//
// Copyright (C) 2008 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

// ****** Bluespec imports ******

import FIFO::*;
import FShow::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"


// ****** Timing Model imports *****

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/l1_cache_base_types.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"

// ****** Modules ******

interface SMTFetch;
    method Action setNumThreadsPerCore(Bit#(32) numThreads);
endinterface

// mkFetch

module [HASIM_MODULE] mkFetch (SMTFetch);

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_fetch.out");


    // ****** Model State (per instance) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(ISA_ADDRESS)))    pcsPool <- mkMultiplexed(mkReg(multithreaded(`PROGRAM_START_ADDR)));
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(IMEM_EPOCH)))  epochsPool <- mkMultiplexed(mkReg(multithreaded(initIMemEpoch(0, 0, 0, 0))));
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(INSTQ_CREDIT_COUNT))) creditsPool <- mkMultiplexed(mkReg(multithreaded(fromInteger(valueof(NUM_INSTQ_CREDITS)))));

    MULTIPLEXED#(NUM_CPUS, Reg#(THREAD_ID)) curThreadPool <- mkMultiplexed(mkReg(0));
    Reg#(THREAD_ID) maxThreadId <- mkReg(maxBound);

    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, MULTITHREADED#(INSTQ_CREDIT_COUNT))                creditFromInstQ <- mkPortRecv_Multiplexed("InstQ_to_Fet_credit", 1);

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(THREAD_ID, ISA_ADDRESS, IMEM_EPOCH))  newPCFromPCCalc <- mkPortRecv_Multiplexed("PCCalc_to_Fet_newpc", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ITLB_INPUT) pcToITLB <- mkPortSend_Multiplexed("CPU_to_ITLB_req");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) pcToBP <- mkPortSend_Multiplexed("Fet_to_BP_pc");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) pcToLP <- mkPortSend_Multiplexed("Fet_to_LP_pc");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) newPCFromLP     <- mkPortRecvDependent_Multiplexed("LP_to_Fet_newpc");

    // ****** Local Controller ******
        
    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = creditFromInstQ.ctrl;
    inports[1]  = newPCFromPCCalc.ctrl;
    outports[0] = pcToITLB.ctrl;
    outports[1] = pcToBP.ctrl;
    outports[2] = pcToLP.ctrl;
    
    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventFet <- mkEventRecorder_Multiplexed(`EVENTS_FETCH_INSTRUCTION_FET);

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCycles  <- mkStatCounter_Multiplexed(`STATS_FETCH_TOTAL_CYCLES);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statFet     <- mkStatCounter_Multiplexed(`STATS_FETCH_INSTS_FETCHED);


    // ****** Rules ******
    
    // stage1_LPReq
    
    // Send the current pc to the line predictor to predict the next pc.
    // The pc we send to the line predictor is whatever we think pc is, unless
    // pccalc sends us a redirected pc, in which case we used the redirected pc.
    //
    // Also update the number of instq credits we have if the instq gives us
    // more.
    //
    // Ports read:
    // * creditFromInstQ
    // * newPCFromPCCalc
    //
    // Ports written:
    // * pcToLP

    (* conservative_implicit_conditions *)
    rule stage1_LPReq (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        statCycles.incr(cpu_iid);
        debugLog.nextModelCycle(cpu_iid);
        modelCycle.send(extend(cpu_iid));
        
        // Get our local state using the instance.
        Reg#(MULTITHREADED#(ISA_ADDRESS)) pcsReg = pcsPool[cpu_iid];
        let pcs = pcsReg;
        Reg#(MULTITHREADED#(IMEM_EPOCH)) epochs = epochsPool[cpu_iid];
        Reg#(MULTITHREADED#(INSTQ_CREDIT_COUNT)) credits = creditsPool[cpu_iid];
        Reg#(THREAD_ID) cur_thread = curThreadPool[cpu_iid];

        // Get credits from the instruction queue and update our count of them.
        let m_creditFromInstQ <- creditFromInstQ.receive(cpu_iid);

        if (m_creditFromInstQ matches tagged Valid .newcredits)
        begin
            function Bit#(n) add(Bit#(n) a, Bit#(n) b) = a + b;

            credits <= zipWith(add, credits, newcredits);
        end

        
        let pc_for_line_prediction = pcs[cur_thread];

        // Get the next PC from PCCalc for redirects
        let m_pcFromPCCalc <- newPCFromPCCalc.receive(cpu_iid);
        
        // Update the PC and front end epochs.
        if (m_pcFromPCCalc matches tagged Valid {.thread, .new_pc, .new_epoch})
        begin
            debugLog.record_next_cycle(cpu_iid, $format("REDIRECT TO PC:0x%h", new_pc) + $format(" THREAD:0x%0h", thread) + $format(" EPOCH:0x%0h", new_epoch));

            pcs[thread] = new_pc;
            pcsReg <= pcs;
            epochs[thread] <= new_epoch;

            pc_for_line_prediction = pcs[cur_thread];
        end

        // Send the pc to the line predictor
        // We always request a line prediction, even if we don't have a credit
        // in the instruction queue.
        pcToLP.send(cpu_iid, tagged Valid pc_for_line_prediction);

        stage2Ctrl.ready(cpu_iid);
    endrule

    // stage2_fetchReq
    // If we have a credit for the instruction queue, send fetch request to
    // ITLB and Branch Predictors.
    //
    // Ports read:
    // * newPCFromLP
    //
    // Ports written:
    // * pcToITLB
    // * pcToBP

    (* conservative_implicit_conditions *)
    rule stage2_fetchReq (True);
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get our local state using the instance.
        Reg#(THREAD_ID) cur_thread = curThreadPool[cpu_iid];
        Reg#(MULTITHREADED#(ISA_ADDRESS)) pcs = pcsPool[cpu_iid];
        let epoch = epochsPool[cpu_iid][cur_thread];
        Reg#(MULTITHREADED#(INSTQ_CREDIT_COUNT)) credits = creditsPool[cpu_iid];

        // Get the line prediction
        // assert isValid(m_line_prediction)
        let m_line_prediction <- newPCFromLP.receive(cpu_iid);
        let line_prediction = validValue(m_line_prediction);
        
        // See if we have any credits for the instruction queue.
        if (credits[cur_thread] != 0)
        begin
        
            // The instructionQ still has room...
            // Send the current PC to the ITLB and Branch predictor and line
            // predictor
            pcToITLB.send(cpu_iid, tagged Valid initIMemBundle(epoch, pcs[cur_thread], line_prediction, getContextId(cpu_iid, cur_thread)));
            pcToBP.send(cpu_iid, tagged Valid pcs[cur_thread]);

            // Set the pc as predicted
            pcs[cur_thread] <= line_prediction;

            // Use the credit
            credits[cur_thread] <= credits[cur_thread] - 1;
        
            // End of model cycle. (Path 1)
            eventFet.recordEvent(cpu_iid, tagged Valid truncate(pcs[cur_thread]));
            statFet.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("FETCH THREAD: 0x%h, ADDR:0x%h", cur_thread, pcs[cur_thread]) + $format(" EPOCH:0x%0h", epoch));
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else
        begin

            // We have no credits for the instructionQ.
            // Nothing we can do.
            debugLog.record(cpu_iid, $format("BUBBLE"));
            eventFet.recordEvent(cpu_iid, tagged Invalid);
            
            // Don't send anything to the ITLB.
            
            // Don't request a new address translation or branch prediction.
            pcToITLB.send(cpu_iid, tagged Invalid);
            pcToBP.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end

        // Round robin over the threads
        cur_thread <= (cur_thread == maxThreadId ? 0 : cur_thread + 1);
        
    endrule

    method Action setNumThreadsPerCore(Bit#(32) numThreads);
        maxThreadId <= truncate(numThreads-1);
    endmethod

endmodule

