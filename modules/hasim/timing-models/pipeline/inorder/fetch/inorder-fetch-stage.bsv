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

typedef struct
{
    ISA_ADDRESS pc;
    IMEM_EPOCH epoch;
    INSTQ_CREDIT_COUNT credits;
}
FETCH_STATE deriving (Eq, Bits);

FETCH_STATE initFetchState = 
    FETCH_STATE
    {
        pc: `PROGRAM_START_ADDR,
        epoch: initialIMemEpoch,
        credits: fromInteger(valueof(NUM_INSTQ_CREDITS))
    };

// mkFetch

module [HASIM_MODULE] mkFetch ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_fetch.out");


    // ****** Model State (per instance) ******

    MULTIPLEXED_STATE_POOL#(NUM_CPUS, FETCH_STATE) statePool <- mkMultiplexedStatePool(initFetchState);

    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, INSTQ_CREDIT_COUNT)                     creditFromInstQ <- mkPortRecv_Multiplexed("InstQ_to_Fet_credit", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH))  newPCFromPCCalc <- mkPortRecv_Multiplexed("PCCalc_to_Fet_newpc", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ITLB_INPUT) pcToITLB <- mkPortSend_Multiplexed("CPU_to_ITLB_req");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) pcToBP <- mkPortSend_Multiplexed("Fet_to_BP_pc");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) pcToLP <- mkPortSend_Multiplexed("Fet_to_LP_pc");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) newPCFromLP     <- mkPortRecvDependent_Multiplexed("LP_to_Fet_newpc");

    // ****** Local Controller ******
        
    Vector#(3, INSTANCE_CONTROL_IN#(NUM_CPUS))  inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS))  depports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = creditFromInstQ.ctrl;
    inports[1]  = newPCFromPCCalc.ctrl;
    inports[2]  = statePool.ctrl;
    depports[0] = newPCFromLP.ctrl;
    outports[0] = pcToITLB.ctrl;
    outports[1] = pcToBP.ctrl;
    outports[2] = pcToLP.ctrl;
    
    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("Fetch", inports, depports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, FETCH_STATE) stage2Ctrl <- mkBufferedStageController();


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
        modelCycle.send(cpu_iid);
        
        // Get our local state using the instance.
        let local_state <- statePool.extractState(cpu_iid);
        
        // Get credits from the instruction queue and update our count of them.
        let m_creditFromInstQ <- creditFromInstQ.receive(cpu_iid);
        local_state.credits = local_state.credits + fromMaybe(0, m_creditFromInstQ);

        // Get the next PC from PCCalc for redirects
        let m_pcFromPCCalc <- newPCFromPCCalc.receive(cpu_iid);
        
        // Update the PC and front end epochs.
        if (m_pcFromPCCalc matches tagged Valid {.new_pc, .new_epoch})
        begin
            debugLog.record_next_cycle(cpu_iid, $format("REDIRECT TO PC:0x%h", new_pc) + $format(" EPOCH:0x%0h", new_epoch));

            local_state.pc = new_pc;
            local_state.epoch = new_epoch;
        end

        // Send the pc to the line predictor
        // We always request a line prediction, even if we don't have a credit
        // in the instruction queue. (Is this OKAY?)
        pcToLP.send(cpu_iid, tagged Valid local_state.pc);

        stage2Ctrl.ready(cpu_iid, local_state);

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

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // Get the line prediction
        // assert isValid(m_line_prediction)
        let m_line_prediction <- newPCFromLP.receive(cpu_iid);
        let line_prediction = validValue(m_line_prediction);
        
        // See if we have any credits for the instruction queue.
        if (local_state.credits != 0)
        begin
        
            // The instructionQ still has room...
            // Send the current PC to the ITLB and Branch predictor and line
            // predictor
            pcToITLB.send(cpu_iid, tagged Valid initIMemBundle(local_state.epoch, local_state.pc, line_prediction, cpu_iid));
            pcToBP.send(cpu_iid, tagged Valid local_state.pc);

            // Set the pc as predicted
            local_state.pc = line_prediction;

            // Use the credit
            local_state.credits = local_state.credits - 1;
        
            // End of model cycle. (Path 1)
            eventFet.recordEvent(cpu_iid, tagged Valid truncate(local_state.pc));
            statFet.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("FETCH ADDR:0x%h", local_state.pc));
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
        
        statePool.insertState(cpu_iid, local_state);
        
    endrule

endmodule
