//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_fetch.out");


    // ****** Model State (per instance) ******

    MULTIPLEXED_STATE_POOL#(MAX_NUM_CPUS, FETCH_STATE) statePool <- mkMultiplexedStatePool(initFetchState);

    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_CREDIT_COUNT) creditFromInstQ <- mkPortRecv_Multiplexed("InstQ_to_Fet_credit", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH)) newPCFromPCCalc <- mkPortRecv_Multiplexed("PCCalc_to_Fet_newpc", 1);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ITLB_INPUT) pcToITLB <- mkPortSend_Multiplexed("CPU_to_ITLB_req");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ISA_ADDRESS) pcToBP <- mkPortSend_Multiplexed("Fet_to_BP_pc");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BRANCH_ATTR) attrToPCCALC <- mkPortSend_Multiplexed("Fet_to_PCCALC_attr");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BRANCH_ATTR) newPCFromBP <- mkPortRecvDependent_Multiplexed("BP_to_Fet_newpc");

    // ****** Local Controller ******
        
    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  depports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = creditFromInstQ.ctrl;
    inports[1]  = newPCFromPCCalc.ctrl;
    inports[2]  = statePool.ctrl;
    depports[0] = newPCFromBP.ctrl;
    outports[0] = pcToITLB.ctrl;
    outports[1] = pcToBP.ctrl;
    outports[2] = attrToPCCALC.ctrl;
    
    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("Fetch", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, FETCH_STATE) stage2Ctrl <- mkBufferedStageController();


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventFet <- mkEventRecorder_Multiplexed(`EVENTS_FETCH_INSTRUCTION_FET);

    STAT_VECTOR#(MAX_NUM_CPUS) statCycles <-
        mkStatCounter_Multiplexed(statName("MODEL_FETCH_TOTAL_CYCLES",
                                           "Total Cycles"));
    STAT_VECTOR#(MAX_NUM_CPUS) statFet <-
        mkStatCounter_Multiplexed(statName("MODEL_FETCH_INSTS_FETCHED",
                                           "Instructions Fetched"));

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
    // * pcToBP

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
        pcToBP.send(cpu_iid, tagged Valid local_state.pc);

        stage2Ctrl.ready(cpu_iid, local_state);

    endrule

    // stage2_fetchReq
    // If we have a credit for the instruction queue, send fetch request to
    // ITLB and Branch Predictors.
    //
    // Ports read:
    // * newPCFromBP
    //
    // Ports written:
    // * pcToITLB
    // * attrToPCCALC

    (* conservative_implicit_conditions *)
    rule stage2_fetchReq (True);

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // Get the branch prediction
        let m_branch_prediction <- newPCFromBP.receive(cpu_iid);
        
        // See if we have any credits for the instruction queue.
        if (local_state.credits != 0)
        begin
        
            // Set the pc as predicted
            ISA_ADDRESS next_pc;
            if (m_branch_prediction matches tagged Valid .pred &&&
                pred matches tagged BranchTaken .tgt)
            begin
                next_pc = tgt;
            end
            else
            begin
                next_pc = local_state.pc + 4;
            end

            // The instructionQ still has room...
            // Send the current PC to the ITLB
            pcToITLB.send(cpu_iid, tagged Valid initIMemBundle(local_state.epoch, local_state.pc, next_pc, cpu_iid));
            // Send details of the branch prediction to PCCALC
            attrToPCCALC.send(cpu_iid, m_branch_prediction);

            // Use the credit
            local_state.credits = local_state.credits - 1;
        
            // End of model cycle. (Path 1)
            eventFet.recordEvent(cpu_iid, tagged Valid truncate(local_state.pc));
            statFet.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("FETCH ADDR:0x%h", local_state.pc));

            // Update local state for next cycle
            local_state.pc = next_pc;

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
            attrToPCCALC.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        
        statePool.insertState(cpu_iid, local_state);
        
    endrule

endmodule
