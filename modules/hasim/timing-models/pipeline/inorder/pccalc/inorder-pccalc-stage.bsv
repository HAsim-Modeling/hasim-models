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

import FShow::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model imports *****

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"


// ****** Local Datatypes ******


// PCC1_STATE

// Record if the PCCAlc stage stalls for a rewind or fault.

typedef union tagged
{
    void        STAGE2_noredirect;
    ISA_ADDRESS STAGE2_redirect;
    ISA_ADDRESS STAGE2_rewindRsp;
}
PCC_STAGE2_STATE deriving (Bits, Eq);


// ASSUMPTIONS made by pccalc:
//   The input from BranchPrediction refers to the same instruction bundle as
//   the input from IMem.
// and probably more I don't realize yet.

// PATHS:
// 1. No fault from back end.
// 2. Fault from back end.
// 3. Rewind from back end.

// ****** Modules ******


module [HASIM_MODULE] mkPCCalc
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_pccalc.out");

    // ****** Model State (per instance) ******

    MULTIPLEXED_REG#(MAX_NUM_CPUS, IMEM_EPOCH)  epochPool <- mkMultiplexedReg(initialIMemEpoch);

    // ****** Ports ******


    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_ENQUEUE)                        enqToInstQ <- mkPortSend_Multiplexed("Fet_to_InstQ_enq");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                               clearToInstQ <- mkPortSend_Multiplexed("Fet_to_InstQ_clear");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH))  nextPCToFetch <- mkPortSend_Multiplexed("PCCalc_to_Fet_newpc");

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN, ISA_ADDRESS))                    faultFromCom  <- mkPortRecv_Multiplexed("Com_to_Fet_fault", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple3#(TOKEN, TOKEN_FAULT_EPOCH, ISA_ADDRESS)) rewindFromExe <- mkPortRecv_Multiplexed("Exe_to_Fet_rewind", 1);

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, IMEM_OUTPUT) rspFromIMem <- mkPortRecv_Multiplexed("IMem_to_Fet_response", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BRANCH_ATTR) attrFromBP  <- mkPortRecv_Multiplexed("Fet_to_PCCALC_attr", 4);


    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();

    inctrls[0] = faultFromCom.ctrl;
    inctrls[1] = rewindFromExe.ctrl;
    inctrls[2] = rspFromIMem.ctrl;
    inctrls[3] = attrFromBP.ctrl;
    outctrls[0]  = enqToInstQ.ctrl;
    outctrls[1]  = nextPCToFetch.ctrl;
    outctrls[2]  = clearToInstQ.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("PC Calc", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, PCC_STAGE2_STATE) stage2Ctrl <- mkStageController();


    // ****** Stats ******
    STAT_VECTOR#(MAX_NUM_CPUS) statLpBpMismatches <-
        mkStatCounter_Multiplexed(statName("MODEL_PCCALC_LP_BP_MISMATCHES",
                                           "Line Prediction/Branch Prediction Mismatches"));

    // ****** Rules ******
    
    // Ports read:
    // * rewindFromExe
    // * faultFromCom
    // * rspFromIMem
    // * attrFromBP
    //
    // Ports written:
    // * enqToInstQ
    // * clearToInstQ

    (* conservative_implicit_conditions *)
    rule stage1_nextPC (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our state from the instance.
        Reg#(IMEM_EPOCH) epochReg = epochPool.getReg(cpu_iid);
        IMEM_EPOCH epoch = epochReg;

        // Receive potential new PCs from our incoming ports.
        let m_rewind   <- rewindFromExe.receive(cpu_iid);
        let m_fault    <- faultFromCom.receive(cpu_iid);
        let m_imem_rsp <- rspFromIMem.receive(cpu_iid);
        let m_attr     <- attrFromBP.receive(cpu_iid);

        PCC_STAGE2_STATE stage2_redirect_state = tagged STAGE2_noredirect;

        Bool back_end_fault = False;

        // Check for faults and rewinds from the back end.
        // This is only to see if we need to redirect the pc. It has nothing
        // to do with the instruction queue enqueing.
        //
        // If there is a fault or a rewind we increment the epoch, so that the
        // next stage will never try to overide redirect, because the epoch is
        // gaurenteed not to match.
        if (m_fault matches tagged Valid { .tok, .addr })
        begin
            // A fault occurred. Redirect to the given handler address.
            // (If there's no handler this will just be the faulting instruction.
            debugLog.record_next_cycle(cpu_iid, fshow("1: FAULT: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            //rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            epoch.fault = epoch.fault + 1;

            back_end_fault = True;
            stage2_redirect_state = tagged STAGE2_rewindRsp addr;

        end
        else if (m_rewind matches tagged Valid { .tok, .fault_epoch, .addr} &&&
                 fault_epoch == epoch.fault)
        begin

            // A branch misprediction occured.
            // Epoch check ensures we haven't already redirected from a fault.
            debugLog.record_next_cycle(cpu_iid, fshow("1: REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            //rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            epoch.branch = epoch.branch + 1;

            back_end_fault = True;
            stage2_redirect_state = tagged STAGE2_rewindRsp addr;
        end
        else
        begin

            debugLog.record_next_cycle(cpu_iid, fshow("1: NO FAULT/REWIND"));
        
        end

        // If we need to redirect, we put the pc to redirect to in here.
        Maybe#(ISA_ADDRESS) redirect = Invalid;
        Bool enqueue = ?;

        // Figure out what to do with the INSTQ.
        if (m_imem_rsp matches tagged Valid .imem_rsp)
        begin

            // If m_imem_rsp is valid, it means m_attr must be valid.  The direct
            // path from fetch to here must be the same latency as the path
            // through the ITLB and ICACHE.  Since this is a static property,
            // we check only in simulation.
            if (! isValid(m_attr))
            begin
                $display("Branch attr invalid!  Fet_to_PCCALC_attr latency is probably wrong.");
                $stop(1);
            end

            let attr = validValue(m_attr);

            // Check for problems with this bundle.
            if (imem_rsp.bundle.epoch.fault != epoch.fault || imem_rsp.bundle.epoch.branch != epoch.branch || imem_rsp.bundle.epoch.prediction != epoch.prediction)
            begin
                // Epoch is wrong. No need to redirect.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("1: WRONG EPOCH"));
            end
            else if (imem_rsp.response matches tagged IMEM_bad_epoch)
            begin
                // This was dropped by an earlier stage for some reason.
                // Reclaim the credit from the instruction queue.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("1 WRONG EPOCH (PREVIOUSLY DROPPED)"));
            end
            else if (imem_rsp.response matches tagged IMEM_itlb_fault)
            begin
                // An ITLB fault occured. Increment the itlb epoch, redirect
                // pc to the handler address.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("1: ITLB FAULT"));
        
                // For now we just go to whatever address we were trying to
                // execute. This will likely have to change in the future.
                redirect = tagged Valid imem_rsp.bundle.virtualAddress;
                epoch.iTLB = epoch.iTLB + 1;
            
            end
            else if (imem_rsp.response matches tagged IMEM_icache_retry)
            begin
                // ICACHE Retry. We need to increment the iCache epoch,
                // redirect the PC.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("1: ICACHE RETRY"));

                redirect = tagged Valid imem_rsp.bundle.virtualAddress;
                epoch.iCache = epoch.iCache + 1;
            end
            else
            begin
                debugLog.record_next_cycle(cpu_iid, $format("1: NO REDIRECT"));
                // Normal flow. Everything's happy. Enqueue the bundle.
                enqueue = True;
            end

            // Check for icache miss. This is independant of anything we've
            // done so far, because regardless of whether or not we drop the
            // incoming bundle, we need to tell the instruction queue to expect
            // a delayed reply from icache.
            Maybe#(L1_ICACHE_MISS_ID) miss_id = tagged Invalid;
            if (imem_rsp.response matches tagged IMEM_icache_miss .id)
            begin 
                miss_id = tagged Valid id;
            end

            Maybe#(FETCH_BUNDLE) bundle = Invalid;
            if (enqueue)
            begin
                // Make appropriate INSTQ_ENQUEUE
                bundle = tagged Valid FETCH_BUNDLE {
                    branchEpoch: imem_rsp.bundle.epoch.branch,
                    faultEpoch: imem_rsp.bundle.epoch.fault,
                    pc: imem_rsp.bundle.virtualAddress,
                    inst: imem_rsp.bundle.instruction,
                    branchAttr: attr
                };
                debugLog.record_next_cycle(cpu_iid, $format("1: ENQ PC: 0x%h inst: 0x%h", imem_rsp.bundle.virtualAddress, imem_rsp.bundle.instruction));
            end
            else
            begin
                debugLog.record_next_cycle(cpu_iid, $format("1: RECLAIM CREDIT"));
            end

            let instq_enq = INSTQ_ENQUEUE {
                bundle: bundle,
                missID: miss_id
            };
            enqToInstQ.send(cpu_iid, tagged Valid instq_enq);

        end
        else
        begin
            // There's a bubble from the front end.
            debugLog.record_next_cycle(cpu_iid, $format("1: BUBBLE"));
            enqToInstQ.send(cpu_iid, Invalid);
        end

        // Now we deal with pc redirection.
        if (!back_end_fault &&& redirect matches tagged Valid .new_pc)
        begin
            stage2_redirect_state = tagged STAGE2_redirect new_pc;
        end

        clearToInstQ.send(cpu_iid, back_end_fault ? tagged Valid (?) : Invalid);

        stage2Ctrl.ready(cpu_iid, stage2_redirect_state);

        epochReg <= epoch;
    endrule
    

    // stage2_redirect
    
    // Get fault response, rewind response, or do pc redirection.
    // Ports read:
    //  (none)
    // Ports written:
    // * nextPCToFetch

    rule stage2_redirect (True);

        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();

        // Get our state based on the current cpu instance.
        let epoch = epochPool.getReg(cpu_iid);

            
        // Redirect as appropriate.
        if (state matches tagged STAGE2_noredirect) 
        begin
            debugLog.record(cpu_iid, $format("2: NO REDIRECT"));
            nextPCToFetch.send(cpu_iid, tagged Invalid);
        end
        else if (state matches tagged STAGE2_redirect .new_pc)
        begin
            debugLog.record(cpu_iid, $format("2: REDIRECT FINISH (NO REWIND)"));
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, epoch));
        end
        else if (state matches tagged STAGE2_rewindRsp .new_pc)
        begin
            //let rsp = rewindToToken.getResp();
            //rewindToToken.deq();
            debugLog.record(cpu_iid, $format("2: REWIND RSP + REDIRECT FINISH"));
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, epoch));

        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

endmodule

