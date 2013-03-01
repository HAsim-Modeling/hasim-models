//
// Copyright (C) 2009 Massachusetts Institute of Technology
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


// ASSUMPTIONS made by pccalc:
//   The input from BranchPrediction refers to the same instruction bundle as
//   the input from IMem.
// and probably more I don't realize yet.

// PATHS:
// 1. No fault from back end.
// 2. Fault from back end.
// 3. Rewind from back end.

typedef struct {
    ISA_ADDRESS addr;
    Maybe#(TOKEN) rewind;
} PCC_REDIRECT deriving(Bits, Eq);

// ****** Modules ******


module [HASIM_MODULE] mkPCCalc
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_pccalc.out");

    // ****** Model State (per instance) ******

    MULTIPLEXED#(MAX_NUM_CPUS, Reg#(MULTITHREADED#(IMEM_EPOCH)))  epochsPool <- mkMultiplexed(mkReg(multithreaded(initialIMemEpoch)));

    MULTIPLEXED#(MAX_NUM_CPUS, Reg#(MULTITHREADED#(Maybe#(PCC_REDIRECT)))) redirectsPool <- mkMultiplexed(mkReg(multithreaded(Invalid)));

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");
 /*
    Connection_Client#(FUNCP_REQ_HANDLE_FAULT,
                       FUNCP_RSP_HANDLE_FAULT)       handleFault <- mkConnection_Client("funcp_handleFault");
 */

    // ****** Ports ******


    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_ENQUEUE)                  enqToInstQ <- mkPortSend_Multiplexed("Fet_to_InstQ_enq");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, THREAD_ID)                      clearToInstQ  <- mkPortSend_Multiplexed("Fet_to_InstQ_clear");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, Tuple3#(THREAD_ID, ISA_ADDRESS, IMEM_EPOCH))  nextPCToFetch <- mkPortSend_Multiplexed("PCCalc_to_Fet_newpc");

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN, ISA_ADDRESS))                    faultFromCom  <- mkPortRecv_Multiplexed("Com_to_Fet_fault", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple3#(TOKEN, TOKEN_FAULT_EPOCH, ISA_ADDRESS)) rewindFromExe <- mkPortRecv_Multiplexed("Exe_to_Fet_rewind", 1);

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, IMEM_OUTPUT) rspFromIMem <- mkPortRecv_Multiplexed("IMem_to_Fet_response", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ISA_ADDRESS) predFromBP  <- mkPortRecv_Multiplexed("BP_to_Fet_pred", 2);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BRANCH_ATTR) attrFromBP  <- mkPortRecv_Multiplexed("BP_to_Fet_attr", 2);


    // ****** Local Controller ******

    Vector#(5, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();

    inctrls[0] = faultFromCom.ctrl;
    inctrls[1] = rewindFromExe.ctrl;
    inctrls[2] = rspFromIMem.ctrl;
    inctrls[3] = predFromBP.ctrl;
    inctrls[4] = attrFromBP.ctrl;
    outctrls[0]  = enqToInstQ.ctrl;
    outctrls[1]  = nextPCToFetch.ctrl;
    outctrls[2]  = clearToInstQ.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    STAGE_CONTROLLER_VOID#(MAX_NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();

    // True if we should get a rewind response, false otherwise.
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage3Ctrl <- mkStageController();


    // ****** Stats ******
    STAT_VECTOR#(MAX_NUM_CPUS) statLpBpMismatches <-
        mkStatCounter_Multiplexed(statName("PCCALC_LP_BP_MISMATCHES",
                                           "Line Prediction/Branch Prediction Mismatches"));

    // ****** Rules ******
    
    // Ports read:
    // * rewindFromExe
    // * faultFromCom
    // * rspFromIMem
    // * predFromBP
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
        Reg#(MULTITHREADED#(IMEM_EPOCH)) epochsReg = epochsPool[cpu_iid];
        MULTITHREADED#(IMEM_EPOCH) epochs = epochsReg;
        Reg#(MULTITHREADED#(Maybe#(PCC_REDIRECT))) redirectsReg = redirectsPool[cpu_iid];
        let redirects = redirectsReg;

        // Receive potential new PCs from our incoming ports.
        let m_rewind     <- rewindFromExe.receive(cpu_iid);
        let m_fault      <- faultFromCom.receive(cpu_iid);
        let m_imem_rsp  <- rspFromIMem.receive(cpu_iid);
        let m_pred_pc    <- predFromBP.receive(cpu_iid);
        let m_attr <- attrFromBP.receive(cpu_iid);

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
            THREAD_ID thread = tokThreadId(tok);
            debugLog.record_next_cycle(cpu_iid, fshow("FAULT: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            epochs[thread].fault = epochs[thread].fault + 1;

            if (isValid(redirects[thread]))
            begin
                // TODO: handle this case!
                debugLog.record_next_cycle(cpu_iid, fshow("WARNING: redirect collision in pccalc"));
            end
            redirects[thread] = tagged Valid PCC_REDIRECT { addr: addr, rewind: tagged Valid tok };

        end

        if (m_rewind matches tagged Valid { .tok, .fault_epoch, .addr} &&&
                 fault_epoch == epochs[tokThreadId(tok)].fault)
        begin

            // A branch misprediction occured.
            // Epoch check ensures we haven't already redirected from a fault.
            THREAD_ID thread = tokThreadId(tok);
            debugLog.record_next_cycle(cpu_iid, fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            epochs[thread].branch = epochs[thread].branch + 1;

            if (isValid(redirects[thread]))
            begin
                // TODO: handle this case!
                debugLog.record_next_cycle(cpu_iid, fshow("WARNING: redirect collision in pccalc"));
            end
            redirects[thread] = tagged Valid PCC_REDIRECT { addr: addr, rewind: tagged Valid tok};
        end

        Bool enqueue = ?;

        // Figure out what to do with the INSTQ.
        if (m_imem_rsp matches tagged Valid .imem_rsp)
        begin

            // If m_imem_rsp is valid, it means m_pred_pc and m_attr must be
            // valid.
            // assert isValid(m_pred_pc);
            let pred_pc = validValue(m_pred_pc);
            let attr = validValue(m_attr);
            let thread = ctxThreadId(imem_rsp.bundle.ctx_id);

            // Check for problems with this bundle.
            if (imem_rsp.bundle.epoch != epochs[thread])
            begin
                // Epoch is wrong. No need to redirect.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("WRONG EPOCH"));
            end
            else if (imem_rsp.response matches tagged IMEM_itlb_fault)
            begin
                // An ITLB fault occured. Increment the itlb epoch, redirect
                // pc to the handler address.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("ITLB FAULT"));
        
                // For now we just go to whatever address we were trying to
                // execute. This will likely have to change in the future.
                if (isValid(redirects[thread]))
                begin
                    // TODO: handle this case!
                    debugLog.record_next_cycle(cpu_iid, fshow("WARNING: redirect collision in pccalc"));
                end
                redirects[thread] = tagged Valid PCC_REDIRECT{ addr: imem_rsp.bundle.virtualAddress, rewind: Invalid};
                epochs[thread].iTLB = epochs[thread].iTLB + 1;
            
            end
            else if (imem_rsp.response matches tagged IMEM_icache_retry)
            begin
                // ICACHE Retry. We need to increment the iCache epoch,
                // redirect the PC.
                enqueue = False;
                debugLog.record_next_cycle(cpu_iid, $format("ICACHE RETRY"));

                if (isValid(redirects[thread]))
                begin
                    // TODO: handle this case!
                    debugLog.record_next_cycle(cpu_iid, fshow("WARNING: redirect collision in pccalc"));
                end
                redirects[thread] = tagged Valid PCC_REDIRECT { addr: imem_rsp.bundle.virtualAddress, rewind: Invalid};
                epochs[thread].iCache = epochs[thread].iCache + 1;
            end
            else if (imem_rsp.bundle.linePrediction != pred_pc)
            begin
                // Line prediction doesn't match branch prediction.
                // Increment the epoch (we use iCache epoch for this).
                // Redirect to the branch prediction.
                // Even though the line prediction was wrong, this instruction
                // is still correct, so we still want to enqueue it on the
                // INSTQ.
                enqueue = True;
                statLpBpMismatches.incr(cpu_iid);
                debugLog.record_next_cycle(cpu_iid, $format("LP BP Mismatch.  LP: 0x%0h, BP: 0x%0h", imem_rsp.bundle.linePrediction, pred_pc));

                if (isValid(redirects[thread]))
                begin
                    // TODO: handle this case!
                    debugLog.record_next_cycle(cpu_iid, fshow("WARNING: redirect collision in pccalc"));
                end
                redirects[thread] = tagged Valid PCC_REDIRECT { addr: pred_pc, rewind: Invalid};
                epochs[thread].iCache = epochs[thread].iCache+1;
            end
            else
            begin
                // Normal flow. Everything's happy. Enqueue the bundle.
                enqueue = True;
                debugLog.record_next_cycle(cpu_iid, fshow("HAPPY"));
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
                    branchAttr: attr,
                    thread: thread
                };
            end

            let instq_enq = INSTQ_ENQUEUE {
                bundle: bundle,
                missID: miss_id,
                thread: thread
            };
            enqToInstQ.send(cpu_iid, tagged Valid instq_enq);

        end
        else
        begin
            // There's a bubble from the front end.
            enqToInstQ.send(cpu_iid, Invalid);
            debugLog.record_next_cycle(cpu_iid, fshow("BUBBLE"));
        end

        stage2Ctrl.ready(cpu_iid);

        epochsReg <= epochs;
        redirectsReg <= redirects;
    endrule
    

    // stage2_redirect
    
    // Get fault response, rewind response, or do pc redirection.
    // Ports read:
    //  (none)
    // Ports written:
    // * nextPCToFetch

    rule stage2_redirect (True);

        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get our state based on the current cpu instance.
        let epochs = epochsPool[cpu_iid];
        Reg#(MULTITHREADED#(Maybe#(PCC_REDIRECT))) redirects = redirectsPool[cpu_iid];

        Bool rewind = False;

        // Pick a redirect to do if we can find one.
        let m_idx = findIndex(isValid, redirects);
        if (m_idx matches tagged Valid .idx)
        begin
            THREAD_ID thread = unpack(pack(idx));

            let redirect = validValue(redirects[thread]);
            nextPCToFetch.send(cpu_iid, tagged Valid tuple3(thread, redirect.addr, epochs[thread]));

            if (redirect.rewind matches tagged Valid .tok)
            begin
                rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
                clearToInstQ.send(cpu_iid, tagged Valid thread);
                rewind = True;
            end
            else
            begin
                clearToInstQ.send(cpu_iid, Invalid);
            end

            redirects[thread] <= Invalid;

        end
        else
        begin
            // No redirects to do.
            nextPCToFetch.send(cpu_iid, tagged Invalid);
            clearToInstQ.send(cpu_iid, tagged Invalid);
        end

        stage3Ctrl.ready(cpu_iid, rewind);
    endrule

    rule stage3_rewindResp; 

        match {.cpu_iid, .rewind} <- stage3Ctrl.nextReadyInstance();
    
        if (rewind)
        begin
            rewindToToken.deq();
        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

endmodule

