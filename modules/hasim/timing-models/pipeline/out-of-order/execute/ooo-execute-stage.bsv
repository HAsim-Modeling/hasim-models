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

import FShow::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

`include "asim/provides/funcp_interface.bsh"


// ****** Generated Files ******

`include "asim/dict/EVENTS_EXECUTE.bsh"
`include "asim/dict/STATS_EXECUTE.bsh"


// mkExecute

// Inorder execute module which models a lone single-stage ALU. 
// Performs address calculation for memory ops.
// Performs branch resolution and resteers the fetch unit on a misprediction.
// Also trains the branch predictor.

// All operations, even non-memory ones, are sent to the MemQ.

// This module is pipelined across instances. Stages:

// Stage 1 -> Stage 2
// These stages never stall.

// Possible ways the model cycle can end:
//   Path 1: Either the MemQ is full, or the IssueQ is empty, so there's a bubble.
//   Path 2: The instruction in the IssueQ is of the wrong epoch, so we drop it. stall.
//   Path 3: The instruction was executed and enqued in the MemQ.

typedef union tagged
{
    DMEM_BUNDLE STAGE2_junk;
    BUNDLE      STAGE2_exeRsp;
    void        STAGE2_bubble;
}
EXE_STAGE2_STATE deriving (Bits, Eq);

typedef struct
{
    Maybe#(TOKEN) token;
    TOKEN_BRANCH_EPOCH expectedEpoch;
}
EXE_BRANCH_INFO deriving (Eq, Bits);

EXE_BRANCH_INFO initialBranchInfo =
    EXE_BRANCH_INFO
    {
        token: tagged Invalid,
        expectedEpoch: 0
    };

module [HASIM_MODULE] mkExecute ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_execute.out");

    // ****** Ports ******

    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, BUNDLE) bundleFromIssueQ <- mkPortStallRecv_Multiplexed("IssueQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE)     bundleToMemQ <- mkPortStallSend_Multiplexed("DTLBQ");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple3#(TOKEN, TOKEN_FAULT_EPOCH, ISA_ADDRESS)) rewindToFet <- mkPortSend_Multiplexed("Exe_to_Fet_rewind");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple2#(TOKEN, TOKEN_FAULT_EPOCH)) rewindToDec  <- mkPortSend_Multiplexed("Exe_to_Dec_mispredict");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BRANCH_PRED_TRAIN) trainingToBP <- mkPortSend_Multiplexed("Exe_to_BP_training");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE) writebackToDec <- mkPortSend_Multiplexed("Exe_to_Dec_writeback");

    // ****** Local State ******
    
    MULTIPLEXED_STATE_POOL#(NUM_CPUS, EXE_BRANCH_INFO) statePool <- mkMultiplexedStatePool(initialBranchInfo);

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS) getResults <- mkConnection_Client("funcp_getResults");


    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromIssueQ.ctrl.in;
    inports[1]  = bundleToMemQ.ctrl.in;
    inports[2]  = statePool.ctrl;
    outports[0] = bundleToMemQ.ctrl.out;
    outports[1] = rewindToFet.ctrl;
    outports[2] = writebackToDec.ctrl;
    outports[3] = trainingToBP.ctrl;
    outports[4] = bundleFromIssueQ.ctrl.out;
    outports[5] = rewindToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, Tuple2#(EXE_STAGE2_STATE, EXE_BRANCH_INFO)) stage2Ctrl <- mkStageController();


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventExe <- mkEventRecorder_Multiplexed(`EVENTS_EXECUTE_INSTRUCTION_EXECUTE);

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statMispred <- mkStatCounter_Multiplexed(`STATS_EXECUTE_BPRED_MISPREDS);


    // ****** Helper functions ******

    // nonBranchPred 
    
    // Used for handling branch prediction / rewind feedback
    // processing for all non-branch instructions.

    function Action nonBranchPred(BUNDLE bundle, FUNCP_RSP_GET_RESULTS rsp, BRANCH_ATTR branchAttr);
    action
    
        let tok = bundle.token;
        
        // Get our local state from the context.
        let cpu_iid = tokCpuInstanceId(tok);
        
        // Calculate the next PC.
        let tgt = rsp.instructionAddress + zeroExtend(rsp.instructionSize);

        // Marshall up some training info.
        BRANCH_PRED_TRAIN train;
        train.token = tok;
        train.branchPC = rsp.instructionAddress;
        train.exeResult = tagged NotBranch;

        if (branchAttr matches tagged NotBranch)
        begin
            // Don't rewind the PC or train the BP.
            rewindToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);
            trainingToBP.send(cpu_iid, tagged Invalid);
        end
        else if (branchAttr matches tagged BranchNotTaken .pred_tgt &&&
                 tgt == pred_tgt)
        begin
            // Treated non-branch instruction as a branch but got the right answer.
            // Train BP but no need to rewind
            debugLog.record(cpu_iid, fshow("NON-BRANCH PREDICTED NOT TAKEN BRANCH: ") + fshow(tok));
            rewindToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);

            train.predCorrect = True;
            trainingToBP.send(cpu_iid, tagged Valid train);
        end
        else
        begin
        
            // Treated a non-branch as a branch and got the wrong answer.
            debugLog.record(cpu_iid, fshow("NON-BRANCH PREDICTED BRANCH: ") + fshow(tok));
            statMispred.incr(cpu_iid);

            // Rewind the PC and train the BP.
            rewindToFet.send(cpu_iid, tagged Valid tuple3(tok, bundle.faultEpoch, tgt));
            rewindToDec.send(cpu_iid, tagged Valid tuple2(tok, bundle.faultEpoch));
            train.predCorrect = False;
            trainingToBP.send(cpu_iid, tagged Valid train);

        end
        
    endaction
    endfunction

    
    // stage1_begin
    
    // Begin a new model cycle. Check if the IssueQ has an 
    // instruction and the MemQ has space. If so, then send it to
    // the functional partition for execution.

    (* conservative_implicit_conditions *)
    rule stage1_begin (True);
    
        // Get our local state from the context.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        let oldest_branch_info <- statePool.extractState(cpu_iid);

        // Do we have an instruction to execute, and a place to put it?
        let m_bundle <- bundleFromIssueQ.receive(cpu_iid);
        let can_enq <- bundleToMemQ.canEnq(cpu_iid);

        if (m_bundle matches tagged Valid .bundle &&& can_enq)
        begin

            // Yes... but is it something we should be executing?
            let tok = bundle.token;
        
            // See if we've recently seen a branch.
            if (oldest_branch_info.token matches tagged Valid .branch_tok)
            begin
                // If this inst is younger, we should only execute it if it's from
                // the new epoch.
                if (!tokenIsOlderOrEq(tokTokenId(tok), tokTokenId(branch_tok)))
                begin
                    if (bundle.branchEpoch == oldest_branch_info.expectedEpoch)
                    begin

                        // The new epoch has arrived.
                        oldest_branch_info.token = tagged Invalid;

                        // This token is older than the oldest branch we've seen, 
                        // so we should execute it as normal. It may even be a branch itself.
                        debugLog.record_next_cycle(cpu_iid, fshow("EXEC (NEW EPOCH): ") + fshow(tok) + $format(", pc = 0x%h", bundle.pc));

                        // Have the functional partition execute it.
                        getResults.makeReq(initFuncpReqGetResults(tok));

                        // In the next stage we'll get the response and deal with it.
                        stage2Ctrl.ready(cpu_iid, tuple2(tagged STAGE2_exeRsp bundle, oldest_branch_info));

                    end
                    else
                    begin

                        // It was sitting in the issue q after a mispredicted branch. Mark as dummy.
                        debugLog.record_next_cycle(cpu_iid, fshow("MARK AS DUMMY: ") + fshow(tok));
                        tok.dummy = True;

                        let dmem_bundle = initDMemBundle(tok, bundle.effAddr, bundle.faultEpoch, bundle.isLoad, bundle.isStore, bundle.isTerminate, bundle.dests);

                        // In the next stages we'll pass the junk instruction on, and mark its destinations ready.
                        stage2Ctrl.ready(cpu_iid, tuple2(tagged STAGE2_junk dmem_bundle, oldest_branch_info));

                    end
                end
                else
                begin
                    // This token is older than the oldest branch we've seen, 
                    // so we should execute it as normal. It may even be a branch itself.
                    debugLog.record_next_cycle(cpu_iid, fshow("EXEC (OLDER): ") + fshow(tok) + $format(", pc = 0x%h", bundle.pc));

                    // Have the functional partition execute it.
                    getResults.makeReq(initFuncpReqGetResults(tok));

                    // In the next stage we'll get the response and deal with it.
                    stage2Ctrl.ready(cpu_iid, tuple2(tagged STAGE2_exeRsp bundle, oldest_branch_info));

                end
            end
            else
            begin
            
                // Execute as normal.         
                debugLog.record_next_cycle(cpu_iid, fshow("EXEC: ") + fshow(tok) + $format(", pc = 0x%h", bundle.pc));

                // Have the functional partition execute it.
                getResults.makeReq(initFuncpReqGetResults(tok));
                
                // In the next stage we'll get the response and deal with it.
                stage2Ctrl.ready(cpu_iid, tuple2(tagged STAGE2_exeRsp bundle, oldest_branch_info));

            end

        end
        else
        begin

            // A bubble. 
            debugLog.record_next_cycle(cpu_iid, fshow("BUBBLE"));
            
            // Tell the next stage to propogate the bubble.
            stage2Ctrl.ready(cpu_iid, tuple2(tagged STAGE2_bubble, oldest_branch_info));

        end
    
    endrule

    // stage2_results
    
    // Get the response from the functional partition and resolve 
    // any branches.
    
    // If we got here then we know the IssueQ is not empty and the MemQ is not full.

    rule stage2_results (True);
    
        match {.cpu_iid, {.state, .oldest_branch_info}} <- stage2Ctrl.nextReadyInstance();
    
        // Handle writing our output ports based on what the previous stage decided.

        if (state matches tagged STAGE2_bubble)
        begin
            
            // Last stage determined that there was a bubble, or that we should delay for a cycle.
            
            // Propogate the bubble.
            bundleFromIssueQ.noDeq(cpu_iid);
            bundleToMemQ.noEnq(cpu_iid);
            rewindToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);
            trainingToBP.send(cpu_iid, tagged Invalid);
            writebackToDec.send(cpu_iid, tagged Invalid);

            // End the model cycle. (Path 1)
            eventExe.recordEvent(cpu_iid, tagged Invalid);
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE2_junk .dmem_bundle)
        begin
        
            // Bad path due to branch. Send the junk bundle on.
            bundleToMemQ.doEnq(cpu_iid, dmem_bundle);

            // Dequeue the IssueQ
            bundleFromIssueQ.doDeq(cpu_iid);

            // Tell decode the instruction was dropped, so its dests are "ready."
            writebackToDec.send(cpu_iid, tagged Valid genBusMessage(dmem_bundle.token, dmem_bundle.dests));

            // Propogate the bubble.
            rewindToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);
            trainingToBP.send(cpu_iid, tagged Invalid);

            // End the model cycle. (Path 2)
            eventExe.recordEvent(cpu_iid, tagged Invalid);
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        else if (state matches tagged STAGE2_exeRsp .bundle)
        begin

            let new_bundle = bundle;

            // Get the response from the functional partition.
            let rsp = getResults.getResp();
            getResults.deq();

            // Get our local state from the context.
            let tok = rsp.token;
            let res = rsp.result;

            // Dequeue the IssueQ.
            bundleFromIssueQ.doDeq(cpu_iid);

            // Let's begin to gather some training data for the branch predictor.
            let pc = bundle.pc;
            BRANCH_PRED_TRAIN train;
            train.token = tok;
            train.branchPC = pc;

            // What was the result of execution?
            case (res) matches
                tagged RBranchTaken .addr:
                begin

                    // A branch was taken.
                    debugLog.record(cpu_iid, fshow("BRANCH TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
                    train.exeResult = tagged BranchTaken addr;

                    if (bundle.branchAttr matches tagged BranchTaken .tgt &&& tgt == addr)
                    begin

                        // It was predicted correctly. Train, but don't resteer.
                        debugLog.record(cpu_iid, fshow("(AS PREDICTED)"));
                        rewindToFet.send(cpu_iid, tagged Invalid);
                        rewindToDec.send(cpu_iid, tagged Invalid);
                        train.predCorrect = True;
                        trainingToBP.send(cpu_iid, tagged Valid train);

                    end
                    else
                    begin

                        // The branch predictor predicted NotTaken.
                        debugLog.record(cpu_iid, $format("(MIS PREDICTED)  END-OF-EPOCH:%d", bundle.branchEpoch));
                        statMispred.incr(cpu_iid);

                        // Rewind the PC to the actual target and train the BP.
                        rewindToFet.send(cpu_iid, tagged Valid tuple3(tok, bundle.faultEpoch, addr));
                        rewindToDec.send(cpu_iid, tagged Valid tuple2(tok, bundle.faultEpoch));
                        train.predCorrect = False;
                        trainingToBP.send(cpu_iid, tagged Valid train);
                        
                        // Record info so we won't execute younger instructions.
                        oldest_branch_info.token = tagged Valid tok;
                        oldest_branch_info.expectedEpoch = oldest_branch_info.expectedEpoch + 1;

                    end
                end
                tagged RBranchNotTaken .addr:
                begin

                    // It was a branch, but it was not taken.
                    debugLog.record(cpu_iid, fshow("BRANCH NOT-TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
                    train.exeResult = tagged BranchNotTaken addr;

                    case (bundle.branchAttr) matches
                        tagged BranchNotTaken .tgt:
                        begin

                            // The predictor got it right. No need to resteer.
                            debugLog.record(cpu_iid, fshow("(AS PREDICTED)"));
                            rewindToFet.send(cpu_iid, tagged Invalid);
                            rewindToDec.send(cpu_iid, tagged Invalid);
                            train.predCorrect = True;
                            trainingToBP.send(cpu_iid, tagged Valid train);

                        end
                        tagged BranchTaken .tgt:
                        begin

                            // The predictor got it wrong.
                            statMispred.incr(cpu_iid);

                            // The branch predictor predicted NotTaken.
                            debugLog.record(cpu_iid, $format("(MIS PREDICTED)  END-OF-EPOCH:%d", bundle.branchEpoch));

                            // Resteer the PC to the actual destination and train.
                            rewindToFet.send(cpu_iid, tagged Valid tuple3(tok, bundle.faultEpoch, addr));
                            rewindToDec.send(cpu_iid, tagged Valid tuple2(tok, bundle.faultEpoch));
                            train.predCorrect = False;
                            trainingToBP.send(cpu_iid, tagged Valid train);

                            // Record info so we won't execute younger instructions.
                            oldest_branch_info.token = tagged Valid tok;
                            oldest_branch_info.expectedEpoch = oldest_branch_info.expectedEpoch + 1;

                        end
                        tagged NotBranch:
                        begin

                            // The predictor was wrong, but it was harmless.
                            // Note: Should we train in this case?
                            debugLog.record(cpu_iid, fshow("(ACCIDENTALLY PREDICTED)"));
                            rewindToFet.send(cpu_iid, tagged Invalid);
                            rewindToDec.send(cpu_iid, tagged Invalid);
                            trainingToBP.send(cpu_iid, tagged Invalid);

                        end

                    endcase


                end
                tagged REffectiveAddr .ea:
                begin

                    // The instruction was a memory operation to this address.
                    debugLog.record(cpu_iid, fshow("EFF ADDR: ") + fshow(tok) + fshow(" ADDR:") + fshow(ea));

                    // Update the bundle for the DMem module.
                    new_bundle.effAddr = ea;

                    // Use the standard branch prediction for non-branches.
                    nonBranchPred(new_bundle, rsp, bundle.branchAttr);

                end
                tagged RNop:
                begin

                    // The instruction was some kind of instruction we don't care about.
                    nonBranchPred(bundle, rsp, bundle.branchAttr);

                end
                tagged RTerminate .pf:
                begin

                    // Like a nop, but if this instruction commits, then simulation should end.
                    rewindToFet.send(cpu_iid, tagged Invalid);
                    rewindToDec.send(cpu_iid, tagged Invalid);
                    new_bundle.isTerminate = tagged Valid pf;
                    trainingToBP.send(cpu_iid, tagged Invalid);

                end

            endcase

            if (bundle.isLoad)
            begin

                // The destinations won't be ready until the Mem stage is finished.
                debugLog.record(cpu_iid, fshow(tok) + fshow(": load -- dest regs not yet valid"));
                // No writebacks to report.
                writebackToDec.send(cpu_iid, tagged Invalid);

            end
            else
            begin

                // The destinations of this token are ready.
                debugLog.record(cpu_iid, fshow(tok) + fshow(": marking dest regs valid"));

                // Send the writeback to decode.
                writebackToDec.send(cpu_iid, tagged Valid genBusMessage(tok, bundle.dests));

            end

            // Update the bundle with any token updates from the FP.
            new_bundle.token = tok;

            // Enqueue the instuction in the MemQ.
            bundleToMemQ.doEnq(cpu_iid, initDMemBundle(new_bundle.token, bundle.effAddr, bundle.faultEpoch, bundle.isLoad, bundle.isStore, new_bundle.isTerminate, bundle.dests));

            // End the model cycle. (Path 3)
            eventExe.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));
            localCtrl.endModelCycle(cpu_iid, 3);

        end
        
        statePool.insertState(cpu_iid, oldest_branch_info);

    endrule

endmodule
