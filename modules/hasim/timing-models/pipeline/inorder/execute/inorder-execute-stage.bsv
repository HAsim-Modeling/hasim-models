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
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"

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

// This module is pipelined across contexts. Stages:

// Stage 1 -> Stage 2
// These stages never stall.

// Possible ways the model cycle can end:
//   Path 1: Either the MemQ is full, or the IssueQ is empty, so there's a bubble.
//   Path 2: The instruction in the IssueQ is of the wrong epoch, so we drop it. stall.
//   Path 3: The instruction was executed and enqued in the MemQ.


module [HASIM_MODULE] mkExecute ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_execute.out");

    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(TOKEN_EPOCH)) ctx_epoch <- mkMultiCtx(mkReg(initEpoch(0, 0)));


    // ****** Ports ******

    PORT_STALL_RECV_MULTICTX#(BUNDLE) bundleFromIssueQ <- mkPortStallRecv_MultiCtx("IssueQ");
    PORT_STALL_SEND_MULTICTX#(BUNDLE)     bundleToMemQ <- mkPortStallSend_MultiCtx("MemQ");

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, ISA_ADDRESS)) rewindToFet <- mkPortSend_MultiCtx("Exe_to_Fet_rewind");
    PORT_SEND_MULTICTX#(BRANCH_PRED_TRAIN)          trainingToBP <- mkPortSend_MultiCtx("Exe_to_BP_training");

    PORT_SEND_MULTICTX#(BUS_MESSAGE) writebackToDec <- mkPortSend_MultiCtx("Exe_to_Dec_writeback");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS) getResults <- mkConnection_Client("funcp_getResults");


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(5, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromIssueQ.ctrl;
    inports[1]  = bundleToMemQ.ctrl;
    outports[0] = bundleToMemQ.ctrl;
    outports[1] = rewindToFet.ctrl;
    outports[2] = writebackToDec.ctrl;
    outports[3] = trainingToBP.ctrl;
    outports[4] = bundleFromIssueQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTICTX eventExe <- mkEventRecorder_MultiCtx(`EVENTS_EXECUTE_INSTRUCTION_EXECUTE);

    STAT_RECORDER_MULTICTX statMispred <- mkStatCounter_MultiCtx(`STATS_EXECUTE_BPRED_MISPREDS);


    // ****** Helper functions ******

    // goodEpoch
    
    // Is a token in the correct epoch?

    function Bool goodEpoch(TOKEN tok);
        
        let ctx = tokContextId(tok);
        let epoch = ctx_epoch[ctx];
        return tokEpoch(tok) == epoch;

    endfunction


    // nonBranchPred 
    
    // Used for handling branch prediction / rewind feedback
    // processing for all non-branch instructions.

    function Action nonBranchPred(TOKEN tok,
                                  FUNCP_RSP_GET_RESULTS rsp,
                                  BRANCH_ATTR branchAttr);
    action
        
        // Get our local state from the context.
        let ctx = tokContextId(tok);
        Reg#(TOKEN_EPOCH) epoch = ctx_epoch[ctx];
        
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
            rewindToFet.send(ctx, tagged Invalid);
            trainingToBP.send(ctx, tagged Invalid);
        end
        else if (branchAttr matches tagged BranchNotTaken .pred_tgt &&&
                 tgt == pred_tgt)
        begin
            // Treated non-branch instruction as a branch but got the right answer.
            // Train BP but no need to rewind
            debugLog.record(ctx, fshow("NON-BRANCH PREDICTED NOT TAKEN BRANCH: ") + fshow(tok));
            rewindToFet.send(ctx, tagged Invalid);

            train.predCorrect = True;
            trainingToBP.send(ctx, tagged Valid train);
        end
        else
        begin
        
            // Treated a non-branch as a branch and got the wrong answer.
            debugLog.record(ctx, fshow("NON-BRANCH PREDICTED BRANCH: ") + fshow(tok));
            statMispred.incr(ctx);

            epoch.branch <= epoch.branch + 1;

            // Rewind the PC and train the BP.
            rewindToFet.send(ctx, tagged Valid tuple2(tok, tgt));
            train.predCorrect = False;
            trainingToBP.send(ctx, tagged Valid train);

        end
    endaction
    endfunction

    // stage1_begin
    
    // Begin a new model cycle. Check if the IssueQ has an 
    // instruction and the MemQ has space. If so, then send it to
    // the functional partition for execution.

    rule stage1_begin (True);
    
        // Get our local state from the context.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
        Reg#(TOKEN_EPOCH) epoch = ctx_epoch[ctx];

        // Do we have an instruction to execute, and a place to put it?
        if (!bundleFromIssueQ.canDeq(ctx) || !bundleToMemQ.canEnq(ctx))
        begin

            // A bubble. 
            debugLog.record_next_cycle(ctx, fshow("BUBBLE"));

            // Propogate the bubble.
            bundleFromIssueQ.noDeq(ctx);
            bundleToMemQ.noEnq(ctx);
            rewindToFet.send(ctx, tagged Invalid);
            trainingToBP.send(ctx, tagged Invalid);
            writebackToDec.send(ctx, tagged Invalid);

            // End the model cycle. (Path 1)
            eventExe.recordEvent(ctx, tagged Invalid);
            localCtrl.endModelCycle(ctx, 1);

        end
        else
        begin

            // Yes... but is it something we should be executing?
            let bundle = bundleFromIssueQ.peek(ctx);
            let tok = bundle.token;
        
            if (goodEpoch(tok))
            begin
                
                // It's on the good path.            
                debugLog.record_next_cycle(ctx, fshow("EXEC: ") + fshow(tok));

                // Have the functional partition execute it.
                // It will be returned to the next stage.
                getResults.makeReq(initFuncpReqGetResults(tok));

            end
            else
            begin
            
                // We've got to flush.
            
                debugLog.record_next_cycle(ctx, fshow("FLUSH: ") + fshow(tok));

                if (tokFaultEpoch(tok) != epoch.fault)
                begin

                    //
                    // New fault epoch.  Fault handler just before commit forced a fault
                    // (rewind).  Update the epoch completely since execute may have
                    // requested rewinds after the fault for mispredicted branches.
                    // These requests would have been ignored, causing the branchEpoch
                    // counter here to be out of sync.
                    //
                    // To handle the new fault epoch this cycle is treated as a bubble.
                    // The incoming token remains in decode and will be executed next
                    // cycle, now that it will appear to be on the good path.
                    //
                    epoch <= tokEpoch(tok);
                    // Don't dequeue the IssueQ.
                    bundleFromIssueQ.noDeq(ctx);
                    // Don't send any register writebacks.
                    writebackToDec.send(ctx, tagged Invalid);

                end
                else
                begin

                    //
                    // Bad path due to branch.  Drop the incoming token.
                    //
                    // Dequeue the IssueQ
                    bundleFromIssueQ.doDeq(ctx);
                    // Tell decode the instruction was dropped, so its dests are "ready."
                    writebackToDec.send(ctx, tagged Valid genBusMessage(tok, bundle.dests, True));

                end
                
                // Don't enqueue anything in the MemQ.
                bundleToMemQ.noEnq(ctx);

                // Propogate the bubble.
                rewindToFet.send(ctx, tagged Invalid);
                trainingToBP.send(ctx, tagged Invalid);

                // End the model cycle. (Path 2)
                eventExe.recordEvent(ctx, tagged Invalid);
                localCtrl.endModelCycle(ctx, 2);
                
            end

        end
    
    endrule

    // stage2_results
    
    // Get the response from the functional partition and resolve 
    // any branches.
    
    // If we got here then we know the IssueQ is not empty and the MemQ is not full.

    rule stage2_results (True);
    
        // Get the response from the functional partition.
        let rsp = getResults.getResp();
        getResults.deq();

        // Get our local state from the context.
        let tok = rsp.token;
        let res = rsp.result;
        let ctx = tokContextId(tok);
        Reg#(TOKEN_EPOCH) epoch = ctx_epoch[ctx];

        // assert bundleFromIssueQ.canDeq(), since otherwise we wouldn't go down this path.
        let bundle = bundleFromIssueQ.peek(ctx);

        // Dequeue the IssueQ.
        bundleFromIssueQ.doDeq(ctx);
        
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
                debugLog.record(ctx, fshow("BRANCH TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h END-OF-EPOCH:%d", addr, epoch.branch));
                train.exeResult = tagged BranchTaken addr;

                if (bundle.branchAttr matches tagged BranchTaken .tgt &&& tgt == addr)
                begin
                    
                    // It was predicted correctly. Train, but don't resteer.
                    rewindToFet.send(ctx, tagged Invalid);
                    train.predCorrect = True;
                    trainingToBP.send(ctx, tagged Valid train);

                end
                else
                begin
                    
                    // The branch predictor predicted NotTaken.
                    statMispred.incr(ctx);
                    epoch.branch <= epoch.branch + 1;

                    // Rewind the PC to the actual target and train the BP.
                    rewindToFet.send(ctx, tagged Valid tuple2(tok, addr));
                    train.predCorrect = False;
                    trainingToBP.send(ctx, tagged Valid train);

                end
            end
            tagged RBranchNotTaken .addr:
            begin
            
                // It was a branch, but it was not taken.
                debugLog.record(ctx, fshow("BRANCH NOT-TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
                train.exeResult = tagged BranchNotTaken addr;

                case (bundle.branchAttr) matches
                    tagged BranchNotTaken .tgt:
                    begin

                        // The predictor got it right. No need to resteer.
                        rewindToFet.send(ctx, tagged Invalid);
                        train.predCorrect = True;
                        trainingToBP.send(ctx, tagged Valid train);

                    end
                    tagged BranchTaken .tgt:
                    begin

                        // The predictor got it wrong.
                        statMispred.incr(ctx);
                        epoch.branch <= epoch.branch + 1;
                        
                        // Resteer the PC to the actual destination and train.
                        rewindToFet.send(ctx, tagged Valid tuple2(tok, addr));
                        train.predCorrect = False;
                        trainingToBP.send(ctx, tagged Valid train);

                    end
                    tagged NotBranch:
                    begin

                        // The predictor was wrong, but it was harmless.
                        // Note: Should we train in this case?
                        rewindToFet.send(ctx, tagged Invalid);
                        trainingToBP.send(ctx, tagged Invalid);

                    end

                endcase


            end
            tagged REffectiveAddr .ea:
            begin
                
                // The instruction was a memory operation to this address.
                debugLog.record(ctx, fshow("EFF ADDR: ") + fshow(tok) + fshow(" ADDR:") + fshow(ea));

                // Update the bundle for the DMem module.
                bundle.effAddr = ea;

                // Use the standard branch prediction for non-branches.
                nonBranchPred(tok, rsp, bundle.branchAttr);

            end
            tagged RNop:
            begin

                // The instruction was some kind of instruction we don't care about.
                nonBranchPred(tok, rsp, bundle.branchAttr);

            end
            tagged RTerminate .pf:
            begin
                
                // Like a nop, but if this instruction commits, then simulation should end.
                rewindToFet.send(ctx, tagged Invalid);
                bundle.isTerminate = tagged Valid pf;
                trainingToBP.send(ctx, tagged Invalid);

            end
            
        endcase

        if (bundle.isLoad)
        begin

            // The destinations won't be ready until the Mem stage is finished.
            debugLog.record(ctx, fshow(tok) + fshow(": load -- dest regs not yet valid"));
            // No writebacks to report.
            writebackToDec.send(ctx, tagged Invalid);

        end
        else
        begin

            // The destinations of this token are ready.
            debugLog.record(ctx, fshow(tok) + fshow(": marking dest regs valid"));

            // Send the writeback to decode.
            writebackToDec.send(ctx, tagged Valid genBusMessage(tok, bundle.dests, False));

        end

        // Update the bundle with any token updates from the FP.
        bundle.token = tok;

        // Enqueue the instuction in the MemQ.
        bundleToMemQ.doEnq(ctx, bundle);

        // End the model cycle. (Path 3)
        eventExe.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
        localCtrl.endModelCycle(ctx, 3);

    endrule

endmodule
