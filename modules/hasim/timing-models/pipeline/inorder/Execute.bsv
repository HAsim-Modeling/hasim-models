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

import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

`include "asim/provides/funcp_interface.bsh"

//import PipelineTypes::*;
import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_EXECUTE.bsh"
`include "asim/dict/STATS_EXECUTE.bsh"

typedef enum { EXECUTE_STATE_EXEC, EXECUTE_STATE_WORK } EXECUTE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkExecute ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_execute.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("dec2exe");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("exe2mem");

    Port_Send#(Tuple2#(TOKEN, ISA_ADDRESS))      rewindQ <- mkPort_Send("rewind");
    Port_Send#(BRANCH_PRED_TRAIN)               bptrainQ <- mkPort_Send("bp_train");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("exe_bus");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS) getResults <- mkConnection_Client("funcp_getResults");

    Reg#(EXECUTE_STATE) state <- mkReg(EXECUTE_STATE_EXEC);

    Reg#(TOKEN_EPOCH) epoch <- mkReg(initEpoch(0, 0));

    //Local Controller
    Vector#(1, Port_Control) inports  = newVector();
    Vector#(4, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = rewindQ.ctrl;
    outports[2] = busQ.ctrl;
    outports[3] = bptrainQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_exe <- mkEventRecorder(`EVENTS_EXECUTE_INSTRUCTION_EXECUTE);
    //Stats
    Stat stat_mpred <- mkStatCounter(`STATS_EXECUTE_BPRED_MISPREDS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS, Reg#(Bool)) prfValid = newVector();

    function Bool good_epoch (TOKEN tok) = tokEpoch(tok) == epoch;

    //
    // nonBranchPred is used for handling branch prediction / rewind feedback
    // processing for all non-branch instructions.
    //
    function Action nonBranchPred(TOKEN tok,
                                  FUNCP_RSP_GET_RESULTS rsp,
                                  BRANCH_ATTR branchAttr);
    action
        let tgt = rsp.instructionAddress + zeroExtend(rsp.instructionSize);

        BRANCH_PRED_TRAIN train;
        train.token = tok;
        train.branchPC = rsp.instructionAddress;
        train.exeResult = tagged NotBranch;

        if (branchAttr matches tagged NotBranch)
        begin
            rewindQ.send(Invalid);
            bptrainQ.send(Invalid);
        end
        else if (branchAttr matches tagged BranchNotTaken .pred_tgt &&&
                 tgt == pred_tgt)
        begin
            // Treated non-branch instruction as a branch but got the right answer.
            // Train BP but no need to rewind
            debugLog.record(fshow("NON-BRANCH PREDICTED NOT TAKEN BRANCH: ") + fshow(tok));
            rewindQ.send(Invalid);

            train.predCorrect = True;
            bptrainQ.send(tagged Valid train);
        end
        else
        begin
            debugLog.record(fshow("NON-BRANCH PREDICTED BRANCH: ") + fshow(tok));
            stat_mpred.incr();
            epoch.branch <= epoch.branch + 1;
            rewindQ.send(Valid(tuple2(tok, tgt)));

            train.predCorrect = False;
            bptrainQ.send(tagged Valid train);
        end
    endaction
    endfunction

    rule flush (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .bundle } &&& !good_epoch(tok));
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        
        debugLog.record(fshow("FLUSH: ") + fshow(tok));

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
            inQ.pass();
            busQ.send(Invalid);
        end
        else
        begin
            //
            // Bad path due to branch.  Drop the incoming token.
            //
            let x <- inQ.receive();
            busQ.send(Valid(bundle.dests));
        end

        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        bptrainQ.send(Invalid);
        event_exe.recordEvent(Invalid);
    endrule

    rule exec (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .* } &&& good_epoch(tok));
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        if (outQ.canSend)
        begin
            debugLog.record(fshow("EXEC: ") + fshow(tok));
            getResults.makeReq(initFuncpReqGetResults(tok));
            state <= EXECUTE_STATE_WORK;
        end
        else
        begin
           debugLog.record(fshow("STALL PROPAGATED"));
           inQ.pass();
           outQ.pass();
           rewindQ.send(Invalid);
           bptrainQ.send(Invalid);
           busQ.send(Invalid);
           event_exe.recordEvent(Invalid);
        end
    endrule

    rule bubble (state == EXECUTE_STATE_EXEC &&& inQ.peek() == Invalid);
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        debugLog.record(fshow("BUBBLE"));
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        bptrainQ.send(Invalid);
        busQ.send(Invalid);
        event_exe.recordEvent(Invalid);
    endrule

    rule results (state == EXECUTE_STATE_WORK);
        getResults.deq();

        let rsp = getResults.getResp();
        let tok = rsp.token;
        let res = rsp.result;

        let x <- inQ.receive();
        if (x matches tagged Valid { .tok2, .bndl })
        begin
            let bundle = bndl;
            let pc = bundle.pc;

            BRANCH_PRED_TRAIN train;
            train.token = tok;
            train.branchPC = pc;

            case (res) matches
              tagged RBranchTaken .addr:
                begin
                    train.exeResult = BranchTaken(addr);

                    if (bundle.branchAttr matches tagged BranchTaken .tgt &&& tgt == addr)
                    begin
                        rewindQ.send(Invalid);

                        train.predCorrect = True;
                        bptrainQ.send(tagged Valid train);
                    end
                    else
                    begin
                        stat_mpred.incr();
                        epoch.branch <= epoch.branch + 1;
                        rewindQ.send(Valid(tuple2(tok,addr)));

                        train.predCorrect = False;
                        bptrainQ.send(tagged Valid train);
                    end
                    debugLog.record(fshow("BRANCH TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h END-OF-EPOCH:%d", addr, epoch.branch));
                end
              tagged RBranchNotTaken .addr:
                begin
                    train.exeResult = tagged BranchNotTaken addr;

                    case (bundle.branchAttr) matches
                        tagged BranchNotTaken .tgt:
                            begin
                                rewindQ.send(Invalid);

                                train.predCorrect = True;
                                bptrainQ.send(tagged Valid train);
                            end
                        tagged BranchTaken .tgt:
                            begin
                                stat_mpred.incr();
                                epoch.branch <= epoch.branch + 1;
                                rewindQ.send(Valid(tuple2(tok,addr)));

                                train.predCorrect = False;
                                bptrainQ.send(tagged Valid train);
                            end
                        tagged NotBranch:
                            begin
                                rewindQ.send(Invalid);
                                bptrainQ.send(Invalid);
                            end
                    endcase
                    debugLog.record(fshow("BRANCH NOT-TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
                end
              tagged REffectiveAddr .ea:
                begin
                    debugLog.record(fshow("EFF ADDR: ") + fshow(tok) + fshow(" ADDR:") + fshow(ea));
                    bundle.effAddr = ea;

                    nonBranchPred(tok, rsp, bundle.branchAttr);
                end
              tagged RNop:
                begin
                    nonBranchPred(tok, rsp, bundle.branchAttr);
                end
              tagged RTerminate .pf:
                begin
                    rewindQ.send(Invalid);
                    bundle.isTerminate = Valid(pf);
                    bptrainQ.send(Invalid);
                end
            endcase
            outQ.send(Valid(tuple2(tok, bundle)));
            busQ.send(Invalid);
            event_exe.recordEvent(Valid(zeroExtend(pack(tok.index))));
            state <= EXECUTE_STATE_EXEC;
        end
    endrule

endmodule
