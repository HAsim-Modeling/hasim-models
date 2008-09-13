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


import fpga_components::*;

`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/hasim_branch_pred.bsh"

typedef 12 BTB_IDX_SZ;

typedef Bit#(2) BTB_OFFSET;
typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE,TAdd#(BTB_IDX_SZ,2))) BTB_TAG;
typedef Bit#(BTB_IDX_SZ) BTB_INDEX;

typedef enum { BP_STATE_TRAIN, BP_STATE_BTB, BP_STATE_BP } BP_STATE deriving (Bits, Eq);

function BTB_INDEX getIndex (ISA_ADDRESS a);
    Tuple3#(BTB_TAG,BTB_INDEX,BTB_OFFSET) tup = unpack(a);
    match { .tag, .idx, .off } = tup;
    // assert off = 0b00
    return idx;
endfunction
function BTB_TAG getTag (ISA_ADDRESS a);
    Tuple3#(BTB_TAG,BTB_INDEX,BTB_OFFSET) tup = unpack(a);
    match { .tag, .idx, .off } = tup;
    // assert off = 0b00
    return tag;
endfunction

module [HASIM_MODULE] mkBranchPredictor ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_bp.out");

    Port_Receive#(Tuple2#(TOKEN,ISA_ADDRESS)) pcQ <- mkPort_Receive("bp_req", 0);
    Port_Send#(ISA_ADDRESS)    nextpcQ <- mkPort_Send("bp_reply_pc");
    Port_Send#(BRANCH_ATTR)      predQ <- mkPort_Send("bp_reply_pred");
    Port_Receive#(BRANCH_PRED_TRAIN) trainQ <- mkPort_Receive("bp_train", 1);

    BRAM#(BTB_INDEX, Maybe#(Tuple2#(BTB_TAG,ISA_ADDRESS))) btb <- mkBRAMInitialized(Invalid);

    Reg#(ISA_ADDRESS) pc <- mkReg(0);
    Reg#(BP_STATE) state <- mkReg(BP_STATE_TRAIN);

    BranchPred bPred <- mkBranchPred();

    //Local Controller
    Vector#(2, Port_Control) inports  = newVector();
    Vector#(2, Port_Control) outports = newVector();
    inports[0]  = pcQ.ctrl;
    inports[1]  = trainQ.ctrl;
    outports[0] = nextpcQ.ctrl;
    outports[1] = predQ.ctrl;

    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule train (state == BP_STATE_TRAIN);
        local_ctrl.startModelCC();
        let x <- trainQ.receive();
        if (x matches tagged Valid .bpt)
        begin
            let pc = bpt.branchPC;
            Bool taken = False;

            if (bpt.exeResult matches tagged BranchTaken .tgt)
            begin
                taken = True;

                // Update branch target buffer
                btb.write(getIndex(pc), Valid(tuple2(getTag(pc),tgt)));
                debugLog.record($format("BTB TRAIN: %h -> %h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
            else if (bpt.exeResult matches tagged NotBranch)
            begin
                // BTB must be an alias for a different branch.  Remove BTB
                // entry.
                // Update branch target buffer
                btb.write(getIndex(pc), Invalid);
                debugLog.record($format("BTB TRAIN: %h not branch", pc) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
            
            // Derive original prediction from true path and whether prediction was correct
            Bool pred = (bpt.predCorrect ? taken : ! taken);

            // Update predictor
            if (bpt.exeResult matches tagged NotBranch)
            begin
                noAction;
            end
            else
            begin
                bPred.upd(bpt.token, bpt.branchPC, pred, taken);
                debugLog.record($format("BP TRAIN: %h, pred: %d, taken: %d", pc, pred, taken));
            end
        end
        state <= BP_STATE_BTB;
    endrule

    rule btbreq (state == BP_STATE_BTB);
        let mpc <- pcQ.receive();
        if (mpc matches tagged Valid { .tok, .addr })
        begin
            btb.readReq(getIndex(addr));
            bPred.getPredReq(tok, addr);
            pc <= addr;
            state <= BP_STATE_BP;
        end
        else
        begin
            nextpcQ.send(Invalid);
            predQ.send(Invalid);
            state <= BP_STATE_TRAIN;
        end
    endrule
    rule bp (state == BP_STATE_BP);
        let x <- btb.readRsp();
        let predTaken <- bPred.getPredResp();

        if (x matches tagged Valid { .tag, .tgt } &&& getTag(pc) == tag)
        begin
            if (predTaken)
            begin
                nextpcQ.send(Valid(tgt));
                predQ.send(Valid(BranchTaken(tgt)));
                debugLog.record($format("PRED: %h -> taken; tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
            else
            begin
                nextpcQ.send(Valid(pc + 4));
                predQ.send(Valid(BranchNotTaken(tgt)));
                debugLog.record($format("PRED: %h -> not-taken; taken-tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
        end
        else
        begin
            nextpcQ.send(Valid(pc + 4));
            predQ.send(Valid(NotBranch));
            debugLog.record($format("PRED: %h -> not-branch", pc));
        end
        state <= BP_STATE_TRAIN;
    endrule
endmodule
