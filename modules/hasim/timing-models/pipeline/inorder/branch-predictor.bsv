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


`include "asim/provides/fpga_components.bsh"

`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/hasim_branch_pred_alg.bsh"

typedef 12 BTB_IDX_SZ;

typedef Bit#(2) BTB_OFFSET;
typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE,TAdd#(BTB_IDX_SZ,2))) BTB_TAG;
typedef Bit#(BTB_IDX_SZ) BTB_INDEX;


module [HASIM_MODULE] mkBranchPredictor ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_bp.out");


    // ****** Model State (per Context) ******
    
    MULTICTX#(BRAM#(BTB_INDEX, Maybe#(Tuple2#(BTB_TAG, ISA_ADDRESS)))) ctx_BTB <- mkMultiCtx(mkBRAMInitialized(Invalid));

    MULTICTX#(FIFO#(ISA_ADDRESS)) ctx_pcQ <- mkMultiCtx(mkFIFO());

    MULTICTX#(BRANCH_PREDICTOR_ALG) ctx_BPAlg <- mkMultiCtx(mkBranchPredAlg());


    // ****** UnModel Pipeline State ******
    
    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();


    // ****** Ports ******

    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, ISA_ADDRESS)) pcFromFet <- mkPortRecvGuarded_MultiCtx("Fet_to_BP_pc", 0);
    PORT_SEND_MULTICTX#(ISA_ADDRESS)                 predToFet <- mkPortSend_MultiCtx("BP_to_Fet_pred");
    PORT_SEND_MULTICTX#(BRANCH_ATTR)                 attrToFet <- mkPortSend_MultiCtx("BP_to_Fet_attr");
    PORT_RECV_MULTICTX#(BRANCH_PRED_TRAIN)     trainingFromExe <- mkPortRecv_MultiCtx("Exe_to_BP_training", 1);


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS) inports  = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    // inports[0]  = pcFromFet.ctrl;
    inports[0]  = trainingFromExe.ctrl;
    outports[0] = predToFet.ctrl;
    outports[1] = attrToFet.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Helper Functions ******


    // getIndex, getTag
    
    // Split an address into an index/tag hash.

    function BTB_INDEX getIndex (ISA_ADDRESS a);
        Tuple3#(BTB_TAG, BTB_INDEX, BTB_OFFSET) tup = unpack(a);
        match { .tag, .idx, .off } = tup;
        // assert off = 0b00
        return idx;
    endfunction

    function BTB_TAG getTag (ISA_ADDRESS a);
        Tuple3#(BTB_TAG, BTB_INDEX, BTB_OFFSET) tup = unpack(a);
        match { .tag, .idx, .off } = tup;
        // assert off = 0b00
        return tag;
    endfunction


    // ****** Rules ******
    

    // stage1_train
    
    // Get the training data and update the branch predictor.

    rule stage1_train (True);
    
        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);

        // Get our local state from the context.
        let btb = ctx_BTB[ctx];
        let bPAlg = ctx_BPAlg[ctx];

        // Check for any new training.
        let m_train <- trainingFromExe.receive(ctx);

        if (m_train matches tagged Valid .bpt)
        begin

            // Let's train the predictor.
            let pc = bpt.branchPC;
            Bool taken = False;

            if (bpt.exeResult matches tagged BranchTaken .tgt)
            begin

                // Update the BTB to note the actual target.            
                debugLog.record_next_cycle(ctx, $format("BTB TRAIN: %h -> %h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
                btb.write(getIndex(pc), tagged Valid tuple2(getTag(pc),tgt));
                taken = True;

            end
            else if (bpt.exeResult matches tagged NotBranch)
            begin

                // BTB must be an alias for a different branch.  Remove BTB entry.
                // Note: this is a bit aggressive. Two-bit predictor semantics could be an alternative?
                debugLog.record_next_cycle(ctx, $format("BTB TRAIN: %h not branch", pc) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
                btb.write(getIndex(pc), Invalid);

            end
            
            // Derive original prediction from true path and whether prediction was correct.
            Bool pred = (bpt.predCorrect ? taken : ! taken);

            // Update predictor
            if (bpt.exeResult matches tagged NotBranch)
            begin

                // Note: Should this be passed to the predictor as well?
                noAction;

            end
            else
            begin

                // Update the predictor with the training.
                debugLog.record_next_cycle(ctx, $format("BP TRAIN: %h, pred: %d, taken: %d", pc, pred, taken));
                bPAlg.upd(bpt.token, bpt.branchPC, pred, taken);

            end

        end
        
        // Proceed to the next stage.
        stage2Q.enq(ctx);
        
    endrule


    // stage2_btbReq
    
    // Make the requests to the branch predictor alg and BTB.

    rule stage2_btbReq (True);

        // Get our context from the previous stage.
        let ctx = stage2Q.first();
        stage2Q.deq();

        // Get our local state from the context.
        let btb = ctx_BTB[ctx];
        let bPAlg = ctx_BPAlg[ctx];
        let pcQ = ctx_pcQ[ctx];

        // Let's see if there was a prediction request.
        let m_pc <- pcFromFet.receive(ctx);

        if (m_pc matches tagged Valid { .tok, .addr })
        begin
        
            // Lookup this PC in the BTB and branch predictor.
            btb.readReq(getIndex(addr));
            bPAlg.getPredReq(tok, addr);

            // Pass the information to the next stage.
            pcQ.enq(addr);
            stage3Q.enq(ctx);

        end
        else
        begin

            // No prediction request. Propogate the bubble.
            debugLog.record(ctx, $format("BUBBLE"));
            predToFet.send(ctx, tagged Invalid);
            attrToFet.send(ctx, tagged Invalid);
            
            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(ctx, 1);

        end

    endrule

    rule stage3_btbRsp (True);

        // Get our context from the previous stage.
        let ctx = stage3Q.first();
        stage3Q.deq();
        
        // Get our local state from the context.
        let btb = ctx_BTB[ctx];
        let bPAlg = ctx_BPAlg[ctx];
        let pcQ = ctx_pcQ[ctx];
        
        // Get the PC associated with this prediction.
        let pc = pcQ.first();
        pcQ.deq();

        // Get the responses from the predictors.
        let m_rsp <- btb.readRsp();
        let predTaken <- bPAlg.getPredResp();

        // Let's see if the BTB has some information for us.
        // A taken prediction is useless if we don't know where to go!
        if (m_rsp matches tagged Valid { .tag, .tgt } &&& getTag(pc) == tag)
        begin

            // The tag match ensures we're not clashing to a different address.

            if (predTaken)
            begin

                // The branch predictor thinks we're taking it, so give the BTB
                // response as the next PC.
                debugLog.record(ctx, $format("PRED: %h -> taken; tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));

                // Send the responses to the Fetch unit.
                predToFet.send(ctx, tagged Valid tgt);
                attrToFet.send(ctx, tagged Valid (tagged BranchTaken tgt));

            end
            else
            begin

                // Well, we have a target, but the BP says not taken, so lets ignore it.
                debugLog.record(ctx, $format("PRED: %h -> not-taken; taken-tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));

                // Send the responses to the Fetch unit.
                predToFet.send(ctx, tagged Valid (pc + 4));
                attrToFet.send(ctx, tagged Valid (tagged BranchNotTaken tgt));

            end
        end
        else
        begin
        
            // Well, the BTB doesn't know about it, so we'll go with PC+4.
            debugLog.record(ctx, $format("PRED: %h -> not-branch", pc));
            predToFet.send(ctx, tagged Valid (pc + 4));
            attrToFet.send(ctx, tagged Valid NotBranch);

        end
        
        // End of model cycle. (Path 2)
        localCtrl.endModelCycle(ctx, 2);

    endrule

endmodule
