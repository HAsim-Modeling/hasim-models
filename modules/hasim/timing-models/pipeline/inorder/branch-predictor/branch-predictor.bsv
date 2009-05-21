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

import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_branch_pred_alg.bsh"

typedef Bit#(`BTB_OFFSET_SIZE) BTB_OFFSET;
typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE,TAdd#(`BTB_IDX_SIZE, `BTB_OFFSET_SIZE))) BTB_TAG;
typedef Bit#(`BTB_IDX_SIZE) BTB_INDEX;

typedef union tagged
{
    void        STAGE2_bubble;
    ISA_ADDRESS STAGE2_btbRsp;
}
BP_STAGE2_STATE deriving (Eq, Bits);


module [HASIM_MODULE] mkBranchPredictor ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_bp.out");


    // ****** Model State (per instance) ******
    
    MULTIPLEXED#(NUM_CPUS, BRAM#(BTB_INDEX, Maybe#(Tuple2#(BTB_TAG, ISA_ADDRESS)))) bTBPool <- mkMultiplexed(mkBRAMInitialized(Invalid));

    MULTIPLEXED#(NUM_CPUS, FIFO#(ISA_ADDRESS)) pcQPool <- mkMultiplexed(mkFIFO());

    MULTIPLEXED#(NUM_CPUS, BRANCH_PREDICTOR_ALG) bPAlgPool <- mkMultiplexed(mkBranchPredAlg());


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS)       pcFromFet <- mkPortRecvGuarded_Multiplexed("Fet_to_BP_pc", 0);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS)       predToFet <- mkPortSend_Multiplexed("BP_to_Fet_pred");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BRANCH_ATTR)       attrToFet <- mkPortSend_Multiplexed("BP_to_Fet_attr");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BRANCH_PRED_TRAIN) trainingFromExe <- mkPortRecv_Multiplexed("Exe_to_BP_training", 1);


    // ****** Local Controller ******

    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    // inports[0]  = pcFromFet.ctrl;
    inports[0]  = trainingFromExe.ctrl;
    outports[0] = predToFet.ctrl;
    outports[1] = attrToFet.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, BP_STAGE2_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER_VOID#(NUM_CPUS)             stage3Ctrl <- mkStageControllerVoid();

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
    

    // stage1_btbReq
    
    // Make the requests to the branch predictor alg and BTB.

    // Ports read:
    // * pcFromFet

    // Ports written:
    // * None

    (* conservative_implicit_conditions *)
    rule stage2_btbReq (True);

        // Get the next active instance.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get the local state for the current instance.
        let btb = bTBPool[cpu_iid];
        let bPAlg = bPAlgPool[cpu_iid];
        let pcQ = pcQPool[cpu_iid];

        // Let's see if there was a prediction request.
        let m_pc <- pcFromFet.receive(cpu_iid);

        if (m_pc matches tagged Valid .addr)
        begin
        
            // Lookup this PC in the BTB and branch predictor.
            btb.readReq(getIndex(addr));
            bPAlg.getPredReq(addr);

            // Pass the information to the next stage.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_btbRsp addr);

        end
        else
        begin

            // No prediction request. Propogate the bubble.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

        end

    endrule

    // stage2_btbRsp
    
    // Get the responses from the prediction structures and process them (if any).
    // Ports read:
    // * None
    
    // Ports written:
    // * predToFet
    // * attrToFet

    rule stage2_btbRsp (True);

        // Get the active instance from the previous stage.
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get our local state from the instance.
        let btb = bTBPool[cpu_iid];
        let bPAlg = bPAlgPool[cpu_iid];

        if (state matches tagged STAGE2_bubble)
        begin
        
            // Just propogate the bubble.
            predToFet.send(cpu_iid, tagged Invalid);
            attrToFet.send(cpu_iid, tagged Invalid);
            
            // Proceed to the next stage.
            stage3Ctrl.ready(cpu_iid);
        
        end
        else if (state matches tagged STAGE2_btbRsp .pc)
        begin

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
                    debugLog.record(cpu_iid, $format("2: PRED: %h -> taken; tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));

                    // Send the responses to the Fetch unit.
                    predToFet.send(cpu_iid, tagged Valid tgt);
                    attrToFet.send(cpu_iid, tagged Valid (tagged BranchTaken tgt));

                end
                else
                begin

                    // Well, we have a target, but the BP says not taken, so lets ignore it.
                    debugLog.record(cpu_iid, $format("2: PRED: %h -> not-taken; taken-tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));

                    // Send the responses to the Fetch unit.
                    predToFet.send(cpu_iid, tagged Valid (pc + 4));
                    attrToFet.send(cpu_iid, tagged Valid (tagged BranchNotTaken tgt));

                end
            end
            else
            begin

                // Well, the BTB doesn't know about it, so we'll go with PC+4.
                debugLog.record(cpu_iid, $format("2: PRED: %h -> not-branch", pc));
                predToFet.send(cpu_iid, tagged Valid (pc + 4));
                attrToFet.send(cpu_iid, tagged Valid NotBranch);

            end
            
            // Proceed to the next stage.
            stage3Ctrl.ready(cpu_iid);

        end

    endrule

    // stage3_train
    
    // Get the training data and update the branch predictor.

    // Ports read:
    // * trainingFromExe
    
    // Ports written:
    // * None

    (* conservative_implicit_conditions *)
    rule stage3_train (True);
    
        // Get the next ready instance.
        let cpu_iid <- stage3Ctrl.nextReadyInstance();

        // Get our local state from the instance.
        let btb = bTBPool[cpu_iid];
        let bPAlg = bPAlgPool[cpu_iid];

        // Check for any new training.
        let m_train <- trainingFromExe.receive(cpu_iid);

        if (m_train matches tagged Valid .bpt)
        begin

            // Let's train the predictor.
            let pc = bpt.branchPC;
            Bool taken = False;

            if (bpt.exeResult matches tagged BranchTaken .tgt)
            begin

                // Update the BTB to note the actual target.            
                debugLog.record_next_cycle(cpu_iid, $format("3: BTB TRAIN: %h -> %h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
                btb.write(getIndex(pc), tagged Valid tuple2(getTag(pc),tgt));
                taken = True;

            end
            else if (bpt.exeResult matches tagged NotBranch)
            begin

                // BTB must be an alias for a different branch.  Remove BTB entry.
                // Note: this is a bit aggressive. Two-bit predictor semantics could be an alternative?
                debugLog.record_next_cycle(cpu_iid, $format("3: BTB TRAIN: %h not branch", pc) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
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
                debugLog.record_next_cycle(cpu_iid, $format("3: BP TRAIN: %h, pred: %d, taken: %d", pc, pred, taken));
                bPAlg.upd(bpt.branchPC, pred, taken);

            end

        end
        
        // End of model cycle (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule



endmodule
