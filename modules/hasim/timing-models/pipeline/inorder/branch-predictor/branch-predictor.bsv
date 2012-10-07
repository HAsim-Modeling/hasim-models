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
`include "asim/provides/common_services.bsh"
`include "asim/provides/mem_services.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/model_structures_base_types.bsh"
`include "asim/provides/hasim_branch_pred_alg.bsh"
`include "asim/provides/hasim_branch_target_buffer.bsh"

`include "asim/dict/VDEV_SCRATCH.bsh"

module [HASIM_MODULE] mkBranchPredictor ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_bp.out");


    // ****** Model State (per instance) ******
    
    BRANCH_PREDICTOR_ALG bpAlg <- mkBranchPredAlg();
    BRANCH_TARGET_BUFFER_ALG btbAlg <- mkBranchTargetPredAlg();


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS)       pcFromFet <- mkPortRecv_Multiplexed("Fet_to_BP_pc", 0);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BRANCH_ATTR)       predToFet <- mkPortSend_Multiplexed("BP_to_Fet_newpc");

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BRANCH_PRED_TRAIN) trainingFromExe <- mkPortRecv_Multiplexed("Exe_to_BP_training", 1);


    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = pcFromFet.ctrl;
    inports[1]  = trainingFromExe.ctrl;
    outports[0] = predToFet.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkNamedLocalController("Branch Predictor", inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, Maybe#(ISA_ADDRESS)) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();


    // ****** Rules ******
    

    // stage1_btbReq
    
    // Make the requests to the branch predictor alg and BTB.

    // Ports read:
    // * pcFromFet

    // Ports written:
    // * None

    (* conservative_implicit_conditions *)
    rule stage1_predLookup (True);
        // Get the next active instance.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Let's see if there was a prediction request.
        let m_pc <- pcFromFet.receive(cpu_iid);

        if (m_pc matches tagged Valid .addr)
        begin
            // Query the branch predictor and BTB in parallel
            bpAlg.getPredReq(cpu_iid, addr);
            btbAlg.getPredReq(cpu_iid, addr);
            debugLog.record_next_cycle(cpu_iid, $format("1: REQ: %h", addr));
        end
        else
        begin
            debugLog.record_next_cycle(cpu_iid, $format("1: NO REQ"));
        end

        stage2Ctrl.ready(cpu_iid, m_pc);
    endrule


    // stage2_predRsp
    
    // Get the responses from the prediction structures and process them (if any).
    // Ports read:
    // * None
    
    // Ports written:
    // * predToFet

    rule stage2_predRsp (True);
        // Get the active instance from the previous stage.
        match {.cpu_iid, .m_pc} <- stage2Ctrl.nextReadyInstance();
        
        if (m_pc matches tagged Valid .pc)
        begin
            let pred_taken <- bpAlg.getPredRsp(cpu_iid);
            let btb_rsp <- btbAlg.getPredRsp(cpu_iid);
            
            if (btb_rsp matches tagged Valid .tgt)
            begin
                if (pred_taken)
                begin
                    // The branch predictor thinks we're taking it, so give the BTB
                    // response as the next PC.
                    debugLog.record(cpu_iid, $format("2: PRED: %h -> taken; tgt=%h", pc, tgt));

                    // Send the responses to the Fetch unit.
                    predToFet.send(cpu_iid, tagged Valid (tagged BranchTaken tgt));
                end
                else
                begin
                    // We have a target, but the BP says not taken,
                    // so lets ignore it.
                    debugLog.record(cpu_iid, $format("2: PRED: %h -> not-taken; taken-tgt=%h", pc, tgt));

                    // Send the responses to the Fetch unit.
                    predToFet.send(cpu_iid, tagged Valid (tagged BranchNotTaken tgt));
                end
            end
            else
            begin
                // The BTB doesn't know about it, so we'll go with PC+4.
                debugLog.record(cpu_iid, $format("2: PRED: %h -> entry invalid", pc));
                predToFet.send(cpu_iid, tagged Valid NotBranch);
            end
        end
        else
        begin
            // No instruction -- just propagate the bubble.
            predToFet.send(cpu_iid, tagged Invalid);
        end

        // Proceed to the next stage.
        stage3Ctrl.ready(cpu_iid);
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
                debugLog.record(cpu_iid, $format("3: BTB TRAIN: %h -> %h", pc, tgt));
                btbAlg.upd(cpu_iid, pc, bpt.predCorrect, tagged Valid tgt);
                taken = True;
            end
            else if (bpt.exeResult matches tagged NotBranch)
            begin
                // BTB must be an alias for a different branch.  Remove BTB entry.
                // Note: this is a bit aggressive. Two-bit predictor semantics
                // could be an alternative?
                debugLog.record(cpu_iid, $format("3: BTB TRAIN: %h not branch", pc));
                btbAlg.upd(cpu_iid, pc, bpt.predCorrect, tagged Invalid);
            end
            
            // Update predictor
            if (bpt.exeResult matches tagged NotBranch)
            begin
                // Note: Should this be passed to the predictor as well?
                noAction;
            end
            else
            begin
                // Update the predictor with the training.
                debugLog.record(cpu_iid, $format("3: BP TRAIN: %h, predCorrect: %d, taken: %d", pc, bpt.predCorrect, taken));
                bpAlg.upd(cpu_iid, pc, bpt.predCorrect, taken);
            end

        end
        
        // End of model cycle (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
    endrule

endmodule
