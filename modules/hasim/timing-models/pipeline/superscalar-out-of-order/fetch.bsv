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

import FShow::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/hasim_branch_pred.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"

`include "asim/provides/hasim_pipeline_types.bsh"

typedef enum
{
    FETCH_STATE_PREDICT_UPDATE,
    FETCH_STATE_FAULT_RESP,
    FETCH_STATE_REWIND_RESP,
    FETCH_STATE_TOKEN_REQ,
    FETCH_STATE_I_TRANSLATE_REQ,
    FETCH_STATE_INST_REQ,
    FETCH_STATE_INST_RESP,
    FETCH_STATE_BRANCH_IMM,
    FETCH_STATE_JUMP_IMM
}
FETCH_STATE
    deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch();
    TIMEP_DEBUG_FILE                                                               debugLog <- mkTIMEPDebugFile("pipe_fet.out");

    PORT_CREDIT_SEND#(FETCH_BUNDLE, `FETCH_NUM, LOG_FETCH_CREDITS)                fetchPort <- mkPortCreditSend("fetch");
    PORT_NO_STALL_RECEIVE#(PREDICT_UPDATE_BUNDLE, `ALU_NUM)               predictUpdatePort <- mkPortNoStallReceive("predictUpdate");
    PORT_NO_STALL_RECEIVE#(REWIND_BUNDLE, 1)                                    resteerPort <- mkPortNoStallReceive("resteer");
    PORT_NO_STALL_RECEIVE#(FAULT_BUNDLE, 1)                                       faultPort <- mkPortNoStallReceive("fault");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT, FUNCP_RSP_NEW_IN_FLIGHT)        newInFlight <- mkConnection_Client("funcp_newInFlight");
    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE, FUNCP_RSP_DO_ITRANSLATE)         iTranslate <- mkConnection_Client("funcp_doITranslate");
    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION, FUNCP_RSP_GET_INSTRUCTION) getInstruction <- mkConnection_Client("funcp_getInstruction");
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");
    Connection_Client#(FUNCP_REQ_HANDLE_FAULT, FUNCP_RSP_HANDLE_FAULT)          handleFault <- mkConnection_Client("funcp_handleFault");

    Reg#(ISA_ADDRESS)                                                                    pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(FETCH_STATE)                                                                 state <- mkReg(FETCH_STATE_PREDICT_UPDATE);
    Reg#(ROB_INDEX)                                                                epochRob <- mkReg(0);
    Reg#(Bool)                                                                 afterResteer <- mkReg(False);

    BranchPred                                                                   branchPred <- mkBranchPred;

    //
    // Local Controller
    //
    // FIXME -- need to enumerate ports so balancing works for events
    Vector#(0, PORT_CONTROLS) inports  = newVector();
    Vector#(0, PORT_CONTROLS) outports = newVector();

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    function Action makeFetchBundle(TOKEN token, ISA_INSTRUCTION inst, ISA_ADDRESS _pc, PRED_TYPE predType, Bool prediction, ISA_ADDRESS predPc);
    action
        getInstruction.deq;
        let bundle = FETCH_BUNDLE{token: token, inst: inst, pc: _pc, predType: predType, prediction: prediction, predPc: predPc, epochRob: epochRob, afterResteer: afterResteer};
        fetchPort.enq(bundle);
        debugLog.record($format("instResp ") + fshow(bundle));
        afterResteer <= False;
        pc <= predPc;
        state <= FETCH_STATE_TOKEN_REQ;
    endaction
    endfunction

    rule predictUpdate(state == FETCH_STATE_PREDICT_UPDATE);
        let dummy <- localCtrl.startModelCycle();
        localCtrl.endModelCycle(dummy, 1);

        if(predictUpdatePort.canReceive)
        begin
            let bundle <- predictUpdatePort.pop;
            debugLog.record($format("predict update received") + fshow(bundle));
            if(bundle.predType == PRED_TYPE_BRANCH_IMM)
            begin
                debugLog.record($format("Branch Imm upd ") + fshow(bundle));
                branchPred.upd(bundle.token, bundle.pc, bundle.pred, bundle.actual);
            end
        end
        else
        begin
            //
            // End of cycle.  Read fault and resteer from ROB.
            //
            predictUpdatePort.done;
            let fault <- faultPort.receive();
            let resteer <- resteerPort.receive();
            
            if (fault.fault)
            begin
                // Fault.  Invoke the functional fault handler.  The fault handler
                // rewinds instructions and returns the next PC.
                debugLog.record($format("faultReq ") + fshow(fault));
                epochRob <= fault.robIndex;
                afterResteer <= True;
                handleFault.makeReq(initFuncpReqHandleFault(fault.token));
                state <= FETCH_STATE_FAULT_RESP;
            end
            else if (resteer.mispredict)
            begin
                // Branch misprediction.  Rewind and resteer.
                debugLog.record($format("rewindReq ") + fshow(resteer));
                pc <= resteer.addr;
                epochRob <= resteer.robIndex;
                afterResteer <= True;
                rewindToToken.makeReq(initFuncpReqRewindToToken(resteer.token));
                state <= FETCH_STATE_REWIND_RESP;
            end
            else
                state <= FETCH_STATE_TOKEN_REQ;
        end
    endrule

    rule faultResp(state == FETCH_STATE_FAULT_RESP);
        let rsp = handleFault.getResp();
        handleFault.deq();
        debugLog.record($format("faultResp resteer to 0x%0x", rsp.nextInstructionAddress));
        pc <= rsp.nextInstructionAddress;
        state <= FETCH_STATE_TOKEN_REQ;
    endrule

    rule rewindResp(state == FETCH_STATE_REWIND_RESP);
        debugLog.record($format("rewindResp "));
        rewindToToken.deq();
        state <= FETCH_STATE_TOKEN_REQ;
    endrule

    rule tokenReq(state == FETCH_STATE_TOKEN_REQ);
        if(fetchPort.canSend())
        begin
            debugLog.record($format("new token req"));
            newInFlight.makeReq(initFuncpReqNewInFlight(0));
            state <= FETCH_STATE_I_TRANSLATE_REQ;
        end
        else
        begin
            debugLog.record($format("end cycle"));
            debugLog.nextModelCycle();
            state <= FETCH_STATE_PREDICT_UPDATE;
            fetchPort.done();
        end
    endrule

    rule iTranslateReq(state == FETCH_STATE_I_TRANSLATE_REQ);
        debugLog.record($format("iTranslate req"));
        let resp = newInFlight.getResp();
        newInFlight.deq();
        iTranslate.makeReq(FUNCP_REQ_DO_ITRANSLATE{token: resp.newToken, address: pc});
        state <= FETCH_STATE_INST_REQ;
    endrule

    rule instReq(state == FETCH_STATE_INST_REQ);
        debugLog.record($format("iTranslate resp"));
        let resp = iTranslate.getResp();
        iTranslate.deq();

        // iTranslate may return multiple responses for unaligned references.
        // Don't act until the last one is received.
        if (! resp.hasMore)
        begin
            debugLog.record($format("inst req"));
            getInstruction.makeReq(FUNCP_REQ_GET_INSTRUCTION{token: resp.token});
            state <= FETCH_STATE_INST_RESP;
        end
    endrule

    rule instResp(state == FETCH_STATE_INST_RESP);
        let resp = getInstruction.getResp;
        debugLog.record($format("inst resp"));
        if(isBranchImm(resp.instruction))
        begin
            branchPred.getPredReq(resp.token, pc);
            debugLog.record($format("Branch Imm"));
            state <= FETCH_STATE_BRANCH_IMM;
        end
        else if(isJumpImm(resp.instruction))
        begin
            debugLog.record($format("Jump Imm"));
            state <= FETCH_STATE_JUMP_IMM;
        end
        else
            makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_NONE, False, pc + 4);
    endrule

    rule branchImm(state == FETCH_STATE_BRANCH_IMM);
        debugLog.record($format("branch imm resp"));
        let resp = getInstruction.getResp;
        let pred <- branchPred.getPredResp;
        let predPc = pred? predPcBranchImm(pc, resp.instruction): pc + 4;
        makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_BRANCH_IMM, pred, predPc);
    endrule

    rule jumpImm(state == FETCH_STATE_JUMP_IMM);
        debugLog.record($format("jump imm resp"));
        let resp = getInstruction.getResp;
        let predPc = predPcJumpImm(pc, resp.instruction);
        makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_JUMP_IMM, False, predPc);
    endrule
endmodule
