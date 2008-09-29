import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

`include "hasim_branch_pred.bsh"
`include "PipelineTypes.bsv"

`include "funcp_simulated_memory.bsh"

typedef enum { FETCH_STATE_PREDICT_UPDATE, FETCH_STATE_REWIND_RESP, FETCH_STATE_TOKEN_REQ, FETCH_STATE_I_TRANSLATE_REQ, FETCH_STATE_INST_REQ, FETCH_STATE_INST_RESP, FETCH_STATE_BRANCH_IMM, FETCH_STATE_JUMP_IMM } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch();
    TIMEP_DEBUG_FILE                                                               debugLog <- mkTIMEPDebugFile("pipe_fet.out");

    PORT_CREDIT_SEND#(FETCH_BUNDLE, `FETCH_NUM, LOG_FETCH_CREDITS)                fetchPort <- mkPortCreditSend("fetch");
    PORT_CREDIT_RECEIVE#(PREDICT_UPDATE_BUNDLE, `ALU_NUM, LOG_ALU_NUM)    predictUpdatePort <- mkPortCreditReceive("predictUpdate");

    PORT_RECEIVE#(REWIND_BUNDLE)                                                resteerPort <- mkPortReceive("resteer");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT, FUNCP_RSP_NEW_IN_FLIGHT)        newInFlight <- mkConnection_Client("funcp_newInFlight");
    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE, FUNCP_RSP_DO_ITRANSLATE)         iTranslate <- mkConnection_Client("funcp_doITranslate");
    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION, FUNCP_RSP_GET_INSTRUCTION) getInstruction <- mkConnection_Client("funcp_getInstruction");
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    Reg#(ISA_ADDRESS)                                                                    pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(FETCH_STATE)                                                                 state <- mkReg(FETCH_STATE_PREDICT_UPDATE);
    Reg#(ROB_INDEX)                                                                epochRob <- mkRegU();
    Reg#(Bool)                                                                 afterResteer <- mkReg(False);
    Reg#(TOKEN_TIMEP_EPOCH)                                                           epoch <- mkReg(0);

    BranchPred                                                                   branchPred <- mkBranchPred;

    Reg#(Bit#(32))                                                                   fpgaCC <- mkReg(0);
    rule inc(True);
        fpgaCC <= fpgaCC + 1;
    endrule

    function Action makeFetchBundle(TOKEN token, ISA_INSTRUCTION inst, ISA_ADDRESS _pc, PRED_TYPE predType, Bool prediction, ISA_ADDRESS predPc, ROB_INDEX _epochRob, Bool _afterResteer);
    action
        getInstruction.deq;
        let bundle = FETCH_BUNDLE{token: token, inst: inst, pc: _pc, predType: predType, prediction: prediction, predPc: predPc, epochRob: _epochRob, afterResteer: _afterResteer};
        fetchPort.enq(bundle);
        debugLog.record($format("instResp ") + fshow(bundle));
        afterResteer <= False;
        state <= FETCH_STATE_TOKEN_REQ;
    endaction
    endfunction

    rule predictUpdate(state == FETCH_STATE_PREDICT_UPDATE);
        if(predictUpdatePort.canReceive)
        begin
            let bundle <- predictUpdatePort.pop;
            if(bundle.predType == PRED_TYPE_BRANCH_IMM)
            begin
                debugLog.record($format("Branch Imm upd ") + fshow(bundle));
                branchPred.upd(bundle.token, bundle.pc, bundle.pred, bundle.actual);
             end
        end
        else
        begin
            predictUpdatePort.done(`ALU_NUM);
            let bundle <- resteerPort.pop();
            debugLog.record($format("rewindReq %d", fpgaCC) + fshow(bundle));
            if(bundle.mispredict)
            begin
                pc <= bundle.addr;
                epochRob <= bundle.robIndex;
                afterResteer <= True;
                rewindToToken.makeReq(FUNCP_REQ_REWIND_TO_TOKEN{token: bundle.token});
                state <= FETCH_STATE_REWIND_RESP;
            end
            else
                state <= FETCH_STATE_TOKEN_REQ;
        end
    endrule

    rule rewindResp(state == FETCH_STATE_REWIND_RESP);
        debugLog.record($format("rewindResp %d", fpgaCC));
        rewindToToken.deq();
        epoch <= epoch + 1;
        state <= FETCH_STATE_TOKEN_REQ;
    endrule

    rule tokenReq(state == FETCH_STATE_TOKEN_REQ);
        if(fetchPort.canSend())
        begin
            newInFlight.makeReq(?);
            state <= FETCH_STATE_I_TRANSLATE_REQ;
        end
        else
        begin
            debugLog.nextModelCycle();
            state <= FETCH_STATE_PREDICT_UPDATE;
            fetchPort.done();
        end
    endrule

    rule iTranslateReq(state == FETCH_STATE_I_TRANSLATE_REQ);
        let resp = newInFlight.getResp();
        resp.newToken.timep_info.epoch = epoch;
        newInFlight.deq();
        iTranslate.makeReq(FUNCP_REQ_DO_ITRANSLATE{token: resp.newToken, address: pc});
        state <= FETCH_STATE_INST_REQ;
    endrule

    rule instReq(state == FETCH_STATE_INST_REQ);
        let resp = iTranslate.getResp();
        iTranslate.deq();

        // iTranslate may return multiple responses for unaligned references.
        // Don't act until the last one is received.
        if (! resp.hasMore)
        begin
            getInstruction.makeReq(FUNCP_REQ_GET_INSTRUCTION{token: resp.token});
            state <= FETCH_STATE_INST_RESP;
        end
    endrule

    rule instResp(state == FETCH_STATE_INST_RESP);
        let resp = getInstruction.getResp;
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
        begin
            makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_NONE, False, 0, epochRob, afterResteer);
            pc <= pc + 4;
        end
    endrule

    rule branchImm(state == FETCH_STATE_BRANCH_IMM);
        let resp = getInstruction.getResp;
        let pred <- branchPred.getPredResp;
        let predPc = pred? predPcBranchImm(pc, resp.instruction): pc + 4;
        makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_BRANCH_IMM, pred, predPc, epochRob, afterResteer);
        pc <= predPc;
    endrule

    rule jumpImm(state == FETCH_STATE_JUMP_IMM);
        let resp = getInstruction.getResp;
        let predPc = predPcJumpImm(pc, resp.instruction);
        makeFetchBundle(resp.token, resp.instruction, pc, PRED_TYPE_JUMP_IMM, False, predPc, epochRob, afterResteer);
        pc <= predPc;
    endrule
endmodule
