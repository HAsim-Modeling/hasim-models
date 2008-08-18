import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

`include "PipelineTypes.bsv"

`include "funcp_simulated_memory.bsh"

typedef enum { FETCH_STATE_REWIND_REQ, FETCH_STATE_REWIND_RESP, FETCH_STATE_TOKEN_REQ, FETCH_STATE_I_TRANSLATE_REQ, FETCH_STATE_INST_REQ, FETCH_STATE_INST_RESP } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch();
    TIMEP_DEBUG_FILE                                                               debugLog <- mkTIMEPDebugFile("pipe_fet.out");

    PORT_BANDWIDTH_CREDIT_SEND#(FETCH_BUNDLE, `FETCH_NUM, `FETCH_CREDITS)         fetchPort <- mkPortBandwidthCreditSend("fetch");

    PORT_RECEIVE#(REWIND_BUNDLE)                                                resteerPort <- mkPortReceive("resteer");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT, FUNCP_RSP_NEW_IN_FLIGHT)        newInFlight <- mkConnection_Client("funcp_newInFlight");
    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE, FUNCP_RSP_DO_ITRANSLATE)         iTranslate <- mkConnection_Client("funcp_doITranslate");
    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION, FUNCP_RSP_GET_INSTRUCTION) getInstruction <- mkConnection_Client("funcp_getInstruction");
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    Reg#(ISA_ADDRESS)                                                                    pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(FETCH_STATE)                                                                 state <- mkReg(FETCH_STATE_REWIND_REQ);
    Reg#(ROB_INDEX)                                                                epochRob <- mkRegU();
    Reg#(Bool)                                                                 afterResteer <- mkReg(False);
    Reg#(TOKEN_TIMEP_EPOCH)                                                           epoch <- mkReg(0);

    Reg#(Bit#(32))                                                                   fpgaCC <- mkReg(0);
    rule inc(True);
        fpgaCC <= fpgaCC + 1;
    endrule

    rule rewindReq(state == FETCH_STATE_REWIND_REQ);
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
            state <= FETCH_STATE_REWIND_REQ;
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
        getInstruction.makeReq(FUNCP_REQ_GET_INSTRUCTION{token: resp.token});
        state <= FETCH_STATE_INST_RESP;
    endrule

    rule instResp(state == FETCH_STATE_INST_RESP);
        let resp = getInstruction.getResp();
        getInstruction.deq();
        let bundle = FETCH_BUNDLE{token: resp.token, inst: resp.instruction, pc: pc, prediction: False, epochRob: epochRob, afterResteer: afterResteer};
        debugLog.record($format("instResp ") + fshow(bundle));
        fetchPort.enq(bundle);
        pc <= pc + 4;
        afterResteer <= False;
        state <= FETCH_STATE_TOKEN_REQ;
    endrule
endmodule
