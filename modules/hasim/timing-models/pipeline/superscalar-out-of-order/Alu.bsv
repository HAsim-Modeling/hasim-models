import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

import FIFOF::*;

`include "PipelineTypes.bsv"

typedef enum {ALU_STATE_FILL, ALU_STATE_WRITEBACK_REQ, ALU_STATE_WRITEBACK_RESP} ALU_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkAlu();
    TIMEP_DEBUG_FILE                                                              debugLog <- mkTIMEPDebugFile("pipe_alu.out");

    PORT_CREDIT_RECEIVE#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                    aluPort <- mkPortCreditReceive("alu", `ALU_CREDITS);
    PORT_CREDIT_SEND#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, LOG_ALU_NUM)        aluWritebackPort <- mkPortCreditSend("aluWriteback");

    Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS)            getResults <- mkConnection_Client("funcp_getResults0");

    FIFOF#(ALU_BUNDLE)                                                             aluFifo <- mkSizedFIFOF(`ALU_CREDITS);
    Reg#(ALU_STATE)                                                                  state <- mkReg(ALU_STATE_FILL);
    Reg#(Bit#(TLog#(TAdd#(`ALU_CREDITS, 1))))                                   aluCredits <- mkReg(`ALU_CREDITS);

    rule fill(state == ALU_STATE_FILL);
        if(aluPort.canReceive())
        begin
            let bundle <- aluPort.pop();
            debugLog.record($format("fill") + fshow(bundle));
            aluFifo.enq(bundle);
            aluCredits <= aluCredits - 1;
        end
        else
        begin
            debugLog.record($format("fill end"));
            aluPort.done(aluCredits);
            state <= ALU_STATE_WRITEBACK_REQ;
        end
    endrule

    rule writebackReq(state == ALU_STATE_WRITEBACK_REQ);
        if(aluFifo.notEmpty() && aluWritebackPort.canSend())
        begin
            debugLog.record($format("writeback req") + fshow(aluFifo.first));
            getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: aluFifo.first().token});
            state <= ALU_STATE_WRITEBACK_RESP;
        end
        else
        begin
            debugLog.record($format("writeback done"));
            debugLog.nextModelCycle();
            aluWritebackPort.done();
            state <= ALU_STATE_FILL;
        end
    endrule

    rule writebackResp(state == ALU_STATE_WRITEBACK_RESP);
        let res = getResults.getResp();
        getResults.deq();
        debugLog.record($format("writeback resp") + fshow(aluFifo.first));
        aluWritebackPort.enq(makeAluWritebackBundle(aluFifo.first(), res));
        aluFifo.deq();
        aluCredits <= aluCredits + 1;
        state <= ALU_STATE_WRITEBACK_REQ;
    endrule
endmodule
