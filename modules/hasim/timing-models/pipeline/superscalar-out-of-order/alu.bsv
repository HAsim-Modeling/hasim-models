import FIFOF::*;
import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"

`include "pipeline_base_types.bsh"

typedef enum {ALU_STATE_WRITEBACK_REQ, ALU_STATE_WRITEBACK_RESP} ALU_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkAlu();
    TIMEP_DEBUG_FILE                                                              debugLog <- mkTIMEPDebugFile("pipe_alu.out");

    PORT_FIFO_RECEIVE#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                      aluFifo <- mkPortFifoReceive("alu", True, `ALU_CREDITS);
    PORT_CREDIT_SEND#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)    aluWritebackPort <- mkPortCreditSend("aluWriteback");

    PORT_CREDIT_SEND#(FUNCP_REQ_GET_RESULTS, `ALU_NUM, LOG_ALU_NUM)          getResultsReq <- mkPortCreditSend("aluGetResultsReq");
    Connection_Receive#(FUNCP_RSP_GET_RESULTS)                              getResultsResp <- mkConnection_Receive("aluGetResultsResp");

    Reg#(ALU_STATE)                                                                  state <- mkReg(ALU_STATE_WRITEBACK_REQ);

    rule writebackReq(state == ALU_STATE_WRITEBACK_REQ);
        if(aluFifo.canReceive() && aluWritebackPort.canSend())
        begin
            debugLog.record($format("writeback req") + fshow(aluFifo.first));
            getResultsReq.enq(FUNCP_REQ_GET_RESULTS{token: aluFifo.first.token});
            state <= ALU_STATE_WRITEBACK_RESP;
        end
        else
        begin
            getResultsReq.done;
            debugLog.record($format("writeback done"));
            debugLog.nextModelCycle();
            aluFifo.done;
            aluWritebackPort.done();
        end
    endrule

    rule writebackResp(state == ALU_STATE_WRITEBACK_RESP);
        let res = getResultsResp.receive();
        getResultsResp.deq();
        debugLog.record($format("writeback resp") + fshow(aluFifo.first));
        aluWritebackPort.enq(makeAluWritebackBundle(aluFifo.first(), res));
        aluFifo.deq();
        state <= ALU_STATE_WRITEBACK_REQ;
    endrule
endmodule
