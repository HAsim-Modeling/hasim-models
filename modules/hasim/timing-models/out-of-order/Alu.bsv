import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

import FIFOF::*;

`include "PipelineTypes.bsv"
`include "DebugFile.bsv"

typedef enum {ALU_STATE_FILL, ALU_STATE_WRITEBACK_REQ, ALU_STATE_WRITEBACK_RESP} ALU_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkAlu();
    DebugFile                                                                        debug <- mkDebugFile("Alu.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(ALU_BUNDLE, `ALU_NUM, `ALU_CREDITS)             aluPort <- mkPortBandwidthCreditReceive("alu", `ALU_CREDITS);
    PORT_BANDWIDTH_CREDIT_SEND#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, `ALU_NUM) aluWritebackPort <- mkPortBandwidthCreditSend("aluWriteback");

    Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS)            getResults <- mkConnection_Client("funcp_getResults0");

    FIFOF#(ALU_BUNDLE)                                                             aluFifo <- mkSizedFIFOF(`ALU_CREDITS);
    Reg#(ALU_STATE)                                                                  state <- mkReg(ALU_STATE_FILL);
    Reg#(Bit#(TLog#(TAdd#(`ALU_CREDITS, 1))))                                   aluCredits <- mkReg(`ALU_CREDITS);

    rule fill(state == ALU_STATE_FILL);
        if(aluPort.canReceive())
        begin
            let bundle <- aluPort.pop();
            aluFifo.enq(bundle);
            aluCredits <= aluCredits - 1;
        end
        else
        begin
            aluPort.done(aluCredits);
            state <= ALU_STATE_WRITEBACK_REQ;
        end
    endrule

    rule writebackReq(state == ALU_STATE_WRITEBACK_REQ);
        if(aluFifo.notEmpty() && aluWritebackPort.canSend())
        begin
            getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: aluFifo.first().token});
            state <= ALU_STATE_WRITEBACK_RESP;
        end
        else
        begin
            debug.endModelCC();
            aluWritebackPort.done();
            state <= ALU_STATE_FILL;
        end
    endrule

    rule writebackResp(state == ALU_STATE_WRITEBACK_RESP);
        let res = getResults.getResp();
        getResults.deq();
        aluWritebackPort.enq(makeAluWritebackBundle(aluFifo.first(), res));
        aluFifo.deq();
        aluCredits <= aluCredits + 1;
        state <= ALU_STATE_WRITEBACK_REQ;
    endrule
endmodule
