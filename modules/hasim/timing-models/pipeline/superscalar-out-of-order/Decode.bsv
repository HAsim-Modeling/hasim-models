import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import FIFOF::*;
import Vector::*;

`include "PipelineTypes.bsv"

typedef enum {DECODE_STATE_REQ_DEPENDENCIES, DECODE_STATE_RESP_DEPENDENCIES} DECODE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDecode();
    TIMEP_DEBUG_FILE                                                                  debugLog <- mkTIMEPDebugFile("pipe_dec.out");

    PORT_FIFO_RECEIVE#(FETCH_BUNDLE, `FETCH_NUM, LOG_FETCH_CREDITS)                fetchBuffer <- mkPortFifoReceive("fetch", True, `FETCH_CREDITS);
    PORT_CREDIT_SEND#(DECODE_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)               decodePort <- mkPortCreditSend("decode");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES, FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    Reg#(DECODE_STATE)                                                                   state <- mkReg(DECODE_STATE_REQ_DEPENDENCIES);

    rule reqDependencies(state == DECODE_STATE_REQ_DEPENDENCIES);
        if(fetchBuffer.canReceive() && decodePort.canSend())
        begin
            getDependencies.makeReq(FUNCP_REQ_GET_DEPENDENCIES{token: fetchBuffer.first().token});
            state <= DECODE_STATE_RESP_DEPENDENCIES;
        end
        else
        begin
            fetchBuffer.done;
            debugLog.nextModelCycle();
            decodePort.done;
        end
    endrule

    rule respDependencies(state == DECODE_STATE_RESP_DEPENDENCIES);
        let resp = getDependencies.getResp();
        getDependencies.deq();
        fetchBuffer.deq();
        let fetchBundle = fetchBuffer.first();
        let decodeBundle = makeDecodeBundle(fetchBundle, extractPhysReg(resp.srcMap), extractPhysReg(resp.dstMap));
        debugLog.record($format("respDependencies ") + fshow(decodeBundle));
        decodePort.enq(decodeBundle);
        state <= DECODE_STATE_REQ_DEPENDENCIES;
    endrule
endmodule
