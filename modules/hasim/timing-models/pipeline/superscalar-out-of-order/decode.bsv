import FIFOF::*;
import Vector::*;
import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"

`include "hasim_pipeline_types.bsh"

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
            debugLog.record($format("dep req: ") + fshow(fetchBuffer.first.token));
            getDependencies.makeReq(FUNCP_REQ_GET_DEPENDENCIES{token: fetchBuffer.first().token});
            state <= DECODE_STATE_RESP_DEPENDENCIES;
        end
        else
        begin
            debugLog.record($format("end cycle"));
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
