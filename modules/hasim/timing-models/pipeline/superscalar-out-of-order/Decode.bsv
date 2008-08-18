import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import FIFOF::*;
import Vector::*;

`include "PipelineTypes.bsv"

typedef enum {DECODE_STATE_FILL, DECODE_STATE_REQ_DEPENDENCIES, DECODE_STATE_RESP_DEPENDENCIES} DECODE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDecode();
    TIMEP_DEBUG_FILE                                                                  debugLog <- mkTIMEPDebugFile("pipe_dec.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(FETCH_BUNDLE, `FETCH_NUM, `FETCH_CREDITS)         fetchPort <- mkPortBandwidthCreditReceive("fetch", fromInteger(`FETCH_CREDITS));
    PORT_BANDWIDTH_CREDIT_SEND#(DECODE_BUNDLE, `DECODE_NUM, DECODE_CREDITS)         decodePort <- mkPortBandwidthCreditSend("decode");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES, FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    FIFOF#(FETCH_BUNDLE)                                                           fetchBuffer <- mkSizedFIFOF(`FETCH_CREDITS);
    Reg#(DECODE_STATE)                                                                   state <- mkReg(DECODE_STATE_FILL);
    Reg#(FETCH_BUFFER_INDEX)                                                      fetchCredits <- mkReg(`FETCH_CREDITS);

    function Vector#(n, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) extractPhysReg(Vector#(n, Maybe#(Tuple2#(ISA_REG_INDEX, FUNCP_PHYSICAL_REG_INDEX))) regMap);
        Vector#(n, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs = newVector();
        for(Integer i = 0; i < valueOf(n); i = i + 1)
        begin
            case (regMap[i]) matches
                tagged Valid {.ar, .pr}: regs[i] = tagged Valid pr;
                tagged Invalid: regs[i] = tagged Invalid;
            endcase
        end
        return regs;
    endfunction

    rule fill(state == DECODE_STATE_FILL);
        if(fetchPort.canReceive())
        begin
            let fetch <- fetchPort.pop();
            fetchBuffer.enq(fetch);
            fetchCredits <= fetchCredits - 1;
        end
        else
        begin
            fetchPort.done(fetchCredits);
            state <= DECODE_STATE_REQ_DEPENDENCIES;
        end
    endrule

    rule reqDependencies(state == DECODE_STATE_REQ_DEPENDENCIES);
        if(fetchBuffer.notEmpty() && decodePort.canSend())
        begin
            getDependencies.makeReq(FUNCP_REQ_GET_DEPENDENCIES{token: fetchBuffer.first().token});
            state <= DECODE_STATE_RESP_DEPENDENCIES;
        end
        else
        begin
            debugLog.nextModelCycle();
            decodePort.done();
            state <= DECODE_STATE_FILL;
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
        fetchCredits <= fetchCredits + 1;
        state <= DECODE_STATE_REQ_DEPENDENCIES;
    endrule
endmodule
