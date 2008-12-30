import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"

`include "hasim_pipeline_types.bsh"

import FIFOF::*;

typedef enum {MEM_ADDRESS_STATE_ADDRESS_REQ, MEM_ADDRESS_STATE_ADDRESS_RESP} MEM_ADDRESS_STATE deriving (Bits, Eq);
 
module [HASIM_MODULE] mkMemAddress();
    TIMEP_DEBUG_FILE                                                              debugLog <- mkTIMEPDebugFile("pipe_mem_addr.out");

    PORT_FIFO_RECEIVE#(MEM_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)                      memFifo <- mkPortFifoReceive("mem", True, `MEM_CREDITS);
    PORT_CREDIT_SEND#(MEM_ADDRESS_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)        memAddressPort <- mkPortCreditSend("memAddress");

    PORT_CREDIT_SEND#(FUNCP_REQ_GET_RESULTS, `MEM_NUM, LOG_MEM_NUM)          getResultsReq <- mkPortCreditSend("memGetResultsReq");
    Connection_Receive#(FUNCP_RSP_GET_RESULTS)                              getResultsResp <- mkConnection_Receive("memGetResultsResp");

    Reg#(MEM_ADDRESS_STATE)                                                          state <- mkReg(MEM_ADDRESS_STATE_ADDRESS_REQ);

    rule addressReq(state == MEM_ADDRESS_STATE_ADDRESS_REQ);
        if(memFifo.canReceive() && memAddressPort.canSend())
        begin
            debugLog.record($format("writeback req") + fshow(memFifo.first));
            getResultsReq.enq(FUNCP_REQ_GET_RESULTS{token: memFifo.first().token});
            state <= MEM_ADDRESS_STATE_ADDRESS_RESP;
        end
        else
        begin
            memFifo.done;
            getResultsReq.done;
            debugLog.record($format("writeback done"));
            debugLog.nextModelCycle();
            memAddressPort.done();
        end
    endrule

    rule addressResp(state == MEM_ADDRESS_STATE_ADDRESS_RESP);
        let res = getResultsResp.receive();
        getResultsResp.deq();
        debugLog.record($format("writeback resp") + fshow(memFifo.first));
        memAddressPort.enq(makeMemAddressBundle(memFifo.first(), res));
        memFifo.deq();
        state <= MEM_ADDRESS_STATE_ADDRESS_REQ;
    endrule
endmodule

typedef enum {MEM_STATE_D_TRANSLATE_REQ, MEM_STATE_MEM_REQ, MEM_STATE_MEM_RESP} MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem();
    TIMEP_DEBUG_FILE                                                            debugLog <- mkTIMEPDebugFile("pipe_mem.out");

    PORT_FIFO_RECEIVE#(MEM_ADDRESS_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)     memAddressFifo <- mkPortFifoReceive("memAddress", True, `MEM_CREDITS);
    PORT_CREDIT_SEND#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)  memWritebackPort <- mkPortCreditSend("memWriteback");

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE, FUNCP_RSP_DO_DTRANSLATE)    doDTranslate <- mkConnection_Client("funcp_doDTranslate");
    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS)                   doLoads <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(FUNCP_REQ_DO_STORES, FUNCP_RSP_DO_STORES)                doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    Reg#(MEM_STATE)                                                                state <- mkReg(MEM_STATE_D_TRANSLATE_REQ);

    rule dTranslate(state == MEM_STATE_D_TRANSLATE_REQ);
        if(memAddressFifo.canReceive() && memWritebackPort.canSend())
        begin
            debugLog.record($format(fshow(memAddressFifo.first().token) + $format(": doDTranslate request")));
            doDTranslate.makeReq(FUNCP_REQ_DO_DTRANSLATE{token: memAddressFifo.first().token});
            state <= MEM_STATE_MEM_REQ;
        end
        else
        begin
            debugLog.record($format("end cycle"));
            memAddressFifo.done;
            debugLog.nextModelCycle();
            memWritebackPort.done();
        end
    endrule

    rule memReq(state == MEM_STATE_MEM_REQ);
        let resp = doDTranslate.getResp();
        doDTranslate.deq();

        debugLog.record(fshow(resp.token) + $format(": doDTranslate response (PA 0x%h%s)", resp.physicalAddress, (resp.hasMore ? ", hasMore" : "")));

        // Has the translation completed or is it multi-part?  If multi-part
        // then stay in the current state and get the rest.  If the translation
        // is done then start the memory request.
        if (! resp.hasMore)
        begin
            if(isaIsLoad(memAddressFifo.first().inst))
            begin
                debugLog.record($format(fshow(resp.token) + $format(": memReq LOAD")));
                doLoads.makeReq(FUNCP_REQ_DO_LOADS{token: resp.token});
            end
            else
            begin
                debugLog.record($format(fshow(resp.token) + $format(": memReq STORE")));
                doStores.makeReq(FUNCP_REQ_DO_STORES{token: resp.token});
            end

            state <= MEM_STATE_MEM_RESP;
        end
    endrule

    rule memResp(state == MEM_STATE_MEM_RESP);
        TOKEN resp_token;

        if(isaIsLoad(memAddressFifo.first().inst))
        begin
            let resp = doLoads.getResp();
            doLoads.deq();
            resp_token = resp.token;
            debugLog.record($format("loads deq"));
        end
        else
        begin
            let resp = doStores.getResp();
            doStores.deq();
            resp_token = resp.token;
            debugLog.record($format("stores deq"));
        end

        debugLog.record($format(fshow(resp_token) + $format(": memResp")));

        memWritebackPort.enq(makeMemWritebackBundle(memAddressFifo.first(), resp_token));
        memAddressFifo.deq();

        state <= MEM_STATE_D_TRANSLATE_REQ;
    endrule
endmodule
