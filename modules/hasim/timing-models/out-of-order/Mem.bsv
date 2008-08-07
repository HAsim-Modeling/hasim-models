import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

import FIFOF::*;

`include "PipelineTypes.bsv"
`include "DebugFile.bsv"

typedef enum {MEM_ADDRESS_STATE_FILL, MEM_ADDRESS_STATE_ADDRESS_REQ, MEM_ADDRESS_STATE_ADDRESS_RESP} MEM_ADDRESS_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMemAddress();
    DebugFile                                                                        debug <- mkDebugFile("MemAddress.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(MEM_BUNDLE, `MEM_NUM, `MEM_CREDITS)             memPort <- mkPortBandwidthCreditReceive("mem", `MEM_CREDITS);
    PORT_BANDWIDTH_CREDIT_SEND#(MEM_ADDRESS_BUNDLE, `MEM_NUM, `MEM_CREDITS) memAddressPort <- mkPortBandwidthCreditSend("memAddress");

    Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS)            getResults <- mkConnection_Client("funcp_getResults1");

    FIFOF#(MEM_BUNDLE)                                                             memFifo <- mkSizedFIFOF(`MEM_CREDITS);
    Reg#(MEM_ADDRESS_STATE)                                                          state <- mkReg(MEM_ADDRESS_STATE_FILL);
    Reg#(Bit#(TLog#(TAdd#(`MEM_CREDITS, 1))))                                   memCredits <- mkReg(`MEM_CREDITS);

    rule fill(state == MEM_ADDRESS_STATE_FILL);
        if(memPort.canReceive())
        begin
            let bundle <- memPort.pop();
            memFifo.enq(bundle);
            memCredits <= memCredits - 1;
        end
        else
        begin
            memPort.done(memCredits);
            state <= MEM_ADDRESS_STATE_ADDRESS_REQ;
        end
    endrule

    rule addressReq(state == MEM_ADDRESS_STATE_ADDRESS_REQ);
        if(memFifo.notEmpty() && memAddressPort.canSend())
        begin
            getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: memFifo.first().token});
            state <= MEM_ADDRESS_STATE_ADDRESS_RESP;
        end
        else
        begin
            debug.endModelCC();
            memAddressPort.done();
            state <= MEM_ADDRESS_STATE_FILL;
        end
    endrule

    rule addressResp(state == MEM_ADDRESS_STATE_ADDRESS_RESP);
        let res = getResults.getResp();
        getResults.deq();
        memAddressPort.enq(makeMemAddressBundle(memFifo.first(), res));
        memFifo.deq();
        memCredits <= memCredits + 1;
        state <= MEM_ADDRESS_STATE_ADDRESS_REQ;
    endrule
endmodule

typedef enum {MEM_STATE_FILL, MEM_STATE_D_TRANSLATE_REQ, MEM_STATE_MEM_REQ, MEM_STATE_MEM_RESP} MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem();
    DebugFile                                                                           debug <- mkDebugFile("Mem.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(MEM_ADDRESS_BUNDLE, `MEM_NUM, `MEM_CREDITS) memAddressPort <- mkPortBandwidthCreditReceive("memAddress", `MEM_CREDITS);
    PORT_BANDWIDTH_CREDIT_SEND#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, `MEM_NUM)    memWritebackPort <- mkPortBandwidthCreditSend("memWriteback");

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE, FUNCP_RSP_DO_DTRANSLATE)         doDTranslate <- mkConnection_Client("funcp_doDTranslate");
    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS)                        doLoads <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(FUNCP_REQ_DO_STORES, FUNCP_RSP_DO_STORES)                     doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    FIFOF#(MEM_ADDRESS_BUNDLE)                                                 memAddressFifo <- mkSizedFIFOF(`MEM_CREDITS);
    Reg#(MEM_STATE)                                                                     state <- mkReg(MEM_STATE_FILL);
    Reg#(Bit#(TLog#(TAdd#(`MEM_CREDITS, 1))))                                      memCredits <- mkReg(`MEM_CREDITS);

    rule fill(state == MEM_STATE_FILL);
        if(memAddressPort.canReceive())
        begin
            let bundle <- memAddressPort.pop();
            memAddressFifo.enq(bundle);
            memCredits <= memCredits - 1;
        end
        else
        begin
            memAddressPort.done(memCredits);
            state <= MEM_STATE_D_TRANSLATE_REQ;
        end
    endrule

    rule dTranslate(state == MEM_STATE_D_TRANSLATE_REQ);
        if(memAddressFifo.notEmpty() && memWritebackPort.canSend())
        begin
            doDTranslate.makeReq(FUNCP_REQ_DO_DTRANSLATE{token: memAddressFifo.first().token});
            state <= MEM_STATE_MEM_REQ;
        end
        else
        begin
            debug.endModelCC();
            memWritebackPort.done();
            state <= MEM_STATE_FILL;
        end
    endrule

    rule memReq(state == MEM_STATE_MEM_REQ);
        doDTranslate.deq();
        if(isaIsLoad(memAddressFifo.first().inst))
            doLoads.makeReq(FUNCP_REQ_DO_LOADS{token: memAddressFifo.first().token});
        else
            doStores.makeReq(FUNCP_REQ_DO_STORES{token: memAddressFifo.first().token});
        state <= MEM_STATE_MEM_RESP;
    endrule

    rule memResp(state == MEM_STATE_MEM_RESP);
        if(isaIsLoad(memAddressFifo.first().inst))
            doLoads.deq();
        else
            doStores.deq();
        memWritebackPort.enq(makeMemWritebackBundle(memAddressFifo.first()));
        memAddressFifo.deq();
        state <= MEM_STATE_D_TRANSLATE_REQ;
        memCredits <= memCredits + 1;
    endrule
endmodule
