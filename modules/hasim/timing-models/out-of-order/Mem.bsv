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

    Connection_Client#(TOKEN, FUNCP_GET_RESULTS_MSG)                            getResults <- mkConnection_Client("funcp_getResults1");

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
            getResults.makeReq(memFifo.first().token);
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

typedef enum {MEM_STATE_FILL, MEM_STATE_MEM_REQ, MEM_STATE_MEM_RESP} MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem();
    DebugFile                                                                           debug <- mkDebugFile("Mem.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(MEM_ADDRESS_BUNDLE, `MEM_NUM, `MEM_CREDITS) memAddressPort <- mkPortBandwidthCreditReceive("memAddress", `MEM_CREDITS);
    PORT_BANDWIDTH_CREDIT_SEND#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, `MEM_NUM)    memWritebackPort <- mkPortBandwidthCreditSend("memWriteback");

    Connection_Client#(TOKEN,TOKEN)                                                   doLoads <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(TOKEN,TOKEN)                                                  doStores <- mkConnection_Client("funcp_doSpeculativeStores");

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
            state <= MEM_STATE_MEM_REQ;
        end
    endrule

    rule memReq(state == MEM_STATE_MEM_REQ);
        if(memAddressFifo.notEmpty() && memWritebackPort.canSend())
        begin
            if(isaIsLoad(memAddressFifo.first().inst))
                doLoads.makeReq(memAddressFifo.first().token);
            else
                doStores.makeReq(memAddressFifo.first().token);
            state <= MEM_STATE_MEM_RESP;
        end
        else
        begin
            debug.endModelCC();
            memWritebackPort.done();
            state <= MEM_STATE_FILL;
        end
    endrule

    rule memResp(state == MEM_STATE_MEM_RESP);
        if(isaIsLoad(memAddressFifo.first().inst))
            doLoads.deq();
        else
            doStores.deq();
        memWritebackPort.enq(makeMemWritebackBundle(memAddressFifo.first()));
        memAddressFifo.deq();
        state <= MEM_STATE_MEM_REQ;
        memCredits <= memCredits + 1;
    endrule
endmodule
