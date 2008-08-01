import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_MEMORY.bsh"

typedef enum { MEM_STATE_REQ, MEM_STATE_LOAD_REQ, MEM_STATE_STORE_REQ, MEM_STATE_LOAD_RSP, MEM_STATE_STORE_RSP} MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem ();

    DebugFile debug <- mkDebugFile("pipe_mem.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("exe2mem");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("mem_bus");

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE, FUNCP_RSP_DO_DTRANSLATE) doDTranslate  <- mkConnection_Client("funcp_doDTranslate");
    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS) doLoads  <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(FUNCP_REQ_DO_STORES,FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    Reg#(MEM_STATE) state <- mkReg(MEM_STATE_REQ);

    //Local Controller
    Vector#(1, Port_Control) inports  = newVector();
    Vector#(2, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_mem <- mkEventRecorder(`EVENTS_MEMORY_INSTRUCTION_MEM);

    function Action finishCycle(TOKEN tok, BUNDLE bundle);
    action

        let x <- inQ.receive();
        outQ.send(x);
        busQ.send(Invalid);
        event_mem.recordEvent(Valid(zeroExtend(tok.index)));
        state <= MEM_STATE_REQ;

    endaction
    endfunction

    rule stall (state == MEM_STATE_REQ && !outQ.canSend);
        local_ctrl.startModelCC();
        debug.startModelCC();
        debug <= fshow("STALL PROPOGATED");
        inQ.pass();
        outQ.pass();
        busQ.send(Invalid);
        event_mem.recordEvent(Invalid);
    endrule

    rule bubble (state == MEM_STATE_REQ && outQ.canSend && !isValid(inQ.peek));
        local_ctrl.startModelCC();
        debug.startModelCC();
        debug <= fshow("BUBBLE");
        let x <- inQ.receive();
        outQ.send(Invalid);
        busQ.send(Invalid);
        event_mem.recordEvent(Invalid);
    endrule

    rule doit (state == MEM_STATE_REQ &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debug <= fshow("FUNCP-REQ: ") + fshow(tok) + fshow(" ") + fshow(bundle);
        local_ctrl.startModelCC();
        debug.startModelCC();
        if (bundle.isLoad)
        begin
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
            state <= MEM_STATE_LOAD_REQ;
        end
        else if (bundle.isStore)
        begin
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
            state <= MEM_STATE_STORE_REQ;
        end
        else
        begin
            
            finishCycle(tok, bundle);
        
        end
    endrule
    
    rule loadReq (state == MEM_STATE_LOAD_REQ);
    
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();
        doLoads.makeReq(initFuncpReqDoLoads(rsp.token));
        state <= MEM_STATE_LOAD_RSP;
    
    endrule

    rule storeReq (state == MEM_STATE_STORE_REQ);
    
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();
        doStores.makeReq(initFuncpReqDoStores(rsp.token));
        state <= MEM_STATE_STORE_RSP;
    
    endrule

    rule loadRsp (state == MEM_STATE_LOAD_RSP &&& inQ.peek() matches tagged Valid { .tok, .bundle });

        doLoads.deq();
        finishCycle(tok, bundle);

    endrule

    rule storeRsp (state == MEM_STATE_STORE_RSP &&& inQ.peek() matches tagged Valid { .tok, .bundle });

        doStores.deq();
        finishCycle(tok, bundle);
        
    endrule

endmodule
