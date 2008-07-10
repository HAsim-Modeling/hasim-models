import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_MEMORY.bsh"

typedef enum { MEM_STATE_REQ, MEM_STATE_LOAD, MEM_STATE_STORE, MEM_STATE_DONE } MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem ();

    DebugFile debug <- mkDebugFile("pipe_mem.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("exe2mem");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("mem_bus");

    Connection_Client#(TOKEN,TOKEN) doLoads  <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(TOKEN,TOKEN) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

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
            doLoads.makeReq(tok);
            state <= MEM_STATE_LOAD;
        end
        else if (bundle.isStore)
        begin
            doStores.makeReq(tok);
            state <= MEM_STATE_STORE;
        end
        else
            state <= MEM_STATE_DONE;
    endrule

    rule load (state == MEM_STATE_LOAD);
        doLoads.deq();
        state <= MEM_STATE_DONE;
    endrule

    rule store (state == MEM_STATE_STORE);
        doStores.deq();
        state <= MEM_STATE_DONE;
    endrule

    rule done (state == MEM_STATE_DONE &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let x <- inQ.receive();
        outQ.send(x);
        busQ.send(Invalid);
        event_mem.recordEvent(Valid(zeroExtend(tok.index)));
        state <= MEM_STATE_REQ;
    endrule

endmodule
