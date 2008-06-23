import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import PipelineTypes::*;
import Vector::*;

typedef enum { STATE_REQ, STATE_LOAD, STATE_STORE } STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem ();

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("exe2mem");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("mem_bus");

    Connection_Client#(TOKEN,TOKEN) doLoads  <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(TOKEN,TOKEN) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    Reg#(STATE) state <- mkReg(STATE_REQ);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //outports[0] = outQ.ctrl;
    //outports[1] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule stall (state == STATE_REQ && !outQ.canSend);
        local_ctrl.startModelCC();
        inQ.pass();
        outQ.pass();
        busQ.send(Invalid);
    endrule

    rule bubble (state == STATE_REQ && outQ.canSend && !isValid(inQ.peek));
        local_ctrl.startModelCC();
        let x <- inQ.receive();
        outQ.send(Invalid);
        busQ.send(Invalid);
    endrule

    rule doit (state == STATE_REQ &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        if (bundle.isLoad)
        begin
            doLoads.makeReq(tok);
            state <= STATE_LOAD;
        end
        else if (bundle.isStore)
        begin
            doStores.makeReq(tok);
            state <= STATE_STORE;
        end
    endrule

    rule load (state == STATE_LOAD);
        doLoads.deq();
        let x <- inQ.receive();
        outQ.send(x);
        busQ.send(Invalid);
        state <= STATE_REQ;
    endrule

    rule store (state == STATE_STORE);
        doStores.deq();
        let x <- inQ.receive();
        busQ.send(Invalid);
        if (x matches tagged Valid { .tok, .bundle })
            outQ.send(x);
        state <= STATE_REQ;
    endrule

endmodule
