import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import PipelineTypes::*;
import Vector::*;

typedef enum { STATE_REQ, STATE_RESULTS, STATE_STORE, STATE_SEND } STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkWriteBack ();

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("wb_bus");

    Connection_Client#(TOKEN,TOKEN) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(TOKEN,TOKEN) commitStores  <- mkConnection_Client("funcp_commitStores");

    Reg#(STATE) state <- mkReg(STATE_REQ);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //outports[0] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule bubble (state == STATE_REQ && !isValid(inQ.peek));
        local_ctrl.startModelCC();
        inQ.pass();
        busQ.send(Invalid);
    endrule

    rule results (state == STATE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        commitResults.makeReq(tok);
        state <= STATE_RESULTS;
    endrule

    rule stores (state == STATE_RESULTS &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        commitResults.deq();
        if (bundle.isStore)
        begin
            commitStores.makeReq(tok);
            state <= STATE_STORE;
        end
        else
            state <= STATE_SEND;
    endrule

    rule stores2 (state == STATE_STORE);
        commitStores.deq();
        state <= STATE_SEND;
    endrule

    rule done (state == STATE_SEND &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let x <- inQ.receive();
        busQ.send(Valid(bundle.dests));
    endrule

endmodule
