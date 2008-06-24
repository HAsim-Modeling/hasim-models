import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import PipelineTypes::*;
import Vector::*;

typedef enum { WB_STATE_REQ, WB_STATE_RESULTS, WB_STATE_STORE, WB_STATE_SEND } WB_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkWriteBack ();

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("wb_bus");

    Connection_Client#(TOKEN,TOKEN) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(TOKEN,TOKEN) commitStores  <- mkConnection_Client("funcp_commitStores");

    Reg#(WB_STATE) state <- mkReg(WB_STATE_REQ);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //outports[0] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule bubble (state == WB_STATE_REQ && !isValid(inQ.peek));
        local_ctrl.startModelCC();
        inQ.pass();
        busQ.send(Invalid);
    endrule

    rule results (state == WB_STATE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        commitResults.makeReq(tok);
        state <= WB_STATE_RESULTS;
    endrule

    rule stores (state == WB_STATE_RESULTS &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        commitResults.deq();
        if (bundle.isStore)
        begin
            commitStores.makeReq(tok);
            state <= WB_STATE_STORE;
        end
        else
            state <= WB_STATE_SEND;
    endrule

    rule stores2 (state == WB_STATE_STORE);
        commitStores.deq();
        state <= WB_STATE_SEND;
    endrule

    rule done (state == WB_STATE_SEND &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let x <- inQ.receive();
        busQ.send(Valid(bundle.dests));
    endrule

endmodule
