import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;
`include "asim/provides/hasim_controller.bsh"

import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_WRITEBACK.bsh"
`include "asim/dict/STATS_WRITEBACK.bsh"

typedef enum { WB_STATE_REQ, WB_STATE_RESULTS, WB_STATE_STORE, WB_STATE_SEND } WB_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkWriteBack ();

    DebugFile debug <- mkDebugFile("pipe_writeback.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("wb_bus");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");

    Reg#(WB_STATE) state <- mkReg(WB_STATE_REQ);

    //Local Controller
    Vector#(1, Port_Control) inports  = newVector();
    Vector#(1, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    outports[0] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_wb <- mkEventRecorder(`EVENTS_WRITEBACK_INSTRUCTION_WRITEBACK);

    //Stats
    Stat stat_wb <- mkStatCounter(`STATS_WRITEBACK_INSTS_COMMITTED);

    // Number of commits (to go along with heartbeat)
    Connection_Send#(MODEL_NUM_COMMITS) linkModelCommit <- mkConnection_Send("model_commits");

    rule bubble (state == WB_STATE_REQ && !isValid(inQ.peek));
        debug <= $format("BUBBLE");
        local_ctrl.startModelCC();
        debug.startModelCC();
        inQ.pass();
        busQ.send(Invalid);
        event_wb.recordEvent(Invalid);
    endrule

    rule results (state == WB_STATE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        debug.startModelCC();
        stat_wb.incr();
        linkModelCommit.send(1);
        commitResults.makeReq(initFuncpReqCommitResults(tok));
        state <= WB_STATE_RESULTS;
    endrule

    rule stores (state == WB_STATE_RESULTS &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        commitResults.deq();
        if (bundle.isStore)
        begin
            commitStores.makeReq(initFuncpReqCommitStores(tok));
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
        debug <= fshow("DONE: ") + fshow(tok) + fshow(" ") + fshow(bundle);
        if (bundle.isTerminate matches tagged Valid .pf)
            local_ctrl.endProgram(pf);
        let x <- inQ.receive();
        busQ.send(Valid(bundle.dests));
        event_wb.recordEvent(Valid(zeroExtend(tok.index)));
        state <= WB_STATE_REQ;
    endrule

endmodule
