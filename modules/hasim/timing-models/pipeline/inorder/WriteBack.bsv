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

typedef enum {
    WB_STATE_REQ, WB_STATE_RETRY_REQ, WB_STATE_DCACHE, WB_STATE_DCACHE_RESP,
    WB_STATE_DCACHE_STALL, WB_STATE_STORE_RESP, WB_STATE_SEND, WB_STATE_PASS2
} WB_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkWriteBack ();

    TIMEP_DEBUG_LOG debugLog <- mkTIMEPDebugFile("pipe_writeback.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("mem2wb");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("wb_bus");

    Port_Send#(Tuple2#(TOKEN, CacheInput)) dcacheQ <- mkPort_Send("cpu_to_dcache_committed");
    Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) dcache_delQ <- mkPort_Receive("dcache_to_cpu_delayed_committed", 0);
    Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) dcache_immQ <- mkPort_Receive("dcache_to_cpu_immediate_committed", 0);

    Port_Send#(TOKEN) sb_deallocQ <- mkPort_Send("storebuffer_dealloc");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");

    Reg#(WB_STATE) state <- mkReg(WB_STATE_REQ);

    //Local Controller
    Vector#(3, Port_Control) inports  = newVector();
    Vector#(3, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = dcache_immQ.ctrl;
    inports[2]  = dcache_delQ.ctrl;
    outports[0] = busQ.ctrl;
    outports[1] = dcacheQ.ctrl;
    outports[2] = sb_deallocQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_wb <- mkEventRecorder(`EVENTS_WRITEBACK_INSTRUCTION_WRITEBACK);

    //Stats
    Stat stat_wb <- mkStatCounter(`STATS_WRITEBACK_INSTS_COMMITTED);

    // Number of commits (to go along with heartbeat)
    Connection_Send#(MODEL_NUM_COMMITS) linkModelCommit <- mkConnection_Send("model_commits");

    rule bubble (state == WB_STATE_REQ && !isValid(inQ.peek));
        debugLog.record($format("BUBBLE"));
        local_ctrl.startModelCC();
        inQ.pass();
        busQ.send(Invalid);
        sb_deallocQ.send(Invalid);
        dcacheQ.send(Invalid);
        state <= WB_STATE_PASS2;
    endrule

    rule pass2 (state == WB_STATE_PASS2);
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        event_wb.recordEvent(Invalid);
        state <= WB_STATE_REQ;
    endrule

    rule results (state == WB_STATE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        commitResults.makeReq(initFuncpReqCommitResults(tok));
        state <= WB_STATE_DCACHE;
    endrule

    rule dcache_retry (state == WB_STATE_RETRY_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        local_ctrl.startModelCC();
        debugLog.record(fshow("RETRYING DCACHE STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.effAddr));
        dcacheQ.send(Valid(tuple2(tok,Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
        state <= WB_STATE_DCACHE_RESP;
    endrule

    rule dcache (state == WB_STATE_DCACHE &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        commitResults.deq();
        if (bundle.isStore) begin
            debugLog.record(fshow("DCACHE STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.effAddr));
            dcacheQ.send(Valid(tuple2(tok,Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
        end
        else
            dcacheQ.send(Invalid);
        state <= WB_STATE_DCACHE_RESP;
    endrule

    rule dcache_resp (state == WB_STATE_DCACHE_RESP &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        if (del == Invalid &&& imm matches tagged Valid { .tok, tagged Miss_retry .* })
        begin
            sb_deallocQ.send(Invalid);
            state <= WB_STATE_DCACHE_STALL;
        end
        else
        begin
            if (bundle.isStore)
            begin
                sb_deallocQ.send(Valid(tok));
                commitStores.makeReq(initFuncpReqCommitStores(tok));
                state <= WB_STATE_STORE_RESP;
            end
            else
            begin
                sb_deallocQ.send(Invalid);
                state <= WB_STATE_SEND;
            end
        end
    endrule

    rule store_resp (state == WB_STATE_STORE_RESP);
        commitStores.deq();
        state <= WB_STATE_SEND;
    endrule

    rule dcache_stall (state == WB_STATE_DCACHE_STALL &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.record(fshow("DCACHE RETRY ") + fshow(tok));
        inQ.pass();
        busQ.send(Invalid);
        event_wb.recordEvent(Invalid);
        state <= WB_STATE_RETRY_REQ;
    endrule

    rule done (state == WB_STATE_SEND &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.record(fshow("DONE: ") + fshow(tok) + fshow(" ") + fshow(bundle));
        if (bundle.isTerminate matches tagged Valid .pf)
            local_ctrl.endProgram(pf);
        let x <- inQ.receive();
        busQ.send(Valid(bundle.dests));
        event_wb.recordEvent(Valid(zeroExtend(tok.index)));
        stat_wb.incr();
        linkModelCommit.send(1);
        state <= WB_STATE_REQ;
    endrule

endmodule
