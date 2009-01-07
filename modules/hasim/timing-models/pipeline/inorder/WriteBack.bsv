//
// Copyright (C) 2008 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

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

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_writeback.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("mem2wb");

    Port_Send#(TOKEN) commitQ <- mkPort_Send("commit_bus");

    Port_Send#(TOKEN) faultQ <- mkPort_Send("fault");

    Port_Send#(Tuple2#(TOKEN, CacheInput)) dcacheQ <- mkPort_Send("cpu_to_dcache_committed");
    Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) dcache_delQ <- mkPort_Receive("dcache_to_cpu_delayed_committed", 0);
    Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) dcache_immQ <- mkPort_Receive("dcache_to_cpu_immediate_committed", 0);

    Port_Send#(TOKEN) sb_deallocQ <- mkPort_Send("storebuffer_dealloc");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");

    Reg#(WB_STATE) state <- mkReg(WB_STATE_REQ);
    Reg#(TOKEN_FAULT_EPOCH) faultEpoch <- mkReg(0);

    //Local Controller
    Vector#(3, Port_Control) inports  = newVector();
    Vector#(4, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = dcache_immQ.ctrl;
    inports[2]  = dcache_delQ.ctrl;
    outports[0] = commitQ.ctrl;
    outports[1] = dcacheQ.ctrl;
    outports[2] = sb_deallocQ.ctrl;
    outports[3] = faultQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_wb <- mkEventRecorder(`EVENTS_WRITEBACK_INSTRUCTION_WRITEBACK);

    //Stats
    Stat stat_wb <- mkStatCounter(`STATS_WRITEBACK_INSTS_COMMITTED);

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");

    rule bubble (state == WB_STATE_REQ && !isValid(inQ.peek));
        debugLog.record($format("BUBBLE"));
        inQ.pass();
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        commitQ.send(Invalid);
        sb_deallocQ.send(Invalid);
        dcacheQ.send(Invalid);
        faultQ.send(Invalid);
        state <= WB_STATE_PASS2;
    endrule

    rule pass2 (state == WB_STATE_PASS2);
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        event_wb.recordEvent(Invalid);
        state <= WB_STATE_REQ;
    endrule

    rule results (state == WB_STATE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();

        if (! tokIsPoisoned(tok) && (tokFaultEpoch(tok) == faultEpoch))
        begin
            //
            // Normal commit flow for a good instruction.
            //
            debugLog.record(fshow("COMMIT: ") + fshow(tok) + fshow(" ") + fshow(bundle));
            commitResults.makeReq(initFuncpReqCommitResults(tok));
            faultQ.send(Invalid);
            state <= WB_STATE_DCACHE;
        end
        else
        begin
            //
            // Exception flow.
            //
            let x <- inQ.receive();
            dcacheQ.send(Invalid);

            // Instruction no longer in flight
            commitQ.send(Valid(tok));
            state <= WB_STATE_PASS2;

            // Drop token from store buffer
            if (bundle.isStore())
                sb_deallocQ.send(Valid(tok));
            else
                sb_deallocQ.send(Invalid);

            if (tokFaultEpoch(tok) != faultEpoch)
            begin
                // Draining following earlier fault
                debugLog.record(fshow("DRAIN: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                faultQ.send(Invalid);
            end
            else
            begin
                // Fault.  Redirect the front end to handle the fault.
                debugLog.record(fshow("FAULT: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                faultEpoch <= faultEpoch + 1;
                faultQ.send(tagged Valid tok);
            end
        end
    endrule

    rule dcache_retry (state == WB_STATE_RETRY_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        debugLog.record(fshow("RETRYING DCACHE STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.effAddr));
        dcacheQ.send(Valid(tuple2(tok,Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
        faultQ.send(Invalid);
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
        commitQ.send(Invalid);
        event_wb.recordEvent(Invalid);
        state <= WB_STATE_RETRY_REQ;
    endrule

    rule done (state == WB_STATE_SEND &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.record(fshow("DONE: ") + fshow(tok) + fshow(" ") + fshow(bundle));
        if (bundle.isTerminate matches tagged Valid .pf)
            local_ctrl.endProgram(pf);
        let x <- inQ.receive();
        commitQ.send(Valid(tok));
        event_wb.recordEvent(Valid(zeroExtend(pack(tok.index))));
        stat_wb.incr();
        linkModelCommit.send(tuple2(0, 1));
        state <= WB_STATE_REQ;
    endrule

endmodule
