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

import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_MEMORY.bsh"

typedef enum {
    MEM_STATE_REQ, MEM_STATE_DCACHE_REQ, MEM_STATE_DCACHE_REPLY,
    MEM_STATE_FUNCP1, MEM_STATE_FUNCP2, MEM_STATE_FUNCP3,
    MEM_STATE_SB_STALL, MEM_STATE_DCACHE_STALL, MEM_STATE_PASS2
} MEM_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkMem ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_mem.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("exe2mem");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("mem2wb");

    Port_Send#(BUS_MESSAGE) busQ <- mkPort_Send("mem_bus");

    Port_Send#(Tuple2#(TOKEN, CacheInput)) dcacheQ <- mkPort_Send("cpu_to_dcache_speculative");
    Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) dcache_delQ <- mkPort_Receive("dcache_to_cpu_delayed_speculative", 0);
    Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) dcache_immQ <- mkPort_Receive("dcache_to_cpu_immediate_speculative", 0);

    Port_Send#(Tuple2#(TOKEN, CacheInput)) sbQ <- mkPort_Send("storebuffer_req");
    Port_Receive#(Tuple2#(TOKEN, SB_RESPONSE)) sb_respQ <- mkPort_Receive("storebuffer_resp", 0);

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE, FUNCP_RSP_DO_DTRANSLATE) doDTranslate  <- mkConnection_Client("funcp_doDTranslate");
    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS) doLoads  <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(FUNCP_REQ_DO_STORES,FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    Reg#(Bool) stalled <- mkReg(False);

    Reg#(MEM_STATE) state <- mkReg(MEM_STATE_REQ);

    //Local Controller
    Vector#(4, Port_Control) inports  = newVector();
    Vector#(4, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = dcache_immQ.ctrl;
    inports[2]  = dcache_delQ.ctrl;
    inports[3]  = sb_respQ.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = busQ.ctrl;
    outports[2] = dcacheQ.ctrl;
    outports[3] = sbQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_mem <- mkEventRecorder(`EVENTS_MEMORY_INSTRUCTION_MEM);

    function Action finishCycle(TOKEN tok, BUNDLE bundle);
    action
        let x <- inQ.receive();
        outQ.send(tagged Valid tuple2(tok, bundle));

        if (bundle.isLoad)
        begin
            busQ.send(Valid(genBusMessage(tok, bundle.dests, False)));
            debugLog.record(fshow(tok) + fshow(": marking load dest reg(s) valid"));
        end
        else
        begin
            busQ.send(Invalid);
        end

        event_mem.recordEvent(Valid(zeroExtend(pack(tok.index))));
        state <= MEM_STATE_REQ;
    endaction
    endfunction

    rule stall (state == MEM_STATE_REQ && !outQ.canSend);
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        debugLog.record(fshow("STALL PROPOGATED"));
        inQ.pass();
        outQ.pass();
        busQ.send(Invalid);
        sbQ.send(Invalid);
        dcacheQ.send(Invalid);
        state <= MEM_STATE_PASS2;
    endrule

    rule bubble (state == MEM_STATE_REQ && outQ.canSend && !isValid(inQ.peek));
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        debugLog.record(fshow("BUBBLE"));
        let x <- inQ.receive();
        outQ.send(Invalid);
        busQ.send(Invalid);
        sbQ.send(Invalid);
        dcacheQ.send(Invalid);
        state <= MEM_STATE_PASS2;
    endrule

    rule pass2 (state == MEM_STATE_PASS2);
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        let sbr <- sb_respQ.receive();
        event_mem.recordEvent(Invalid);
        state <= MEM_STATE_REQ;
    endrule

    rule storebuf_req (state == MEM_STATE_REQ &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();
        if (bundle.isLoad)
        begin
            if (stalled) begin
                sbQ.send(Invalid);
            end
            else begin
                debugLog.record(fshow("SB LOAD ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                sbQ.send(Valid(tuple2(tok, Data_read_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
            end
        end
        else if (bundle.isStore)
        begin
            debugLog.record(fshow("SB STORE ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
            sbQ.send(Valid(tuple2(tok, Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
        end
        else
        begin
            debugLog.record(fshow("NO-MEMORY ") + fshow(tok));
            sbQ.send(Invalid);
        end
        state <= MEM_STATE_DCACHE_REQ;
    endrule

    rule storebuf_reply (state == MEM_STATE_DCACHE_REQ &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let m <- sb_respQ.receive();
        case (m) matches
            tagged Invalid:
            begin
                dcacheQ.send(Invalid);
                state <= MEM_STATE_DCACHE_REPLY;
            end
            tagged Valid { .tok, .x }:
            case (x) matches
              SB_HIT:
                begin
                    debugLog.record(fshow("SB HIT ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                    dcacheQ.send(Invalid);
                    state <= MEM_STATE_DCACHE_REPLY;
                end
              SB_MISS:
                begin
                    debugLog.record(fshow("DCACHE LOAD ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                    dcacheQ.send(Valid(tuple2(tok, Data_read_mem_ref(tuple2(?/*passthru*/, bundle.effAddr)))));
                    state <= MEM_STATE_DCACHE_REPLY;
                end
              SB_STALL:
                begin
                    debugLog.record(fshow("SB STALL ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                    dcacheQ.send(Invalid);
                    state <= MEM_STATE_SB_STALL;
                end
            endcase
        endcase
    endrule

    rule dcache_reply (state == MEM_STATE_DCACHE_REPLY);
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        case (del) matches
            tagged Invalid:
                case (imm) matches
                    tagged Invalid:
                    begin
                        if (stalled)
                            state <= MEM_STATE_DCACHE_STALL;
                        else
                            state <= MEM_STATE_FUNCP1;
                    end
                    tagged Valid { .tok, .msg }:
                    begin
                        case (msg) matches
                            tagged Hit .*:
                                state <= MEM_STATE_FUNCP1;
                            tagged Miss_servicing .*:
                            begin
                                stalled <= True;
                                state <= MEM_STATE_DCACHE_STALL;
                            end
                            tagged Hit_servicing .*:
                                noAction;
                                //assertion failure
                            tagged Miss_retry .*:
                                state <= MEM_STATE_DCACHE_STALL;
                        endcase
                    end
                endcase
            tagged Valid { .tok, .msg }:
            begin
                //assert (stalled)
                //assert (imm == Invalid)
                case (msg) matches
                    tagged Miss_response .*:
                    begin
                        stalled <= False;
                        state <= MEM_STATE_FUNCP1;
                    end
                    tagged Hit_response .*:
                        noAction;
                        // assertion failure
                endcase
            end
        endcase
    endrule

    rule funcp1 (state == MEM_STATE_FUNCP1 &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        if (bundle.isLoad)
        begin
            debugLog.record(fshow("FUNCP-REQ LOAD DTRANS ") + fshow(tok));
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
            state <= MEM_STATE_FUNCP2;
        end
        else if (bundle.isStore)
        begin
            debugLog.record(fshow("FUNCP-REQ STORE DTRANS") + fshow(tok));
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
            state <= MEM_STATE_FUNCP2;
        end
        else
            finishCycle(tok, bundle);
    endrule

    rule funcp2 (state == MEM_STATE_FUNCP2 &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();
        // Wait for all translation responses.  Stay in this state if more are
        // coming.  Otherwise, do the operation.
        if (! rsp.hasMore)
        begin
            if (bundle.isLoad)
            begin
                debugLog.record(fshow("FUNCP-REQ LOAD ") + fshow(tok));
                doLoads.makeReq(initFuncpReqDoLoads(rsp.token));
            end
            else if (bundle.isStore)
            begin
                debugLog.record(fshow("FUNCP-REQ STORE ") + fshow(tok));
                doStores.makeReq(initFuncpReqDoStores(rsp.token));
            end

            state <= MEM_STATE_FUNCP3;
        end
    endrule

    rule funcp3 (state == MEM_STATE_FUNCP3 &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let out_tok = tok;

        if (bundle.isLoad)
        begin
            let rsp = doLoads.getResp();
            out_tok = rsp.token;
            doLoads.deq();
        end
        else if (bundle.isStore)
        begin
            let rsp = doStores.getResp();
            out_tok = rsp.token;
            doStores.deq();
        end

        finishCycle(out_tok, bundle);
    endrule

    rule sb_stall (state == MEM_STATE_SB_STALL &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        let imm <- dcache_immQ.receive();
        let del <- dcache_delQ.receive();
        inQ.pass();
        outQ.send(Invalid);
        busQ.send(Invalid);
        event_mem.recordEvent(Invalid);
        state <= MEM_STATE_REQ;
    endrule

    rule dcache_stall (state == MEM_STATE_DCACHE_STALL &&& inQ.peek() matches tagged Valid { .tok, .bundle });
        if (stalled)
            debugLog.record(fshow("DCACHE STALL ") + fshow(tok));
        else
            debugLog.record(fshow("DCACHE RETRY ") + fshow(tok));
        inQ.pass();
        outQ.send(Invalid);
        busQ.send(Invalid);
        event_mem.recordEvent(Invalid);
        state <= MEM_STATE_REQ;
    endrule

endmodule
