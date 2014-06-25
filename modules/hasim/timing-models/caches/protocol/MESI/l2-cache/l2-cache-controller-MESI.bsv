//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ******* Library Imports *******

import Vector::*;
import FIFO::*;
import DefaultValue::*;
import FShow::*;


// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/fpga_components.bsh"


// ******* HAsim Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/hasim_modellib.bsh"

`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/funcp_memstate_base_types.bsh"
`include "awb/provides/funcp_interface.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_l2_cache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"

// ******* Generated File Imports *******

`include "awb/dict/EVENTS_L2.bsh"


// ****** Local Definitions *******

//
// States associated with entries.  Invalid isn't listed because it is provided
// as part of the cache algorithm.
//
typedef enum
{
    // Modified
    L2C_STATE_M,
    // Exclusive
    L2C_STATE_E,
    // Shared
    L2C_STATE_S
}
L2C_ENTRY_STATE
    deriving (Eq, Bits);

instance FShow#(L2C_ENTRY_STATE);
    function Fmt fshow(L2C_ENTRY_STATE state);
        let str =
            case (state) matches
                L2C_STATE_M: return "M";
                L2C_STATE_E: return "E";
                L2C_STATE_S: return "S";
                default: return "UNDEFINED";
            endcase;

        return $format(str);
    endfunction
endinstance

instance FShow#(Tuple2#(L2C_ENTRY_STATE, L2C_ENTRY_STATE));
    function Fmt fshow(Tuple2#(L2C_ENTRY_STATE, L2C_ENTRY_STATE) states);
        match {.s_from, .s_to} = states;
        return fshow(s_from) + $format("->") + fshow(s_to);
    endfunction
endinstance



typedef `L2_MISS_ID_SIZE L2_MISS_ID_SIZE;
typedef CACHE_MISS_INDEX#(L2_MISS_ID_SIZE) L2_MISS_ID;
typedef CACHE_MISS_TOKEN#(L2_MISS_ID_SIZE) L2_MISS_TOKEN;
typedef TExp#(L2_MISS_ID_SIZE) NUM_L2_MISS_IDS;


typedef Maybe#(CACHE_PROTOCOL_MSG) L2C_OPER;
function L2C_OPER l2cInval() = tagged Invalid;

//
// L2C_LOCAL_STATE --
//   L2 State to pass between pipeline stages.
//
typedef struct
{
    Bool fromLLCDeq;
    Vector#(2, Bool) fromL1Deq;

    Vector#(2, Bool) toLLCQNotFull;
    Vector#(2, Bool) toLLCQUsed;
    Vector#(2, CACHE_PROTOCOL_MSG) toLLCQData;
    
    Bool cacheUpdUsed;
    Bool cacheUpdInval;
    Bool cacheUpdAlloc;
    L2_CACHE_IDX cacheUpdIdx;
    L2C_ENTRY_STATE cacheUpdState;
    LINE_ADDRESS cacheUpdPAddr;
    
    Bool toL1QNotFull;
    Maybe#(CACHE_PROTOCOL_MSG) toL1Q;
}
L2C_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(L2C_LOCAL_STATE);
    defaultValue = L2C_LOCAL_STATE { 
        fromLLCDeq: False,
        fromL1Deq: replicate(False),
        toLLCQNotFull: replicate(False),
        toLLCQUsed: replicate(False),
        toLLCQData: ?,
        cacheUpdUsed: False,
        cacheUpdInval: False,
        cacheUpdAlloc: False,
        cacheUpdIdx: ?,
        cacheUpdState: ?,
        cacheUpdPAddr: ?,
        toL1QNotFull: False,
        toL1Q: tagged Invalid
        };
endinstance


//
// mkL2Cache --
//
// A model of an L2 Cache that is unpipelined.
//
// Note that the module itself is implemented as a pipeline, though the target
// model carries out all actions in one model cycle.
//
module [HASIM_MODULE] mkL2Cache#(String portFromL1Name,
                                 String portToL1Name,
                                 String reqToLLCName,
                                 String rspFromLLCName)
    // Interface:
    ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l2_data.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    L2_CACHE_ALG#(MAX_NUM_CPUS, L2C_ENTRY_STATE) l2Alg <-
        mkL2CacheAlg(constFn(True));


    // ****** Ports ******

    // Queues to/from core hierarchy.
    Vector#(2, PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                            CACHE_PROTOCOL_MSG)) portFromL1 = newVector();
    portFromL1[0] <- mkPortStallRecv_Multiplexed(portFromL1Name + "_0");
    portFromL1[1] <- mkPortStallRecv_Multiplexed(portFromL1Name + "_1");

    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) portToL1 <-
        mkPortStallSend_Multiplexed(portToL1Name + "_0");
    
    // Queues to/from coherence engine.
    Vector#(2, PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                            CACHE_PROTOCOL_MSG)) reqToLLC = newVector();
    reqToLLC[0] <- mkPortStallSend_Multiplexed(reqToLLCName + "_0");
    reqToLLC[1] <- mkPortStallSend_Multiplexed(reqToLLCName + "_1");
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspFromLLC <-
        mkPortStallRecv_Multiplexed(rspFromLLCName);
    
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = portFromL1[0].ctrl.in;
    inctrls[1]  = portFromL1[1].ctrl.in;
    inctrls[2]  = portToL1.ctrl.in;
    inctrls[3]  = reqToLLC[0].ctrl.in;
    inctrls[4]  = reqToLLC[1].ctrl.in;
    inctrls[5]  = rspFromLLC.ctrl.in;

    outctrls[0] = portFromL1[0].ctrl.out;
    outctrls[1] = portFromL1[1].ctrl.out;
    outctrls[2] = portToL1.ctrl.out;
    outctrls[3] = reqToLLC[0].ctrl.out;
    outctrls[4] = reqToLLC[1].ctrl.out;
    outctrls[5] = rspFromLLC.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L2 Cache", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage4Ctrl <-
        mkStageController();


    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statReadHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_READ_HIT",
                                           "L2 Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_READ_MISS",
                                           "L2 Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_READ_RETRY",
                                           "L2 Read Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_WRITE_HIT",
                                           "L2 Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_WRITE_MISS",
                                           "L2 Write Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_WRITE_RETRY",
                                           "L2 Write Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statFillRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_FILL_RETRY",
                                           "L2 Fill Retries"));

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_L2_HIT);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_L2_MISS);


    // ****** Assertions ******

    Vector#(1, ASSERTION_STR_CLIENT) assertNode <- mkAssertionStrClientVec();

    let assertStateOk <-
        mkAssertionStrChecker("l2-cache-controller-unpipelined.bsv: Invalid cache state",
                              ASSERT_ERROR,
                              assertNode[0]);

    let assertNotBlocked <- mkAssertionSimOnly("l2-cache-controller-unpipelined.bsv: Unexpected blocked line",
                                               ASSERT_ERROR);

    // ****** Functions ******

    //
    // getOper --
    //   Pick out the L2C_OPER argument from a stage controller payload.
    //   Assumes the operation is always the first element in the payload
    //   tuple.
    //
    function L2C_OPER getOper(STAGE_CONTROLLER#(MAX_NUM_CPUS, t_ARGS) ctrl)
        provisos (Has_tpl_1#(t_ARGS, L2C_OPER));

        match {.cpu_iid, .payload} = ctrl.peekReadyInstance();
        return tpl_1(payload);
    endfunction


    // ****** Rules ******

    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = defaultValue;

        // Check whether the request ports have room for any new requests.
        local_state.toL1QNotFull <- portToL1.canEnq(cpu_iid);
        local_state.toLLCQNotFull[0] <- reqToLLC[0].canEnq(cpu_iid);
        local_state.toLLCQNotFull[1] <- reqToLLC[1].canEnq(cpu_iid);
        
        // Collect incoming messages
        let m_from_llc <- rspFromLLC.receive(cpu_iid);
        Vector#(2, Maybe#(CACHE_PROTOCOL_MSG)) m_from_l1 = newVector();
        m_from_l1[0] <- portFromL1[0].receive(cpu_iid);
        m_from_l1[1] <- portFromL1[1].receive(cpu_iid);


        L2C_OPER m_oper = l2cInval();

        //
        // Consider all incoming messages and process them in priority order.
        // A request is processed only if all possible needed output ports
        // are available.
        //

        if (m_from_l1[1] matches tagged Valid .wb_inval)
        begin
            // The writeback channel has highest priority.
            if (local_state.toLLCQNotFull[1])
            begin
                local_state.fromL1Deq[1] = True;
                m_oper = m_from_l1[1];

                debugLog.record(cpu_iid, $format("1: FROM L1[1]: ") + fshow(wb_inval));
            end
            else
            begin
                debugLog.record(cpu_iid, $format("1: RETRY FROM L1[1]: ") + fshow(wb_inval));
            end
        end
        else if (m_from_llc matches tagged Valid .msg)
        begin
            // Responses and coherence traffic from the LLC are next.
            if (local_state.toL1QNotFull)
            begin
                local_state.fromLLCDeq = True;
                m_oper = m_from_llc;

                debugLog.record(cpu_iid, $format("1: FROM LLC: ") + fshow(msg));
            end
            else
            begin
                debugLog.record(cpu_iid, $format("1: RETRY FROM LLC: ") + fshow(msg));
            end
        end
        else if (m_from_l1[0] matches tagged Valid .req)
        begin
            // New requests have the lowest priority.
            if (local_state.toL1QNotFull && local_state.toLLCQNotFull[0])
            begin
                local_state.fromL1Deq[0] = True;
                m_oper = m_from_l1[0];

                debugLog.record(cpu_iid, $format("1: FROM L1[0]: ") + fshow(req));
            end
            else
            begin
                debugLog.record(cpu_iid, $format("1: RETRY FROM L1[0]: ") + fshow(req));
            end
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        stage2Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    rule stage2 (True);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        if (m_oper matches tagged Valid .oper)
        begin
            Bool upd_replacement = False;
            Bool is_load = False;

            if (oper.kind matches tagged REQ_LOAD .fill_meta)
            begin
                upd_replacement = True;
                is_load = ! fill_meta.exclusive;
            end

            // Look up the line
            l2Alg.lookupByAddrReq(cpu_iid, oper.linePAddr,
                                  upd_replacement, is_load);

            debugLog.record(cpu_iid, $format("2: LOOKUP: upd %0d, isLoad %0d, ", upd_replacement, is_load) + fshow(oper));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("2: Bubble"));
        end

        stage3Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    // ====================================================================
    //
    //  Stage 3:  Exactly one stage3 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage3_REQ_LOAD (getOper(stage3Ctrl) matches tagged Valid .oper &&&
                          oper.kind matches tagged REQ_LOAD .load_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let entry <- l2Alg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = oper.linePAddr;

        // Hit/miss events will be recorded in this rule.
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;

        if (entry.state matches tagged Valid .state &&&
            (state.opaque != L2C_STATE_S) || ! load_meta.exclusive)
        begin
            // Line is in the cache and either in M/E state or
            // the request doesn't need exclusive access.
            CACHE_PROTOCOL_RSP_LOAD rsp_meta = defaultValue;
            rsp_meta.exclusive = load_meta.exclusive;

            local_state.toL1Q = tagged Valid cacheMsg_RspLoad(oper.linePAddr,
                                                              oper.opaque,
                                                              rsp_meta);

            statReadHit.incr(cpu_iid);
            evt_hit = tagged Valid resize({ oper.linePAddr, 1'b0 });

            debugLog.record(cpu_iid, $format("3: REQ_LOAD HIT: ") + fshow(oper) + $format(", ") + fshow(entry.idx));
        end
        else
        begin
            local_state.toLLCQUsed[0] = True;
            local_state.toLLCQData[0] = oper;

            statReadMiss.incr(cpu_iid);
            evt_miss = tagged Valid resize({ oper.linePAddr, 1'b0 });

            debugLog.record(cpu_iid, $format("3: REQ_LOAD MISS: ") + fshow(oper));
        end

        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage3_RSP_LOAD (getOper(stage3Ctrl) matches tagged Valid .oper &&&
                          oper.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let entry <- l2Alg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = oper.linePAddr;

        //
        // In general we just leave the L2 unchanged and let the
        // response flow to the L1.  The L2 will be a victim cache
        // for L1 capacity evictions.  The only exception is when
        // the line is already present in S state and the response
        // elevates it to E.
        //
        if (entry.state matches tagged Valid .state &&&
            state.opaque == L2C_STATE_S &&&
            fill_meta.exclusive)
        begin
            // Elevate to exclusive.
            local_state.cacheUpdUsed = True;
            local_state.cacheUpdState = L2C_STATE_E;
            debugLog.record(cpu_iid, $format("3: RSP_LOAD: ") + fshow(entry.idx) + $format(", S->E"));
        end

        // Forward response to L1
        local_state.toL1Q = m_oper;

        debugLog.record(cpu_iid, $format("3: FWD TO L1: ") + fshow(oper));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    //
    // Writeback/inval from L1 due to capacity.  The message will end here
    // and not be forwarded to the LLC.  Treat the L2 as a victim cache
    // and allocate the line in the L2.
    //
    rule stage3_L1_VICTIM (getOper(stage3Ctrl) matches tagged Valid .oper &&&
                           oper.kind matches tagged WB_INVAL .wb_meta &&&
                           ! wb_meta.toDir);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let entry <- l2Alg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = oper.linePAddr;

        if (entry.state matches tagged Valid .state)
        begin
            // Line already present in the L2.  Update state.
            case (state.opaque)
                L2C_STATE_M:
                begin
                    // Nothing to do.
                    noAction;
                end

                L2C_STATE_E:
                begin
                    if (wb_meta.dirty)
                    begin
                        local_state.cacheUpdUsed = True;
                        local_state.cacheUpdState = L2C_STATE_M;
                    end
                end

                L2C_STATE_S:
                begin
                    // Nothing to do.
                    if (wb_meta.dirty || wb_meta.exclusive)
                    begin
                        assertStateOk(False);
                        debugLog.record(cpu_iid, $format("3: WB INVAL ERROR: In S but exclusive, ") + fshow(oper));
                    end
                end
            endcase

            if (local_state.cacheUpdUsed)
                debugLog.record(cpu_iid, $format("3: WB INVAL ALREADY PRESENT: ") + fshow(entry.idx) + $format(", ") + fshow(tuple2(state.opaque, local_state.cacheUpdState)));
            else
                debugLog.record(cpu_iid, $format("3: WB INVAL ALREADY PRESENT: ") + fshow(entry.idx));
        end
        else if (entry.state matches tagged Blocked)
        begin
            // This should never happen, since this cache has no transition
            // states.
            assertNotBlocked(False);
        end
        else
        begin
            // Line not in the cache.  Add it.
            local_state.cacheUpdUsed = True;
            local_state.cacheUpdAlloc = True;

            if (wb_meta.dirty)
                local_state.cacheUpdState = L2C_STATE_M;
            else if (wb_meta.exclusive)
                local_state.cacheUpdState = L2C_STATE_E;
            else
                local_state.cacheUpdState = L2C_STATE_S;

            debugLog.record(cpu_iid, $format("3: INSERT L1 VICTIM: ") + fshow(entry.idx) + $format(", ") + fshow(local_state.cacheUpdState));

            // Is there currently a line in the cache?  Write back to the
            // LLC if it is dirty.
            if (entry.state matches tagged MustEvict .state &&&
                state.opaque == L2C_STATE_M)
            begin
                local_state.toLLCQUsed[1] = True;
                CACHE_PROTOCOL_WB_INVAL m = defaultValue;
                m.dirty = True;
                m.exclusive = True;
                local_state.toLLCQData[1] = cacheMsg_WBInval(state.linePAddr,
                                                             ?,
                                                             m);

                debugLog.record(cpu_iid, $format("3: L2 VICTIM: ") + fshow(entry.idx) + $format(", ") + fshow(local_state.toLLCQData[1]));
            end
        end

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    //
    // Coherence invalidation, triggered by the directory controller.
    // Evict the line, if present, and pass the result down to the LLC.
    //
    rule stage3_WB_INVAL (getOper(stage3Ctrl) matches tagged Valid .oper &&&
                          oper.kind matches tagged WB_INVAL .wb_meta &&&
                          wb_meta.toDir);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let entry <- l2Alg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = oper.linePAddr;

        let new_wb_meta = wb_meta;
        if (entry.state matches tagged Valid .state)
        begin
            // Line is currently in the cache.  Drop it.
            local_state.cacheUpdInval = True;

            // Merge dirty/exclusive states from L1 and L2
            new_wb_meta.dirty = new_wb_meta.dirty ||
                                (state.opaque == L2C_STATE_M);
            new_wb_meta.exclusive = new_wb_meta.exclusive ||
                                    (state.opaque == L2C_STATE_M) ||
                                    (state.opaque == L2C_STATE_E);

            debugLog.record(cpu_iid, $format("3: INVAL: ") + fshow(entry.idx) + $format(", ") + fshow(state.opaque) + $format("->I"));

            if (state.opaque == L2C_STATE_S &&
                (new_wb_meta.dirty || new_wb_meta.exclusive))
            begin
                assertStateOk(False);
                debugLog.record(cpu_iid, $format("3: WB INVAL ERROR: In S but exclusive, ") + fshow(oper));
            end
        end

        // Forward the updated message to the LLC.
        local_state.toLLCQUsed[1] = True;
        local_state.toLLCQData[1] = cacheMsg_WBInval(oper.linePAddr,
                                                     oper.opaque,
                                                     new_wb_meta);

        debugLog.record(cpu_iid, $format("3: WB TODIR: ") + fshow(local_state.toLLCQData[1]));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    rule stage3_FORCE_INVAL (getOper(stage3Ctrl) matches tagged Valid .oper &&&
                             oper.kind matches tagged FORCE_INVAL .meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let entry <- l2Alg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = oper.linePAddr;

        // Forward to L1.  The L2 will see it again as a response
        // from L1 to LLC.
        local_state.toL1Q = m_oper;

        debugLog.record(cpu_iid, $format("3: FWD TO L1: ") + fshow(oper));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage3_Invalid (getOper(stage3Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("3: Bubble"));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule



    // ====================================================================
    //
    //   Flow merges back to a single rule for the final message routing.
    //
    // ====================================================================

    rule stage4_end (True);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("4: Done"));

        //
        // I/O queue updates...
        //

        // Incoming queue from LLC
        if (local_state.fromLLCDeq)
        begin
            rspFromLLC.doDeq(cpu_iid);
        end
        else
        begin
            rspFromLLC.noDeq(cpu_iid);
        end

        // Outgoing queue to L1
        if (local_state.toL1Q matches tagged Valid .msg)
        begin
            portToL1.doEnq(cpu_iid, msg); 
            debugLog.record(cpu_iid, $format("4: TO L1: ") + fshow(msg));
        end
        else
        begin
            portToL1.noEnq(cpu_iid);
        end

        for (Integer i = 0; i < 2; i = i + 1)
        begin
            // Incoming queues from L1
            if (local_state.fromL1Deq[i])
            begin
                portFromL1[i].doDeq(cpu_iid);
            end
            else
            begin
                portFromL1[i].noDeq(cpu_iid);
            end

            // Outgoing queues toward LLC
            if (local_state.toLLCQUsed[i])
            begin
                reqToLLC[i].doEnq(cpu_iid, local_state.toLLCQData[i]);
                debugLog.record(cpu_iid, $format("4: TO LLC[%0d]: ", i) + fshow(local_state.toLLCQData[i]));
            end
            else
            begin
                reqToLLC[i].noEnq(cpu_iid);
            end
        end
        
        // Take care of the cache update.
        if (local_state.cacheUpdInval)
        begin
            l2Alg.invalidate(cpu_iid, local_state.cacheUpdIdx);

            debugLog.record(cpu_iid, $format("4: INVAL: ") + fshow(local_state.cacheUpdIdx));
        end
        else if (local_state.cacheUpdUsed)
        begin
            CACHE_ENTRY_STATE#(L2C_ENTRY_STATE) state =
                initCacheEntryState(local_state.cacheUpdPAddr,
                                    local_state.cacheUpdState == L2C_STATE_M,
                                    local_state.cacheUpdState);

            if (local_state.cacheUpdAlloc)
            begin
                // Allocating a new entry
                l2Alg.allocate(cpu_iid,
                               local_state.cacheUpdIdx,
                               state,
                               CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("4: ALLOC: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                // Updating an existing entry
                l2Alg.update(cpu_iid,
                             local_state.cacheUpdIdx,
                             state);

                debugLog.record(cpu_iid, $format("4: UPDATE: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
        end


        // End of model cycle.
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
