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

// ******* Library Imports *******

import Vector::*;
import DefaultValue::*;


// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"


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
`include "awb/provides/l1_cache_base_types.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_l1_dcache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"


// ****** Local Definitions *******

//
// States associated with entries.  Invalid isn't listed because it is provided
// as part of the cache algorithm.
//
typedef enum
{
    // Modified
    L1D_STATE_M,
    // Exclusive
    L1D_STATE_E,
    // Shared
    L1D_STATE_S,
    // Fill to shared in progress.  No data yet.
    L1D_STATE_FILL_S,
    // Fill to exclusive in progress.  No data yet.
    L1D_STATE_FILL_E,
    // Shared to exclusive transition in progress.
    L1D_STATE_S_E
}
L1D_ENTRY_STATE
    deriving (Eq, Bits);

instance FShow#(L1D_ENTRY_STATE);
    function Fmt fshow(L1D_ENTRY_STATE state);
        let str =
            case (state) matches
                L1D_STATE_M: return "M";
                L1D_STATE_E: return "E";
                L1D_STATE_S: return "S";
                L1D_STATE_FILL_S: return "FILL_S";
                L1D_STATE_FILL_E: return "FILL_E";
                L1D_STATE_S_E: return "S_E";
                default: return "UNDEFINED";
            endcase;

        return $format(str);
    endfunction
endinstance

instance FShow#(Tuple2#(L1D_ENTRY_STATE, L1D_ENTRY_STATE));
    function Fmt fshow(Tuple2#(L1D_ENTRY_STATE, L1D_ENTRY_STATE) states);
        match {.s_from, .s_to} = states;
        return fshow(s_from) + $format("->") + fshow(s_to);
    endfunction
endinstance


typedef union tagged
{
    L1_DCACHE_MISS_TOKEN MULTI_FILL;
    CACHE_PROTOCOL_MSG   CACHE_MSG;
    DCACHE_LOAD_INPUT    LOAD_REQ;
    DCACHE_STORE_INPUT   STORE_REQ;
    void Invalid;
}
L1D_OPER
    deriving (Eq, Bits);

// newL1DCOper --
//   The compiler sometimes fails to infer the type when initialized.  This
//   function makes types clear.
function L1D_OPER newL1DCOper(L1D_OPER oper) = oper;


// DC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Maybe#(L1_DCACHE_MISS_TOKEN) missTokToFree;

    Vector#(2, Bool) toL2QNotFull;
    Vector#(2, Bool) toL2QUsed;
    Vector#(2, CACHE_PROTOCOL_MSG) toL2QData;
    
    Bool cacheUpdUsed;
    Bool cacheUpdInval;
    L1_DCACHE_IDX cacheUpdIdx;
    L1D_ENTRY_STATE cacheUpdState;
    LINE_ADDRESS cacheUpdPAddr;

    Maybe#(DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImm;
    Maybe#(DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImm;
    
    Maybe#(DCACHE_LOAD_OUTPUT_DELAYED)  loadRsp;
    Maybe#(DCACHE_STORE_OUTPUT_DELAYED) storeRsp;
}
DC_LOCAL_STATE
    deriving (Eq, Bits);


instance DefaultValue#(DC_LOCAL_STATE);
    defaultValue = DC_LOCAL_STATE { 
        missTokToFree: tagged Invalid,
        toL2QNotFull: ?,
        toL2QUsed: replicate(False),
        toL2QData: ?,
        cacheUpdUsed: False,
        cacheUpdInval: False,
        cacheUpdIdx: ?,
        cacheUpdPAddr: ?,
        cacheUpdState: ?,
        loadRspImm: tagged Invalid,
        storeRspImm: tagged Invalid,
        loadRsp: tagged Invalid,
        storeRsp: tagged Invalid
        };
endinstance



// mkL1DCache
//
// A model of an L1 DCache that is unpipelined.  (The model itself is pipelined,
// but the target machine is not.)  Lookup and response takes a single model
// cycle.
//
module [HASIM_MODULE] mkL1DCache ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_data.out");

    // ****** Submodels ******


    //
    // The cache algorithm which determines hits, misses, and evictions.
    // The mayEvict function is used inside the algorithm when returning
    // a candidate for replacement.  mayEvict indicates when an entry
    // may not be victimized.
    //
    function Bool mayEvict(L1D_ENTRY_STATE state) = (pack(state) <= pack(L1D_STATE_S));

    L1_DCACHE_ALG#(MAX_NUM_CPUS, L1D_ENTRY_STATE) dCacheAlg <-
        mkL1DCacheAlg(mayEvict);

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, DCACHE_MISS_ID_SIZE) outstandingMisses <- mkCoalescingCacheMissTracker();

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_load", 0);

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_INPUT) storeReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_store", 0);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_immediate");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_delayed");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_immediate");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED) storeRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_delayed");

    // Queues to and from the memory hierarchy, encapsulated as StallPorts.
    Vector#(2, PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                            CACHE_PROTOCOL_MSG)) portToL2 = newVector();
    portToL2[0] <- mkPortStallSend_Multiplexed("L1_DCache_OutQ_0");
    portToL2[1] <- mkPortStallSend_Multiplexed("L1_DCache_OutQ_1");

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) portFromL2 <- mkPortStallRecv_Multiplexed("L1_DCache_InQ_0");


    // ****** Local Controller ******

    Vector#(5, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(7, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = storeReqFromCPU.ctrl;
    inports[2] = portFromL2.ctrl.in;
    inports[3] = portToL2[0].ctrl.in;
    inports[4] = portToL2[1].ctrl.in;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;
    outports[4] = portFromL2.ctrl.out;
    outports[5] = portToL2[0].ctrl.out;
    outports[6] = portToL2[1].ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L1 DCache Controller", inports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1D_OPER,
                                            DC_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1D_OPER,
                                            DC_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple3#(L1D_OPER,
                                            DC_LOCAL_STATE,
                                            Bool)) stage4Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1D_OPER,
                                            DC_LOCAL_STATE)) stage5Ctrl <-
        mkStageController();


    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statReadHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_READ_HIT",
                                           "L1 DCache Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_READ_MISS",
                                           "L1 DCache Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_READ_RETRY",
                                           "L1 DCache Read Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadBypass <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_READ_BYPASS",
                                           "L1 DCache Read Bypasses"));

    STAT_VECTOR#(MAX_NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_HIT",
                                           "L1 DCache Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_MISS",
                                           "L1 DCache Write Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_RETRY",
                                           "L1 DCache Write Retries"));

    // ****** Assertions ******

    Vector#(4, ASSERTION_STR_CLIENT) assertNode <- mkAssertionStrClientVec();

    let assertL2MsgOk <-
        mkAssertionStrChecker("l1-dcache-controller-MESI.bsv: Unexpected message from L2",
                              ASSERT_ERROR,
                              assertNode[0]);

    let assertValidExclusiveFlag <-
        mkAssertionStrChecker("l1-dcache-controller-MESI.bsv: Exclusive flag set for LOAD or clear for STORE",
                              ASSERT_ERROR,
                              assertNode[1]);

    let assertInFillState <-
        mkAssertionStrChecker("l1-dcache-controller-MESI.bsv: Entry not in proper FILL state",
                              ASSERT_ERROR,
                              assertNode[2]);

    let assertNewToken <-
        mkAssertionStrChecker("l1-dcache-controller-MESI.bsv: Entry unexpected merged FILL token",
                              ASSERT_ERROR,
                              assertNode[3]);


    // ****** Functions ******

    //
    // getOper --
    //   Pick out the L1D_OPER argument from a stage controller payload.
    //   Assumes the operation is always the first element in the payload
    //   tuple.
    //
    function L1D_OPER getOper(STAGE_CONTROLLER#(MAX_NUM_CPUS, t_ARGS) ctrl)
        provisos (Has_tpl_1#(t_ARGS, L1D_OPER));

        match {.cpu_iid, .payload} = ctrl.peekReadyInstance();
        return tpl_1(payload);
    endfunction


    // ****** Rules ******

    // stage1_pickOperation
    
    // Consider all requests and pick one to process.

    // Ports read:
    // * portFromL2
    // * storeReqFromCPU
    // * loadReqFromCPU
    
    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        DC_LOCAL_STATE local_state = defaultValue;

        // Check if the toL2Q has room for any new requests.
        local_state.toL2QNotFull[0] <- portToL2[0].canEnq(cpu_iid);
        local_state.toL2QNotFull[1] <- portToL2[1].canEnq(cpu_iid);
        
        // Consume incoming messages
        let m_fromL2 <- portFromL2.receive(cpu_iid);
        let m_cpu_req_store <- storeReqFromCPU.receive(cpu_iid);
        let m_cpu_req_load <- loadReqFromCPU.receive(cpu_iid);

        //
        // Pick an action for the model cycle.
        //

        L1D_OPER oper = tagged Invalid;

        // Are any previously returned fills going to multiple loads?
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin
            oper = tagged MULTI_FILL miss_tok;
            debugLog.record(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
        end
        else if (m_fromL2 matches tagged Valid .fromL2)
        begin
            oper = tagged CACHE_MSG fromL2;

            // Request/response from lower in the cache hierarchy.
            if (fromL2.kind matches tagged RSP_LOAD .fill_meta)
            begin
                // Fill response.
                L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fromL2.opaque);
                debugLog.record(cpu_iid, $format("1: ") + fshow(fromL2) + $format(", idx %0d", miss_tok.index));
            end
            else if (fromL2.kind matches tagged FORCE_INVAL .inval_meta)
            begin
                // Writeback or invalidation request.
                debugLog.record(cpu_iid, $format("1: ") + fshow(fromL2));
            end
            else
            begin
                // Unexpected message!
                debugLog.record(cpu_iid, $format("1: UNEXPECTED: ") + fshow(fromL2));
                assertL2MsgOk(False);
            end
        end
        else if (m_cpu_req_store matches tagged Valid .req)
        begin
            oper = tagged STORE_REQ req;

            let line_addr = toLineAddress(req.physicalAddress);
            debugLog.record(cpu_iid, $format("1: STORE REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));
        end
        else if (m_cpu_req_load matches tagged Valid .req)
        begin
            oper = tagged LOAD_REQ req;

            let line_addr = toLineAddress(req.physicalAddress);
            debugLog.record(cpu_iid, $format("1: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        // If there are load and store requests then some immediate response
        // is required.  Initialize a retry response by default.
        if (m_cpu_req_store matches tagged Valid .req)
        begin
            local_state.storeRspImm = tagged Valid initDCacheStoreRetry();
        end
        if (m_cpu_req_load matches tagged Valid .req)
        begin
            local_state.loadRspImm = tagged Valid initDCacheLoadRetry(req);
        end

        stage2Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    // ====================================================================
    //
    //  Stage 2:  Exactly one stage2 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage2_MULTI_FILL (getOper(stage2Ctrl) matches tagged MULTI_FILL .miss_tok);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        // A fill that came in previously is going to multiple miss tokens.
        local_state.missTokToFree = tagged Valid miss_tok;

        // Now send the fill to the right place.
        // Start by looking up if it was a load or store based on miss ID.
        if (missTokIsLoad(miss_tok))
        begin
            // The fill is a load response. Return it to the CPU.
            debugLog.record(cpu_iid, $format("2: FILL MULTIPLE RSP LOAD: %0d", miss_tok.index));
            local_state.loadRsp = tagged Valid initDCacheLoadMissRsp(miss_tok.index);
        end
        else
        begin
            // The fill is a completed store.
            debugLog.record(cpu_iid, $format("2: FILL MULTIPLE RSP STORE: %0d", miss_tok.index));
            local_state.storeRsp = tagged Valid initDCacheStoreDelayOk(miss_tok.index);
        end

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_FILL_RSP (getOper(stage2Ctrl) matches tagged CACHE_MSG .fill &&&
                          fill.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        // Record fill meta data that will be written to the cache.
        local_state.cacheUpdUsed = True;
        local_state.cacheUpdPAddr = fill.linePAddr;

        // Deallocate the Miss ID.
        L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

        // Free the token in the next stage, in case we had to retry.
        local_state.missTokToFree = tagged Valid miss_tok;

        // Now send the fill to the right place.
        if (missTokIsLoad(miss_tok))
        begin
            // The fill is a load response. Return it to the CPU.
            debugLog.record(cpu_iid, $format("2: MEM RSP LOAD: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
            local_state.loadRsp = tagged Valid initDCacheLoadMissRsp(miss_tok.index);
            assertValidExclusiveFlag(fill_meta.exclusive == False);
        end
        else
        begin
            // The fill is a store response. Tell the CPU the entry has
            // been updated.
            debugLog.record(cpu_iid, $format("2: MEM RSP STORE: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
            local_state.storeRsp = tagged Valid initDCacheStoreDelayOk(miss_tok.index);
//            assertValidExclusiveFlag(fill_meta.exclusive == True);
            let new_meta = fill_meta;
            new_meta.exclusive = True;
            oper = tagged CACHE_MSG cacheMsg_RspLoad(fill.linePAddr, fill.opaque, new_meta);
        end

        // Pick the victim
        dCacheAlg.lookupByAddrReq(cpu_iid,
                                  fill.linePAddr,
                                  False,
                                  missTokIsLoad(miss_tok));

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_INVAL_REQ (getOper(stage2Ctrl) matches tagged CACHE_MSG .inval &&&
                           inval.kind matches tagged FORCE_INVAL .inval_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        // Record fill meta data that will be written to the cache.
        local_state.cacheUpdUsed = True;
        local_state.cacheUpdPAddr = inval.linePAddr;

        debugLog.record(cpu_iid, $format("2: INVAL: line 0x%h", inval.linePAddr));

        // Look up the line
        dCacheAlg.lookupByAddrReq(cpu_iid, inval.linePAddr, False, True);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_LOAD_REQ (getOper(stage2Ctrl) matches tagged LOAD_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        let line_addr = toLineAddress(req.physicalAddress);

        debugLog.record(cpu_iid, $format("2: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

        // Look up the address in the cache.
        dCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, True);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_STORE_REQ (getOper(stage2Ctrl) matches tagged STORE_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        let line_addr = toLineAddress(req.physicalAddress);

        debugLog.record(cpu_iid, $format("2: STORE REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

        // Look up the address in the cache.
        dCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, False);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_Invalid (getOper(stage2Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("2: Bubble"));

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    // ====================================================================
    //
    //  Stage 3:  Exactly one stage3 rule must fire for proper pipelining.
    //
    // ====================================================================


    rule stage3_MULTI_FILL (getOper(stage3Ctrl) matches tagged MULTI_FILL .miss_tok);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        debugLog.record(cpu_iid, $format("3: Bubble MULTI_FILL"));

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_FILL_RSP (getOper(stage3Ctrl) matches tagged CACHE_MSG .fill &&&
                          fill.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        //
        // This protocol evicts at request.  The entry should already
        // be present in the FILL state.
        //
        let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

        local_state.cacheUpdIdx = entry.idx;

        L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
        if (missTokIsLoad(miss_tok))
        begin
            // Load was serviced this cycle.  Free the fill token.
            debugLog.record(cpu_iid, $format("3: FREE LOAD: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
            outstandingMisses.reportLoadDone(cpu_iid, fill.linePAddr);
        end

        // Expect a hit in the cache, since the cache is used to track misses.
        if (entry.state matches tagged Valid .state)
        begin
            Bool error = False;

            //
            // State transition as a result of the fill...
            //

            case (state.opaque)
                L1D_STATE_FILL_S:
                begin
                    local_state.cacheUpdState = L1D_STATE_S;
                end

                L1D_STATE_FILL_E:
                begin
                    if (fill_meta.exclusive)
                    begin
                        // Fill for exclusive access complete
                        local_state.cacheUpdState = L1D_STATE_E;
                    end
                    else
                    begin
                        // There must also be a fill for exclusive pending.
                        // The current response was for a load before a store.
                        // Mark the entry shared in transition to exclusive.
                        local_state.cacheUpdState = L1D_STATE_S_E;
                    end
                end

                L1D_STATE_S_E:
                begin
                    if (fill_meta.exclusive)
                    begin
                        local_state.cacheUpdState = L1D_STATE_E;
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: S_E FILL with not exlusive, probably a bug: line 0x%h, ", fill.linePAddr) + fshow(entry.idx));
                        error = True;
                    end
                end

                default:
                begin
                    debugLog.record(cpu_iid, $format("3: Unexpected FILL state: line 0x%h, ", fill.linePAddr) + fshow(entry.idx) + $format(", ") + fshow(state.opaque));
                    error = True;
                end
            endcase

            debugLog.record(cpu_iid, $format("3: FILL: line 0x%h, ", fill.linePAddr) + fshow(entry.idx) + $format(", ") + fshow(tuple2(state.opaque, local_state.cacheUpdState)));
            assertInFillState(! error);
        end
        else
        begin
            // Error:  line should be in the cache already in FILL state.
            assertInFillState(False);
            debugLog.record(cpu_iid, $format("3: ERROR: Filled line not present, line 0x%h", fill.linePAddr));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule




    rule stage3_INVAL_REQ (getOper(stage3Ctrl) matches tagged CACHE_MSG .inval &&&
                           inval.kind matches tagged FORCE_INVAL .inval_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

        local_state.cacheUpdIdx = entry.idx;

        CACHE_PROTOCOL_WB_INVAL rsp_meta = defaultValue;
        rsp_meta.toDir = inval_meta.fromDir;

        if (! local_state.toL2QNotFull[1])
        begin
            // Can't forward the writeback response because the port is busy
            local_state.cacheUpdUsed = False;
            oper = newL1DCOper(tagged Invalid);
            debugLog.record(cpu_iid, $format("3: INVAL RETRY LATER: line 0x%h", inval.linePAddr));
        end
        else if (entry.state matches tagged Valid .state)
        begin
            case (state.opaque)
                L1D_STATE_M:
                begin
                    local_state.cacheUpdInval = True;
                    rsp_meta.exclusive = True;
                    rsp_meta.dirty = True;
                end

                L1D_STATE_E:
                begin
                    local_state.cacheUpdInval = True;
                    rsp_meta.exclusive = True;
                end

                L1D_STATE_S:
                begin
                    local_state.cacheUpdInval = True;
                end

                L1D_STATE_S_E:
                begin
                    // Drop shared but still waiting for fill for write
                    local_state.cacheUpdState = L1D_STATE_FILL_E;
                end
            endcase

            local_state.toL2QUsed[1] = True;
            local_state.toL2QData[1] = cacheMsg_WBInval(inval.linePAddr,
                                                        ?,
                                                        rsp_meta);
            if (local_state.cacheUpdInval)
                debugLog.record(cpu_iid, $format("3: INVAL ENTRY: line 0x%h, ", inval.linePAddr) + fshow(entry.idx));
            else
                debugLog.record(cpu_iid, $format("3: INVAL ENTRY: line 0x%h, ", inval.linePAddr) + fshow(entry.idx) + $format(", ") + fshow(tuple2(state.opaque, local_state.cacheUpdState)));
        end
        else
        begin
            // The line is not present.
            local_state.toL2QUsed[1] = True;
            local_state.toL2QData[1] = cacheMsg_WBInval(inval.linePAddr,
                                                        ?,
                                                        rsp_meta);
            debugLog.record(cpu_iid, $format("3: INVAL NOT PRESENT: line 0x%h", inval.linePAddr));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    //
    // evictLineMsg --
    //   Generate an eviction message for a current line.  Used by multiple
    //   stage3 rules below.
    //
    function CACHE_PROTOCOL_MSG evictLineMsg(CACHE_ENTRY_STATE#(L1D_ENTRY_STATE) state);
        CACHE_PROTOCOL_WB_INVAL inval_meta = defaultValue;
        inval_meta.exclusive = (state.opaque == L1D_STATE_M ||
                                state.opaque == L1D_STATE_E);
        inval_meta.dirty = (state.opaque == L1D_STATE_M);

        return cacheMsg_WBInval(state.linePAddr, ?, inval_meta);
    endfunction


    rule stage3_LOAD_REQ (getOper(stage3Ctrl) matches tagged LOAD_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        //
        // Did the load hit in the cache?
        //
        let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

        // Is there space in the miss tracker for a new L2 request?
        let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);
        let line_addr = toLineAddress(req.physicalAddress);

        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = line_addr;

        if (entry.state matches tagged Valid .state)
        begin
            // Line is present in the cache.
            case (state.opaque)
                L1D_STATE_M,
                L1D_STATE_E,
                L1D_STATE_S,
                L1D_STATE_S_E:
                begin
                    // A hit, so give the data back.
                    local_state.loadRspImm = tagged Valid initDCacheLoadHit(req);
                    statReadHit.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD HIT: line 0x%h, state ", state.linePAddr) + fshow(state.opaque));
                end

                default:
                begin
                    // One of the fill states.  If the load can be merged then
                    // add it to an outstanding fill request.  Otherwise, retry
                    // later.
                    if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) &&
                        can_allocate)
                    begin
                        // A fill of this address is already in flight.  Latch
                        // on to it and don't generate a new request.
                        new_miss_tok_req = True;
                        outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                        statReadMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS (ALREADY OUTSTANDING): line 0x%h, state ", line_addr) + fshow(state.opaque));
                    end
                    else
                    begin
                        // Miss, but miss tracker is full or L2 is busy.
                        debugLog.record(cpu_iid, $format("3: LOAD BUSY RETRY: line 0x%h, can alloc %0d, state ", line_addr, can_allocate) + fshow (state.opaque));
                    end
                end
            endcase
        end
        else if (entry.state matches tagged Blocked)
        begin
            // No victim available.  Retry.
            debugLog.record(cpu_iid, $format("3: LOAD BLOCKED (no victim avail): line 0x%h", line_addr));
        end
        else if (! can_allocate)
        begin
            // No miss token available
            debugLog.record(cpu_iid, $format("3: NO MISS TOKEN: line 0x%h", line_addr));
        end
        else if (! all(id, local_state.toL2QNotFull))
        begin
            // Channel for requesting fill or sending writeback is full.
            debugLog.record(cpu_iid, $format("3: L2Q REQ FULL: line 0x%h", line_addr));
        end
        else
        begin
            // A miss and the necessary request channels are all available.

            // Miss token should be new since no miss is currently outstanding
            // for this address.  (Misses are recorded in the cache state tag.)
            assertNewToken(! outstandingMisses.loadOutstanding(cpu_iid, line_addr));

            // Generate the eviction message
            if (entry.state matches tagged MustEvict .state)
            begin
                let msg = evictLineMsg(state);
                local_state.toL2QData[1] = msg;
                local_state.toL2QUsed[1] = True;

                debugLog.record(cpu_iid, $format("3: EVICT INVAL: line 0x%h, ", state.linePAddr) + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(msg));
            end

            // Allocate the next miss ID
            new_miss_tok_req = True;
            outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

            // Record that we are using the memory queue.
            local_state.toL2QUsed[0] = True;

            // Update cache state
            local_state.cacheUpdUsed = True;
            local_state.cacheUpdState = L1D_STATE_FILL_S;

            statReadMiss.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("3: LOAD MISS: line 0x%h, ", line_addr) + fshow(local_state.cacheUpdIdx));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule
        

    rule stage3_STORE_REQ (getOper(stage3Ctrl) matches tagged STORE_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        //
        // Did the load hit in the cache?
        //
        let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

        // Is there space in the miss tracker for a new L2 request?
        let can_allocate = outstandingMisses.canAllocateStore(cpu_iid);
        let line_addr = toLineAddress(req.physicalAddress);

        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = line_addr;

        if (entry.state matches tagged Valid .state)
        begin
            // Line is present in the cache.
            case (state.opaque)
                L1D_STATE_M,
                L1D_STATE_E:
                begin
                    // A hit, so give the data back.
                    local_state.storeRspImm = tagged Valid initDCacheStoreOk();
                    statWriteHit.incr(cpu_iid);

                    if (state.opaque == L1D_STATE_E)
                    begin
                        // Transition to modified
                        local_state.cacheUpdUsed = True;
                        local_state.cacheUpdState = L1D_STATE_M;
                        debugLog.record(cpu_iid, $format("3: STORE HIT M->E: line 0x%h, state ", state.linePAddr) + fshow(state.opaque));
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: STORE HIT M: line 0x%h, state ", state.linePAddr) + fshow(state.opaque));
                    end
                end

                L1D_STATE_S,
                L1D_STATE_FILL_S:
                begin
                    // Present but shared or fill outstanding for shared.
                    // Request exclusive control and respond RETRY to CPU.
                    if (can_allocate)
                    begin
                        // Get a MissID to go to memory.
                        new_miss_tok_req = True;
                        outstandingMisses.allocateStoreReq(cpu_iid);

                        // Record that we're using the toL2Q.
                        local_state.toL2QUsed[0] = True;

                        // Update entry state to transition to exclusive.
                        local_state.cacheUpdUsed = True;
                        local_state.cacheUpdState =
                            (state.opaque == L1D_STATE_S) ? L1D_STATE_S_E :
                                                            L1D_STATE_FILL_E;

                        statWriteMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: STORE ") + fshow(tuple2(state.opaque, local_state.cacheUpdState)) + $format(": line 0x%h", state.linePAddr));
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: STORE NO TOKEN: line 0x%h, state ", line_addr) + fshow (state.opaque));
                    end
                end

                default:
                begin
                    debugLog.record(cpu_iid, $format("3: STORE BUSY RETRY: line 0x%h, can alloc %0d, state ", line_addr, can_allocate) + fshow (state.opaque));
                end
            endcase
        end
        else if (entry.state matches tagged Blocked)
        begin
            // No victim available.  Retry.
            debugLog.record(cpu_iid, $format("3: STORE BLOCKED (no victim avail): line 0x%h", line_addr));
        end
        else if (! can_allocate)
        begin
            // No miss token available
            debugLog.record(cpu_iid, $format("3: NO MISS TOKEN: line 0x%h", line_addr));
        end
        else if (! all(id, local_state.toL2QNotFull))
        begin
            // Channel for requesting fill or sending writeback is full.
            debugLog.record(cpu_iid, $format("3: L2Q REQ FULL: line 0x%h", line_addr));
        end
        else
        begin
            // A miss and the necessary request channels are all available.

            // Miss token should be new since no miss is currently outstanding
            // for this address.  (Misses are recorded in the cache state tag.)
            assertNewToken(! outstandingMisses.loadOutstanding(cpu_iid, line_addr));

            // Generate the eviction message
            if (entry.state matches tagged MustEvict .state)
            begin
                let msg = evictLineMsg(state);
                local_state.toL2QData[1] = msg;
                local_state.toL2QUsed[1] = True;

                debugLog.record(cpu_iid, $format("3: EVICT INVAL: line 0x%h, ", state.linePAddr) + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(msg));
            end

            // Allocate the next miss ID
            new_miss_tok_req = True;
            outstandingMisses.allocateStoreReq(cpu_iid);

            // Record that we are using the memory queue.
            local_state.toL2QUsed[0] = True;

            // Update cache state
            local_state.cacheUpdUsed = True;
            local_state.cacheUpdState = L1D_STATE_FILL_E;

            statWriteMiss.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("3: STORE MISS: line 0x%h, ", line_addr) + fshow(local_state.cacheUpdIdx));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_Invalid (getOper(stage3Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        debugLog.record(cpu_iid, $format("3: Bubble"));

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    // ====================================================================
    //
    //  Stage 4:  Not much left to do.  All operations trigger the same
    //            rule.
    //
    // ====================================================================

    rule stage4 (True);
        match {.cpu_iid, {.oper,
                          .local_state,
                          .new_miss_tok_req}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_FILL .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble MULTI_FILL"));
            end

            tagged CACHE_MSG .msg:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble CACHE_MSG"));
            end

            tagged LOAD_REQ .req:
            begin
                if (new_miss_tok_req)
                begin
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);
                    local_state.loadRspImm = tagged Valid initDCacheLoadMiss(req, miss_tok.index);

                    if (! local_state.toL2QUsed[0])
                    begin
                        debugLog.record(cpu_iid, $format("4: LOAD MISS (ALREADY OUTSTANDING): %0d", miss_tok.index));
                    end
                    else
                    begin
                        // Use the opaque bits to store the miss token.
                        let line_addr = toLineAddress(req.physicalAddress);
                        let l2_req = cacheMsg_ReqLoad(line_addr,
                                                      toMemOpaque(miss_tok),
                                                      defaultValue);
                        local_state.toL2QData[0] = l2_req;

                        debugLog.record(cpu_iid, $format("4: LOAD MISS: line 0x%h, tok %0d", line_addr, miss_tok.index));
                    end
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("4: Bubble LOAD_REQ"));
                end
            end

            tagged STORE_REQ .req:
            begin
                if (new_miss_tok_req)
                begin
                    let miss_tok <- outstandingMisses.allocateStoreRsp(cpu_iid);

                    // Use the opaque bits to store the miss token.
                    let line_addr = toLineAddress(req.physicalAddress);
                    CACHE_PROTOCOL_REQ_LOAD fill_req = defaultValue;
                    fill_req.exclusive = True;
                    let l2_req = cacheMsg_ReqLoad(line_addr,
                                                  toMemOpaque(miss_tok),
                                                  fill_req);
                    local_state.toL2QData[0] = l2_req;

                    // Tell the CPU to delay until the store returns.
                    local_state.storeRspImm = tagged Valid initDCacheStoreDelay(miss_tok.index);
                    debugLog.record(cpu_iid, $format("4: STORE MISS: line 0x%h, tok %0d", line_addr, miss_tok.index));
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("4: Bubble STORE_REQ"));
                end
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble"));
            end
        endcase


        // Send immediate responses.
        loadRspImmToCPU.send(cpu_iid, local_state.loadRspImm);
        storeRspImmToCPU.send(cpu_iid, local_state.storeRspImm);

        // Update retry statistics.
        if (local_state.loadRspImm matches tagged Valid .rsp &&&
            rsp.rspType matches tagged DCACHE_retry)
        begin
            statReadRetry.incr(cpu_iid);
        end

        if (local_state.storeRspImm matches tagged Valid .rsp &&&
            rsp matches tagged DCACHE_retryStore)
        begin
            statWriteRetry.incr(cpu_iid);
        end


        stage5Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    (* conservative_implicit_conditions *)
    rule stage5 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        loadRspDelToCPU.send(cpu_iid, local_state.loadRsp);
        storeRspDelToCPU.send(cpu_iid, local_state.storeRsp);

        if (oper matches tagged CACHE_MSG .msg)
        begin
            portFromL2.doDeq(cpu_iid);
        end
        else
        begin
            portFromL2.noDeq(cpu_iid);
        end

        // Take care of the memory queue.
        for (Integer i = 0; i < 2; i = i + 1)
        begin
            if (local_state.toL2QUsed[i])
            begin
                portToL2[i].doEnq(cpu_iid, local_state.toL2QData[i]);
            end
            else
            begin
                portToL2[i].noEnq(cpu_iid);
            end
        end

        // Take care of the cache update.
        if (local_state.cacheUpdInval)
        begin
            dCacheAlg.invalidate(cpu_iid, local_state.cacheUpdIdx);

            debugLog.record(cpu_iid, $format("5: INVAL: ") + fshow(local_state.cacheUpdIdx));
        end
        else if (local_state.cacheUpdUsed)
        begin
            Bool is_dirty = (local_state.cacheUpdState == L1D_STATE_M);
            let state = initCacheEntryState(local_state.cacheUpdPAddr,
                                            is_dirty,
                                            local_state.cacheUpdState);

            if ((local_state.cacheUpdState == L1D_STATE_FILL_S) ||
                (local_state.cacheUpdState == L1D_STATE_FILL_E))
            begin
                // Fill allocates a new line.
                dCacheAlg.allocate(cpu_iid,
                                   local_state.cacheUpdIdx,
                                   state,
                                   CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("5: ALLOC: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                dCacheAlg.update(cpu_iid,
                                 local_state.cacheUpdIdx,
                                 state);

                debugLog.record(cpu_iid, $format("5: UPDATE: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
        end
        
        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        debugLog.record(cpu_iid, $format("5: Done"));

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule
endmodule
