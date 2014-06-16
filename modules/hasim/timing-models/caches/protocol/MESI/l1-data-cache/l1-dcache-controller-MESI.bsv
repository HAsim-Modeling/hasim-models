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

typedef union tagged
{
    L1_DCACHE_MISS_TOKEN MULTI_LOAD;
    CACHE_PROTOCOL_MSG   FILL_RSP;
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

    Bool toL2QNotFull;
    Bool toL2QUsed;
    CACHE_PROTOCOL_MSG toL2QData;
    
    Bool writePortUsed;
    Bool writeDataDirty;
    L1_DCACHE_IDX writePortIdx;
    LINE_ADDRESS writePortData;

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
        toL2QNotFull: True,
        toL2QUsed: False,
        toL2QData: ?,
        writePortUsed: False,
        writeDataDirty: False,
        writePortIdx: ?,
        writePortData: ?,
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

    // The cache algorithm which determines hits, misses, and evictions.
    L1_DCACHE_ALG#(MAX_NUM_CPUS, void) dCacheAlg <-
        mkL1DCacheAlg(constFn(True));

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

    let assertRspOk <- mkAssertionSimOnly("l1-dcache-controller-unpipelined.bsv: Unexpected response kind",
                                          ASSERT_ERROR);


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
        let toL2Q_not_full <- portToL2[0].canEnq(cpu_iid);
        local_state.toL2QNotFull = toL2Q_not_full;
        
        let dummy <- portToL2[1].canEnq(cpu_iid);

        // Consume incoming messages
        let m_fill <- portFromL2.receive(cpu_iid);
        let m_cpu_req_store <- storeReqFromCPU.receive(cpu_iid);
        let m_cpu_req_load <- loadReqFromCPU.receive(cpu_iid);

        //
        // Pick an action for the model cycle.
        //

        L1D_OPER oper = tagged Invalid;

        // Are any previously returned fills going to multiple loads?
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin
            oper = tagged MULTI_LOAD miss_tok;
            debugLog.record(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
        end
        else if (m_fill matches tagged Valid .fill)
        begin
            assertRspOk(cacheMsg_IsRspLoad(fill));

            oper = tagged FILL_RSP fill;

            L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
            debugLog.record(cpu_iid, $format("1: FILL RSP: idx %0d, line 0x%h", miss_tok.index, fill.linePAddr));
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


    rule stage2 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
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
            end

            tagged FILL_RSP .fill:
            begin
                // Record fill meta data that will be written to the cache.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.linePAddr;
                local_state.writeDataDirty = False;

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
                end
                else
                begin
                    // The fill is a store response. Tell the CPU the entry has
                    // been updated.
                    debugLog.record(cpu_iid, $format("2: MEM RSP STORE: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
                    local_state.storeRsp = tagged Valid initDCacheStoreDelayOk(miss_tok.index);
                end

                // Pick the victim
                dCacheAlg.lookupByAddrReq(cpu_iid,
                                          fill.linePAddr,
                                          False,
                                          missTokIsLoad(miss_tok));
            end

            tagged LOAD_REQ .req:
            begin
                let line_addr = toLineAddress(req.physicalAddress);

                debugLog.record(cpu_iid, $format("2: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

                // Look up the address in the cache.
                dCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, True);
            end

            tagged STORE_REQ .req:
            begin
                let line_addr = toLineAddress(req.physicalAddress);

                debugLog.record(cpu_iid, $format("2: STORE REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

                // Look up the address in the cache.
                dCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, False);
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("2: Bubble"));
            end
        endcase

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage3 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        Bool new_miss_tok_req = False;

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble MULTI_LOAD"));
            end

            tagged FILL_RSP .fill:
            begin
                //
                // Does the cache entry being replaced require a writeback?
                //
                let evict <- dCacheAlg.lookupByAddrRsp(cpu_iid);

                local_state.writePortIdx = evict.idx;

                if (evict.state matches tagged Blocked)
                begin
                    // No available victim due to temporary state of the old
                    // line.  Wait for it to settle.
                    oper = newL1DCOper(tagged Invalid);
                    local_state.writePortUsed = False;
                    local_state.missTokToFree = tagged Invalid;

                    debugLog.record(cpu_iid, $format("3: BLOCKED EVICTION: new line 0x%h", fill.linePAddr));
                end
                else if (evict.state matches tagged MustEvict .state &&&
                         state.dirty)
                begin
                    // Victim is dirty.  Arrange for writeback.
                    if (local_state.toL2QNotFull)
                    begin
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION: new line 0x%h, old line 0x%h", fill.linePAddr, state.linePAddr));

                        // Record that we're using the toL2Q.
                        local_state.toL2QUsed = True;
                        CACHE_PROTOCOL_WB_INVAL wb = defaultValue;
                        wb.inval = True;
                        wb.isWriteback = True;
                        local_state.toL2QData = cacheMsg_WBInval(state.linePAddr, ?, wb);
                    end
                    else
                    begin
                        // Need to write back but L2 is busy.  Retry later.
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION WB BLOCKED: new line 0x%h, old line 0x%h", fill.linePAddr, state.linePAddr));

                        oper = newL1DCOper(tagged Invalid);
                        local_state.writePortUsed = False;
                        local_state.missTokToFree = tagged Invalid;
                    end
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("3: CLEAN EVICTION: line 0x%h", fill.linePAddr));
                end

                L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
                if (local_state.writePortUsed && missTokIsLoad(miss_tok))
                begin
                    // Load was serviced this cycle.  Free to the fill token.
                    debugLog.record(cpu_iid, $format("3: FREE LOAD: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
                    outstandingMisses.reportLoadDone(cpu_iid, fill.linePAddr);
                end
            end

            tagged LOAD_REQ .req:
            begin
                //
                // Did the load hit in the cache?
                //
                let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // A hit, so give the data back.
                    local_state.loadRspImm = tagged Valid initDCacheLoadHit(req);
                    statReadHit.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD HIT: line 0x%h", state.linePAddr));
                end
                else
                begin
                    // A miss.
                    let line_addr = toLineAddress(req.physicalAddress);

                    // Is there space in the miss tracker for a new L2 request?
                    let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);

                    if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) &&
                        can_allocate)
                    begin
                        // A fill of this address is already in flight.  Latch
                        // on to it and don't generate a new request.
                        new_miss_tok_req = True;
                        outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                        statReadMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS (ALREADY OUTSTANDING): line 0x%h", line_addr));
                    end
                    else if (can_allocate && local_state.toL2QNotFull)
                    begin
                        // Allocate the next miss ID
                        new_miss_tok_req = True;
                        outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                        // Record that we are using the memory queue.
                        local_state.toL2QUsed = True;

                        statReadMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS: line 0x%h", line_addr));
                    end
                    else
                    begin
                        // Miss, but miss tracker is full or L2 is busy.
                        local_state.loadRspImm = tagged Valid initDCacheLoadRetry(req);

                        debugLog.record(cpu_iid, $format("3: LOAD RETRY: line 0x%h, can alloc %0d, l2 notFull %0d", line_addr, can_allocate, local_state.toL2QNotFull));
                    end
                end
            end

            tagged STORE_REQ .req:
            begin
                //
                // Did the store hit in the cache?
                //
                let entry <- dCacheAlg.lookupByAddrRsp(cpu_iid);

                // Set up the write to the cache in case it is possible.  The
                // write will be enabled by setting local_state.writePortUsed
                // later, if necessary.
                let line_addr = toLineAddress(req.physicalAddress);
                local_state.writePortIdx = entry.idx;
                local_state.writePortData = line_addr;

                if (entry.state matches tagged Valid .state)
                begin
                    // Hit!
                    local_state.storeRspImm = tagged Valid initDCacheStoreOk();
                    statWriteHit.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: STORE HIT: line 0x%h", line_addr));

                    // We're writeback.  Update the line in the cache.
                    local_state.writePortUsed = True;
                    local_state.writeDataDirty = True;

                    // If there was also a load request this cycle and the load
                    // is the same address as the store then bypass the value
                    // and service the load this cycle.  The load request is stored
                    // as a retry request, since it lost to the store earlier
                    // in this pipeline.
                    if (local_state.loadRspImm matches tagged Valid .rsp &&&
                       rsp.rspType matches tagged DCACHE_retry &&&
                       rsp.bundle.physicalAddress == req.physicalAddress)
                    begin
                        local_state.loadRspImm = tagged Valid initDCacheLoadHit(rsp.bundle);

                        debugLog.record(cpu_iid, $format("3: LOAD BYPASS: PA 0x%h", rsp.bundle.physicalAddress));
                        statReadBypass.incr(cpu_iid);
                    end
                end
                else
                begin
                    // A miss.  We need to bring the line into the cache to deal
                    // with any coherence issues.
                    let can_allocate = outstandingMisses.canAllocateStore(cpu_iid);

                    if (can_allocate && local_state.toL2QNotFull)
                    begin
                        // Get a MissID to go to memory.
                        new_miss_tok_req = True;
                        outstandingMisses.allocateStoreReq(cpu_iid);

                        // Record that we're using the toL2Q.
                        local_state.toL2QUsed = True;

                        // Tell the CPU to delay until the store returns.
                        statWriteMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: STORE MISS: line 0x%h", line_addr));
                    end
                    else
                    begin
                        // Can't allocate a miss tracker or L2 is busy.
                        // The CPU will have to retry this store.
                        local_state.storeRspImm = tagged Valid initDCacheStoreRetry();
                        debugLog.record(cpu_iid, $format("3: STORE RETRY: line 0x%h, can alloc %0d, l2 notFull %0d", line_addr, can_allocate, local_state.toL2QNotFull));
                    end
                end
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble"));
            end
        endcase

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage4 (True);
        match {.cpu_iid, {.oper,
                          .local_state,
                          .new_miss_tok_req}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble MULTI_LOAD"));
            end

            tagged FILL_RSP .fill:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble FILL"));
            end

            tagged LOAD_REQ .req:
            begin
                if (new_miss_tok_req)
                begin
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);
                    local_state.loadRspImm = tagged Valid initDCacheLoadMiss(req, miss_tok.index);

                    if (! local_state.toL2QUsed)
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
                        local_state.toL2QData = l2_req;

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
                    local_state.toL2QData = l2_req;

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

        debugLog.record(cpu_iid, $format("5: Done"));

        loadRspDelToCPU.send(cpu_iid, local_state.loadRsp);
        storeRspDelToCPU.send(cpu_iid, local_state.storeRsp);

        if (oper matches tagged FILL_RSP .rsp)
        begin
            portFromL2.doDeq(cpu_iid);
        end
        else
        begin
            portFromL2.noDeq(cpu_iid);
        end

        // Take care of the memory queue.
        if (local_state.toL2QUsed)
        begin
            portToL2[0].doEnq(cpu_iid, local_state.toL2QData);
        end
        else
        begin
            portToL2[0].noEnq(cpu_iid);
        end
        
        portToL2[1].noEnq(cpu_iid);

        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
            let state = initCacheEntryState(local_state.writePortData,
                                            local_state.writeDataDirty,
                                            ?);
            if (oper matches tagged FILL_RSP .rsp)
            begin
                // Fill allocates a new line.
                dCacheAlg.allocate(cpu_iid,
                                   local_state.writePortIdx,
                                   state,
                                   CACHE_ALLOC_FILL);
            end
            else
            begin
                // Store is the only other path.  It updates the state of an
                // existing entry.
                dCacheAlg.update(cpu_iid,
                                 local_state.writePortIdx,
                                 state);
            end
        end
        
        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule
endmodule
