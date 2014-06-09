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


typedef `L2_MISS_ID_SIZE L2_MISS_ID_SIZE;
typedef CACHE_MISS_INDEX#(L2_MISS_ID_SIZE) L2_MISS_ID;
typedef CACHE_MISS_TOKEN#(L2_MISS_ID_SIZE) L2_MISS_TOKEN;
typedef TExp#(L2_MISS_ID_SIZE) NUM_L2_MISS_IDS;


typedef union tagged
{
    CACHE_PROTOCOL_MSG FILL_RSP;
    CACHE_PROTOCOL_MSG LOAD_REQ;
    CACHE_PROTOCOL_MSG STORE_REQ;
    void Invalid;
}
L2C_OPER
    deriving (Eq, Bits);

// newL2COper --
//   The compiler sometimes fails to infer the type when initialized.  This
//   function makes types clear.
function L2C_OPER newL2COper(L2C_OPER oper) = oper;


//
// L2C_LOCAL_STATE --
//   L2 State to pass between pipeline stages.
//
typedef struct
{
    Maybe#(L2_MISS_TOKEN) missTokToFree;

    Bool memQNotFull;
    Bool memQUsed;
    CACHE_PROTOCOL_MSG memQData;
    
    Bool writePortUsed;
    L2_CACHE_IDX writePortIdx;
    Bool writeDataDirty;
    LINE_ADDRESS writePortData;
    
    Bool coreQNotFull;
    Bool coreQUsed;
    CACHE_PROTOCOL_MSG coreQData;
}
L2C_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(L2C_LOCAL_STATE);
    defaultValue = L2C_LOCAL_STATE { 
        missTokToFree: tagged Invalid,
        memQNotFull: True,
        memQUsed: False,
        memQData: ?,
        coreQNotFull: True,
        coreQUsed: False,
        coreQData: ?,
        writePortUsed: False,
        writePortIdx: ?,
        writeDataDirty: False,
        writePortData: ?
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
module [HASIM_MODULE] mkL2Cache#(String reqFromL1Name,
                                 String rspToL1Name,
                                 String reqToMemoryName,
                                 String rspFromMemoryName)
    // Interface:
    ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l2_data.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    L2_CACHE_ALG#(MAX_NUM_CPUS, void) l2Alg <-
        mkL2CacheAlg(constFn(True));

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, L2_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();

    // A RAM To map our miss IDs into the original opaques, that we return to higher levels.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS, L2_MISS_ID, L2_MISS_TOKEN) opaquesPool <-
        mkMemory_Multiplexed(mkBRAM);

    // ****** Ports ******

    // Queues to/from core hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqFromCore <-
        mkPortStallRecv_Multiplexed(reqFromL1Name);
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToCore <-
        mkPortStallSend_Multiplexed(rspToL1Name);
    
    // Queues to/from coherence engine.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToUncore <-
        mkPortStallSend_Multiplexed(reqToMemoryName);
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspFromUncore <-
        mkPortStallRecv_Multiplexed(rspFromMemoryName);
    
    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    inctrls[2]  = reqToUncore.ctrl.in;
    inctrls[3]  = rspFromUncore.ctrl.in;

    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqToUncore.ctrl.out;
    outctrls[3] = rspFromUncore.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L2 Cache", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage4Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2C_OPER,
                                            L2C_LOCAL_STATE)) stage5Ctrl <-
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

    let assertReqOk <- mkAssertionSimOnly("l2-cache-controller-unpipelined.bsv: Unexpected request kind",
                                          ASSERT_ERROR);
    let assertRspOk <- mkAssertionSimOnly("l2-cache-controller-unpipelined.bsv: Unexpected response kind",
                                          ASSERT_ERROR);


    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = defaultValue;

        // Check whether the request ports have room for any new requests.
        let can_enq_uncore_req <- reqToUncore.canEnq(cpu_iid);
        let can_enq_core_rsp <- rspToCore.canEnq(cpu_iid);
        local_state.memQNotFull = can_enq_uncore_req;
        local_state.coreQNotFull = can_enq_core_rsp;
        
        // Now check for responses from the cache coherence engine.
        let m_uncore_rsp <- rspFromUncore.receive(cpu_iid);
        let m_core_req <- reqFromCore.receive(cpu_iid);


        L2C_OPER oper = tagged Invalid;

        if (m_uncore_rsp matches tagged Valid .rsp &&&
            can_enq_core_rsp)
        begin
            //
            // FILL received and the core response port is available.
            //
            let fill = cacheMsgFromMemRsp(rsp);
            assertRspOk(cacheMsg_IsRspLoad(fill));

            oper = tagged FILL_RSP fill;

            L2_MISS_TOKEN miss_tok = fromMemOpaque(rsp.opaque);
            debugLog.record(cpu_iid, $format("1: FILL RSP: %0d, ", miss_tok) + fshow(rsp));
        end
        else if (m_core_req matches tagged Valid .req)
        begin
            //
            // New request from core received.
            //
            if (cacheMsg_IsReqLoad(req))
            begin
                oper = tagged LOAD_REQ req;
                debugLog.record(cpu_iid, $format("1: LOAD REQ: ") + fshow(req));
            end
            else if (cacheMsg_IsWBInval(req))
            begin
                oper = tagged STORE_REQ req;
                debugLog.record(cpu_iid, $format("1: STORE REQ: ") + fshow(req));
            end
            else
            begin
                assertReqOk(False);
            end
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        stage2Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        case (oper) matches
            tagged FILL_RSP .fill:
            begin
                // Fill will write to the cache.  The write may still be turned
                // off later in this pipeline and retried in another model cycle
                // if a needed eviction is blocked.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.linePAddr;
                local_state.writeDataDirty = False;

                // Get the Miss ID.
                L2_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = tagged Valid miss_tok;

                // Only respond to loads.
                if (missTokIsLoad(miss_tok))
                begin
                    local_state.coreQUsed = True;
                    local_state.coreQData = fill;

                    // The destination and opaque are stored in opaquesPool.
                    // Retrieve them and overwrite dst/opaque in coreQData
                    // in stage 2.
                    opaquesPool.readReq(cpu_iid, missTokIndex(miss_tok));

                    debugLog.record(cpu_iid, $format("2: FILL LOAD RSP: %0d, ", miss_tok) + fshow(fill));
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("2: FILL RSP: %0d, ", miss_tok) + fshow(fill));
                end

                // Pick the victim
                l2Alg.lookupByAddrReq(cpu_iid,
                                      fill.linePAddr,
                                      False,
                                      missTokIsLoad(miss_tok));
            end

            tagged LOAD_REQ .req:
            begin
                // Look up the address in the cache.
                l2Alg.lookupByAddrReq(cpu_iid, req.linePAddr, True, True);
                debugLog.record(cpu_iid, $format("2: LOAD REQ: ") + fshow(req));
            end

            tagged STORE_REQ .req:
            begin
                // Look up the address in the cache.
                l2Alg.lookupByAddrReq(cpu_iid, req.linePAddr, True, False);
                debugLog.record(cpu_iid, $format("2: STORE REQ: ") + fshow(req));
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("2: Bubble"));
            end
        endcase

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage3 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        // Hit/miss events will be recorded in this rule.
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;

        case (oper) matches
            tagged FILL_RSP .fill:
            begin
                if (local_state.coreQUsed)
                begin
                    //
                    // Restore the opaque to the context that sent the request
                    // to this cache.  This cache modified only the bits required
                    // for a local miss token.  Merge the unmodified bits (still
                    // in the opaque) and the preserved, overwritten bits (stored
                    // in opaquesPool).
                    //
                    let prev_opaque <- opaquesPool.readRsp(cpu_iid);
                    local_state.coreQData.opaque =
                        updateMemOpaque(local_state.coreQData.opaque,
                                        prev_opaque);
                end

                let evict <- l2Alg.lookupByAddrRsp(cpu_iid);

                local_state.writePortIdx = evict.idx;

                if (evict.state matches tagged Blocked)
                begin
                    // No available victim due to temporary state of the old
                    // line.  Wait for it to settle.
                    oper = newL2COper(tagged Invalid);
                    local_state = defaultValue;

                    statFillRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: BLOCKED EVICTION: new line 0x%h", fill.linePAddr));
                end
                else if (evict.state matches tagged MustEvict .state &&&
                         state.dirty)
                begin
                    // Is there any room in the memQ?
                    if (local_state.memQNotFull)
                    begin
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION: new line 0x%h, old line 0x%h, ", fill.linePAddr, state.linePAddr) + fshow(evict.idx));

                        // Record that we're using the memQ.
                        local_state.memQUsed = True;
                        CACHE_PROTOCOL_WB_INVAL wb = defaultValue;
                        wb.inval = True;
                        wb.isWriteback = True;
                        local_state.memQData = cacheMsg_WBInval(state.linePAddr, ?, wb);
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION WB BLOCKED: new line 0x%h, old line 0x%h, ", fill.linePAddr, state.linePAddr) + fshow(evict.idx));

                        // Retry on a later model cycle.
                        statFillRetry.incr(cpu_iid);
                        oper = newL2COper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
                else
                begin
                    // Clean eviction.  No writeback needed.
                    if (evict.state matches tagged Valid .state)
                    begin
                        debugLog.record(cpu_iid, $format("3: ALREADY PRESENT: line 0x%h, ", fill.linePAddr) + fshow(evict.idx));
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: CLEAN EVICTION: line 0x%h", fill.linePAddr) + fshow(evict.idx));
                    end
                end
            end

            tagged LOAD_REQ .req:
            begin
                let entry <- l2Alg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // Hit!
                    if (local_state.coreQNotFull)
                    begin
                        // Load response indicates data present.
                        local_state.coreQUsed = True;
                        local_state.coreQData = cacheMsg_RspLoad(req.linePAddr,
                                                                 req.opaque,
                                                                 defaultValue);

                        statReadHit.incr(cpu_iid);
                        evt_hit = tagged Valid resize({ req.linePAddr, 1'b0 });

                        debugLog.record(cpu_iid, $format("3: LOAD HIT: ") + fshow(req) + $format(", ") + fshow(entry.idx));
                    end
                    else
                    begin
                        // A load hit, but the response port is busy.
                        statReadRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD HIT RETRY: ") + fshow(req) + $format(", ") + fshow(entry.idx));

                        oper = newL2COper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
                else
                begin
                    // Miss.  Fill the line from memory.
                    if (outstandingMisses.canAllocateLoad(cpu_iid) &&
                        local_state.memQNotFull)
                    begin
                        // Allocate the next miss ID.
                        local_state.memQUsed = True;
                        outstandingMisses.allocateLoadReq(cpu_iid,
                                                          req.linePAddr);

                        statReadMiss.incr(cpu_iid);
                        evt_miss = tagged Valid resize({ req.linePAddr, 1'b0 });

                        debugLog.record(cpu_iid, $format("3: LOAD MISS: ") + fshow(req));
                    end
                    else
                    begin
                        // The request must stall.
                        statReadRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS RETRY"));

                        oper = newL2COper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
            end

            tagged STORE_REQ .req:
            begin
                let entry <- l2Alg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // Hit!  Writeback sets the dirty bit.
                    local_state.writePortUsed = True;
                    local_state.writePortIdx = entry.idx;
                    local_state.writePortData = req.linePAddr;
                    local_state.writeDataDirty = True;

                    statWriteHit.incr(cpu_iid);
                    evt_hit = tagged Valid resize({ req.linePAddr, 1'b1 });

                    debugLog.record(cpu_iid, $format("3: STORE HIT: ") + fshow(req) + $format(", ") + fshow(entry.idx));
                end
                else
                begin
                    // Miss.  Fill the line from memory.
                    if (outstandingMisses.canAllocateStore(cpu_iid) &&
                        local_state.memQNotFull)
                    begin
                        // Allocate the next miss ID
                        local_state.memQUsed = True;
                        outstandingMisses.allocateStoreReq(cpu_iid);

                        statWriteMiss.incr(cpu_iid);
                        evt_miss = tagged Valid resize({ req.linePAddr, 1'b1 });

                        debugLog.record(cpu_iid, $format("3: STORE MISS: ") + fshow(req));
                    end
                    else
                    begin
                        // Memory request queue if busy.  The request must stall.
                        statWriteRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: STORE MISS RETRY: ") + fshow(req));

                        oper = newL2COper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble"));
            end
        endcase

        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage4Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage4 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged FILL_RSP .fill:
            begin
                debugLog.record(cpu_iid, $format("4: FILL RSP Bubble"));
            end

            tagged LOAD_REQ .req:
            begin
                if (local_state.memQUsed)
                begin
                    // A fill request is being sent to memory.  Complete the
                    // request details.
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);

                    // Record the state of the outstanding fill, including the
                    // L1 opaque.
                    opaquesPool.write(cpu_iid, missTokIndex(miss_tok),
                                      fromMemOpaque(req.opaque));

                    // Use the opaque bits to store the miss token.
                    // Use the opaque bits to store the miss token.
                    let mem_req = cacheMsg_ReqLoad(req.linePAddr,
                                                   updateMemOpaque(req.opaque, miss_tok),
                                                   defaultValue);
                    local_state.memQData = mem_req;

                    debugLog.record(cpu_iid, $format("4: LOAD MISS: %0d, ", miss_tok.index) + fshow(req));
                end
            end

            tagged STORE_REQ .req:
            begin
                if (local_state.memQUsed)
                begin
                    // A fill request is being sent to memory.  Complete the
                    // request details.
                    let miss_tok <- outstandingMisses.allocateStoreRsp(cpu_iid);

                    // Use the opaque bits to store the miss token.
                    // We use a load to simulate getting exclusive access.
                    CACHE_PROTOCOL_REQ_LOAD fill_req = defaultValue;
                    fill_req.exclusive = True;
                    let mem_req = cacheMsg_ReqLoad(req.linePAddr,
                                                   updateMemOpaque(req.opaque, miss_tok),
                                                   fill_req);
                    local_state.memQData = mem_req;

                    debugLog.record(cpu_iid, $format("4: STORE MISS: %0d, ", miss_tok.index) + fshow(req));
                end
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble"));
            end
        endcase

        stage5Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage5_end (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("5: Done"));

        //
        // Indicate whether requests were consumed.
        //

        if (oper matches tagged FILL_RSP .fill)
        begin
            rspFromUncore.doDeq(cpu_iid);
        end
        else
        begin
            rspFromUncore.noDeq(cpu_iid);
        end


        if (oper matches tagged LOAD_REQ .req)
        begin
            reqFromCore.doDeq(cpu_iid);
        end
        else if (oper matches tagged STORE_REQ .req)
        begin
            reqFromCore.doDeq(cpu_iid);
        end
        else
        begin
            reqFromCore.noDeq(cpu_iid);
        end


        //
        // Send requests/responses.
        //

        // Take care of the memory queue.
        if (local_state.memQUsed)
        begin
            let req = local_state.memQData;
            let mreq = MEMORY_REQ { linePAddr: req.linePAddr,
                                    opaque: req.opaque,
                                    isStore: cacheMsg_IsWBInval(req) };

            reqToUncore.doEnq(cpu_iid, mreq);

            debugLog.record(cpu_iid, $format("5: REQ TO UNCORE: ") + fshow(req));
        end
        else
        begin
            reqToUncore.noEnq(cpu_iid);
        end
        
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
            CACHE_ENTRY_STATE#(void) state =
                initCacheEntryState(local_state.writePortData,
                                    local_state.writeDataDirty,
                                    ?);

            if (oper matches tagged FILL_RSP .rsp)
            begin
                // Fill allocates a new line.
                l2Alg.allocate(cpu_iid,
                               local_state.writePortIdx,
                               state,
                               CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("5: ALLOC: ") + fshow(local_state.writePortIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                // Store is the only other path.  It updates the state of an
                // existing entry.
                l2Alg.update(cpu_iid,
                             local_state.writePortIdx,
                             state);

                debugLog.record(cpu_iid, $format("5: UPDATE: ") + fshow(local_state.writePortIdx) + $format(", ") + fshow(state));
            end
        end
        
        // Take care of CPU rsp
        if (local_state.coreQUsed)
        begin
            rspToCore.doEnq(cpu_iid, local_state.coreQData); 
        end
        else
        begin
            rspToCore.noEnq(cpu_iid);
        end


        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        // End of model cycle.
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
