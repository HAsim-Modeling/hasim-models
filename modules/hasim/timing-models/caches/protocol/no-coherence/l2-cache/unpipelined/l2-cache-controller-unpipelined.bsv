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

typedef enum
{
    L2_CC_REQ_WB,
    L2_CC_REQ_INVALIDATE
}
L2_CC_REQ deriving (Eq, Bits);


//
// L2_LOCAL_STATE --
//   L2 State to pass between pipeline stages.
//
typedef struct
{
    L2_MISS_TOKEN missTokToFree;

    Bool memQNotFull;
    Bool memQUsed;
    CACHE_PROTOCOL_MSG memQData;
    
    Bool writePortUsed;
    Bool writeDataDirty;
    LINE_ADDRESS writePortData;
    
    Bool coreQNotFull;
    Bool coreQUsed;
    CACHE_PROTOCOL_MSG coreQData;

    Maybe#(CACHE_PROTOCOL_MSG) req;
}
L2_LOCAL_STATE deriving (Eq, Bits);

// initLocalState
//
// A fresh local state for the first stage.

function L2_LOCAL_STATE initLocalState();
    return 
        L2_LOCAL_STATE 
        { 
            missTokToFree: ?,
            memQNotFull: True,
            memQUsed: False,
            memQData: ?,
            coreQNotFull: True,
            coreQUsed: False,
            coreQData: ?,
            writePortUsed: False,
            writeDataDirty: False,
            writePortData: 0,
            req: tagged Invalid
        };
endfunction

//
// memQAvailable --
//   The memQ is available if it is notFull AND someone has
//   not already used it.
//
function Bool memQAvailable(L2_LOCAL_STATE local_state);
    return local_state.memQNotFull && !local_state.memQUsed;
endfunction


// mkL2Cache

// A model of an L2 Cache that is unpipelined.
// That is, writes to the cache occur in the same model cycle as hit checks.
// In a more realistic model a writeQ would track pending updates.
// Also there is no victim buffer.
//
// Note that the module itself is implemented as a pipeline, though the target
// model carries out all actions in one model cycle.

module [HASIM_MODULE] mkL2Cache#(String reqFromL1Name,
                                 String rspToL1Name,
                                 String reqToMemoryName,
                                 String rspFromMemoryName)
    // Interface:
    ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l2_data.out");

    // ****** Submodels ******

    // Make an interface to the cache coherence protocol.
    let ccifc <- mkL2CacheCoherenceInterface(reqToMemoryName, rspFromMemoryName);

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(MAX_NUM_CPUS, VOID) l2Alg <- mkL2CacheAlg();

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
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, L2_CC_REQ) reqFromCC <-
        mkPortStallRecv_Multiplexed("CC_to_L2_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqToCC <-
        mkPortStallSend_Multiplexed("L2_to_CC_req");
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspFromCC <-
        mkPortStallRecv_Multiplexed("CC_to_L2_rsp");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToCC <-
        mkPortStallSend_Multiplexed("L2_to_CC_rsp");
    
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    inctrls[2]  = reqFromCC.ctrl.in;
    inctrls[3]  = reqToCC.ctrl.in;
    inctrls[4]  = rspFromCC.ctrl.in;
    inctrls[5]  = rspToCC.ctrl.in;

    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqFromCC.ctrl.out;
    outctrls[3] = reqToCC.ctrl.out;
    outctrls[4] = rspFromCC.ctrl.out;
    outctrls[5] = rspToCC.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L2 Cache", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2_LOCAL_STATE, Bool)) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, L2_LOCAL_STATE) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L2_LOCAL_STATE,
                                            Maybe#(CACHE_PROTOCOL_MSG))) stage4Ctrl <-
        mkStageController();

    Reg#(Maybe#(Tuple4#(CPU_INSTANCE_ID,
                        CACHE_PROTOCOL_MSG,
                        Maybe#(CACHE_ENTRY#(VOID)),
                        L2_LOCAL_STATE))) stage3Stall <- mkReg(tagged Invalid);


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
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_WRITE_RETRY",
                                           "L2 Write Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statFillRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L2_FILL_RETRY",
                                           "L2 Fill Retries"));

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_L2_HIT);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_L2_MISS);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventFill <- mkEventRecorder_Multiplexed(`EVENTS_L2_FILL);

    // ****** Assertions ******

    let assertReqOk <- mkAssertionSimOnly("l2-cache-controller-unpipelined.bsv: Unexpected request kind",
                                          ASSERT_ERROR);
    let assertRspOk <- mkAssertionSimOnly("l2-cache-controller-unpipelined.bsv: Unexpected response kind",
                                          ASSERT_ERROR);


    //
    // Stage 1 --
    //   Consume responses from memory.
    //
    (* conservative_implicit_conditions *)
    rule stage1_fill (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = initLocalState();

        // Check if the CC engine has room for any new requests.
        let can_enq_cc_req <- reqToCC.canEnq(cpu_iid);
        let can_enq_cc_rsp <- rspToCC.canEnq(cpu_iid);
        let can_enq_core_rsp <- rspToCore.canEnq(cpu_iid);
        local_state.memQNotFull = can_enq_cc_req;
        local_state.coreQNotFull = can_enq_core_rsp;
        
        // Now check for responses from the cache coherence engine.
        let m_cc_rsp <- rspFromCC.receive(cpu_iid);

        // Also check for new requests from the cache coherence engine.
        let m_cc_req <- reqFromCC.receive(cpu_iid);

        // Unused by L2. Should be handling invalidation writebacks.
        rspToCC.noEnq(cpu_iid);

        // L2 drops invalidates at this point. 
        // TODO: They should be passed on to L1C via SEPARATE fifos.
        if (m_cc_req matches tagged Valid .req)
        begin
            reqFromCC.doDeq(cpu_iid);
        end
        else
        begin
            reqFromCC.noDeq(cpu_iid);
        end

        Bool read_opaques = False;

        //
        // Have a response from memory?
        //
        if (m_cc_rsp matches tagged Valid .rsp)
        begin
            assertRspOk(cacheMsg_IsRspLoad(rsp));

            // Yes.  Can it be sent up toward the core?
            if (local_state.coreQNotFull)
            begin
                // Yes.  Consume the response.
                let fill = rsp;

                // We want to use the cache write port.
                // Since we're the highest priority we don't have to check if
                // someone else has it. Just record that we're using it so
                // no one else will.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.linePAddr;
                local_state.writeDataDirty = False;

                // Get the Miss ID.
                L2_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = miss_tok;

                // Return the fill to higher levels.
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP: %0d, LINE: 0x%h", miss_tok.index, fill.linePAddr));

                // Only respond to loads.
                if (missTokIsLoad(miss_tok))
                begin
                    local_state.coreQUsed = True;
                    local_state.coreQData = fill;

                    // Replace the opaque with the one for higher levels.
                    opaquesPool.readReq(cpu_iid, missTokIndex(miss_tok));
                    read_opaques = True;
                end

                // See if our allocation will evict a dirty line for writeback.
                // This check will be finished in the following stage.
                l2Alg.evictionCheckReq(cpu_iid, fill.linePAddr);
            end
            else
            begin
                // Can't consume the response because there is no space to
                // forward it. 
                L2_MISS_TOKEN miss_tok = fromMemOpaque(rsp.opaque);
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP RETRY: %0d, LINE: 0x%h", miss_tok.index, rsp.linePAddr));
            end
        end
        else
        begin
            // There's no response from the below.
            debugLog.record_next_cycle(cpu_iid, $format("1: NO MEM RSP"));
        end

        stage2Ctrl.ready(cpu_iid, tuple2(local_state, read_opaques));
    endrule


    //
    // stage2_evictAndCPUReq --
    //   Finish fill evictions and request lookups for any load/stores.
    //
    rule stage2_evictAndCPUReq (True);

        match {.cpu_iid, .local_state, .read_opaques} <- stage2Ctrl.nextReadyInstance();

        if (read_opaques)
        begin
            //
            // Restore the opaque to the context that sent the request to this
            // cache.  This cache modified only the bits required for a local
            // miss token.  Merge the unmodified bits (still in the opaque)
            // and the preserved, overwritten bits (stored in opaquesPool).
            //
            let prev_opaque <- opaquesPool.readRsp(cpu_iid);
            local_state.coreQData.opaque = updateMemOpaque(local_state.coreQData.opaque,
                                                           prev_opaque);
        end

        // See if we started an eviction in the previous stage.
        if (local_state.writePortUsed)
        begin
            let m_evict <- l2Alg.evictionCheckRsp(cpu_iid);

            // If our fill evicted a dirty line we must write it back.
            if (m_evict matches tagged Valid .evict &&& evict.dirty)
            begin
                // Is there any room in the memQ?
                if (memQAvailable(local_state))
                begin
                    debugLog.record(cpu_iid, $format("2: DIRTY EVICTION: 0x%h", evict.physicalAddress));

                    // Record that we're using the memQ.
                    local_state.memQUsed = True;
                    local_state.memQData = cacheMsg_StoreReq(evict.physicalAddress, ?);
                    outstandingMisses.free(cpu_iid, local_state.missTokToFree);
                
                    // Acknowledge the fill.
                    rspFromCC.doDeq(cpu_iid);
                end
                else
                begin
                    // The queue is full, so retry the fill next cycle. No dequeue.
                    rspFromCC.noDeq(cpu_iid);
                    
                    debugLog.record(cpu_iid, $format("2: DIRTY EVICTION RETRY: 0x%h", evict.physicalAddress));

                    // Yield the writePort and rspPort to lower-priority users.
                    // The fill update will not happen this model cycle.
                    // Don't free the token.
                    local_state.writePortUsed = False;
                    local_state.coreQUsed = False;
                end
            end
            else
            begin
                // We finished the fill succesfully and no writeback is needed.
                // Dequeue the fill and free the miss.
                debugLog.record(cpu_iid, $format("2: CLEAN EVICTION"));
                outstandingMisses.free(cpu_iid, local_state.missTokToFree);
                rspFromCC.doDeq(cpu_iid);
            end

            // Note that the actual cache update will be done later, so that
            // any lookups this model cycle don't see it accidentally.
        end
        else
        begin
            // No fill received
            rspFromCC.noDeq(cpu_iid);
        end

        if (local_state.writePortUsed)
        begin
            eventFill.recordEvent(cpu_iid,
                                  tagged Valid resize({ local_state.writePortData, 1'b0 }));
        end
        else
        begin
            eventFill.recordEvent(cpu_iid, tagged Invalid);
        end


        // Now read the request port from the L1.
        let m_core_req <- reqFromCore.receive(cpu_iid);

        // Deal with any load/store requests.
        if (m_core_req matches tagged Valid .req)
        begin
            assertReqOk(cacheMsg_IsReqLoad(req) || cacheMsg_IsReqStore(req));

            // See if the cache algorithm hit or missed.
            l2Alg.loadLookupReq(cpu_iid, req.linePAddr);
            debugLog.record(cpu_iid, $format("2: REQ: LINE: 0x%h", req.linePAddr));

            // Finish the request in the next stage.
            local_state.req = tagged Valid req;
        end
        else
        begin
            debugLog.record(cpu_iid, $format("2: NO REQ"));
        end
        
        // Pass our information to the next stage.
        stage3Ctrl.ready(cpu_iid, local_state);
    endrule
    
    
    //
    // stage3_cpuRspStoreReq --
    //   Finish up any load/stores to see if they hit or miss.
    //   Begin handling any store requests.
    //    
    rule stage3_cpuRspCCReq (!isValid(stage3Stall));
        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage3Ctrl.nextReadyInstance();

        // See if we need to finish any cpu responses.
        if (local_state.req matches tagged Valid .req)
        begin
            // Get the lookup response.
            let m_entry <- l2Alg.loadLookupRsp(cpu_iid);
            stage3Stall <= tagged Valid tuple4(cpu_iid, req, m_entry, local_state);
        end
        else
        begin
            // No request.  Propogate the bubble.
            reqFromCore.noDeq(cpu_iid);
            eventHit.recordEvent(cpu_iid, tagged Invalid);
            eventMiss.recordEvent(cpu_iid, tagged Invalid);
            stage4Ctrl.ready(cpu_iid, tuple2(local_state, tagged Invalid));
        end
    endrule
    

    (* conservative_implicit_conditions *)
    rule stage3_STALL (stage3Stall matches tagged Valid {.cpu_iid,
                                                         .req,
                                                         .m_entry,
                                                         .ls});
        let local_state = ls;

        //
        // All events here will use the low bit to indicate whether the operation
        // is a load (0) or a store (1).  The remainer of the event data is
        // whatever fits from the low bits of the PA.
        //
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;
        Maybe#(CACHE_PROTOCOL_MSG) new_miss_tok_req = tagged Invalid;

        //
        // Is the line already in the cache?
        //
        if (m_entry matches tagged Valid .entry)
        begin
            // Yes:  hit
            if (cacheMsg_IsReqStore(req))
            begin
                if (!local_state.writePortUsed)
                begin
                    // We're writeback, so we don't need the memQ.
                    // Note that we don't need to do an eviction check since
                    // we hit, so we'll just overwrite the existing value.
                    // In other words, the writes will be coalesced and only
                    // one writeback to memory will occur.
                    local_state.writePortUsed = True;
                    local_state.writePortData = req.linePAddr;
                    local_state.writeDataDirty = True;

                    // No response to a store. Don't change the coreQData in
                    // case there was a fill.
                    statWriteHit.incr(cpu_iid);
                    evt_hit = tagged Valid resize({ req.linePAddr, 1'b1 });
                    debugLog.record(cpu_iid, $format("3: STORE HIT"));
                    reqFromCore.doDeq(cpu_iid);
                end
                else
                begin
                    // The store must retry because a fill happened and the
                    // cache's write port is busy.
                    statWriteRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: STORE HIT RETRY"));
                    reqFromCore.noDeq(cpu_iid);
                end
            end
            else if (local_state.coreQNotFull && ! local_state.coreQUsed)
            begin
                // A load hit, so give the data back. We won't need the
                // memory queue.
                local_state.coreQData = cacheMsg_LoadRsp(req.linePAddr, req.opaque);
                local_state.coreQUsed = True;
                statReadHit.incr(cpu_iid);
                evt_hit = tagged Valid resize({ req.linePAddr, 1'b0 });
                debugLog.record(cpu_iid, $format("3: LOAD HIT"));
                reqFromCore.doDeq(cpu_iid);
            end
            else
            begin
                // A load hit, but the port is already in use, or the queue
                // is full, so retry.
                statReadRetry.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: LOAD HIT RETRY"));
                reqFromCore.noDeq(cpu_iid);
            end
        end
        else
        begin
            //
            // Miss path.
            //
            if (cacheMsg_IsReqStore(req))
            begin
                if (outstandingMisses.canAllocateStore(cpu_iid) &&
                    memQAvailable(local_state))
                begin
                    // Allocate the next miss ID.
                    new_miss_tok_req = tagged Valid req;
                    outstandingMisses.allocateStoreReq(cpu_iid);

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // A miss, so no response. (Don't change the response in case there's an existing fill)
                    //statWriteMiss.incr(cpu_iid);
                    evt_miss = tagged Valid resize({ req.linePAddr, 1'b1 });
                    debugLog.record(cpu_iid, $format("3: STORE MISS: ") + fshow(req));
                    reqFromCore.doDeq(cpu_iid);
                end
                else
                begin
                    // The request must stall.
                    statWriteRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: STORE MISS RETRY"));
                    reqFromCore.noDeq(cpu_iid);
                end
            end
            else
            begin
                // A load miss. But do we have a free missID to track the fill?
                // And is the memQ not full and free for us to use?
                if (outstandingMisses.canAllocateLoad(cpu_iid) &&
                    memQAvailable(local_state))
                begin
                    // Allocate the next miss ID.
                    new_miss_tok_req = tagged Valid req;
                    outstandingMisses.allocateLoadReq(cpu_iid, req.linePAddr);

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // A miss, so no response. (Don't change the response in case
                    // there's an existing fill.)
                    statReadMiss.incr(cpu_iid);
                    evt_miss = tagged Valid resize({ req.linePAddr, 1'b0 });
                    debugLog.record(cpu_iid, $format("3: LOAD MISS: ") + fshow(req));
                    reqFromCore.doDeq(cpu_iid);
                end
                else
                begin
                    // The request must stall.
                    statReadRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD MISS RETRY"));
                    reqFromCore.noDeq(cpu_iid);
                end
            end // cache load miss
        end
        
        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage3Stall <= tagged Invalid;
        stage4Ctrl.ready(cpu_iid, tuple2(local_state, new_miss_tok_req));
    endrule
    

    rule stage4_end (True);
        match {.cpu_iid, {.local_state, .new_miss_tok_req}} <- stage4Ctrl.nextReadyInstance();

        if (new_miss_tok_req matches tagged Valid .req)
        begin
            if (cacheMsg_IsReqStore(req))
            begin
                let miss_tok <- outstandingMisses.allocateStoreRsp(cpu_iid);

                // Use the opaque bits to store the miss token.
                // Note that we use a load to simulate getting exclusive access.
                let mem_req = cacheMsg_LoadReq(req.linePAddr,
                                               updateMemOpaque(req.opaque, miss_tok));
                local_state.memQData = mem_req;

                debugLog.record(cpu_iid, $format("4: STORE MISS: %0d, ", miss_tok.index) + fshow(req));
            end
            else
            begin
                let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);

                // Record the original opaque for returning.
                opaquesPool.write(cpu_iid, missTokIndex(miss_tok),
                                  fromMemOpaque(req.opaque));

                // Use the opaque bits to store the miss token.
                let mem_req = cacheMsg_LoadReq(req.linePAddr,
                                               updateMemOpaque(req.opaque, miss_tok));
                local_state.memQData = mem_req;

                debugLog.record(cpu_iid, $format("4: LOAD MISS: %0d, ", miss_tok.index) + fshow(req));
            end
        end

        // Take care of the memory queue.
        if (local_state.memQUsed)
        begin
            reqToCC.doEnq(cpu_iid, local_state.memQData);
        end
        else
        begin
            reqToCC.noEnq(cpu_iid);
        end

        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
            l2Alg.allocate(cpu_iid,
                           local_state.writePortData,
                           local_state.writeDataDirty,
                           0);
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

        // End of model cycle. (Path 1)
        debugLog.record(cpu_iid, $format("4: END CYCLE"));
        localCtrl.endModelCycle(cpu_iid, 1); 
    endrule
endmodule


//
// mkL2CacheCoherenceInterface --
//   Coherence engine, interposed between the L2 cache and the uncore.
//
module [HASIM_MODULE] mkL2CacheCoherenceInterface#(String reqToMemoryName,
                                                   String rspFromMemoryName)
    // Interface:
    ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l2_coherence.out");

    // Requests from / responses to L2
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqFromL2 <-
        mkPortStallRecv_Multiplexed("L2_to_CC_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToL2 <-
        mkPortStallSend_Multiplexed("CC_to_L2_rsp");

    // Coherence traffic to L2
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, L2_CC_REQ) reqToL2 <-
        mkPortStallSend_Multiplexed("CC_to_L2_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspFromL2 <-
        mkPortStallRecv_Multiplexed("L2_to_CC_rsp");
    
    // Requests to / responses from uncore (L3).  In the no-coherence model,
    // the LLC communicates with MEMORY_REQ/MEMORY_RSP.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToUncore <-
        mkPortStallSend_Multiplexed(reqToMemoryName);
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspFromUncore <-
        mkPortStallRecv_Multiplexed(rspFromMemoryName);

    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = reqToL2.ctrl.in;
    inctrls[1]  = reqFromL2.ctrl.in;
    inctrls[2]  = rspFromL2.ctrl.in;
    inctrls[3]  = rspToL2.ctrl.in;
    inctrls[4]  = reqToUncore.ctrl.in;
    inctrls[5]  = rspFromUncore.ctrl.in;

    outctrls[0] = reqToL2.ctrl.out;
    outctrls[1] = reqFromL2.ctrl.out;
    outctrls[2] = rspFromL2.ctrl.out;
    outctrls[3] = rspToL2.ctrl.out;
    outctrls[4] = reqToUncore.ctrl.out;
    outctrls[5] = rspFromUncore.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L2 Coherence", inctrls, outctrls);

    rule stage1 (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        //
        // No coherence implemented.
        //
        let can_enq_l2_req <- reqToL2.canEnq(cpu_iid);
        reqToL2.noEnq(cpu_iid);

        let m_l2_rsp <- rspFromL2.receive(cpu_iid);
        if (m_l2_rsp matches tagged Valid .rsp)
        begin
            rspFromL2.doDeq(cpu_iid);
        end
        else
        begin
            rspFromL2.noDeq(cpu_iid);
        end


        //
        // Forward L2 requests to uncore.
        //
        let can_enq_uncore_req <- reqToUncore.canEnq(cpu_iid);
        let m_l2_req <- reqFromL2.receive(cpu_iid);
        if (can_enq_uncore_req &&& m_l2_req matches tagged Valid .req)
        begin
            let mreq = MEMORY_REQ { physicalAddress: req.linePAddr,
                                    opaque: req.opaque,
                                    isStore: cacheMsg_IsReqStore(req) };

            reqToUncore.doEnq(cpu_iid, mreq);
            reqFromL2.doDeq(cpu_iid);
            debugLog.record_next_cycle(cpu_iid, $format("1: FWD L2 Req"));
        end
        else
        begin
            reqToUncore.noEnq(cpu_iid);
            reqFromL2.noDeq(cpu_iid);

            if (isValid(m_l2_req))
            begin
                debugLog.record_next_cycle(cpu_iid, $format("1: RETRY L2 Req"));
            end
        end


        //
        // Forward uncore responses to L2.
        //
        let can_enq_l2_rsp <- rspToL2.canEnq(cpu_iid);
        let m_uncore_rsp <- rspFromUncore.receive(cpu_iid);
        if (can_enq_l2_rsp &&& m_uncore_rsp matches tagged Valid .rsp)
        begin
            rspToL2.doEnq(cpu_iid, cacheMsgFromMemRsp(rsp));
            rspFromUncore.doDeq(cpu_iid);
            debugLog.record_next_cycle(cpu_iid, $format("1: FWD Uncore Rsp"));
        end
        else
        begin
            rspToL2.noEnq(cpu_iid);
            rspFromUncore.noDeq(cpu_iid);

            if (isValid(m_uncore_rsp))
            begin
                debugLog.record_next_cycle(cpu_iid, $format("1: RETRY Uncore Rsp"));
            end
        end

        debugLog.record_next_cycle(cpu_iid, $format("1: Done"));

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 
    endrule
endmodule
