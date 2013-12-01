// ******* Library Imports *******

import Vector::*;
import FIFO::*;

// ******* Application Imports *******

`include "asim/provides/soft_connections.bsh"
`include "asim/provides/common_services.bsh"


// ******* HAsim Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/hasim_modellib.bsh"

`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"
`include "asim/provides/hasim_l1_dcache_alg.bsh"
`include "asim/provides/hasim_miss_tracker.bsh"

// ******* Generated File Imports *******

`include "asim/dict/PARAMS_HASIM_L1_DCACHE.bsh"

// ****** Local Definitions *******

// DC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Maybe#(L1_DCACHE_MISS_TOKEN) missTokToFree;

    Bool memQNotFull;
    Bool memQUsed;
    MEMORY_REQ memQData;
    
    Bool writePortUsed;
    Bool writeDataDirty;
    LINE_ADDRESS writePortData;
    
    Maybe#(DCACHE_LOAD_INPUT)  loadReq;
    Maybe#(DCACHE_STORE_INPUT) storeReq;
    
    Maybe#(DCACHE_LOAD_OUTPUT_DELAYED)  loadRsp;
    Maybe#(DCACHE_STORE_OUTPUT_DELAYED) storeRsp;
    Maybe#(LINE_ADDRESS) fillToReport;

    Bool loadBypass;
    Bool storeBypass;
    
}
DC_LOCAL_STATE deriving (Eq, Bits);


// initLocalState
//
// A fresh local state for the first stage.

function DC_LOCAL_STATE initLocalState();

    return 
        DC_LOCAL_STATE 
        { 
            missTokToFree: tagged Invalid,
            memQNotFull: True,
            memQUsed: False,
            memQData: ?,
            writePortUsed: False,
            writeDataDirty: False,
            writePortData: 0,
            loadReq: tagged Invalid,
            storeReq: tagged Invalid,
            loadRsp: tagged Invalid,
            storeRsp: tagged Invalid,
            fillToReport: tagged Invalid,
            loadBypass: False,
            storeBypass: False
        };

endfunction

// memQAvailable
//
// The memQ is available if it is notFull AND someone has
// not already used it.

function Bool memQAvailable(DC_LOCAL_STATE local_state);

    return local_state.memQNotFull && !local_state.memQUsed;

endfunction


// mkL1DCache

// A model of an L1 DCache that is unpipelined.
// That is, writes to the cache occur in the same model cycle as hit checks.
// In a more realistic model a writeQ would track pending updates.
// Also there is no victim buffer.
//
// Note that the module itself is implmented as a pipeline, though the target
// model carries out all actions in one model cycle.

module [HASIM_MODULE] mkL1DCache ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_data.out");

    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();
    Param#(1) writeThroughParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_L1_WRITE_THROUGH, paramNode);
    Bool simulatingWriteHitWriteThrough = (writeThroughParam == 1);
    
 
    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(MAX_NUM_CPUS, VOID) dCacheAlg <- mkL1DCacheAlg();

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
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToMemQ      <- mkPortStallSend_Multiplexed("L1_DCache_OutQ");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) fillFromMemory <- mkPortStallRecv_Multiplexed("L1_DCache_InQ");


    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) depports = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = fillFromMemory.ctrl.in;
    inports[2] = reqToMemQ.ctrl.in;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;
    outports[4] = reqToMemQ.ctrl.out;
    outports[5] = fillFromMemory.ctrl.out;
    depports[0] = storeReqFromCPU.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("L1 DCache Controller", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage4Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage5Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage6Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DC_LOCAL_STATE) stage7Ctrl <- mkStageController();


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
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_HIT",
                                           "L1 DCache Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_MISS",
                                           "L1 DCache Write Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_DCACHE_WRITE_RETRY",
                                           "L1 DCache Write Retries"));


    // ****** Rules ******

    // stage1_fill
    
    // See if there are any new fill responses from memory.

    // Ports read:
    // * fillFromMemory
    
    // Ports written:
    // * loadRspDelayedtoCPU
    // * storeRspDelayedToCPU
    
    (* conservative_implicit_conditions *)
    rule stage1_fill (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = initLocalState();

        // Check if the memQ has room for any new requests.
        let memQ_not_full <- reqToMemQ.canEnq(cpu_iid);
        local_state.memQNotFull = memQ_not_full;
        
        // Check for fills.
        let m_fill <- fillFromMemory.receive(cpu_iid);

        // Check if there are any previously returned fills that are going to multiple loads.
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin

            // A fill that came in previously is going to multiple miss tokens.
            local_state.missTokToFree = tagged Valid miss_tok;
            
            // Return it to the CPU.
            debugLog.record_next_cycle(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
            
            // Now send the fill to the right place.
            // Start by looking up if it was a load or store based on miss ID.
            if (missTokIsLoad(miss_tok))
            begin

                // The fill is a load response. Return it to the CPU.
                debugLog.record_next_cycle(cpu_iid, $format("1: FILL MULTIPLE RSP LOAD: %0d", miss_tok.index));
                local_state.loadRsp = tagged Valid initDCacheLoadMissRsp(miss_tok.index);
                local_state.storeRsp = tagged Invalid;
                
            end
            else
            begin
            
                debugLog.record_next_cycle(cpu_iid, $format("1: FILL MULTIPLE RSP STORE: %0d", miss_tok.index));
                local_state.loadRsp = tagged Invalid;
                local_state.storeRsp = tagged Valid initDCacheStoreDelayOk(miss_tok.index);

            end

        end
        else if (m_fill matches tagged Valid .fill)
        begin


            // We want to use the cache write port.
            // Since we're the highest priority we don't have to check if
            // someone else has it. Just record that we're using it so
            // no one else will.
            local_state.writePortUsed = True;
            local_state.writePortData = fill.physicalAddress;
            local_state.writeDataDirty = False;

            // Deallocate the Miss ID.
            L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
            
            // Free the token in the next stage, in case we had to retry.
            local_state.missTokToFree = tagged Valid miss_tok;
            
            // Now send the fill to the right place.
            // Start by looking up if it was a load or store based on miss ID.
            if (missTokIsLoad(miss_tok))
            begin

                // The fill is a load response. Return it to the CPU.
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP LOAD: %0d, LINE: 0x%h", miss_tok.index, fill.physicalAddress));
                local_state.loadRsp = tagged Valid initDCacheLoadMissRsp(miss_tok.index);
                local_state.storeRsp = tagged Invalid;
                local_state.fillToReport = tagged Valid fill.physicalAddress;
                
            end
            else
            begin

                // The fill is a store response. Tell the CPU the entry has been loaded
                // so it's OK to perform their store with no coherence issues.
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP STORE: %0d, LINE: 0x%h", miss_tok.index, fill.physicalAddress));
                local_state.loadRsp = tagged Invalid;
                local_state.storeRsp = tagged Valid initDCacheStoreDelayOk(miss_tok.index);

            end

            // See if our allocation will evict a dirty line for writeback.
            // This check will be finished in the following stage.
            let fill_addr = fill.physicalAddress;
            dCacheAlg.evictionCheckReq(cpu_iid, fill_addr);

        end
        else
        begin

            // Tell the CPU there's no delayed responses.
            debugLog.record_next_cycle(cpu_iid, $format("1: NO MEM RSP"));
            local_state.loadRsp = tagged Invalid;
            local_state.storeRsp = tagged Invalid;

        end

        // Pass this instance on to the next stage.        
        stage2Ctrl.ready(cpu_iid, local_state);

    endrule

    // stage2_evictAndLoadReq
    
    // Finish fill evictions and request lookups for any loads.
    
    // Ports Read:
    // * loadReqFromCPU
    
    // Ports Written:
    // * None
    
    rule stage2_evictRsp (True);

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // See if we started an eviction in the previous stage.
        if (local_state.writePortUsed)
        begin

            let m_evict <- dCacheAlg.evictionCheckRsp(cpu_iid);

            // If our fill evicted a dirty line we must write it back.
            if (m_evict matches tagged Valid .evict &&& evict.dirty)
            begin

                // Is there any room in the memQ?
                if (memQAvailable(local_state))
                begin

                    debugLog.record(cpu_iid, $format("2: DIRTY EVICTION: 0x%h", evict.physicalAddress));

                    // Record that we're using the memQ.
                    local_state.memQUsed = True;
                    local_state.memQData = initMemStore(evict.physicalAddress);
                
                    // Acknowledge the fill.
                    fillFromMemory.doDeq(cpu_iid);
                
                end
                else
                begin
                
                    // The queue is full, so retry the fill next cycle. No dequeue.
                    fillFromMemory.noDeq(cpu_iid);
                    
                    debugLog.record(cpu_iid, $format("2: DIRTY EVICTION RETRY: 0x%h", evict.physicalAddress));

                    // Yield the writePort to lower-priority users.
                    // The fill update will not happen this model cycle.
                    // Don't free the token.
                    local_state.writePortUsed = False;
                    local_state.missTokToFree = tagged Invalid;
                    local_state.fillToReport = tagged Invalid;
                    local_state.loadRsp = tagged Invalid;
                    local_state.storeRsp = tagged Invalid;

                end

            end
            else
            begin

                // We finished the fill succesfully with no writeback, so dequeue it and free the miss.
                debugLog.record(cpu_iid, $format("2: CLEAN EVICTION"));
                fillFromMemory.doDeq(cpu_iid);

            end

            // Note that the actual dCache update will be done later, so that
            // any lookups this model cycle don't see it accidentally.

        end
        else
        begin
        
            // No dequeue.
            fillFromMemory.noDeq(cpu_iid);
        
        end
        
        loadRspDelToCPU.send(cpu_iid, local_state.loadRsp);
        storeRspDelToCPU.send(cpu_iid, local_state.storeRsp);
        
        if (local_state.fillToReport matches tagged Valid .addr)
        begin
            outstandingMisses.reportLoadDone(cpu_iid, addr);
        end
        
        // Continue in the next stage.
        stage3Ctrl.ready(cpu_iid, local_state);
        
    endrule
    
    rule stage3_loadReq (True);

        match {.cpu_iid, .local_state} <- stage3Ctrl.nextReadyInstance();

        // Now read the load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);

        // Deal with any load requests.
        if (msg_from_cpu matches tagged Valid .req)
        begin

            let line_addr = toLineAddress(req.physicalAddress);

            // See if it's served by the fill from this cycle
            if (local_state.writePortUsed && line_addr == local_state.writePortData)
            begin
                
                // The fill was for this address, so we'll serve it, bypassing the cache.
                debugLog.record_next_cycle(cpu_iid, $format("3: LOAD BYPASS: 0x%h LINE: 0x%h", req.physicalAddress, line_addr));

                // Pass the bypass to the next stage.
                local_state.loadBypass = True;
                local_state.loadReq = tagged Valid req;

            end
            else
            begin
            
                // See if the cache algorithm hit or missed.
                dCacheAlg.loadLookupReq(cpu_iid, line_addr);
                debugLog.record(cpu_iid, $format("3: LOAD REQ: 0x%h LINE: 0x%h", req.physicalAddress, line_addr));

                // Finish the request in the next stage.
                local_state.loadReq = tagged Valid req;
                
            end

        end
        else
        begin

            debugLog.record(cpu_iid, $format("3: NO LOAD"));
        
        end
        
        // Pass our information to the next stage.
        stage4Ctrl.ready(cpu_iid, local_state);

    endrule
    
    // stage3_loadRsp
    
    // Finish up any loads to see if they hit or miss.
    // Begin handling any store requests.
    
    // Ports Read:
    // * storeReqFromCPU
    
    // Ports Written:
    // * loadRspImmToCPU

    rule stage4_loadRsp (True);

        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage4Ctrl.nextReadyInstance();

        // See if we need to finish any load responses
        if (local_state.loadBypass)
        begin
            
            // A bypass, which is as good as a hit, so give the data back. We won't need the memory queue.
            loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadHit(validValue(local_state.loadReq)));
            statReadHit.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("4: LOAD HIT (BYPASSED)"));

        end
        else if (local_state.loadReq matches tagged Valid .req)
        begin

            // Get the lookup response.
            let m_entry <- dCacheAlg.loadLookupRsp(cpu_iid);

            // Does the cache contain this addresss?
            if (m_entry matches tagged Valid .entry)
            begin

                // A hit, so give the data back. We won't need the memory queue.
                loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadHit(req));
                statReadHit.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("4: LOAD HIT"));

            end
            else
            begin

                // A miss. But do we have a free missID to track the fill with?
                // And is the memQ not full and free for us to use?
                // And is there already a load outstanding to this address?

                let line_addr = toLineAddress(req.physicalAddress);
                let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);
                
                if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) && can_allocate)
                begin
                
                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateLoad(cpu_iid, line_addr);
                    
                    // No fill to memory necessary.
                    // Tell the CPU their load missed, but we're handling it.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadMiss(req, miss_tok.index));
                    statReadMiss.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("4: LOAD MISS (ALREADY OUTSTANDING): %0d", miss_tok.index));

                end
                else if (can_allocate && memQAvailable(local_state))
                begin

                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateLoad(cpu_iid, toLineAddress(req.physicalAddress));

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    let mem_req = initMemLoad(line_addr);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    local_state.memQData = mem_req;

                    // Tell the CPU their load missed, but we're handling it.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadMiss(req, miss_tok.index));
                    statReadMiss.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("4: LOAD MISS: %0d", miss_tok.index));

                end
                else
                begin

                    // The CPU must retry.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadRetry(req));
                    
                    // Record the retry.
                    statReadRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("4: LOAD RETRY. MEMQ: %0d, MISS TOK: %0d", pack(memQAvailable(local_state)), pack(outstandingMisses.canAllocateLoad(cpu_iid))));

                end

            end // cache load miss
        end
        else
        begin

            // Propogate the bubble.
            loadRspImmToCPU.send(cpu_iid, tagged Invalid);

        end
        
        stage5Ctrl.ready(cpu_iid, local_state);
   
    endrule
    
    rule stage5_storeReq (True);
    
        // Get our work so far.
        match {.cpu_iid, .local_state} <- stage5Ctrl.nextReadyInstance();

        // Now deal with any store requests.
        let m_store_from_cpu <- storeReqFromCPU.receive(cpu_iid);

        if (m_store_from_cpu matches tagged Valid .req)
        begin

            let line_addr = toLineAddress(req.physicalAddress);

            // See if it's served by the fill from this cycle
            if (local_state.writePortUsed && line_addr == local_state.writePortData)
            begin
                
                // The fill was for this address, so we'll serve it, bypassing the cache.
                debugLog.record_next_cycle(cpu_iid, $format("5: STORE BYPASS: 0x%h LINE: 0x%h", req.physicalAddress, line_addr));

                // Pass the bypass to the next stage.
                local_state.storeBypass = True;
                local_state.storeReq = tagged Valid req;

            end
            else
            begin

                // See if the cache algorithm hit or missed.
                dCacheAlg.storeLookupReq(cpu_iid, line_addr);
                debugLog.record(cpu_iid, $format("5: STORE REQ: 0x%h, LINE: 0x%h", req.physicalAddress, line_addr));

                // Finish handling the request in the next stage.
                local_state.storeReq = tagged Valid req;
            
            end

        end
        else
        begin
        
            debugLog.record(cpu_iid, $format("5: NO STORE"));

        end
        
        // Pass everything on to the next stage.
        stage6Ctrl.ready(cpu_iid, local_state);

    endrule

    // stage6_storeRsp
    
    // Finish handling any stores and see if they hit or miss.
    // A store hit which is writethrough needs to grab the memory queue.

    // Additionally, now that all arbitration is done we can actually
    // make the request to memory and update the cache. 
    
    // Ports Read:
    // * None
    
    // Ports Written:
    // * storeRspImmToCPU
    // * reqToMemory

    rule stage6_storeRsp (True);

        // Get our work so far.
        match {.cpu_iid, .local_state} <- stage6Ctrl.nextReadyInstance();

        // Finish up any stores we started in the last stage.
        if (local_state.storeBypass)
        begin
            
            // A bypass, which is as good as a hit, so give the data back. We won't need the memory queue.
            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk());
            statWriteHit.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("6: STORE HIT (BYPASSED)"));

        end
        else 
        if (local_state.storeReq matches tagged Valid .req)
        begin

            // See if the cache currently contains the requested value.
            let m_entry <- dCacheAlg.storeLookupRsp(cpu_iid);

            if (m_entry matches tagged Valid .entry)
            begin

                // A hit, but what if someone else is using the write port?
                if (local_state.writePortUsed)
                begin

                    // Tell the CPU to retry.
                    storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry());

                    // Record the conflict.
                    statWriteRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("6: STORE HIT RETRY (WRITE PORT USED)"));

                end
                else
                begin

                    // What we do next depends on policy.
                    if (simulatingWriteHitWriteThrough())
                    begin

                        // Since we're writethrough we need to enqueue into the memQ
                        // in order for a store to succeed.
                        if (memQAvailable(local_state))
                        begin

                            // Since we're writethrough mark the data clean so
                            // it won't be written back later.
                            let line_addr = toLineAddress(req.physicalAddress);
                            local_state.writePortUsed = True;
                            local_state.writePortData = line_addr;
                            local_state.writeDataDirty = False;

                            // Record that we are using the memQ (since we're writethrough).
                            local_state.memQUsed = True;
                            local_state.memQData = initMemStore(line_addr);

                            // Tell the CPU that the store succeeded.
                            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk());
                            statWriteHit.incr(cpu_iid);
                            debugLog.record(cpu_iid, $format("6: STORE HIT"));

                        end
                        else
                        begin

                            // Tell the CPU to retry.
                            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry());
                            
                            // Record the conflict.
                            statWriteRetry.incr(cpu_iid);
                            debugLog.record(cpu_iid, $format("6: STORE HIT RETRY (MEMQ UNAVAILABLE FOR WRITE THROUGH)"));

                        end

                    end // writethrough 
                    else
                    begin

                        // We're writeback, so we don't need the memQ,
                        // we can just overwrite the line.
                        // Note that we don't need to do an eviction check since
                        // we hit, so we'll just overwrite the existing value.
                        // In other words, the writes will be coalesced and only
                        // one writeback to memory will occur.
                        
                        let line_addr = toLineAddress(req.physicalAddress);
                        local_state.writePortUsed = True;
                        local_state.writePortData = line_addr;
                        local_state.writeDataDirty = True;

                        // Tell the CPU the store succeeded.
                        storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk());
                        statWriteHit.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("6: STORE HIT"));


                    end // writeback

                end // using write port

            end // store hit
            else
            begin

                // A miss. So we need to bring the line into the cache to deal with 
                // any coherence issues. (Which are handled explicitly at lower levels.)
                // But did a higher-priority guy already use the mem port?
                // And can we get a Miss ID?
                if (memQAvailable(local_state) && outstandingMisses.canAllocateStore(cpu_iid))
                begin

                    // The memory queue is free. Let' bring the data into the cache.
                    // This will resolve any coherence issues at lower levels. 

                    // Get a MissID to go to memory.
                    let miss_tok <- outstandingMisses.allocateStore(cpu_iid);

                    // Record that we're using the memQ.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    let line_addr = toLineAddress(req.physicalAddress);
                    let mem_req = initMemLoad(line_addr);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    local_state.memQData = mem_req;

                    // Tell the CPU to delay until the store returns.
                    storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreDelay(miss_tok.index));
                    statWriteMiss.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("6: STORE MISS: %0d", miss_tok.index));

                end
                else
                begin

                    // The CPU will have to retry this store.
                    storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry());
                    statWriteRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("6: STORE RETRY. MEMQ: %0d, MISS TOK: %0d", pack(memQAvailable(local_state)), pack(outstandingMisses.canAllocateStore(cpu_iid))));

                end

            end // cache write miss

        end
        else
        begin

            // No Store. Propogate the bubble.
            storeRspImmToCPU.send(cpu_iid, tagged Invalid);

        end
        
        stage7Ctrl.ready(cpu_iid, local_state);
        
    endrule
    
    rule stage7_memReq (True);
    
    
        match {.cpu_iid, .local_state} <- stage7Ctrl.nextReadyInstance();
        debugLog.record(cpu_iid, $format("7: DONE"));
        
        // Take care of the memory queue.
        if (local_state.memQUsed)
        begin

            reqToMemQ.doEnq(cpu_iid, local_state.memQData);

        end
        else
        begin

            reqToMemQ.noEnq(cpu_iid);

        end
        
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
        
            dCacheAlg.allocate(cpu_iid, local_state.writePortData, local_state.writeDataDirty, 0);
        
        end
        
        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 

    endrule

endmodule
