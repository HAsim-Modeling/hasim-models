import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"
`include "asim/provides/hasim_last_level_cache_alg.bsh"
`include "asim/provides/hasim_miss_tracker.bsh"

// ******* Generated File Imports *******

`include "asim/dict/STATS_LLC.bsh"
`include "asim/dict/EVENTS_LLC.bsh"

// ****** Local Definitions *******

typedef enum
{
    LLC_CC_REQ_WB,
    LLC_CC_REQ_INVALIDATE
}
LLC_CC_REQ deriving (Eq, Bits);

typedef CORE_MEMORY_REQ LLC_CC_RSP;

typedef CORE_MEMORY_REQ CC_REQ;
typedef CORE_MEMORY_RSP CC_RSP;


// LLC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    LLC_MISS_TOKEN missTokToFree;

    Bool memQNotFull;
    Bool memQUsed;
    MEMORY_REQ memQData;
    
    Bool writePortUsed;
    Bool writeDataDirty;
    LINE_ADDRESS writePortData;
    
    Bool coreQNotFull;
    Bool coreQUsed;
    MEMORY_RSP coreQData;

    Maybe#(MEMORY_REQ)  loadReq;
    
}
LLC_LOCAL_STATE deriving (Eq, Bits);


// initLocalState
//
// A fresh local state for the first stage.

function LLC_LOCAL_STATE initLocalState();

    return 
        LLC_LOCAL_STATE 
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
            loadReq: tagged Invalid
        };

endfunction


// memQAvailable
//
// The memQ is available if it is notFull AND someone has
// not already used it.

function Bool memQAvailable(LLC_LOCAL_STATE local_state);

    return local_state.memQNotFull && !local_state.memQUsed;

endfunction

typedef `LLC_MISS_ID_SIZE LLC_MISS_ID_SIZE;
typedef CACHE_MISS_INDEX#(LLC_MISS_ID_SIZE) LLC_MISS_ID;
typedef CACHE_MISS_TOKEN#(LLC_MISS_ID_SIZE) LLC_MISS_TOKEN;
typedef TExp#(LLC_MISS_ID_SIZE) NUM_LLC_MISS_IDS;


module [HASIM_MODULE] mkLastLevelCache();

    // Make an interface to the cache coherence protocol.
    let ccifc <- mkCacheCoherenceInterface();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(NUM_CPUS, VOID) llcAlg <- mkLastLevelCacheAlg();

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(NUM_CPUS, LLC_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();

    // A RAM To map our miss IDs into the original opaques, that we return to higher levels.
    MULTIPLEXED_LUTRAM#(NUM_CPUS, LLC_MISS_ID, MEM_OPAQUE) opaquesPool <- mkMultiplexedLUTRAM(?);

    // ****** Ports ******

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqFromCore <- mkPortStallRecv_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspToCore   <- mkPortStallSend_Multiplexed("L1_Cache_InQ");
    
    // Queues to/from coherence engine.
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, LLC_CC_REQ) reqFromCC <- mkPortStallRecv_Multiplexed("CC_to_LLC_req");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, CC_REQ)     reqToCC   <- mkPortStallSend_Multiplexed("LLC_to_CC_req");
    
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, CC_RSP)     rspFromCC <- mkPortStallRecv_Multiplexed("CC_to_LLC_rsp");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, LLC_CC_RSP) rspToCC   <- mkPortStallSend_Multiplexed("LLC_to_CC_rsp");
    
    Vector#(6, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    inctrls[2]  = reqFromCC.ctrl.in;
    inctrls[3]  = reqToCC.ctrl.in;
    inctrls[4]  = rspFromCC.ctrl.in;
    inctrls[5]  = rspToCC.ctrl.in;
    outctrls[0]  = reqFromCore.ctrl.out;
    outctrls[1]  = rspToCore.ctrl.out;
    outctrls[2]  = reqFromCC.ctrl.out;
    outctrls[3]  = reqToCC.ctrl.out;
    outctrls[4]  = rspFromCC.ctrl.out;
    outctrls[5]  = rspToCC.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    STAGE_CONTROLLER#(NUM_CPUS, LLC_LOCAL_STATE) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(NUM_CPUS, LLC_LOCAL_STATE) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(NUM_CPUS, LLC_LOCAL_STATE) stage4Ctrl <- mkStageController(); // XXX TMP
    Reg#(Maybe#(Tuple4#(CPU_INSTANCE_ID, MEMORY_REQ, Maybe#(CACHE_ENTRY#(VOID)), LLC_LOCAL_STATE))) stage3Stall <- mkReg(tagged Invalid); //XXX TMP

    // ****** Stats ******

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statReadHit    <- mkStatCounter_Multiplexed(`STATS_LLC_READ_HIT);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statReadMiss   <- mkStatCounter_Multiplexed(`STATS_LLC_READ_MISS);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statReadRetry  <- mkStatCounter_Multiplexed(`STATS_LLC_READ_RETRY);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statWriteHit   <- mkStatCounter_Multiplexed(`STATS_LLC_WRITE_HIT);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statWriteRetry <- mkStatCounter_Multiplexed(`STATS_LLC_WRITE_RETRY);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statFillRetry  <- mkStatCounter_Multiplexed(`STATS_LLC_FILL_RETRY);

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_LLC_HIT);
    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_LLC_MISS);


    (* conservative_implicit_conditions *)
    rule stage1_fill (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our local state from the pools.
        LUTRAM#(LLC_MISS_ID, MEM_OPAQUE) opaques = opaquesPool.getRAM(cpu_iid);

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

        // Unused by LLC. Should be handling invalidation writebacks.
        rspToCC.noEnq(cpu_iid);

        // LLC drops invalidates at this point. 
        // TODO: They should be passed on to L1C via SEPARATE fifos.
        if (m_cc_req matches tagged Valid .req)
        begin
            reqFromCC.doDeq(cpu_iid);
        end
        else
        begin
            reqFromCC.noDeq(cpu_iid);
        end

        if (m_cc_rsp matches tagged Valid .rsp)
        begin

            if (local_state.coreQNotFull)
            begin

                let fill = initMemRsp(rsp.physicalAddress, rsp.opaque);

                // We want to use the cache write port.
                // Since we're the highest priority we don't have to check if
                // someone else has it. Just record that we're using it so
                // no one else will.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.physicalAddress;
                local_state.writeDataDirty = False;

                // Get the Miss ID.
                LLC_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = miss_tok;

                // Return the fill to higher levels.
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP: %0d, LINE: 0x%h", miss_tok.index, fill.physicalAddress));
                // Replace the opaque with the one for higher levels.
                fill.opaque = opaques.sub(missTokIndex(miss_tok));

                // Only respond to loads.
                if (missTokIsLoad(miss_tok))
                begin
                    local_state.coreQUsed = True;
                    local_state.coreQData = fill;
                end

                // See if our allocation will evict a dirty line for writeback.
                // This check will be finished in the following stage.
                llcAlg.evictionCheckReq(cpu_iid, fill.physicalAddress);

            end
            else
            begin
                // Get the Miss ID.
                LLC_MISS_TOKEN miss_tok = fromMemOpaque(rsp.opaque);
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP RETRY: %0d, LINE: 0x%h", miss_tok.index, rsp.physicalAddress));
            end

        end
        else
        begin

            // There's no responses to the CPU.
            debugLog.record_next_cycle(cpu_iid, $format("1: NO MEM RSP"));

        end

        // Pass this instance on to the next stage.        
        stage2Ctrl.ready(cpu_iid, local_state);

    endrule

    // stage2_evictAndCPUReq
    
    // Finish fill evictions and request lookups for any load/stores.
    
    // Ports Read:
    // * loadReqFromCPU
    
    // Ports Written:
    // * None

    rule stage2_evictAndCPUReq (True);

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // See if we started an eviction in the previous stage.
        if (local_state.writePortUsed)
        begin

            let m_evict <- llcAlg.evictionCheckRsp(cpu_iid);

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

                // We finished the fill succesfully with no writeback, so dequeue it and free the miss.
                debugLog.record(cpu_iid, $format("2: CLEAN EVICTION"));
                outstandingMisses.free(cpu_iid, local_state.missTokToFree);
                rspFromCC.doDeq(cpu_iid);

            end

            // Note that the actual cache update will be done later, so that
            // any lookups this model cycle don't see it accidentally.

        end
        else
        begin
        
            // No dequeue.
            rspFromCC.noDeq(cpu_iid);
        
        end

        // Now read the input port.
        let m_core_req <- reqFromCore.receive(cpu_iid);

        // Deal with any load/store requests.
        if (m_core_req matches tagged Valid .req)
        begin

            // See if the cache algorithm hit or missed.
            llcAlg.loadLookupReq(cpu_iid, req.physicalAddress);
            debugLog.record(cpu_iid, $format("2: REQ: LINE: 0x%h", req.physicalAddress));

            // Finish the request in the next stage.
            local_state.loadReq = tagged Valid req;
            
        end
        else
        begin

            debugLog.record(cpu_iid, $format("2: NO REQ"));

        end
        
        // Pass our information to the next stage.
        stage3Ctrl.ready(cpu_iid, local_state);

    endrule
    
    
    // stage3_cpuRspStoreReq
    
    // Finish up any load/stores to see if they hit or miss.
    // Begin handling any store requests.
    
    // Ports Read:
    // * storeReqFromCPU
    
    // Ports Written:
    // * loadRspImmToCPU

    rule stage3_cpuRspCCReq (!isValid(stage3Stall));

        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage3Ctrl.nextReadyInstance();

        // Get our local state from the pools.
        LUTRAM#(LLC_MISS_ID, MEM_OPAQUE) opaques = opaquesPool.getRAM(cpu_iid);

        // See if we need to finish any cpu responses.
        if (local_state.loadReq matches tagged Valid .req)
        begin

            // Get the lookup response.
            let m_entry <- llcAlg.loadLookupRsp(cpu_iid);
            stage3Stall <= tagged Valid tuple4(cpu_iid, req, m_entry, local_state);
        
        end
        else
        begin

            // Propogate the bubble.
            reqFromCore.noDeq(cpu_iid);
            eventHit.recordEvent(cpu_iid, tagged Invalid);
            eventMiss.recordEvent(cpu_iid, tagged Invalid);
            stage4Ctrl.ready(cpu_iid, local_state);

        end

    endrule
    
    (* conservative_implicit_conditions *)
    rule stage3_STALL (stage3Stall matches tagged Valid {.cpu_iid, .req, .m_entry, .ls});
    
        // Get our local state from the pools.
        LUTRAM#(LLC_MISS_ID, MEM_OPAQUE) opaques = opaquesPool.getRAM(cpu_iid);

        let local_state = ls;

        //
        // All events here will use the low bit to indicate whether the operation
        // is a load (0) or a store (1).  The remainer of the event data is
        // whatever fits from the low bits of the PA.
        //
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;

        if (m_entry matches tagged Valid .entry)
        begin

            if (req.isStore)
            begin

                if (!local_state.writePortUsed)
                begin

                    // We're writeback, so we don't need the memQ,
                    // we can just overwrite the line.
                    // Note that we don't need to do an eviction check since
                    // we hit, so we'll just overwrite the existing value.
                    // In other words, the writes will be coalesced and only
                    // one writeback to memory will occur.

                    local_state.writePortUsed = True;
                    local_state.writePortData = req.physicalAddress;
                    local_state.writeDataDirty = True;

                    // No response to a store. Don't change the coreQData in case there was a fill.
                    statWriteHit.incr(cpu_iid);
                    evt_hit = tagged Valid resize({ req.physicalAddress, 1'b1 });
                    debugLog.record(cpu_iid, $format("3: STORE HIT"));
                    reqFromCore.doDeq(cpu_iid);
                
                end
                else
                begin
                
                    // The store must retry because a fill happened.
                    statWriteRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: STORE HIT RETRY"));
                    reqFromCore.noDeq(cpu_iid);
                
                end
                
            end
            else if (local_state.coreQNotFull && !local_state.coreQUsed)
            begin

                // A load hit, so give the data back. We won't need the memory queue.
                local_state.coreQData = initMemRsp(req.physicalAddress, req.opaque);
                local_state.coreQUsed = True;
                statReadHit.incr(cpu_iid);
                evt_hit = tagged Valid resize({ req.physicalAddress, 1'b0 });
                debugLog.record(cpu_iid, $format("3: LOAD HIT"));
                reqFromCore.doDeq(cpu_iid);

            end
            else
            begin
            
                // A load hit, but the port is already in use, or the queue is full, so retry.
                statReadRetry.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: LOAD HIT RETRY"));
                reqFromCore.noDeq(cpu_iid);

            end

        end
        else
        begin
            if (req.isStore)
            begin
                if (outstandingMisses.canAllocateStore(cpu_iid) && memQAvailable(local_state))
                begin
                    
                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateStore(cpu_iid);

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    // Note that we use a load to simulate getting exclusive access.
                    let mem_req = initMemLoad(req.physicalAddress);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    local_state.memQData = mem_req;

                    // A miss, so no response. (Don't change the response in case there's an existing fill)
                    //statWriteMiss.incr(cpu_iid);
                    evt_miss = tagged Valid resize({ req.physicalAddress, 1'b1 });
                    debugLog.record(cpu_iid, $format("3: STORE MISS: %0d", miss_tok.index));
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
            
                // A load miss. But do we have a free missID to track the fill with?
                // And is the memQ not full and free for us to use?
                if (outstandingMisses.canAllocateLoad(cpu_iid) && memQAvailable(local_state))
                begin

                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateLoad(cpu_iid, req.physicalAddress);

                    // Record the original opaque for returning.
                    opaques.upd(missTokIndex(miss_tok), req.opaque);

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    let mem_req = initMemLoad(req.physicalAddress);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    local_state.memQData = mem_req;

                    // A miss, so no response. (Don't change the response in case there's an existing fill)
                    statReadMiss.incr(cpu_iid);
                    evt_miss = tagged Valid resize({ req.physicalAddress, 1'b0 });
                    debugLog.record(cpu_iid, $format("3: LOAD MISS: %0d", miss_tok.index));
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
        
        end // cache miss
        
        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage3Stall <= tagged Invalid;
        stage4Ctrl.ready(cpu_iid, local_state);
        
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage4_end (True);
    
        match {.cpu_iid, .local_state} <- stage4Ctrl.nextReadyInstance();

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
        
            llcAlg.allocate(cpu_iid, local_state.writePortData, local_state.writeDataDirty, 0);
        
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
        localCtrl.endModelCycle(cpu_iid, 1); 

    endrule


endmodule

`define LANE_LLC_REQ 0
`define LANE_LLC_RSP 1

module [HASIM_MODULE] mkCacheCoherenceInterface();

    // Queues to/from last level cache.
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, LLC_CC_REQ) reqToLLC   <- mkPortStallSend_Multiplexed("CC_to_LLC_req");
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, CC_REQ)     reqFromLLC <- mkPortStallRecv_Multiplexed("LLC_to_CC_req");
    
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, LLC_CC_RSP) rspFromLLC <- mkPortStallRecv_Multiplexed("LLC_to_CC_rsp");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, CC_RSP)     rspToLLC   <- mkPortStallSend_Multiplexed("CC_to_LLC_rsp");
    
    // Interface to OCN looks like lanes and virtual channels.   
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqFromOCN    <- mkPortRecv_Multiplexed("CoreMemInQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqToOCN      <- mkPortSend_Multiplexed("CoreMemOutQ_enq");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditFromOCN <- mkPortRecv_Multiplexed("CoreMemInQ_credit", 1);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditToOCN   <- mkPortSend_Multiplexed("CoreMemOutQ_credit");

    Vector#(6, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = reqToLLC.ctrl.in;
    inctrls[1]  = reqFromLLC.ctrl.in;
    inctrls[2]  = rspFromLLC.ctrl.in;
    inctrls[3]  = rspToLLC.ctrl.in;
    inctrls[4]  = enqFromOCN.ctrl;
    inctrls[5]  = creditFromOCN.ctrl;
    outctrls[0]  = reqToLLC.ctrl.out;
    outctrls[1]  = reqFromLLC.ctrl.out;
    outctrls[2]  = rspFromLLC.ctrl.out;
    outctrls[3]  = rspToLLC.ctrl.out;
    outctrls[4]  = enqToOCN.ctrl;
    outctrls[5]  = creditToOCN.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER#(NUM_CPUS, Maybe#(OCN_MSG)) stage3Ctrl <- mkStageController();

    MULTIPLEXED_REG#(NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCreditsPool  <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputNotFullsPool <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRspPool <- mkMultiplexedReg(tagged Invalid);
    MULTIPLEXED_REG#(NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReqPool <- mkMultiplexedReg(tagged Invalid);
    MEMORY_IFC_MULTIPLEXED#(NUM_CPUS, MEM_OPAQUE, LINE_ADDRESS)     physAddrPool       <- mkMemory_Multiplexed(mkBRAMInitialized(~0));

    function Maybe#(VC_IDX) vcToEnq(INSTANCE_ID#(NUM_CPUS) cpu_iid, LANE_IDX ln);
    
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(cpu_iid);
        Maybe#(VC_IDX) res = tagged Invalid;
        
        for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
        begin
            res = (notFulls[ln][vc]) ? tagged Valid fromInteger(vc) : res;
        end
        
        return res;
    
    endfunction
    
    function STATION_ID getDst(LINE_ADDRESS addr);
    
        // TODO: have home nodes for caches?
        // For now just send everything to the memory controller.
        return 0; // XXX
    
    endfunction

    (* conservative_implicit_conditions *)
    rule stage1_updateCredits (True);

        let cpu_iid <- localCtrl.startModelCycle();
        let m_credit <- creditFromOCN.receive(cpu_iid);
        
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(cpu_iid);
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCredits = outputCreditsPool.getReg(cpu_iid);
        
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_not_fulls = notFulls;
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_credits   = outputCredits;
        
        if (m_credit matches tagged Valid .creds)
        begin
        
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                
                    match {.credit, .not_full} = creds[ln][vc];
                    new_not_fulls[ln][vc] = not_full;
                    new_credits[ln][vc] = credit;
                
                end
                
            end
        
        end
        
        notFulls <= new_not_fulls;
        outputCredits <= new_credits;
        
        stage2Ctrl.ready(cpu_iid);
    
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage2_LLCReq (True);
    
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRsp = packetizingRspPool.getReg(cpu_iid);
        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReq = packetizingReqPool.getReg(cpu_iid);

        // Start by checking for new responses from the LLC.
        // These are higher priority.
        let m_llc_rsp <- rspFromLLC.receive(cpu_iid);

        // Also check for new ShReq/ExcReq from the LLC.
        let m_llc_req <- reqFromLLC.receive(cpu_iid);

        if (packetizingRsp matches tagged Valid {.op, .vc_idx})
        begin
        
            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(cpu_iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Invalid;
            rspFromLLC.noDeq(cpu_iid);
            reqFromLLC.noDeq(cpu_iid);

        end
        else if (packetizingReq matches tagged Valid {.op, .vc_idx})
        begin

            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(cpu_iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            packetizingReq <= tagged Invalid;
            rspFromLLC.noDeq(cpu_iid);
            reqFromLLC.noDeq(cpu_iid);

        end
        else if (m_llc_rsp matches tagged Valid .rsp &&& vcToEnq(cpu_iid, `LANE_LLC_RSP) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(cpu_iid), dst: getDst(rsp.physicalAddress), isStore: False};
            enqToOCN.send(cpu_iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Valid tuple2(rsp.opaque, vc_idx);
            rspFromLLC.doDeq(cpu_iid);
            reqFromLLC.noDeq(cpu_iid);

        end
        else if (m_llc_req matches tagged Valid .req &&& vcToEnq(cpu_iid, `LANE_LLC_REQ) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(cpu_iid), dst: getDst(req.physicalAddress), isStore: req.isStore};
            packetizingReq <= tagged Valid tuple2(req.opaque, vc_idx);
            enqToOCN.send(cpu_iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            rspFromLLC.noDeq(cpu_iid);
            reqFromLLC.doDeq(cpu_iid);
            if (!req.isStore)
            begin
                physAddrPool.write(cpu_iid, req.opaque, req.physicalAddress);
            end

        end
        else
        begin

            enqToOCN.send(cpu_iid, tagged Invalid);
            rspFromLLC.noDeq(cpu_iid);
            reqFromLLC.noDeq(cpu_iid);
        
        end
        
        // Route enqueues from the OCN to the correct place. This ignores
        // virtual channels - just lanes.
        let m_enq <- enqFromOCN.receive(cpu_iid);

        // Need to read the PA associated with an incoming message?  Stage 3
        // is separate from this step to wait for the BRAM read.
        if (m_enq matches tagged Valid {.ln, .vc_idx, .msg} &&&
            ln == `LANE_LLC_RSP &&&
            msg matches tagged FLIT_BODY .info)
        begin
            physAddrPool.readReq(cpu_iid, info.opaque);
        end
        else
        begin
            // Trigger a read request just so one is outstanding.  It is needed
            // for the schedule, but not the data.
            physAddrPool.readReq(cpu_iid, unpack(0));
        end

        stage3Ctrl.ready(cpu_iid, m_enq);

    endrule

    (* conservative_implicit_conditions *)
    rule stage3_LLCRsp (True);
    
        match {.cpu_iid, .m_enq} <- stage3Ctrl.nextReadyInstance();

        let pa <- physAddrPool.readRsp(cpu_iid);

        let can_enq_req <- reqToLLC.canEnq(cpu_iid);
        let can_enq_rsp <- rspToLLC.canEnq(cpu_iid);
        
        if (m_enq matches tagged Valid {.ln, .vc_idx, .msg})
        begin
        
            if (ln == `LANE_LLC_REQ)
            begin

                // assert can_enq_req
                case (msg) matches
                    tagged FLIT_HEAD .info:
                    begin
                        // Drop heads at this point.
                        reqToLLC.noEnq(cpu_iid);
                        rspToLLC.noEnq(cpu_iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.doEnq(cpu_iid, LLC_CC_REQ_WB); //TODO: actually distinguish.
                       rspToLLC.noEnq(cpu_iid);
                    end
                endcase

            end
            else if (ln == `LANE_LLC_RSP)
            begin
            
                // assert can_enq_rsp
                case (msg) matches
                    tagged FLIT_HEAD .info:
                    begin
                        // Drop heads at this point.
                        reqToLLC.noEnq(cpu_iid);
                        rspToLLC.noEnq(cpu_iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.noEnq(cpu_iid);
                       rspToLLC.doEnq(cpu_iid, initMemRsp(pa, info.opaque));
                    end

                endcase

            end
            else
            begin

                reqToLLC.noEnq(cpu_iid);
                rspToLLC.noEnq(cpu_iid);

            end

        end
        else
        begin
        
            reqToLLC.noEnq(cpu_iid);
            rspToLLC.noEnq(cpu_iid);
        
        end
        
        VC_CREDIT_INFO creds = newVector();
        creds[`LANE_LLC_REQ] = newVector();
        creds[`LANE_LLC_RSP] = newVector();

        for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
        begin

            let have_credit_req = can_enq_req; // XXX capacity - occupancy > round-trip latency.
            let not_full_req = can_enq_req;
            creds[`LANE_LLC_REQ][vc] = tuple2(have_credit_req, not_full_req);

            let have_credit_rsp = can_enq_rsp; // XXX capacity - occupancy > round-trip latency.
            let not_full_rsp = can_enq_rsp;
            creds[`LANE_LLC_RSP][vc] = tuple2(have_credit_rsp, not_full_rsp);

        end
        
        creditToOCN.send(cpu_iid, tagged Valid creds);
        localCtrl.endModelCycle(cpu_iid, 0);
        
    endrule

endmodule
