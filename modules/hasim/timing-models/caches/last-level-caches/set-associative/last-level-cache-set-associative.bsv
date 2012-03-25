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
`include "asim/provides/hasim_interconnect.bsh"
`include "asim/provides/hasim_last_level_cache_alg.bsh"

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


// DC_LOCAL_STATE
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

    Maybe#(DCACHE_LOAD_INPUT)  loadReq;
    
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

function Bool memQAvailable(DC_LOCAL_STATE local_state);

    return local_state.memQNotFull && !local_state.memQUsed;

endfunction


module [HASIM_MODULE] mkLastLevelCache();

    // Make an interface to the cache coherence protocol.
    let ccifc <- mkCacheCoherenceInterface();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(NUM_CPUS, VOID) dCacheAlg <- mkLastLevelCacheAlg();

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(NUM_CPUS, LLC_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();

    // A RAM To map our miss IDs into the original opaques, that we return to higher levels.
    MULTIPLEXED_LUTRAM#(LLC_MISS_ID, MEM_OPAQUE) opaques <- mkMultiplexedLUTRAM(?);

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

    STAGE_CONTROLLER#(NUM_CPUS, DC_LOCAL_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, DC_LOCAL_STATE) stage3Ctrl <- mkStageController();

    // ****** Stats ******

    STAT_VECTOR#(NUM_CPUS) statReadHit <-
        mkStatCounter_Multiplexed(statName("L1_LLC_READ_HIT", "LLC Read Hits"));
    STAT_VECTOR#(NUM_CPUS) statReadMiss <-
        mkStatCounter_Multiplexed(statName("L1_LLC_READ_MISS", "LLC Read Misses"));
    STAT_VECTOR#(NUM_CPUS) statReadRetry <-
        mkStatCounter_Multiplexed(statName("L1_LLC_READ_RETRY", "LLC Read Retries"));
    STAT_VECTOR#(NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("L1_LLC_WRITE_HIT", "LLC Write Hits"));
    STAT_VECTOR#(NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("L1_LLC_WRITE_RETRY", "LLC Write Retries"));


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
        let can_enq_cc_req <- reqToCC.canEnq(iid);
        let can_enq_cc_rsp <- rspToCC.canEnq(iid);
        let can_enq_core_rsp <- rspToCore.canEnq(iid);
        local_state.memQNotFull = can_enq_cc_req;
        local_state.coreQNotFull = can_enq_core_rsp;
        
        // Now check for responses from the cache coherence engine.
        let m_cc_rsp <- rspFromCC.receive(iid);

        // Also check for new requests from the cache coherence engine.
        let m_cc_req <- reqFromCC.receive(iid);

        // Unused by LLC. Should be handling invalidation writebacks.
        rspToCC.noEnq(iid);

        // LLC drops invalidates at this point. 
        // TODO: They should be passed on to L1C via SEPARATE fifos.
        if (m_cc_req matches tagged Valid .req)
        begin
            reqFromCC.doDeq();
        end
        else
        begin
            reqFromCC.noDeq();
        end

        if (m_cc_rsp matches tagged Valid .rsp)
        begin

            if (local_state.coreQnotFull)
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
                L1_DCACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = miss_tok;

                // Return the fill to higher levels.
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP: %0d, LINE: 0x%h", miss_tok.index, fill.physicalAddress));
                // Replace the opaque with the one for higher levels.
                fill.memRsp = opaques.sub(miss_tok);

                local_state.coreQData = fill;
                local_state.coreQUsed = True;

                // See if our allocation will evict a dirty line for writeback.
                // This check will be finished in the following stage.
                let fill_addr = fill.physicalAddress;
                llcAlg.evictionCheckReq(cpu_iid, fill_addr);

            end
            else
            begin
                debugLog.record_next_cycle(cpu_iid, $format("1: MEM RSP RETRY: %0d, LINE: 0x%h", miss_tok.index, fill.physicalAddress));
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
                    local_state.rspToCPU = tagged Invalid;
                
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
        let m_core_req <- reqFromCore.receive(iid);

        // Deal with any load/store requests.
        if (m_core_req matches tagged Valid .req)
        begin

            // See if the cache algorithm hit or missed.
            let line_addr = toLineAddress(req.physicalAddress);
            llcAlg.loadLookupReq(cpu_iid, line_addr);
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

    rule stage3_cpuRspCCReq (True);

        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage3Ctrl.nextReadyInstance();

        // Get our local state from the pools.
        LUTRAM#(LLC_MISS_ID, MEM_OPAQUE) opaques = opaquesPool.getRAM(cpu_iid);

        // See if we need to finish any cpu responses.
        if (local_state.loadReq matches tagged Valid .req)
        begin

            // Get the lookup response.
            let m_entry <- llcAlg.loadLookupRsp(cpu_iid);

            // Does the cache contain this addresss? And have we not already made a fill response?
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
                        debugLog.record(cpu_iid, $format("3: STORE HIT"));
                        reqFromCore.doDeq();
                    
                    end
                    else
                    begin
                    
                        // The store must retry because a fill happened.
                        statWriteRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: STORE HIT RETRY");
                        reqFromCore.noDeq();
                    
                    end
                    
                end
                else if (local_state.coreQNotFull && !local_state.coreQUsed)
                begin

                    // A load hit, so give the data back. We won't need the memory queue.
                    local_state.coreQData = initMemRsp(req.physicalAddress, req.opaque);
                    local_state.coreQUsed = True;
                    statReadHit.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD HIT"));
                    reqFromCore.doDeq();

                end
                else
                begin
                
                    // A load hit, but the port is already in use, or the queue is full, so retry.
                    statReadRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD HIT RETRY"));
                    reqFromCore.noDeq();

                end

            end
            else
            begin

                // A miss. But do we have a free missID to track the fill with?
                // And is the memQ not full and free for us to use?
                if (outstandingMisses.canAllocateLoad(cpu_iid) && memQAvailable(local_state))
                begin

                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateLoad(cpu_iid, toLineAddress(req.physicalAddress));
                    
                    // Record the original opaque for returning.
                    opaques.upd(miss_tok, req.opaque);

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    let line_addr = toLineAddress(req.physicalAddress);
                    let mem_req = initMemLoad(line_addr);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    local_state.memQData = mem_req;

                    // A miss, so no response. (Don't change the response in case there's an existing fill)
                    statReadMiss.incr(cpu_iid);
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
        end
        else
        begin

            // Propogate the bubble.
            reqFromCore.noDeq(cpu_iid, tagged Invalid);

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

    MULTIPLEXED_REG#(NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCreditsPool  <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputNotFullsPool <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRspPool <- mkMultiplexedReg(tagged Invalid);
    MULTIPLEXED_REG#(NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReqPool <- mkMultiplexedReg(tagged Invalid);
    MULTIPLEXED_LUTRAM#(NUM_CPUS, MEM_OPAQUE, LINE_ADDRESS)         physAddrPool       <- mkMultiplexedLUTRAM(~0);

    function Maybe#(VC_IDX) vcToEnq(INSTANCE_ID#(NUM_CPUS) iid, LANE_IDX ln);
    
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(iid);
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

        let iid <- localCtrl.startModelCycle();
        let m_credit <- creditFromOCN.receive(iid);
        
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(iid);
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCredits = outputCreditsPool.getReg(iid);
        
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
        
        stage2Ctrl.ready(iid);
    
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage2_LLCReq (True);
    
        let iid <- stage2Ctrl.nextReadyInstance();

        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRsp = packetizingRspPool.getReg(iid);
        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReq = packetizingReqPool.getReg(iid);
        LUTRAM#(MEM_OPAQUE, LINE_ADDRESS)         physAddr       = physAddrPool.getRAM(iid);

        // Start by checking for new responses from the LLC.
        // These are higher priority.
        let m_llc_rsp <- rspFromLLC.receive(iid);

        // Also check for new ShReq/ExcReq from the LLC.
        let m_llc_req <- reqFromLLC.receive(iid);

        if (packetizingRsp matches tagged Valid {.op, .vc_idx})
        begin
        
            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Invalid;
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (packetizingReq matches tagged Valid {.op, .vc_idx})
        begin

            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            packetizingReq <= tagged Invalid;
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (m_llc_rsp matches tagged Valid .rsp &&& vcToEnq(iid, `LANE_LLC_RSP) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(iid), dst: getDst(rsp.physicalAddress), isStore: False};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Valid tuple2(rsp.opaque, vc_idx);
            rspFromLLC.doDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (m_llc_req matches tagged Valid .req &&& vcToEnq(iid, `LANE_LLC_REQ) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(iid), dst: getDst(req.physicalAddress), isStore: req.isStore};
            packetizingReq <= tagged Valid tuple2(req.opaque, vc_idx);
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            rspFromLLC.noDeq(iid);
            reqFromLLC.doDeq(iid);
            physAddr.upd(req.opaque, req.physicalAddress);

        end
        else
        begin

            enqToOCN.send(iid, tagged Invalid);
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);
        
        end
        
        // Route enqueues from the OCN to the correct place. This ignores virtual channels - just lanes.
        let m_enq <- enqFromOCN.receive(iid);

        let can_enq_req <- reqToLLC.canEnq(iid);
        let can_enq_rsp <- rspToLLC.canEnq(iid);
        
        if (m_enq matches tagged Valid {.ln, .vc_idx, .msg})
        begin
        
            if (ln == `LANE_LLC_REQ)
            begin

                // assert can_enq_req
                case (msg) matches
                    tagged FLIT_HEAD .info:
                    begin
                        // Drop heads at this point.
                        reqToLLC.noEnq(iid);
                        rspToLLC.noEnq(iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.doEnq(iid, LLC_CC_REQ_WB); //TODO: actually distinguish.
                       rspToLLC.noEnq(iid);
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
                        reqToLLC.noEnq(iid);
                        rspToLLC.noEnq(iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.noEnq(iid);
                       rspToLLC.doEnq(iid, initMemRsp(physAddr.sub(info.opaque), info.opaque));
                    end

                endcase

            end
            else
            begin

                reqToLLC.noEnq(iid);
                rspToLLC.noEnq(iid);

            end

        end
        else
        begin
        
            reqToLLC.noEnq(iid);
            rspToLLC.noEnq(iid);
        
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
        
        creditToOCN.send(iid, tagged Valid creds);
        localCtrl.endModelCycle(iid, 0);
        
    endrule

endmodule
