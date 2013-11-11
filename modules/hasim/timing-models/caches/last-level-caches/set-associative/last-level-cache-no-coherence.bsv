import Vector::*;

// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/fpga_components.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_chip_topology.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_last_level_cache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"
`include "awb/provides/hasim_interconnect_common.bsh"


// ******* Generated File Imports *******

`include "awb/dict/EVENTS_LLC.bsh"
`include "awb/dict/TOPOLOGY.bsh"


// ****** Local Definitions *******

typedef struct
{
    STATION_ID src;
    MEMORY_REQ req;
}
LLC_REQ
    deriving (Eq, Bits);

typedef struct
{
    STATION_ID dst;
    MEMORY_RSP rsp;
}
LLC_RSP
    deriving (Eq, Bits);


typedef enum
{
    LLC_CC_REQ_WB,
    LLC_CC_REQ_INVALIDATE
}
LLC_CC_REQ deriving (Eq, Bits);

typedef MEMORY_REQ LLC_CC_RSP;

typedef MEMORY_REQ CC_REQ;
typedef MEMORY_RSP CC_RSP;


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



//
// mkLastLevelCache --
//   The primary routing module.  This module takes in requests from the
//   core and routes them either to the local copy of the distributed
//   cache or across the OCN.  It also manages coherence messages from the
//   local cache to other distributed instances and messages arriving
//   from the OCN to the local cache.
//
//   Very little work is done here.  It is just a stage for passing
//   messages to the right places.
//
module [HASIM_MODULE] mkLastLevelCache();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc.out");

    // Instantiate other LLC components
    let llc_distrib <- mkDistributedLastLevelCache();
    let llc_ocn <- mkLLCNetworkConnection();

    //
    // Requests from the local core arrive here.  This router will forward
    // the request to the correct distributed LLC segment.
    //
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqFromCore <-
        mkPortStallRecv_Multiplexed("CorePvtCache_to_UncoreQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspToCore <-
        mkPortStallSend_Multiplexed("Uncore_to_CorePvtCacheQ");
    
    //
    // Queues to/from local LLC segment.  These may either come from the local
    // core or a remote core.
    //
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToLLC <-
        mkPortStallSend_Multiplexed("CC_to_LLC_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspFromLLC <-
        mkPortStallRecv_Multiplexed("LLC_to_CC_rsp");
    
    //
    // Messages that travel from this portion of the distrubted LLC to/from
    // memory controllers.  This module will route the messages through the
    // network.
    //
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CC_REQ) cacheToMem <-
        mkPortStallRecv_Multiplexed("LLC_to_MEM_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CC_RSP) memToCache <-
        mkPortStallSend_Multiplexed("MEM_to_LLC_rsp");

    //
    // Ports from the OCN to the LLC.
    //
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqFromOCN <-
        mkPortStallRecv_Multiplexed("OCN_to_LLC_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspFromOCN <-
        mkPortStallRecv_Multiplexed("OCN_to_LLC_rsp");

    //
    // Ports from the LLC to the OCN.
    //
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 Tuple2#(STATION_ID, MEMORY_REQ)) reqToOCN <-
        mkPortStallSend_Multiplexed("LLC_to_OCN_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 Tuple2#(STATION_ID, MEMORY_RSP)) rspToOCN <-
        mkPortStallSend_Multiplexed("LLC_to_OCN_rsp");


    Vector#(10, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = reqFromCore.ctrl.in;
    inctrls[1] = rspToCore.ctrl.in;
    inctrls[2] = reqToLLC.ctrl.in;
    inctrls[3] = rspFromLLC.ctrl.in;
    inctrls[4] = cacheToMem.ctrl.in;
    inctrls[5] = memToCache.ctrl.in;
    inctrls[6] = reqFromOCN.ctrl.in;
    inctrls[7] = rspFromOCN.ctrl.in;
    inctrls[8] = reqToOCN.ctrl.in;
    inctrls[9] = rspToOCN.ctrl.in;

    Vector#(10, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqToLLC.ctrl.out;
    outctrls[3] = rspFromLLC.ctrl.out;
    outctrls[4] = cacheToMem.ctrl.out;
    outctrls[5] = memToCache.ctrl.out;
    outctrls[6] = reqFromOCN.ctrl.out;
    outctrls[7] = rspFromOCN.ctrl.out;
    outctrls[8] = reqToOCN.ctrl.out;
    outctrls[9] = rspToOCN.ctrl.out;


    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("LLC Coherence", inctrls, outctrls);
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(LANE_IDX)) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple2#(LANE_IDX, OCN_FLIT))) stage3Ctrl <- mkStageController();

    //
    // Map core ID to network station ID.  This table is computed by the
    // software-side topology manager.
    //
    let coreToStationMapInit <-
        mkTopologyParamStream(`TOPOLOGY_NET_CORE_STATION_ID_MAP);
    // The table holds 8 entries for every memory controller.
    LUTRAM#(CPU_INSTANCE_ID, STATION_ID) coreToStationMap <-
        mkLUTRAMWithGet(coreToStationMapInit);

    //
    // Physical addresses are assigned to memory controllers during setup
    // by software.  The map table is larger than the number of controllers
    // in order to enable relatively even mapping even when the number of
    // controllers isn't a power of two.  A large map also makes it
    // unnecessary to hash the addresses.
    //

    let ctrlAddrMapInit <- mkTopologyParamStream(`TOPOLOGY_NET_MEM_CTRL_MAP);
    // The table holds 8 entries for every memory controller.
    LUTRAM#(Bit#(TLog#(TMul#(8, MAX_NUM_MEM_CTRLS))),
            STATION_ID) memCtrlDstForAddr <-
        mkLUTRAMWithGet(ctrlAddrMapInit);

    function STATION_ID getMemCtrlDstForAddr(LINE_ADDRESS addr);
        return memCtrlDstForAddr.sub(resize(addr));
    endfunction


    //
    // The LLC is distributed across all cores, with each address having a
    // single entry in a particular LLC.  We use a mapping table, similar
    // to the memory controller table above.  The table is initialized
    // by software.
    //
    
    let llcAddrMapInit <- mkTopologyParamStream(`TOPOLOGY_NET_LLC_ADDR_MAP);
    // The table holds 8 entries for every cache instance.
    LUTRAM#(Bit#(TLog#(TMul#(8, MAX_NUM_CPUS))), STATION_ID) llcDstForAddr <-
        mkLUTRAMWithGet(llcAddrMapInit);

    function STATION_ID getLLCDstForAddr(LINE_ADDRESS addr);
        return llcDstForAddr.sub(resize(addr));
    endfunction


    (* conservative_implicit_conditions *)
    rule stage1_routing (True);
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        let station_iid = coreToStationMap.sub(cpu_iid);

        //
        // Collect incoming messages.  The "receive" operation here is not a
        // commitment to process a message.  For that, doDeq() must be called.
        //
        let m_reqFromCore <- reqFromCore.receive(cpu_iid);
        let m_rspFromLLC <- rspFromLLC.receive(cpu_iid);
        let m_cacheToMem <- cacheToMem.receive(cpu_iid);
        let m_reqFromOCN <- reqFromOCN.receive(cpu_iid);
        let m_rspFromOCN <- rspFromOCN.receive(cpu_iid);


        // Check credits for sending to output ports.
        let can_enq_rspToCore <- rspToCore.canEnq(cpu_iid);
        let can_enq_reqToLLC <- reqToLLC.canEnq(cpu_iid);
        let can_enq_memToCache <- memToCache.canEnq(cpu_iid);
        let can_enq_reqToOCN <- reqToOCN.canEnq(cpu_iid);
        let can_enq_rspToOCN <- rspToOCN.canEnq(cpu_iid);

        //
        // Make routing choices.
        //

        if (m_reqFromCore matches tagged Valid .req &&& can_enq_reqToLLC)
        begin
            reqFromCore.doDeq(cpu_iid);
            reqToLLC.doEnq(cpu_iid, req);
        end
        else
        begin
            reqFromCore.noDeq(cpu_iid);
            reqToLLC.noEnq(cpu_iid);
        end

        if (m_rspFromLLC matches tagged Valid .rsp &&& can_enq_rspToCore)
        begin
            rspFromLLC.doDeq(cpu_iid);
            rspToCore.doEnq(cpu_iid, rsp);
        end
        else
        begin
            rspFromLLC.noDeq(cpu_iid);
            rspToCore.noEnq(cpu_iid);
        end

        if (m_cacheToMem matches tagged Valid .req &&& can_enq_reqToOCN)
        begin
            cacheToMem.doDeq(cpu_iid);
            let dst = getMemCtrlDstForAddr(req.physicalAddress);
            reqToOCN.doEnq(cpu_iid, tuple2(dst, req));
        end
        else
        begin
            cacheToMem.noDeq(cpu_iid);
            reqToOCN.noEnq(cpu_iid);
        end

        if (m_rspFromOCN matches tagged Valid .rsp &&& can_enq_memToCache)
        begin
            rspFromOCN.doDeq(cpu_iid);
            memToCache.doEnq(cpu_iid, rsp);
        end
        else
        begin
            rspFromOCN.noDeq(cpu_iid);
            memToCache.noEnq(cpu_iid);
        end

        rspToOCN.noEnq(cpu_iid);
        if (m_reqFromOCN matches tagged Valid .req)
        begin
            reqFromOCN.doDeq(cpu_iid);
        end
        else
        begin
            reqFromOCN.noDeq(cpu_iid);
        end
        
        localCtrl.endModelCycle(cpu_iid, 0);
    endrule
endmodule


//
// mkDistributedLastLevelCache --
//   Cache management.  Each core has an associated portion of the distributed
//   LLC.
//
module [HASIM_MODULE] mkDistributedLastLevelCache();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc_distrib.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(MAX_NUM_CPUS, VOID) llcAlg <- mkLastLevelCacheAlg();

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, LLC_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();

    // A RAM To map our miss IDs into the original opaques, that we return to higher levels.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS, LLC_MISS_ID, LLC_MISS_TOKEN) opaquesPool <- mkMemory_Multiplexed(mkBRAM);

    // ****** Ports ******

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqFromCore <-
        mkPortStallRecv_Multiplexed("CC_to_LLC_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspToCore <-
        mkPortStallSend_Multiplexed("LLC_to_CC_rsp");
    
    // Requests to memory from the LLC instance responsible for an address.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CC_REQ) reqToCC <-
        mkPortStallSend_Multiplexed("LLC_to_MEM_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CC_RSP) rspFromCC <-
        mkPortStallRecv_Multiplexed("MEM_to_LLC_rsp");
    
    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = reqFromCore.ctrl.in;
    inctrls[1] = rspToCore.ctrl.in;
    inctrls[2] = reqToCC.ctrl.in;
    inctrls[3] = rspFromCC.ctrl.in;

    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqToCC.ctrl.out;
    outctrls[3] = rspFromCC.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("LLC", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(LLC_LOCAL_STATE, Bool)) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, LLC_LOCAL_STATE) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, LLC_LOCAL_STATE) stage4Ctrl <- mkStageController(); // XXX TMP
    Reg#(Maybe#(Tuple4#(CPU_INSTANCE_ID, MEMORY_REQ, Maybe#(CACHE_ENTRY#(VOID)), LLC_LOCAL_STATE))) stage3Stall <- mkReg(tagged Invalid); //XXX TMP

    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statReadHit <-
        mkStatCounter_Multiplexed(statName("LLC_READ_HIT",
                                           "LLC Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadMiss <-
        mkStatCounter_Multiplexed(statName("LLC_READ_MISS",
                                           "LLC Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadRetry <-
        mkStatCounter_Multiplexed(statName("LLC_READ_RETRY",
                                           "LLC Read Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("LLC_WRITE_HIT",
                                           "LLC Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("LLC_WRITE_RETRY",
                                           "LLC Write Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statFillRetry <-
        mkStatCounter_Multiplexed(statName("LLC_FILL_RETRY",
                                           "LLC Fill Retries"));

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_LLC_HIT);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_LLC_MISS);


    (* conservative_implicit_conditions *)
    rule stage1_fill (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = initLocalState();

        // Check if the CC engine has room for any new requests.
        let can_enq_cc_req <- reqToCC.canEnq(cpu_iid);
        let can_enq_core_rsp <- rspToCore.canEnq(cpu_iid);
        local_state.memQNotFull = can_enq_cc_req;
        local_state.coreQNotFull = can_enq_core_rsp;
        
        // Now check for responses from the cache coherence engine.
        let m_cc_rsp <- rspFromCC.receive(cpu_iid);

        Bool read_opaques = False;

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
        stage2Ctrl.ready(cpu_iid, tuple2(local_state, read_opaques));

    endrule

    // stage2_evictAndCPUReq
    
    // Finish fill evictions and request lookups for any load/stores.
    
    // Ports Read:
    // * loadReqFromCPU
    
    // Ports Written:
    // * None

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
                    mem_req.opaque = updateMemOpaque(req.opaque, miss_tok);
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
                    opaquesPool.write(cpu_iid, missTokIndex(miss_tok),
                                      fromMemOpaque(req.opaque));

                    // Record that we are using the memory queue.
                    local_state.memQUsed = True;

                    // Use the opaque bits to store the miss token.
                    let mem_req = initMemLoad(req.physicalAddress);
                    mem_req.opaque = updateMemOpaque(req.opaque, miss_tok);
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


//
// Lanes used for primary memory operations (load and store requests).  These
// may be requests from a core to the instance of the LLC distributed cache
// that handles the request's address.  They may also be requests from LLC
// instances to memory controllers.
//
`define LANE_MEMOP_REQ 0
`define LANE_MEMOP_RSP 1

//
// mkLLCNetworkConnection --
//   The lowest level connection between a core/LLC and the on-chip-network.
//   This module converts LLC messages used in the core and LLC into multi-flit
//   packets that traverse the network.
//
module [HASIM_MODULE] mkLLCNetworkConnection();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc_ocn_connection.out");

    //
    // Messages from the core and LLC destined for the OCN arrive here.
    // Requests and responses travel on separate OCN channels to avoid
    // deadlock, so each class gets a separate port.
    //
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 Tuple2#(STATION_ID, MEMORY_REQ)) reqFromLLC <-
        mkPortStallRecv_Multiplexed("LLC_to_OCN_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 Tuple2#(STATION_ID, MEMORY_RSP)) rspFromLLC <-
        mkPortStallRecv_Multiplexed("LLC_to_OCN_rsp");

    //
    // Messages from the OCN to an LLC.
    //
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToLLC <-
        mkPortStallSend_Multiplexed("OCN_to_LLC_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspToLLC <-
        mkPortStallSend_Multiplexed("OCN_to_LLC_rsp");

    //
    // Wrapped interfaces to/from the OCN.  The wrappers simplify
    // the protocol for credit management.
    //
    PORT_OCN_LOCAL_SEND_MULTIPLEXED#(MAX_NUM_CPUS) ocnSend <-
        mkLocalNetworkPortSend("CoreMemOutQ",
                               "CoreMemInQ",
                               debugLog);

    PORT_OCN_LOCAL_RECV_MULTIPLEXED#(MAX_NUM_CPUS) ocnRecv <-
        mkLocalNetworkPortRecv("CoreMemOutQ",
                               "CoreMemInQ",
                               debugLog);


    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = ocnSend.ctrl.in;
    inctrls[1] = ocnRecv.ctrl.in;
    inctrls[2] = reqFromLLC.ctrl.in;
    inctrls[3] = rspFromLLC.ctrl.in;
    inctrls[4] = reqToLLC.ctrl.in;
    inctrls[5] = rspToLLC.ctrl.in;

    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = ocnSend.ctrl.out;
    outctrls[1] = ocnRecv.ctrl.out;
    outctrls[2] = reqFromLLC.ctrl.out;
    outctrls[3] = rspFromLLC.ctrl.out;
    outctrls[4] = reqToLLC.ctrl.out;
    outctrls[5] = rspToLLC.ctrl.out;


    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("LLC Network Connection", inctrls, outctrls);
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(Bool, Maybe#(LANE_IDX))) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple3#(Bool,
                                            Maybe#(OCN_PACKET_HANDLE),
                                            Maybe#(Tuple2#(LANE_IDX, OCN_FLIT)))) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage4Ctrl <- mkBufferedStageController();

    //
    // Messages are broken into flits in this module when transmitted on the
    // OCN.  For now we always use packets that are two flits.  These
    // registers hold the tail flits after a head is transmitted.
    //
    Vector#(NUM_LANES, MULTIPLEXED_REG#(MAX_NUM_CPUS,
                                        Maybe#(OCN_FLIT_OPAQUE))) packetizingToOCNPool <-
        replicateM(mkMultiplexedReg(tagged Invalid));
    Vector#(NUM_LANES, MULTIPLEXED_REG#(MAX_NUM_CPUS,
                                        Maybe#(OCN_FLIT_OPAQUE))) packetizingFromOCNPool <-
        replicateM(mkMultiplexedReg(tagged Invalid));

    //
    // Side memories hold the actual contents of a packet instead of forcing all
    // datapaths in the simulated OCN to be wide enough to pass a full packet.
    //
    OCN_PACKET_PAYLOAD_CLIENT payloadStorage <- mkNetworkPacketPayloadClient(1);


    (* conservative_implicit_conditions *)
    rule stage1_sendToOCN (True);
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Check credits for sending to the network
        let can_enq <- ocnSend.canEnq(cpu_iid);

        // Remaining packets for all outbound lanes.
        Vector#(NUM_LANES, Reg#(Maybe#(OCN_FLIT_OPAQUE))) packetizingToOCN =
            map(getMultiplexedReg(cpu_iid), packetizingToOCNPool);

        //
        // Collect messages heading toward the OCN.  The "receive" operation
        // here is not a commitment to process a message.  For that, doDeq()
        // must be called.
        //
        let m_reqFromLLC <- reqFromLLC.receive(cpu_iid);
        let m_rspFromLLC <- rspFromLLC.receive(cpu_iid);

        //
        // Pick a winner.
        //

        Bool did_deq_reqFromLLC = False;
        Bool did_deq_rspFromLLC = False;
        Bool did_payload_storage_write = False;

        // Function that will be used to test whether a buffered flit remains
        // to be sent on a port.  The function returns true iff there is a
        // flit and the port can accept a message this cycle.
        function Bool oldFlitRemains(Tuple2#(Bool, Reg#(Maybe#(OCN_FLIT_OPAQUE))) state);
            match {.can_enq_port, .flit} = state;
            return can_enq_port && isValid(flit);
        endfunction

        if (findIndex(oldFlitRemains,
                      zip(can_enq, packetizingToOCN)) matches tagged Valid .lane)
        begin
            //
            // Complete an LLC response by sending an old flit.
            //
            let op = validValue(packetizingToOCN[lane]);
            let msg = tagged FLIT_BODY OCN_FLIT_BODY {opaque: op, isTail: True};
            ocnSend.doEnq(cpu_iid, pack(lane), msg);
            packetizingToOCN[lane] <= tagged Invalid;

            debugLog.record(cpu_iid, $format("1: Lane %0d: tail", lane));
        end
        else if (m_rspFromLLC matches tagged Valid {.dst, .rsp} &&&
                 can_enq[`LANE_MEMOP_RSP])
        begin
            //
            // Start a response.
            //
            let msg = tagged FLIT_HEAD OCN_FLIT_HEAD {src: zeroExtend(cpu_iid),
                                                      dst: dst,
                                                      isStore: False};
            ocnSend.doEnq(cpu_iid, `LANE_MEMOP_RSP, msg);
            packetizingToOCN[`LANE_MEMOP_RSP] <= tagged Valid zeroExtend(rsp.opaque);
            rspFromLLC.doDeq(cpu_iid);
            did_deq_rspFromLLC = True;

            debugLog.record(cpu_iid, $format("1: Lane %0d: Response to node %0d", `LANE_MEMOP_RSP, dst));
        end
        else if (m_reqFromLLC matches tagged Valid {.dst, .req} &&&
                 can_enq[`LANE_MEMOP_REQ] &&&
                 payloadStorage.allocNotEmpty)
        begin
            //
            // Start a request from the cache to a memory controller.  Note
            // that this fires only if there is a free entry in the payloadStorage
            // heap.  This is an artifical restriction.  Just make the heap
            // large enough.
            //
            let msg = tagged FLIT_HEAD OCN_FLIT_HEAD {src: zeroExtend(cpu_iid),
                                                      dst: dst,
                                                      isStore: req.isStore};
            ocnSend.doEnq(cpu_iid, `LANE_MEMOP_REQ, msg);
            reqFromLLC.doDeq(cpu_iid);
            did_deq_reqFromLLC = True;

            let h <- payloadStorage.allocHandle();
            payloadStorage.write(h, zeroExtend(pack(tuple2(req.opaque,
                                                           req.physicalAddress))));
            did_payload_storage_write = True;

            packetizingToOCN[`LANE_MEMOP_REQ] <= tagged Valid h;

            String ld_st = req.isStore ? "STORE" : "LOAD";
            debugLog.record(cpu_iid, $format("1: Lane %0d: Gen %s REQ for LINE 0x%x, ID %0d, handle 0x%x to memctrl node %0d", `LANE_MEMOP_REQ, ld_st, req.physicalAddress, req.opaque, h, dst));
        end
        else
        begin
            //
            // Nothing to send.
            //
            ocnSend.noEnq(cpu_iid);
        end

        if (! did_deq_rspFromLLC)
        begin
            rspFromLLC.noDeq(cpu_iid);
        end

        if (! did_deq_reqFromLLC)
        begin
            reqFromLLC.noDeq(cpu_iid);
        end

        
        //
        // Is a message available incoming from the OCN?  We start here
        // because it is a multi-cycle operation.
        //

        // Only request packets that can be forwarded this cycle.
        let can_enq_reqToLLC <- reqToLLC.canEnq(cpu_iid);
        let can_enq_rspToLLC <- rspToLLC.canEnq(cpu_iid);
        Vector#(NUM_LANES, Bool) can_enq_vec = replicate(False);
        can_enq_vec[`LANE_MEMOP_REQ] = can_enq_reqToLLC;
        can_enq_vec[`LANE_MEMOP_RSP] = can_enq_rspToLLC;

        Maybe#(LANE_IDX) recv_ln = tagged Invalid;
        if (ocnRecv.pickChannel(cpu_iid, can_enq_vec) matches
                tagged Valid {.ln_in, .vc_in})
        begin
            ocnRecv.receiveReq(cpu_iid, ln_in, vc_in);
            ocnRecv.doDeq(cpu_iid, ln_in, vc_in);
            recv_ln = tagged Valid ln_in;
        end
        else
        begin
            ocnRecv.noDeq(cpu_iid);
        end

        stage2Ctrl.ready(cpu_iid, tuple2(did_payload_storage_write, recv_ln));
    endrule


    rule stage2_lookupAddr (True);
        match {.cpu_iid,
               .did_payload_storage_write,
               .m_ln} <- stage2Ctrl.nextReadyInstance();

        Maybe#(OCN_PACKET_HANDLE) m_payload_storage_read = tagged Invalid;

        OCN_FLIT flit = ?;
        if (isValid(m_ln))
        begin
            flit <- ocnRecv.receiveRsp(cpu_iid);
        end

        // Need to read the PA associated with an incoming message?  Stage 3
        // is separate from this step to wait for the BRAM read.
        if (m_ln matches tagged Valid .ln &&&
            ln == `LANE_MEMOP_RSP &&&
            flit matches tagged FLIT_BODY .info)
        begin
            payloadStorage.readReq(info.opaque);
            m_payload_storage_read = tagged Valid info.opaque;
        end

        if (m_ln matches tagged Valid .ln)
        begin
            stage3Ctrl.ready(cpu_iid, tuple3(did_payload_storage_write,
                                             m_payload_storage_read,
                                             tagged Valid tuple2(ln, flit)));
        end
        else
        begin
            stage3Ctrl.ready(cpu_iid, tuple3(did_payload_storage_write,
                                             m_payload_storage_read,
                                             tagged Invalid));
        end
    endrule


    rule stage3_recvFromOCN (True);
        match {.cpu_iid,
               .did_payload_storage_write,
               .m_payload_storage_read,
               .m_flit} <- stage3Ctrl.nextReadyInstance();
        
        Bool did_enq_reqToLLC = False;
        Bool did_enq_rspToLLC = False;

        // Looking up payload contents in the side storage buffer?
        MEM_OPAQUE opaque = ?;
        LINE_ADDRESS pa = ?;
        if (m_payload_storage_read matches tagged Valid .h)
        begin
            let v <- payloadStorage.readRsp();
            Tuple2#(MEM_OPAQUE, LINE_ADDRESS) info = unpack(truncate(v));
            opaque = tpl_1(info);
            pa = tpl_2(info);
            
            // Packet complete.  Release the buffer entry.
            payloadStorage.freeHandle(h);
            debugLog.record(cpu_iid, $format("3: Free 0x%0x", h));
        end

        //
        // The head flit can be ignored.  All we need is in the body (tail).
        //
        if (m_flit matches tagged Valid {.ln, .flit})
        begin
            if (flit matches tagged FLIT_HEAD .info)
            begin
                debugLog.record(cpu_iid, $format("3: Lane %0d: Recv from node %0d", ln, info.src));
            end

            if (flit matches tagged FLIT_BODY .info)
            begin
                if (ln == `LANE_MEMOP_REQ)
                begin
                    reqToLLC.doEnq(cpu_iid, ?); //TODO: actually distinguish.
                    did_enq_reqToLLC = True;
                    debugLog.record(cpu_iid, $format("3: Lane %0d: Recv req tail", ln));
                end
                else if (ln == `LANE_MEMOP_RSP)
                begin
                    let rsp = initMemRsp(pa, opaque);
                    rspToLLC.doEnq(cpu_iid, rsp);
                    did_enq_rspToLLC = True;
                    debugLog.record(cpu_iid, $format("3: Lane %0d: Recv load rsp for LINE 0x%x, ID %0d", ln, rsp.physicalAddress, opaque));
                end
            end
        end

        if (! did_enq_reqToLLC)
        begin
            reqToLLC.noEnq(cpu_iid);
        end
        
        if (! did_enq_rspToLLC)
        begin
            rspToLLC.noEnq(cpu_iid);
        end
        
        stage4Ctrl.ready(cpu_iid, did_payload_storage_write);
    endrule


    //
    // Wait for confirmation that the payload storage buffer has been updated
    // and is visible to the recipient.
    //
    rule stage4_writeAck (True);
        match {.cpu_iid,
               .did_payload_storage_write} <- stage4Ctrl.nextReadyInstance();

        if (did_payload_storage_write)
        begin
            payloadStorage.writeAck();
        end

        localCtrl.endModelCycle(cpu_iid, 0);
    endrule
endmodule
