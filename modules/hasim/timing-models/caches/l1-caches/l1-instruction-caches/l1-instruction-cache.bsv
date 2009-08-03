// ******* Library Imports *******

import Vector::*;
import FIFO::*;


// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/platform_interface.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_memory.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"
`include "asim/provides/hasim_l1_icache_alg.bsh"
`include "asim/provides/hasim_miss_tracker.bsh"

// ******* Generated File Imports *******

`include "asim/dict/STATS_L1_ICACHE.bsh"


// ****** Local Definitions *******


// IC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Bool writePortUsed;
    LINE_ADDRESS writePortData;
    Maybe#(ICACHE_INPUT) loadReq;
    
}
IC_LOCAL_STATE deriving (Eq, Bits);


// initLocalState
//
// A fresh local state for the first stage.

function IC_LOCAL_STATE initLocalState();

    return 
        IC_LOCAL_STATE 
        { 
            writePortUsed: False,
            writePortData: 0,
            loadReq: tagged Invalid
        };

endfunction


// mkL11Cache

// A model of a straightforward L1 ICache.
// There is no victim buffer.
//
// Note that the module itself is implmented as a pipeline, though the target
// model carries out all actions in one model cycle.

module [HASIM_MODULE] mkL1ICache ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_instruction.out");

 
    // ****** Submodules ******

    // The cache algorithm which determines hits, misses, and evictions.
    CACHE_ALG#(NUM_CPUS, VOID) iCacheAlg <- mkL1ICacheAlg();

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(NUM_CPUS, ICACHE_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_ICache_load", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_immediate");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_delayed");

    // Queues to and from the memory hierarchy, encapsulated as StallPorts.
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqToMemQ      <- mkPortStallSend_Multiplexed("L1_ICache_OutQ");
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) fillFromMemory <- mkPortStallRecv_Multiplexed("L1_ICache_InQ");


    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = fillFromMemory.ctrl.in;
    inports[2] = reqToMemQ.ctrl.in;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = reqToMemQ.ctrl.out;
    outports[3] = fillFromMemory.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, IC_LOCAL_STATE) stage2Ctrl <- mkStageController();


    // ****** Stats ******

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statHits    <- mkStatCounter_Multiplexed(`STATS_L1_ICACHE_HIT);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statMisses  <- mkStatCounter_Multiplexed(`STATS_L1_ICACHE_MISS);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statRetries <- mkStatCounter_Multiplexed(`STATS_L1_ICACHE_RETRY);


    // ****** Rules ******

    // stage1_fill
    
    // See if there are any new fill responses from memory.

    // Ports read:
    // * fillFromMemory
    
    // Ports written:
    // * loadRspDelayedtoCPU
    
    (* conservative_implicit_conditions *)
    rule stage1_fill (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = initLocalState();
        
        // Check for fills.
        let m_fill <- fillFromMemory.receive(cpu_iid);

        if (m_fill matches tagged Valid .fill)
        begin

            // Note that the actual cache update will be done later, so that
            // any lookups this model cycle don't see it accidentally.

            local_state.writePortUsed = True;
            local_state.writePortData = fill.physicalAddress;

            // Deallocate the Miss ID.
            L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
            outstandingMisses.free(cpu_iid, miss_tok);
            

            // Return it to the CPU.
            debugLog.record_next_cycle(cpu_iid, $format("MEM RSP: %0d", miss_tok.index));
            let rsp = initICacheMissRsp(miss_tok.index);
            loadRspDelToCPU.send(cpu_iid, tagged Valid rsp);

            // We finished the fill succesfully, so dequeue it.
            fillFromMemory.doDeq(cpu_iid);

        end
        else
        begin

            // Tell the CPU there's no delayed response.
            debugLog.record_next_cycle(cpu_iid, $format("NO RSP"));
            loadRspDelToCPU.send(cpu_iid, tagged Invalid);
            
            // No dequeue.
            fillFromMemory.noDeq(cpu_iid);

        end

        // Now read the load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);

        // Deal with any load requests.
        if (msg_from_cpu matches tagged Valid .req)
        begin

            // See if the cache algorithm hit or missed.
            let line_addr = toLineAddress(req.physicalAddress);
            iCacheAlg.loadLookupReq(cpu_iid, line_addr);
            debugLog.record_next_cycle(cpu_iid, $format("LOAD REQ: 0x%h", line_addr));

            // Finish the request in the next stage.
            local_state.loadReq = tagged Valid req;

        end
        
        // Pass our information to the next stage.
        stage2Ctrl.ready(cpu_iid, local_state);

    endrule
    
    // stage2_loadRsp
    
    // Finish up any loads to see if they hit or miss.
    // Begin handling any store requests.
    
    // Ports Read:
    // * storeReqFromCPU
    
    // Ports Written:
    // * loadRspImmToCPU

    rule stage2_loadRsp (True);

        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // Check if the memQ has room for any new requests.
        let memQ_not_full <- reqToMemQ.canEnq(cpu_iid);

        // See if we need to finish any load responses.
        if (local_state.loadReq matches tagged Valid .req)
        begin

            // Get the lookup response.
            let m_entry <- iCacheAlg.loadLookupRsp(cpu_iid);

            // Does the cache contain this addresss?
            if (m_entry matches tagged Valid .entry)
            begin

                // A hit, so give the data back. We won't need the memory queue.
                loadRspImmToCPU.send(cpu_iid, tagged Valid initICacheHit(req));
                reqToMemQ.noEnq(cpu_iid);
                statHits.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("LOAD HIT"));

            end
            else
            begin

                // A miss. But do we have a free missID to track the fill with?
                // And is the memQ not full?
                if (outstandingMisses.canAllocateLoad(cpu_iid) && memQ_not_full)
                begin

                    // Allocate the next miss ID and give it back to the CPU.
                    let miss_tok <- outstandingMisses.allocateLoad(cpu_iid);
                    
                    // Send the fill request to memory, using the opaque bits for the miss id.
                    let line_addr = toLineAddress(req.physicalAddress);
                    let mem_req = initMemLoad(line_addr);
                    mem_req.opaque = toMemOpaque(miss_tok);
                    reqToMemQ.doEnq(cpu_iid, mem_req);

                    // Tell the CPU their load missed, but we're handling it.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initICacheMiss(req, miss_tok.index));
                    statMisses.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("LOAD MISS: %0d", miss_tok.index));

                end
                else
                begin

                    // The CPU must retry.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initICacheRetry(req));
                    debugLog.record(cpu_iid, $format("LOAD MISS RETRY"));
                    
                    // No request to memory.
                    reqToMemQ.noEnq(cpu_iid);

                    // Record the retry.
                    statRetries.incr(cpu_iid);

                end

            end // cache load miss
        end
        else
        begin

            // Propogate the bubble.
            loadRspImmToCPU.send(cpu_iid, tagged Invalid);
            reqToMemQ.noEnq(cpu_iid);

        end

        
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
        
            iCacheAlg.allocate(cpu_iid, local_state.writePortData, False, ?);
        
        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 

    endrule

endmodule
