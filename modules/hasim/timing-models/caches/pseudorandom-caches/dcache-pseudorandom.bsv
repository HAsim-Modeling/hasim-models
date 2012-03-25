
import FIFO::*;
import Vector::*;
import LFSR::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/funcp_interface.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

// ****** Generated Files ******

`include "asim/dict/PARAMS_HASIM_DCACHE.bsh"


typedef union tagged
{
    DCACHE_LOAD_OUTPUT_DELAYED FILL_load;
    DCACHE_STORE_OUTPUT_DELAYED FILL_store;
}
DCACHE_FILL deriving (Eq, Bits);

function DCACHE_FILL initLoadFill(DMEM_BUNDLE bundle, DCACHE_LOAD_MISS_ID miss_id);

    return tagged FILL_load initDCacheLoadMissRsp(bundle, miss_id);

endfunction

function DCACHE_FILL initStoreFill(STORE_TOKEN st_tok);

    return tagged FILL_store initDCacheStoreDelayOk(st_tok);

endfunction

module [HASIM_MODULE] mkDCache();


    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) loadSeedParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_LOAD_SEED, paramNode);
    Param#(8) storeSeedParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_STORE_SEED, paramNode);
    
    Param#(8) loadRetryChanceParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_LOAD_RETRY_CHANCE, paramNode);
    Param#(8) loadMissChanceParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_LOAD_MISS_CHANCE, paramNode);
    
    Param#(8) storeRetryChanceParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_STORE_RETRY_CHANCE, paramNode);
    Param#(8) storeMissChanceParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_STORE_MISS_CHANCE, paramNode);

    // ****** Local Definitions *******
    
    Bit#(8) dcacheLoadRetryChance = loadRetryChanceParam;
    Bit#(8) dcacheLoadMissChance  = loadRetryChanceParam + loadMissChanceParam;
    
    Bit#(8) dcacheStoreRetryChance = storeRetryChanceParam;
    Bit#(8) dcacheStoreMissChance  = storeRetryChanceParam + storeMissChanceParam;
    
    // ****** Model State *******
    
    // Track the next miss ID to give out.
    MULTIPLEXED#(NUM_CPUS, COUNTER#(DCACHE_LOAD_MISS_ID_SIZE)) nextLoadMissIDPool <- mkMultiplexed(mkLCounter(0));

    // A counter to track number of misses in flight. If we run out of IDs then we make the load retry.
    MULTIPLEXED#(NUM_CPUS, COUNTER#(DCACHE_LOAD_MISS_COUNT)) outstandingLoadMissesPool <- mkMultiplexed(mkLCounter(0));


    // ****** UnModel State ******
    
    MULTIPLEXED#(NUM_CPUS, LFSR#(Bit#(8))) loadLFSRPool  <- mkMultiplexed(mkLFSR_8());
    MULTIPLEXED#(NUM_CPUS, LFSR#(Bit#(8))) storeLFSRPool <- mkMultiplexed(mkLFSR_8());
    
    Reg#(Maybe#(INSTANCE_ID#(NUM_CPUS))) initializingLFSRs <- mkReg(tagged Valid 0);

    // ****** Ports ******

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_load", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_INPUT) storeReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_store", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED) storeRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_delayed");

    // Ports to simulate some latency to memory.
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_FILL) reqToMemory   <- mkPortSend_Multiplexed("DCache_to_memory");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_FILL) rspFromMemory <- mkPortRecv_Multiplexed("DCache_to_memory", `DCACHE_MISS_PENALTY);

    // communication with local controller
    Vector#(3, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(5, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = storeReqFromCPU.ctrl;
    inports[2] = rspFromMemory.ctrl;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;
    outports[4] = reqToMemory.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    // ****** Stats ******

    STAT_VECTOR#(NUM_CPUS) statLoadHits <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_READ_HITS",
                                           "DCache Read Hits"));
    STAT_VECTOR#(NUM_CPUS) statLoadMisses <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_READ_MISSES",
                                           "DCache Read Misses"));
    STAT_VECTOR#(NUM_CPUS) statLoadRetries <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_READ_RETRIES",
                                           "DCache Read Retries"));
    STAT_VECTOR#(NUM_CPUS) statStoreHits <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_WRITE_HITS",
                                           "DCache Write Hits"));
    STAT_VECTOR#(NUM_CPUS) statStoreMisses <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_WRITE_MISSES",
                                           "DCache Write Misses"));
    STAT_VECTOR#(NUM_CPUS) statStoreRetries <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_WRITE_RETRIES",
                                           "DCache Write Retries"));
    STAT_VECTOR#(NUM_CPUS) statPortCollisions <-
        mkStatCounter_Multiplexed(statName("PSEUDORANDOM_DCACHE_PORT_COLLISIONS",
                                           "DCache Port Collisions"));


    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSRs (initializingLFSRs matches tagged Valid .iid);
    
        let loadLFSR  = loadLFSRPool[iid];
        let storeLFSR = storeLFSRPool[iid];
    
        loadLFSR.seed(loadSeedParam);
        storeLFSR.seed(storeSeedParam);
        let new_iid = iid + 1;
        if (new_iid == 0)
        begin
            initializingLFSRs <= tagged Invalid;
        end
        else
        begin
            initializingLFSRs <= tagged Valid new_iid;
        end
    
    endrule

    // stage1_loadReq

    // Ports Read:
    // * rspFromMemory
    // * loadReqFromCPU
    // * storeReqFromCPU
    
    // Ports Written:
    // * loadRspImmToCPU
    // * loadRspDelToCPU
    // * storeRspImmToCPU
    // * storeRspDelToCPU
    // * reqToMemory
    
    rule stage1_loadReq (!isValid(initializingLFSRs));

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // Get our local state.
        let nextLoadMissID = nextLoadMissIDPool[cpu_iid];
        let outstandingLoadMisses = outstandingLoadMissesPool[cpu_iid];

        // First check for fills.
        let m_fill <- rspFromMemory.receive(cpu_iid);
        
        if (m_fill matches tagged Valid .fill)
        begin
       
            // Send the fill to the right place.
            case (fill) matches
                tagged FILL_load .rsp:
                begin
                
                    // The fill is a load response.
                    loadRspDelToCPU.send(cpu_iid, tagged Valid rsp);
                    storeRspDelToCPU.send(cpu_iid, tagged Invalid);
                    outstandingLoadMisses.down();
                
                end
                tagged FILL_store .rsp:
                begin

                    // The fill is a store response.
                    loadRspDelToCPU.send(cpu_iid, tagged Invalid);
                    storeRspDelToCPU.send(cpu_iid, tagged Valid rsp);

                end
            endcase
        
        end
        else
        begin
            
            loadRspDelToCPU.send(cpu_iid, tagged Invalid);
            storeRspDelToCPU.send(cpu_iid, tagged Invalid);
        
        end
        
        let loadLFSR  = loadLFSRPool[cpu_iid];
        let storeLFSR = storeLFSRPool[cpu_iid];
        
        // Now read load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);
        
        // Record the request we should make on our one memory port.
        Maybe#(DCACHE_FILL) req_to_mem = tagged Invalid;

        // Deal with any load requests.
        case (msg_from_cpu) matches

            tagged Invalid:
            begin

                // Propogate the bubble.
                loadRspImmToCPU.send(cpu_iid, tagged Invalid);

                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(cpu_iid, 1);
            end

            tagged Valid .req:
            begin

                // An actual cache would do something with the physical 
                // address to determine hit or miss. We use a pseudo-random 
                // number to determine a response.
        
                let rnd = loadLFSR.value();
                loadLFSR.next();

                if (rnd < dcacheLoadRetryChance)
                begin

                    // They have to retry next cycle.
                    loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadRetry(req));
                    statLoadRetries.incr(cpu_iid);
                    
                    // End of model cycle. (Path 2)
                    localCtrl.endModelCycle(cpu_iid, 2);

                end
                else
                begin
                    
                    if (rnd < dcacheLoadMissChance)
                    begin

                        // A miss, so try to get a Miss ID.
                        let num_outstanding = outstandingLoadMisses.value();
                
                        // We have a miss to allocate if the high bit is not 1.
                        let can_allocate = num_outstanding[valueof(DCACHE_LOAD_MISS_ID_SIZE)] == 0;
                
                        if (can_allocate)
                        begin

                            // Allocate the next miss ID and give it to fetch.
                            let miss_id = nextLoadMissID.value();
                            nextLoadMissID.up();
                            outstandingLoadMisses.up();

                            // Stall the load for some time.
                            req_to_mem = tagged Valid initLoadFill(req, miss_id);
                            statLoadMisses.incr(cpu_iid);

                            // Tell the CPU that a response is coming.
                            loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadMiss(req, miss_id));

                            // End of model cycle. (Path 3)
                            localCtrl.endModelCycle(cpu_iid, 3);
                    
                        end
                        else
                        begin

                            // Tell the load request to retry. This should be quite rare.
                            loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadRetry(req));

                        end
                
                    end
                    else
                    begin
                        
                        // A hit.
                        statLoadHits.incr(cpu_iid);
                        
                        // Give the response to the CPU.
                        loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadHit(req));
                        
                        // End of model cycle. (Path 4)
                        localCtrl.endModelCycle(cpu_iid, 4);
                    
                    end

                end

            end

        endcase
        
        // Now read the store input port.
        let m_store_from_cpu <- storeReqFromCPU.receive(cpu_iid);
        
        // Deal with any store requests.
        case (m_store_from_cpu) matches

            tagged Invalid:
            begin

                // Propogate the bubble.
                storeRspImmToCPU.send(cpu_iid, tagged Invalid);

            end

            tagged Valid .req:
            begin

                // An actual cache would do something with the physical 
                // address to determine hit or miss. We use a pseudo-random 
                // number to determine a response.
        
                let rnd = storeLFSR.value();
                storeLFSR.next();

                if (rnd < dcacheStoreRetryChance)
                begin

                    // They should retry their store on a later model cycle.
                    storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry(req.storeToken));
                    statStoreRetries.incr(cpu_iid);

                end
                else
                begin
                    
                    if (rnd < dcacheStoreMissChance)
                    begin
                        
                        // See if the load was using the port.
                        if (isValid(req_to_mem))
                        begin
                        
                            // The port is taken. They must retry next model cycle.
                            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry(req.storeToken));
                            statPortCollisions.incr(cpu_iid);
                        
                        end
                        else
                        begin

                            // The port is free. We'll use it to delay the store for a while.
                            req_to_mem = tagged Valid initStoreFill(req.storeToken);
                            
                            // Tell the CPU we missed. They should delay until the store comes back.
                            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreDelay(req.storeToken));
                            statStoreMisses.incr(cpu_iid);
                        
                        end
                        
                    end
                    else
                    begin
                        
                        // Tell the CPU we hit.
                        storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk(req.storeToken));
                        statStoreHits.incr(cpu_iid);

                    end

                end

            end

        endcase

        // Make the request to the memory fill port.
        reqToMemory.send(cpu_iid, req_to_mem);

    endrule

endmodule
