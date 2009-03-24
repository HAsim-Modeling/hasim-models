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

`include "asim/provides/memory_base_types.bsh"

// ****** Generated Files ******

`include "asim/dict/PARAMS_HASIM_DCACHE.bsh"
`include "asim/dict/STATS_PSEUDORANDOM_DCACHE.bsh"


typedef union tagged
{
    DCACHE_LOAD_OUTPUT_DELAYED FILL_load;
    DCACHE_STORE_OUTPUT_DELAYED FILL_store;
}
DCACHE_FILL deriving (Eq, Bits);

function DCACHE_FILL initLoadFill(DMEM_BUNDLE bundle);

    return tagged FILL_load initDCacheLoadMissRsp(bundle);

endfunction

function DCACHE_FILL initStoreFill(TOKEN tok);

    return tagged FILL_store initDCacheStoreDelayOk(tok);

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
    

    // ****** UnModel State ******
    
    LFSR#(Bit#(8)) loadLFSR  <- mkLFSR_8();
    LFSR#(Bit#(8)) storeLFSR <- mkLFSR_8();
    
    Reg#(Bool) initializingLFSRs <- mkReg(True);

    // ****** Ports ******

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTICTX#(DCACHE_LOAD_INPUT) loadReqFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_load", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTICTX#(DCACHE_STORE_INPUT) storeReqFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_store", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTICTX#(DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_load_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTICTX#(DCACHE_LOAD_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_load_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTICTX#(DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_store_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTICTX#(DCACHE_STORE_OUTPUT_DELAYED) storeRspDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_store_delayed");

    // Ports to simulate some latency to memory.
    PORT_SEND_MULTICTX#(DCACHE_FILL) reqToMemory   <- mkPortSend_MultiCtx("DCache_to_memory");
    PORT_RECV_MULTICTX#(DCACHE_FILL) rspFromMemory <- mkPortRecv_MultiCtx("DCache_to_memory", `DCACHE_MISS_PENALTY);

    // communication with local controller
    Vector#(3, PORT_CONTROLS) inports = newVector();
    Vector#(5, PORT_CONTROLS) outports = newVector();
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = storeReqFromCPU.ctrl;
    inports[2] = rspFromMemory.ctrl;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;
    outports[4] = reqToMemory.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Stats ******

    STAT_RECORDER_MULTICTX statLoadHits       <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_READ_HITS);
    STAT_RECORDER_MULTICTX statLoadMisses     <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_READ_MISSES);
    STAT_RECORDER_MULTICTX statLoadRetries    <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_READ_RETRIES);
    STAT_RECORDER_MULTICTX statStoreHits      <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_WRITE_HITS);
    STAT_RECORDER_MULTICTX statStoreMisses    <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_WRITE_MISSES);
    STAT_RECORDER_MULTICTX statStoreRetries   <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_WRITE_RETRIES);
    STAT_RECORDER_MULTICTX statPortCollisions <- mkStatCounter_MultiCtx(`STATS_PSEUDORANDOM_DCACHE_DCACHE_PORT_COLLISIONS);


    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSRs (initializingLFSRs);
    
        loadLFSR.seed(loadSeedParam);
        storeLFSR.seed(storeSeedParam);
        initializingLFSRs <= False;
    
    endrule

    // stage1_loadReq
    
    // Stores always hit in the cache. Loads also always hit but we also 

    rule stage1_loadReq (!initializingLFSRs);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // First check for fills.
        let m_fill <- rspFromMemory.receive(ctx);
        
        if (m_fill matches tagged Valid .fill)
        begin
       
            // Send the fill to the right place.
            case (fill) matches
                tagged FILL_load .rsp:
                begin
                
                    // The fill is a load response.
                    loadRspDelToCPU.send(ctx, tagged Valid rsp);
                    storeRspDelToCPU.send(ctx, tagged Invalid);
                
                end
                tagged FILL_store .rsp:
                begin

                    // The fill is a store response.
                    loadRspDelToCPU.send(ctx, tagged Invalid);
                    storeRspDelToCPU.send(ctx, tagged Valid rsp);
                end
            endcase
        
        end
        else
        begin
            
            loadRspDelToCPU.send(ctx, tagged Invalid);
            storeRspDelToCPU.send(ctx, tagged Invalid);
        
        end
        
        // Now read load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(ctx);
        
        // Record the request we should make on our one memory port.
        Maybe#(DCACHE_FILL) req_to_mem = tagged Invalid;

        // Deal with any load requests.
        case (msg_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        loadRspImmToCPU.send(ctx, tagged Invalid);

                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(ctx, 1);
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
	            loadRspImmToCPU.send(ctx, tagged Valid initDCacheLoadRetry(req));
                    statLoadRetries.incr(ctx);
                    
                    // End of model cycle. (Path 2)
                    localCtrl.endModelCycle(ctx, 2);

                end
                else
                begin

                    
                    if (rnd < dcacheLoadMissChance)
                    begin

                        // Stall the load for some time.
                        req_to_mem = tagged Valid initLoadFill(req);
                        statLoadMisses.incr(ctx);

                        
                        // Tell the CPU that a response is coming.
                        loadRspImmToCPU.send(ctx, tagged Valid initDCacheLoadMiss(req));
                        
                        // End of model cycle. (Path 3)
                        localCtrl.endModelCycle(ctx, 3);
                    
                    end
                    else
                    begin
                        
                        // A hit.
                        statLoadHits.incr(ctx);
                        
                        // Give the response to the CPU.
                        loadRspImmToCPU.send(ctx, tagged Valid initDCacheLoadHit(req));
                        
                        // End of model cycle. (Path 4)
                        localCtrl.endModelCycle(ctx, 4);
                    
                    end

                end

	    end

        endcase
        
        // Now read the store input port.
        let m_store_from_cpu <- storeReqFromCPU.receive(ctx);
        
        // Deal with any store requests.
        case (m_store_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        storeRspImmToCPU.send(ctx, tagged Invalid);

	    end

	    tagged Valid {.tok, .phys_addr}:
	    begin

                // An actual cache would do something with the physical 
                // address to determine hit or miss. We use a pseudo-random 
                // number to determine a response.
        
                let rnd = storeLFSR.value();
                storeLFSR.next();

                if (rnd < dcacheStoreRetryChance)
                begin

                    // They should retry their store on a later model cycle.
	            storeRspImmToCPU.send(ctx, tagged Valid initDCacheStoreRetry(tok));
                    statStoreRetries.incr(ctx);

                end
                else
                begin
                    
                    if (rnd < dcacheStoreMissChance)
                    begin
                        
                        // See if the load was using the port.
                        if (isValid(req_to_mem))
                        begin
                        
                            // The port is taken. They must retry next model cycle.
	                    storeRspImmToCPU.send(ctx, tagged Valid initDCacheStoreRetry(tok));
                            statPortCollisions.incr(ctx);
                        
                        end
                        else
                        begin

                            // The port is free. We'll use it to delay the store for a while.
                            req_to_mem = tagged Valid initStoreFill(tok);
                            
                            // Tell the CPU we missed. They should delay until the store comes back.
	                    storeRspImmToCPU.send(ctx, tagged Valid initDCacheStoreDelay(tok));
                            statStoreMisses.incr(ctx);
                        
                        end
                        
                    end
                    else
                    begin
                        
                        // Tell the CPU we hit.
	                storeRspImmToCPU.send(ctx, tagged Valid initDCacheStoreOk(tok));
                        statStoreHits.incr(ctx);
                                                
                    end

                end

	    end

        endcase

        // Make the request to the memory fill port.
        reqToMemory.send(ctx, req_to_mem);

    endrule

endmodule
