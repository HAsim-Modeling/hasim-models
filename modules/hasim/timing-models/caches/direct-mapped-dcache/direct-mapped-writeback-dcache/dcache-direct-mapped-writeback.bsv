import Vector::*;
import FIFO::*;
import DefaultValue::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/platform_interface.bsh"

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/scratchpad_memory_common.bsh"

`include "asim/dict/PARAMS_HASIM_DCACHE.bsh"
`include "asim/dict/VDEV_SCRATCH.bsh"


// ****** Memory Port Assignments ******
`define READ_PORT_LOAD 0
`define READ_PORT_STORE 1


// ****** Local Types *******

typedef Bit#(TSub#(`FUNCP_ISA_P_ADDR_SIZE, TAdd#(`DCACHE_LINE_BITS, `DCACHE_IDX_BITS))) DCACHE_TAG;
typedef Bit#(`DCACHE_LINE_BITS) DCACHE_LINE_OFFSET;
typedef Bit#(`DCACHE_IDX_BITS)  DCACHE_INDEX;

function Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) splitAddressDCache(MEM_ADDRESS addr);

    return unpack(addr);

endfunction

function DCACHE_TAG getDCacheTag(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressDCache(addr);
    return tag;

endfunction

function DCACHE_INDEX getDCacheIndex(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressDCache(addr);
    return idx;

endfunction


function DCACHE_LINE_OFFSET getDCacheOffset(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressDCache(addr);
    return off;

endfunction

typedef union tagged
{
    DCACHE_LOAD_OUTPUT_DELAYED  FILL_load;
    Tuple2#(TOKEN, MEM_ADDRESS) FILL_store;
}
DCACHE_FILL deriving (Eq, Bits);

function DCACHE_FILL initLoadFill(DMEM_BUNDLE bundle);

    return tagged FILL_load initDCacheLoadMissRsp(bundle);

endfunction

function DCACHE_FILL initStoreFill(TOKEN tok, MEM_ADDRESS addr);

    return tagged FILL_store tuple2(tok, addr);

endfunction

typedef union tagged
{
    void DCACHE_storeNop;
    DCACHE_STORE_INPUT DCACHE_storeRead;
    TOKEN DCACHE_storeWrite;
}
DCACHE_STORE_INFO deriving (Eq, Bits);

typedef Tuple2#(Maybe#(DCACHE_LOAD_INPUT), Maybe#(DCACHE_STORE_INPUT)) DCACHE_STAGE2_STATE;
typedef Tuple2#(Maybe#(DCACHE_LOAD_INPUT), Maybe#(DCACHE_STORE_INPUT)) DCACHE_STAGE3_STATE;

module [HASIM_MODULE] mkDCache();


    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(1) alwaysHitParam <- mkDynamicParameter(`PARAMS_HASIM_DCACHE_DCACHE_ALWAYS_HIT, paramNode);
    function Bool alwaysClaimHit() = (alwaysHitParam == 1);

 
    // ****** Model State ******

    // Initialize a scratchpad memory to store our tags in.   
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(MAX_NUM_CPUS, 2, DCACHE_INDEX, Maybe#(DCACHE_TAG)) dCacheTagStore <- mkMultiReadScratchpad_Multiplexed(`VDEV_SCRATCH_DIRECT_MAPPED_WRITETHROUGH_DCACHE_TAGS, defaultValue);

    
    // ****** Ports ******

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_load", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_INPUT) storeReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_store", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED) storeRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_delayed");

    // Ports to simulate some latency to memory.
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_FILL) reqToMemory   <- mkPortSend_Multiplexed("DCache_to_memory");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_FILL) rspFromMemory <- mkPortRecv_Multiplexed("DCache_to_memory", `DCACHE_MISS_PENALTY);


    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(5, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = storeReqFromCPU.ctrl;
    inports[2] = rspFromMemory.ctrl;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;
    outports[4] = reqToMemory.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DCACHE_STAGE2_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DCACHE_STAGE3_STATE) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DCACHE_STAGE3_STATE) stage4Ctrl <- mkStageController();

    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statLoadHits <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_READ_HITS",
                                           "DCache Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statLoadMisses <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_READ_MISSES",
                                           "DCache Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statStoreHits <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_WRITE_HITS",
                                           "DCache Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statStoreMisses <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_WRITE_MISSES",
                                           "DCache Write Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statPortCollisionsRead <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_PORT_COLLISIONS_READ",
                                           "DCache Port Collisions (Read/Read)"));
    STAT_VECTOR#(MAX_NUM_CPUS) statPortCollisionsWrite <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_WRITETHROUGH_DCACHE_PORT_COLLISIONS_WRITE",
                                           "DCache Port Collisions (Read/Write)"));


    // ****** Rules ******

    // stage1_loadStoreReq
    
    // See if there are any new requests from the CPU.

    // Ports read:
    // * loadReqFromCPU
    // * storeReqFromCPU
    
    // Ports written:
    // * None
    
    (* conservative_implicit_conditions *)
    rule stage1_loadStoreReq (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Read load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);
        
        // Record the request we should make on our one memory port.
        Maybe#(DCACHE_LOAD_INPUT)  load_info  = tagged Invalid;
        Maybe#(DCACHE_STORE_INPUT) store_info = tagged Invalid;

        
        // Deal with any load requests.
        case (msg_from_cpu) matches

            tagged Invalid:
            begin

                // Record that the load won't need the memory port.
                load_info = tagged Invalid;
                
            end

            tagged Valid .req:
            begin
            
                // Record that the load may need the memory port.
                load_info = tagged Valid req;
                
                // Look up the index in our tag store.
                let idx = getDCacheIndex(req.physicalAddress);
                dCacheTagStore.readPorts[`READ_PORT_LOAD].readReq(cpu_iid, idx);
                
                // Pass the request on for tag comparison.

            end

        endcase
        
        // Now read the store input port.
        let m_store_from_cpu <- storeReqFromCPU.receive(cpu_iid);
        
        // Deal with any store requests.
        case (m_store_from_cpu) matches

            tagged Invalid:
            begin

                // Record that the store won't need the memory port.
                store_info = tagged Invalid;

            end

            tagged Valid {.tok, .phys_addr}:
            begin
            
                // Record that the store may need the memory port.
                store_info = tagged Valid tuple2(tok, phys_addr);
                
                // Look up the index in our tag store.
                let idx = getDCacheIndex(phys_addr);
                dCacheTagStore.readPorts[`READ_PORT_STORE].readReq(cpu_iid, idx);

            end

        endcase

        // Proceed to the next stage to process responses.
        stage2Ctrl.ready(cpu_iid, tuple2(load_info, store_info));

    endrule
    
    // stage2_rsp
    
    // Get the response from the tag store, make a request to backing store.
    // Get the response from backing store, perform the fill.
    
    // Ports Read:
    // * rspFromMemory
    
    // Ports Written:
    // * loadRspImmToCPU
    // * loadRspDelToCPU
    // * storeRspImmToCPU
    // * storeRspDelToCPU
    // * reqToMemory
    
    rule stage2_rsp (True);
    
        match {.cpu_iid, {.m_load_info, .m_store_info}} <- stage2Ctrl.nextReadyInstance();
    
        // Now handle loads.
        Maybe#(DCACHE_LOAD_INPUT) load_info = tagged Invalid;
    
        if (m_load_info matches tagged Invalid)
        begin
        
            // Propogate the bubble.
            loadRspImmToCPU.send(cpu_iid, tagged Invalid);
                
            // Record that we don't need the memory port since there was no load.
            load_info = tagged Invalid;
        
        end
        else if (m_load_info matches tagged Valid .req)
        begin
    
            // Get the response from the tagStore.
            let m_existing_tag <- dCacheTagStore.readPorts[`READ_PORT_LOAD].readRsp(cpu_iid);

            // See if the tag matches.
            let target_tag = getDCacheTag(req.physicalAddress);
            Bool hit = m_existing_tag matches tagged Valid .existing_tag &&& (existing_tag == target_tag) ? True : False ;

            if (hit || alwaysClaimHit())
            begin

                // A hit, so give the data back.
                loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadHit(req));
                statLoadHits.incr(cpu_iid);

                // Record that we got a hit and don't need the memory port.
                load_info = tagged Invalid;

            end
            else
            begin

                // A miss, so no data to give back immediately.
                loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadMiss(req));
                statLoadMisses.incr(cpu_iid);

                // Record that we got a miss and would like the memory port.
                load_info = tagged Valid req;

            end
        
        end
        
        stage3Ctrl.ready(cpu_iid, tuple2(load_info, m_store_info));
       
    endrule
    
    rule stage3_fill (True);
    
        match {.cpu_iid, .state} <- stage3Ctrl.nextReadyInstance();
    
        // Check for fills.
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
                    
                    // Update the Tag Store.
                    let idx = getDCacheIndex(rsp.physicalAddress);
                    let tag = getDCacheTag(rsp.physicalAddress);
                    dCacheTagStore.write(cpu_iid, idx, tagged Valid tag);
                
                end

                tagged FILL_store {.tok, .phys_addr}:
                begin

                    // The fill is a store response.
                    loadRspDelToCPU.send(cpu_iid, tagged Invalid);
                    storeRspDelToCPU.send(cpu_iid, tagged Valid initDCacheStoreDelayOk(tok));
                    
                    // Update the Tag Store.
                    let idx = getDCacheIndex(phys_addr);
                    let tag = getDCacheTag(phys_addr);
                    dCacheTagStore.write(cpu_iid, idx, tagged Valid tag);

                end
            endcase
        
        end
        else
        begin
            
            loadRspDelToCPU.send(cpu_iid, tagged Invalid);
            storeRspDelToCPU.send(cpu_iid, tagged Invalid);
        
        end
        
        stage4Ctrl.ready(cpu_iid, state);
        
    endrule

    rule stage4_storeRsp (True);
    
        match {.cpu_iid, {.load_info, .m_store_info}} <- stage4Ctrl.nextReadyInstance();
    
        // Next handle stores.
        DCACHE_STORE_INFO store_info = tagged DCACHE_storeNop;
        
        if (m_store_info matches tagged Invalid)
        begin
            
            // No store, so we don't need the memory port for that.
            store_info = tagged DCACHE_storeNop;
            
        end
        else if (m_store_info matches tagged Valid {.tok, .phys_addr})
        begin

            // Get the response from the tagStore.
            let m_existing_tag <- dCacheTagStore.readPorts[`READ_PORT_STORE].readRsp(cpu_iid);

            // See if the tag matches.
            let target_tag = getDCacheTag(phys_addr);
            Bool hit = m_existing_tag matches tagged Valid .existing_tag &&& (existing_tag == target_tag) ? True : False ;

            if (hit || alwaysClaimHit())
            begin

                // A hit, so try to obtain the write port so we can writeback the data.
                // Record that we got a hit so we DO WANT the memory port since we're writethrough.
                store_info = tagged DCACHE_storeWrite tok;

            end
            else
            begin

                // A miss, so tell the user they should delay until the data comes back.

                // Record that we got a miss and would like the memory port to read the data in.
                // (This is more like simulating a coherent cache system.)
                store_info = tagged DCACHE_storeRead tuple2(tok, phys_addr);

            end
            
        end
        
        // Now arbitrate the memory port.
    
        if (load_info matches tagged Valid .req)
        begin

            // The load wants to use the port. Let's give it priority, so it gets it.
            reqToMemory.send(cpu_iid, tagged Valid initLoadFill(req));

            // If the store also wanted it, then the user must retry.
            if (store_info matches tagged DCACHE_storeWrite .tok)
            begin

                // Tell the user to retry... we hit, but could not write through.
                storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry(tok));
                statPortCollisionsWrite.incr(cpu_iid);

            end
            else if (store_info matches tagged DCACHE_storeRead {.tok, .phys_addr})
            begin

                // Tell the user to retry... we missed, and could not make our fill request.
                storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreRetry(tok));
                statPortCollisionsRead.incr(cpu_iid);

            end
            else
            begin

                // We should never get here, but let's put this here for safety.
                storeRspImmToCPU.send(cpu_iid, tagged Invalid);

            end

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (store_info matches tagged DCACHE_storeRead {.tok, .phys_addr})
        begin

            // The load doesn't want the port, so the store can use it to read.
            reqToMemory.send(cpu_iid, tagged Valid initStoreFill(tok, phys_addr));

            // Tell the user to delay until the store miss is filled.
            statStoreMisses.incr(cpu_iid);
            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreDelay(tok));

            // End of model cycle (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);


        end
        else if (store_info matches tagged DCACHE_storeWrite .tok)
        begin

            // The load didn't want the port, so the store can use it to write through.
            // TODO: make this an actual store back to main memory.
            reqToMemory.send(cpu_iid, tagged Invalid);

            // Tell the user their store worked.
            statStoreHits.incr(cpu_iid);
            storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk(tok));

            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(cpu_iid, 3);


        end
        else
        begin

            // Turns out neither wanted the port.
            reqToMemory.send(cpu_iid, tagged Invalid);

            // Propogate the bubble.
            storeRspImmToCPU.send(cpu_iid, tagged Invalid);
    
            
            // End of model cycle. (Path 4)
            localCtrl.endModelCycle(cpu_iid, 4);

        end

    endrule

endmodule
