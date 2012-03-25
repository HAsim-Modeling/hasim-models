import Vector::*;
import FIFO::*;

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

`include "asim/dict/PARAMS_HASIM_ICACHE.bsh"
`include "asim/dict/VDEV_SCRATCH.bsh"

typedef Bit#(TSub#(`FUNCP_ISA_P_ADDR_SIZE, TAdd#(`ICACHE_LINE_BITS, `ICACHE_IDX_BITS))) ICACHE_TAG;
typedef Bit#(`ICACHE_LINE_BITS) ICACHE_LINE_OFFSET;
typedef Bit#(`ICACHE_IDX_BITS)  ICACHE_INDEX;

function Tuple3#(ICACHE_TAG, ICACHE_INDEX, ICACHE_LINE_OFFSET) splitAddressICache(MEM_ADDRESS addr);

    return unpack(addr);

endfunction

function ICACHE_TAG getICacheTag(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressICache(addr);
    return tag;

endfunction

function ICACHE_INDEX getICacheIndex(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressICache(addr);
    return idx;

endfunction


function ICACHE_LINE_OFFSET getICacheOffset(MEM_ADDRESS addr);

    match {.tag, .idx, .off} = splitAddressICache(addr);
    return off;

endfunction

typedef union tagged
{
    void STAGE2_bubble;
    ICACHE_INPUT STAGE2_access;
}
ICACHE_STAGE2_STATE deriving (Eq, Bits);
 
module [HASIM_MODULE] mkICache();
    
    // ***** Dynamic parameters *****
    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(1) alwaysHitParam <- mkDynamicParameter(`PARAMS_HASIM_ICACHE_ICACHE_ALWAYS_HIT, paramNode);
    function Bool alwaysClaimHit() = (alwaysHitParam == 1);


    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)    getInstruction <- mkConnection_Client("funcp_getInstruction");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_INPUT) pcFromFet <- mkPortRecv_Multiplexed("CPU_to_ICache_req", 0);

    // Outgoing ports to CPU
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immToFet <- mkPortSend_Multiplexed("ICache_to_CPU_immediate");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED)   delToFet <- mkPortSend_Multiplexed("ICache_to_CPU_delayed");

    // Ports to simulate some latency to memory.
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED) reqToMemory   <- mkPortSend_Multiplexed("ICache_to_memory");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED) rspFromMemory <- mkPortRecv_Multiplexed("ICache_to_memory", `ICACHE_MISS_PENALTY);


    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = pcFromFet.ctrl;
    inports[1]  = rspFromMemory.ctrl;
    outports[0] = immToFet.ctrl;
    outports[1] = delToFet.ctrl;
    outports[2] = reqToMemory.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);
    
    STAGE_CONTROLLER#(NUM_CPUS, ICACHE_STAGE2_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER_VOID#(NUM_CPUS)                 stage3Ctrl <- mkStageControllerVoid();

    // ****** Stats ******

    STAT_VECTOR#(NUM_CPUS) statHits <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_ICACHE_HITS",
                                           "ICache Hits"));
    STAT_VECTOR#(NUM_CPUS) statMisses <-
        mkStatCounter_Multiplexed(statName("DIRECT_MAPPED_ICACHE_MISSES",
                                           "ICache Misses"));
   
    // ****** Model State (Per Instance) *****
   
    // initialize cache memory
    // Commented out for now until we stabilize memory interface.
    // let cachememory <- mkICacheMemory();

    // Initialize a scratchpad memory to store our tags in.   
    MEMORY_IFC_MULTIPLEXED#(NUM_CPUS, ICACHE_INDEX, Maybe#(ICACHE_TAG)) iCacheTagStore <- mkScratchpad_Multiplexed(`VDEV_SCRATCH_DIRECT_MAPPED_ICACHE_TAGS, SCRATCHPAD_CACHED);

    // Track the next miss ID to give out.
    MULTIPLEXED#(NUM_CPUS, COUNTER#(ICACHE_MISS_ID_SIZE)) nextMissIDPool <- mkMultiplexed(mkLCounter(0));

    // A counter to track number of misses in flight. If we run out of IDs then we make the front-end retry.
    MULTIPLEXED#(NUM_CPUS, COUNTER#(ICACHE_MISS_COUNT)) outstandingMissesPool <- mkMultiplexed(mkLCounter(0));


    // ****** Rules ******
    
    // stage1_handleReq
    
    // Handle a request from the CPU and possibly access the tag store.
    
    // Ports read:
    // * pcFromFet
    
    // Ports written:
    // * None

    (* conservative_implicit_conditions *)
    rule stage1_handleReq (True);
   
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // Read input port.
        let msg_from_cpu <- pcFromFet.receive(cpu_iid);

        // Check for a request.
        case (msg_from_cpu) matches

            tagged Invalid:
            begin

                // Propogate the bubble.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

            end

            tagged Valid .req:
            begin
                
                // Look up the index in our tagStore.
                let idx = getICacheIndex(req.physicalAddress);
                iCacheTagStore.readReq(cpu_iid, idx);

                // Pass it to the next stage through the functional partition, 
                // which actually retrieves the instruction.
                getInstruction.makeReq(initFuncpReqGetInstruction(req.ctx_id, req.physicalAddress, req.offset));
                
                // Continue this instance in the next stage.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_access req);

            end
    
        endcase
    
    endrule

    // stage2_rspAndFill
    
    // Get the response from the scratchpad and also perform the fill from memory.
    
    // Ports read:
    // * rspFromMemory
    
    // Ports written:
    // * immToFet
    // * delToFet
    // * reqToMemory
    

    rule stage2_rsp (True);

        // Get the next instance id.
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get our local state from the instance ID.
        let nextMissID = nextMissIDPool[cpu_iid];
        let outstandingMisses = outstandingMissesPool[cpu_iid];

        if (state matches tagged STAGE2_bubble)
        begin
        
            // Propogate the bubble
            immToFet.send(cpu_iid, tagged Invalid);
            reqToMemory.send(cpu_iid, tagged Invalid);

        end
        else if (state matches tagged STAGE2_access .bundle)
        begin
        
            // Get the response from the tagStore.
            let m_existing_tag <- iCacheTagStore.readRsp(cpu_iid);

            // Get the response from the functional partition.
            let rsp = getInstruction.getResp();
            getInstruction.deq();

            // See if the tag matches.
            let target_tag = getICacheTag(bundle.physicalAddress);
            Bool hit = m_existing_tag matches tagged Valid .existing_tag &&& (existing_tag == target_tag) ? True : False ;

            if (hit || alwaysClaimHit())
            begin

                // A hit, so no request to memory.
                immToFet.send(cpu_iid, tagged Valid initICacheHit(bundle, rsp.instruction));
                reqToMemory.send(cpu_iid, tagged Invalid);

                statHits.incr(cpu_iid);

            end
            else
            begin

                // A miss, so try to get a Miss ID.
                let num_outstanding = outstandingMisses.value();
                
                // We have a miss to allocate if the high bit is not 1.
                let can_allocate = num_outstanding[valueof(ICACHE_MISS_ID_SIZE)] == 0;
                
                if (can_allocate)
                begin
                    // Allocate the next miss ID and give it to fetch.
                    let miss_id = nextMissID.value();
                    nextMissID.up();
                    outstandingMisses.up();
                    immToFet.send(cpu_iid, tagged Valid initICacheMiss(miss_id, bundle));
                    
                    // Send the request to memory with the appropriate miss ID.
                    reqToMemory.send(cpu_iid, tagged Valid initICacheMissRsp(miss_id, bundle, rsp.instruction));
                    statMisses.incr(cpu_iid);

                end
                else
                begin
                    // Since we're out of misses, the CPU must retry.
                    // This should be pretty rare.
                    immToFet.send(cpu_iid, tagged Valid initICacheRetry(bundle));
                    
                    // No request to memory.
                    reqToMemory.send(cpu_iid, tagged Invalid);
                    
                end        

            end

        end
        
        stage3Ctrl.ready(cpu_iid);
        
    endrule
    
    rule stage3_fill (True);
    
        let cpu_iid <- stage3Ctrl.nextReadyInstance();
    
        // Get our local state from the instance ID.
        let outstandingMisses = outstandingMissesPool[cpu_iid];

        // Check for fills.
        let m_fill <- rspFromMemory.receive(cpu_iid);
        
        if (m_fill matches tagged Valid .fill)
        begin
       
            // Get the index and tag.
            let idx = getICacheIndex(fill.bundle.physicalAddress);
            let tag = getICacheTag(fill.bundle.physicalAddress);
       
            // Record that the data is now in the cache.
            iCacheTagStore.write(cpu_iid, idx, tagged Valid tag);
            
            // One less miss is outstanding.
            outstandingMisses.down();
       
            // Send the fill back to fetch.
            delToFet.send(cpu_iid, tagged Valid fill);
                
        end
        else
        begin
            
            // No fill.
            delToFet.send(cpu_iid, tagged Invalid);
        
        end

        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule
