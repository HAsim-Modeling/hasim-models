
import Vector::*;
import FIFO::*;
import LFSR::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_interface.bsh"


`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

// ****** Generated Files ******

`include "asim/dict/PARAMS_HASIM_ICACHE.bsh"
`include "asim/dict/STATS_PSEUDORANDOM_ICACHE.bsh"


// mkICache

// An ICache module that hits or misses based on a pseudorandom number.

typedef union tagged
{
    void STAGE2_bubble;
    ICACHE_INPUT STAGE2_retry;
    Tuple2#(Bool, ICACHE_INPUT) STAGE2_access;
}
STAGE2_STATE deriving (Eq, Bits);

    

module [HASIM_MODULE] mkICache();

    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) seedParam <- mkDynamicParameter(`PARAMS_HASIM_ICACHE_ICACHE_SEED, paramNode);
    
    Param#(8) retryChanceParam <- mkDynamicParameter(`PARAMS_HASIM_ICACHE_ICACHE_RETRY_CHANCE, paramNode);
    Param#(8) missChanceParam  <- mkDynamicParameter(`PARAMS_HASIM_ICACHE_ICACHE_MISS_CHANCE, paramNode);
    
    // ****** Local Definitions *******
    
    Bit#(8) iCacheRetryChance = retryChanceParam;
    Bit#(8) iCacheMissChance  = retryChanceParam + missChanceParam;

    // ****** UnModel State ******
    
    MULTIPLEXED#(NUM_CPUS, LFSR#(Bit#(8))) iLFSRPool  <- mkMultiplexed(mkLFSR_8());
    
    Reg#(Maybe#(INSTANCE_ID#(NUM_CPUS))) initializingLFSR <- mkReg(tagged Valid 0);


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

    STAGE_CONTROLLER#(NUM_CPUS, STAGE2_STATE) stage2Ctrl <- mkStageController();

    // ****** Stats ******

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statHits       <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_HITS);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statMisses     <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_MISSES);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statRetries    <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_RETRIES);


    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSR (initializingLFSR matches tagged Valid .iid);
    
        let iLFSR = iLFSRPool[iid];

        iLFSR.seed(seedParam);
        let next_iid = iid + 1;
        if (next_iid == 0)
        begin
            initializingLFSR <= Invalid;
        end
        else
        begin
            initializingLFSR <= tagged Valid next_iid;
        end
    
    endrule

    // stage1_loadReq
    
    // Determine if stores hit or not using a pseudo-random number from an LFSR.
    
    // Ports Read:
    // * rspFromMemory
    // * pcFromFet
    
    // Ports Written:
    // * delToFet

    (* conservative_implicit_conditions *)
    rule stage1_instReq (!isValid(initializingLFSR));

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // First check for fills.
        let m_fill <- rspFromMemory.receive(cpu_iid);
        
        if (m_fill matches tagged Valid .fill)
        begin
       
            // Send the fill back to fetch.
            delToFet.send(cpu_iid, tagged Valid fill);
                
        end
        else
        begin
            
            // No fill.
            delToFet.send(cpu_iid, tagged Invalid);
        
        end

        // Now read input port.
        let msg_from_cpu <- pcFromFet.receive(cpu_iid);
        
        let iLFSR = iLFSRPool[cpu_iid];

        // Check for a request.
        case (msg_from_cpu) matches

            tagged Invalid:
            begin

                // Propogate the bubble in the next stage.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

            end

            tagged Valid .req:
            begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We use a pseudo-random number to determine hit/miss/retry.

                let rnd = iLFSR.value();
                iLFSR.next();

                if (rnd < iCacheRetryChance)
                begin

                    // They have to retry next cycle.
                    statRetries.incr(cpu_iid);
                    stage2Ctrl.ready(cpu_iid, tagged STAGE2_retry req);

                end
                else
                begin

                    // Pass it to the next stage through the functional partition, 
                    // which actually retrieves the instruction.
                    getInstruction.makeReq(initFuncpReqGetInstruction(cpu_iid, req.physicalAddress, req.offset));

                    if (rnd < iCacheMissChance)
                    begin

                        // A miss.
                        statMisses.incr(cpu_iid);

                        // Pass the miss to the next stage.
                        stage2Ctrl.ready(cpu_iid, tagged STAGE2_access tuple2(False, req));

                    end
                    else
                    begin

                        // A hit.
                        statHits.incr(cpu_iid);

                        // Pass the hit to the next stage.
                        stage2Ctrl.ready(cpu_iid, tagged STAGE2_access tuple2(True, req));

                    end

                end

            end

        endcase
         
    endrule
    
    // stage2_instRsp
    
    // Get the functional partition response (if any).
    // Calculate our response to the processor and also our request to memory, if any.
    
    // Ports Read:
    // * None
    
    // Ports Written:
    // * immToFet
    // * reqToMemory

    rule stage2_instRsp (True);
        
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        if (state matches tagged STAGE2_bubble)
        begin

            // Propogate the bubble.
            immToFet.send(cpu_iid, tagged Invalid);
            reqToMemory.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE2_retry .bundle)
        begin
        
            // Return the retry to the processor.
            immToFet.send(cpu_iid, tagged Valid initICacheRetry(bundle));
            reqToMemory.send(cpu_iid, tagged Invalid);
            
            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        // Take care of the hit or miss.
        else if (state matches tagged STAGE2_access {.hit, .bundle})
        begin
        
            // Get the response from the functional partition.
            let rsp = getInstruction.getResp();
            getInstruction.deq();

            if (hit)
            begin

                // A hit, so no request to memory.
                immToFet.send(cpu_iid, tagged Valid initICacheHit(bundle, rsp.instruction));
                reqToMemory.send(cpu_iid, tagged Invalid);

                // End of model cycle. (Path 3)
                localCtrl.endModelCycle(cpu_iid, 3);

            end
            else
            begin

                // A miss, so delay the response a bit.
                immToFet.send(cpu_iid, tagged Valid initICacheMiss(bundle));
                reqToMemory.send(cpu_iid, tagged Valid initICacheMissRsp(bundle, rsp.instruction));

                // End of model cycle. (Path 4)
                localCtrl.endModelCycle(cpu_iid, 4);

            end
        end

    endrule

endmodule
