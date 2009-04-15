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
    
    LFSR#(Bit#(8)) iLFSR  <- mkLFSR_8();
    
    Reg#(Bool) initializingLFSR <- mkReg(True);


    // ***** Unmodel State ******
    
    FIFO#(Tuple3#(CPU_INSTANCE_ID, Bool, IMEM_BUNDLE)) stage2Q <- mkFIFO();


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

    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) inports = newVector();
    Vector#(3, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = pcFromFet.ctrl;
    inports[1]  = rspFromMemory.ctrl;
    outports[0] = immToFet.ctrl;
    outports[1] = delToFet.ctrl;
    outports[2] = reqToMemory.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    // ****** Stats ******

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statHits       <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_HITS);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statMisses     <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_MISSES);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statRetries    <- mkStatCounter_Multiplexed(`STATS_PSEUDORANDOM_ICACHE_ICACHE_RETRIES);


    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSR (initializingLFSR);
    
        iLFSR.seed(seedParam);
        initializingLFSR <= False;
    
    endrule

    // stage1_loadReq
    
    // Stores always hit in the cache. Loads also always hit but we also 

    rule stage1_instReq (!initializingLFSR);

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

        // Check for a request.
        case (msg_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        immToFet.send(cpu_iid, tagged Invalid);
                reqToMemory.send(cpu_iid, tagged Invalid);

                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(cpu_iid, 1);

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
	            immToFet.send(cpu_iid, tagged Valid initICacheRetry(req));
                    reqToMemory.send(cpu_iid, tagged Invalid);
                    statRetries.incr(cpu_iid);
                    
                    // End of model cycle. (Path 2)
                    localCtrl.endModelCycle(cpu_iid, 2);

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
                        stage2Q.enq(tuple3(cpu_iid, False, req));
                    
                    end
                    else
                    begin
                        
                        // A hit.
                        statHits.incr(cpu_iid);

                        // Pass the hit to the next stage.
                        stage2Q.enq(tuple3(cpu_iid, True, req));
                    
                    end

                end

	    end

        endcase
         
    endrule

    rule stage2_instRsp (True);
        
        // Get the response from the functional partition.
        let rsp = getInstruction.getResp();
        getInstruction.deq();

        match {.cpu_iid, .hit, .bundle} = stage2Q.first();
        stage2Q.deq();

        // Take care of the hit or miss.
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
        
     
    endrule

endmodule
