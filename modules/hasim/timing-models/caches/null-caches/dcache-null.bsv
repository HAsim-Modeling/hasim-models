import FIFO::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/funcp_interface.bsh"

`include "asim/provides/memory_base_types.bsh"

module [HASIM_MODULE] mkDCache();


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

    // communication with local controller
    Vector#(2, PORT_CONTROLS) inports = newVector();
    Vector#(4, PORT_CONTROLS) outports = newVector();
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = storeReqFromCPU.ctrl;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = storeRspImmToCPU.ctrl;
    outports[3] = storeRspDelToCPU.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Rules ******

    // stage1_loadReq
    
    // Stores always hit in the cache. Loads also always hit but we also 

    rule stage1_loadReq (True);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // read store input port
        let store_from_cpu <- storeReqFromCPU.receive(ctx);

        // check request type
        case (store_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        storeRspImmToCPU.send(ctx, tagged Invalid);
	        storeRspDelToCPU.send(ctx, tagged Invalid);

	    end

	    tagged Valid {.tok, .addr}:
	    begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We always hit.

	        storeRspImmToCPU.send(ctx, tagged Valid initDCacheStoreOk(tok));
	        storeRspDelToCPU.send(ctx, tagged Invalid);

	    end

        endcase

        // read load input port
        let msg_from_cpu <- loadReqFromCPU.receive(ctx);

        // check request type
        case (msg_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        loadRspImmToCPU.send(ctx, tagged Invalid);
	        loadRspDelToCPU.send(ctx, tagged Invalid);
                
                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(ctx, 1);

	    end

	    tagged Valid .req:
	    begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We always hit.

	        loadRspImmToCPU.send(ctx, tagged Valid initDCacheLoadHit(req));
	        loadRspDelToCPU.send(ctx, tagged Invalid);

                // End of model cycle. (Path 2)
                localCtrl.endModelCycle(ctx, 2);

	    end

        endcase
         
     

    endrule


endmodule
