import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/memory_base_types.bsh"

module [HASIM_MODULE] mkDCache();

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTICTX#(CACHE_INPUT) specFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_speculative", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTICTX#(CACHE_INPUT) commFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_committed", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTICTX#(CACHE_OUTPUT_IMMEDIATE) specImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_speculative_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTICTX#(CACHE_OUTPUT_DELAYED) specDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_speculative_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTICTX#(CACHE_OUTPUT_IMMEDIATE) commImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_committed_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTICTX#(CACHE_OUTPUT_DELAYED) commDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_committed_delayed");

    // communication with local controller
    Vector#(2, PORT_CONTROLS) inports = newVector();
    Vector#(4, PORT_CONTROLS) outports = newVector();
    inports[0] = specFromCPU.ctrl;
    inports[1] = commFromCPU.ctrl;
    outports[0] = specImmToCPU.ctrl;
    outports[1] = specDelToCPU.ctrl;
    outports[2] = commImmToCPU.ctrl;
    outports[3] = commDelToCPU.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Rules ******

    // stage1_pass
    
    // Always hit in the cache.

    rule stage1_pass (True);

        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // Read message from incoming ports.
        let msg_from_cpu_spec <- specFromCPU.receive(ctx);
        let msg_from_cpu_comm <- commFromCPU.receive(ctx);
        
        // Handle speculative loads.
        // (Stores do nothing.)
        if (msg_from_cpu_spec matches tagged Valid .req_from_cpu_spec &&&
            req_from_cpu_spec.reqType matches tagged CACHE_loadData {.cpu_addr_spec, .ref_addr_spec})
	begin
        
	    specImmToCPU.send(ctx, tagged Valid initCacheHit(req_from_cpu_spec.token, ref_addr_spec));
	    specDelToCPU.send(ctx, tagged Invalid);
	end
        else
        begin
        
	    specImmToCPU.send(ctx, tagged Invalid);
	    specDelToCPU.send(ctx, tagged Invalid);

        end
        
        // Handle non-speculative stores from commit.
        // (Committing Stores does nothing.)
        if (msg_from_cpu_comm matches tagged Valid .req_from_cpu_comm &&&
            req_from_cpu_comm.reqType matches tagged CACHE_writeData {.cpu_addr_comm, .ref_addr_comm})
	begin
        
	    commImmToCPU.send(ctx, tagged Valid initCacheHit(req_from_cpu_comm.token, ref_addr_comm));
	    commDelToCPU.send(ctx, tagged Invalid);
	
        end
        else
        begin
        
	    commImmToCPU.send(ctx, tagged Invalid);
	    commDelToCPU.send(ctx, tagged Invalid);
	
        end
        
        localCtrl.endModelCycle(ctx, 1);

    endrule

endmodule
