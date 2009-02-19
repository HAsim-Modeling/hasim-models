import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import fpga_components::*;

`include "asim/provides/hasim_icache.bsh"

module [HASIM_MODULE] mkDCache();

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheInput)) specFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_speculative", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheInput)) commFromCPU <- mkPortRecv_MultiCtx("CPU_to_DCache_committed", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate)) specImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_speculative_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed)) specDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_speculative_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate)) commImmToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_committed_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed)) commDelToCPU <- mkPortSend_MultiCtx("DCache_to_CPU_committed_delayed");

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
        if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec} &&&
            req_from_cpu_spec matches tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec})
	begin
        
	    specImmToCPU.send(ctx, tagged Valid tuple2(tok_from_cpu_spec, tagged Hit cpu_addr_spec));
	    specDelToCPU.send(ctx, tagged Invalid);
	end
        else
        begin
        
	    specImmToCPU.send(ctx, tagged Invalid);
	    specDelToCPU.send(ctx, tagged Invalid);

        end
        
        // Handle non-speculative stores from commit.
        // (Committing Stores does nothing.)
        if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm} &&&
            req_from_cpu_comm matches tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm})
	begin
        
	    commImmToCPU.send(ctx, tagged Valid tuple2(tok_from_cpu_comm, tagged Hit cpu_addr_comm));
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
