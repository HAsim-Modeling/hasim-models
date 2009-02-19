import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import fpga_components::*;

typedef enum {HandleReq, HandleReq2, HandleTag, HandleRead, HandleStall1, HandleStall2} State deriving (Eq, Bits);

typedef ISA_ADDRESS INST_ADDRESS;
typedef ISA_ADDRESS DATA_ADDRESS;

typedef union tagged {
		 INST_ADDRESS Inst_mem_ref;              // Instruction fetch from ICache
		 INST_ADDRESS Inst_prefetch_ref;         // Instruction prefetch from ICache
		 Tuple2#(INST_ADDRESS, DATA_ADDRESS) Data_read_mem_ref;         // Data read from DCache
		 Tuple2#(INST_ADDRESS, DATA_ADDRESS)  Data_write_mem_ref;        // Data write to DCache
		 Tuple2#(INST_ADDRESS, DATA_ADDRESS) Data_read_prefetch_ref;    // Data read prefetch
		 ISA_ADDRESS Invalidate_line;           // Message to invalidate specific cache line
		 Bool Invalidate_all;                   // Message to entire cache
		 ISA_ADDRESS Flush_line;                // Flush specific cache line            
		 Bool Flush_all;                        // Flush entire cache
		 Bool Kill_all;                         // Kill all current operations of the cache      
		 } 
CacheInput deriving (Eq, Bits);

typedef union tagged{
   INST_ADDRESS Hit;
   INST_ADDRESS Hit_servicing;
   INST_ADDRESS Miss_servicing;
   INST_ADDRESS Miss_retry;
   } 
CacheOutputImmediate deriving (Eq, Bits);

typedef union tagged{
   INST_ADDRESS Miss_response;
   INST_ADDRESS Hit_response;
   }
CacheOutputDelayed deriving (Eq, Bits);


// mkICache

// An ICache module that always hits.

module [HASIM_MODULE] mkICache();


    // ****** Ports ******

    // Incoming port from CPU
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheInput)) pcFromFet <- mkPortRecv_MultiCtx("CPU_to_ICache_req", 0);

    // Outgoing ports to CPU
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate)) immToFet <- mkPortSend_MultiCtx("ICache_to_CPU_immediate");
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed))   delToFet <- mkPortSend_MultiCtx("ICache_to_CPU_delayed");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS) inports = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0] = pcFromFet.ctrl;
    outports[0] = immToFet.ctrl;
    outports[1] = delToFet.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    rule stage1_pass (True);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // read input port
        let msg_from_cpu <- pcFromFet.receive(ctx);

        // check request type
        case (msg_from_cpu) matches
	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        immToFet.send(ctx, tagged Invalid);
	        delToFet.send(ctx, tagged Invalid);

	    end
	    tagged Valid {.tok_from_cpu, .req_from_cpu}:
	    begin

	        case (req_from_cpu) matches
	            tagged Inst_mem_ref .addr:
	            begin

                       // Always hit.
	               immToFet.send(ctx, tagged Valid tuple2(tok_from_cpu, tagged Hit addr));
	               delToFet.send(ctx, tagged Invalid);
	            end
	        endcase

	     end

         endcase
         
         localCtrl.endModelCycle(ctx, 1);

     endrule

endmodule

	       
	       
			 
