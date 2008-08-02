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


module [HASIM_MODULE] mkICache();
   
   // incoming port from CPU
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu <- mkPort_Receive("cpu_to_icache", 0);
   
   // outgoing port to CPU
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Send("icache_to_cpu_immediate");
   
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Send("icache_to_cpu_delayed");
   
   // communication with local controller
   Vector#(1, Port_Control) inports = newVector();
   Vector#(2, Port_Control) outports = newVector();
   inports[0] = port_from_cpu.ctrl;
   outports[0] = port_to_cpu_imm.ctrl;
   outports[1] = port_to_cpu_del.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   rule pass;
      
      // read input port
      let msg_from_cpu <- port_from_cpu.receive();
      
      // check request type
      case (msg_from_cpu) matches
	 tagged Invalid:
	    begin
	       port_to_cpu_imm.send(tagged Invalid);
	       port_to_cpu_del.send(tagged Invalid);
	    end
	 tagged Valid {.tok_from_cpu, .req_from_cpu}:
	    begin
	       case (req_from_cpu) matches
		  tagged Inst_mem_ref .addr:
		     begin
			port_to_cpu_imm.send(tagged Valid tuple2(tok_from_cpu, tagged Hit addr));
			port_to_cpu_del.send(tagged Invalid);
		     end
	       endcase
	    end
      endcase
   endrule
endmodule

	       
	       
			 