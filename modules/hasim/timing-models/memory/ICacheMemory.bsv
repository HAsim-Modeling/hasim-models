import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;

typedef union tagged {
   ISA_ADDRESS Mem_fetch;
		      }
MemInput deriving (Eq, Bits);

typedef union tagged {
   ISA_ADDRESS ValueRet;
		      }
MemOutput deriving (Eq, Bits);

module [HASim_Module] mkICacheMemory();
   // incoming ports
   Port_Receive#(Tuple2#(TOKEN, MemInput)) port_from_icache <- mkPort_Receive("icache_to_memory", 1);
   
   // outgoing ports
   Port_Send#(Tuple2#(TOKEN, MemOutput)) port_to_icache <- mkPort_Send("memory_to_icache");
   
   // communication with local controller 
   Vector#(1, Port_Control) inports  = newVector();
   Vector#(1, Port_Control) outports = newVector();
   inports[0]  = port_from_icache.ctrl;
   outports[0] = port_to_icache.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   rule dummymemory;
      let mem_req <- port_from_icache.receive();
      case (mem_req) matches
	 
	 tagged Invalid:
	    begin
	       port_to_icache.send(tagged Invalid);
	    end
	 tagged Valid {.req_tok, .req_address}:
	    begin
	       case(req_address) matches
		  tagged Mem_fetch .req_pc:
		     begin
			port_to_icache.send(tagged Valid tuple2(req_tok, tagged ValueRet req_pc));
		     end
	       endcase		  
	    end	 
      endcase
   endrule   
endmodule