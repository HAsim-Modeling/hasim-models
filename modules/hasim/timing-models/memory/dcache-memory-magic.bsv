import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"

typedef union tagged {
   ISA_ADDRESS Mem_fetch;
		      }
MemInput deriving (Eq, Bits);

typedef union tagged {
   ISA_ADDRESS ValueRet;
		      }
MemOutput deriving (Eq, Bits);

module [HASIM_MODULE] mkDCacheMemory();
   // incoming ports
   Port_Receive#(Tuple2#(TOKEN, MemInput)) port_from_dcache <- mkPort_Receive("dcache_to_memory", 1);
   
   // outgoing ports
   Port_Send#(Tuple2#(TOKEN, MemOutput)) port_to_dcache <- mkPort_Send("memory_to_dcache");
   
   // communication with local controller 
   Vector#(1, Port_Control) inports  = newVector();
   Vector#(1, Port_Control) outports = newVector();
   inports[0]  = port_from_dcache.ctrl;
   outports[0] = port_to_dcache.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   rule dummymemory;
      let mem_req <- port_from_dcache.receive();
      case (mem_req) matches
	 
	 tagged Invalid:
	    begin
	       port_to_dcache.send(tagged Invalid);
	    end
	 tagged Valid {.req_tok, .req_address}:
	    begin
	       case(req_address) matches
		  tagged Mem_fetch .req_pc:
		     begin
			port_to_dcache.send(tagged Valid tuple2(req_tok, tagged ValueRet req_pc));
		     end
	       endcase		  
	    end	 
      endcase
   endrule   
endmodule
