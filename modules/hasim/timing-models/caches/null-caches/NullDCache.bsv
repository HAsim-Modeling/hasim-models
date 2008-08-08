import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import fpga_components::*;

module [HASIM_MODULE] mkDCache();
   
   // incoming port from CPU with speculative stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_spec <- mkPort_Receive("cpu_to_dcache_speculative", 0);
   
   // incoming port from CPU with committed stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_comm <- mkPort_Receive("cpu_to_dcache_committed", 0);
   
   // outgoing port to CPU with speculative immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm_spec <- mkPort_Send("dcache_to_cpu_immediate_speculative");
   
   // outgoing port to CPU with speculative delayed response
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del_spec <- mkPort_Send("dcache_to_cpu_delayed_speculative");
   
   // outgpong port to CPU with commit immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm_comm <- mkPort_Send("dcache_to_cpu_immediate_committed");
   
   // outgoing port to CPU with commit delayed response
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del_comm <- mkPort_Send("dcache_to_cpu_delayed_committed");
   
   // communication with local controller
   Vector#(2, Port_Control) inports = newVector();
   Vector#(4, Port_Control) outports = newVector();
   inports[0] = port_from_cpu_spec.ctrl;
   inports[1] = port_from_cpu_comm.ctrl;
   outports[0] = port_to_cpu_imm_spec.ctrl;
   outports[1] = port_to_cpu_del_spec.ctrl;
   outports[2] = port_to_cpu_imm_comm.ctrl;
   outports[3] = port_to_cpu_del_comm.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   
   rule pass;
      
      // read message from incoming ports
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      case (tuple2(msg_from_cpu_spec, msg_from_cpu_comm)) matches
	 
	 {tagged Invalid,
	  tagged Invalid}:
	     begin
		port_to_cpu_imm_spec.send(tagged Invalid);
		port_to_cpu_del_spec.send(tagged Invalid);
		port_to_cpu_imm_comm.send(tagged Invalid);
		port_to_cpu_del_comm.send(tagged Invalid);
	     end
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec},
	  tagged Invalid}:
	     begin
		case (req_from_cpu_spec) matches
		   tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec}:
		      begin
			 port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Hit cpu_addr_spec));
			 port_to_cpu_del_spec.send(tagged Invalid);
			 port_to_cpu_imm_comm.send(tagged Invalid);
			 port_to_cpu_del_comm.send(tagged Invalid);
		      end
		endcase
	     end
	 {tagged Invalid, 
	  tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
	     begin
		case (req_from_cpu_comm) matches
		   tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm}:
		      begin
			 port_to_cpu_imm_spec.send(tagged Invalid);
			 port_to_cpu_del_spec.send(tagged Invalid);
			 port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Hit cpu_addr_comm));
			 port_to_cpu_del_comm.send(tagged Invalid);
		      end
		endcase
	     end
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec},
	  tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
	     begin
		case (tuple2(req_from_cpu_spec, req_from_cpu_comm)) matches
		   {tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec},
		    tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm}}:
		       begin
			  port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Hit cpu_addr_spec));
			  port_to_cpu_del_spec.send(tagged Invalid);
			  port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Hit cpu_addr_comm));
			  port_to_cpu_del_comm.send(tagged Invalid);
		       end
		endcase
	     end
      endcase
   endrule
endmodule
	  
	  
	  
	    
	 
			     
