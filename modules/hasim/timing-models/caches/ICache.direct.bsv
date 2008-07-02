import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import hasim_cache_memory::*;
import fpga_components::*;

`include "asim/provides/funcp_base_types.bsh"

/* Cache parameters -- insert in AWB file */
`define ICACHE_LINE_BITS 5   // 32 bytes in cache line
`define ICACHE_IDX_BITS 8     // Number of index bits

typedef Bit#(TSub#(`FUNCP_ISA_ADDR_SIZE, TAdd#(`ICACHE_LINE_BITS, `ICACHE_IDX_BITS))) ICACHE_TAG;
typedef Bit#(`ICACHE_LINE_BITS) ICACHE_LINE_OFFSET;
typedef Bit#(`ICACHE_IDX_BITS) ICACHE_INDEX;

module [HASim_Module] mkICache();
   
   // initialize cache memory
   let cachememory <- mkCacheMemory();
   
   // get a BRAM unit
   BRAM#(`ICACHE_IDX_BITS, Maybe#(ICACHE_TAG)) icache_tag_store <-  mkBramInitialized(tagged Invalid);
   
   // register to hold cache tag/index
   Reg#(ICACHE_TAG) req_icache_tag<- mkReg(0);
   Reg#(ICACHE_INDEX) req_icache_index <- mkReg(0);
   Reg#(TOKEN) req_tok <- mkRegU;
   Reg#(Maybe#(ISA_ADDRESS)) req_addr <- mkReg(tagged Invalid);
   
   /* incoming ports */
   // incoming port from the CPU with 0 latency
   Port_Receive#(Tuple2#(TOKEN, Maybe#(ISA_ADDRESS))) port_from_cpu <- mkPort_Receive("cpu_to_icache", 0); 
   
   // incoming port from memory with 10 latency
   Port_Receive#(Tuple2#(TOKEN, Maybe#(ISA_INSTRUCTION))) port_from_memory <- mkPort_Receive("memory_to_icache", 10);
   
   /* outgoing ports */
   // outgoing port to the CPU with 0 latency
   Port_Send#(Tuple2#(TOKEN, Maybe#(ISA_INSTRUCTION))) port_to_cpu <- mkPort_Send("icache_to_cpu");
   
   // outgoing port to memory with 10 latency
   Port_Send#(Tuple2#(TOKEN, Maybe#(ISA_ADDRESS))) port_to_memory <- mkPort_Send("icache_to_memory");
   
   // communication with local controller 
   Vector#(2, Port_Control) inports  = newVector();
   Vector#(2, Port_Control) outports = newVector();
   inports[0]  = port_from_cpu.ctrl;
   inports[1] = port_from_memory.ctrl;
   outports[0] = port_to_cpu.ctrl;
   outports[1] = port_to_memory.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports); 
   
   // rules
   rule icachefsm (True);
      
      // read request from cpu
      let req_from_cpu <- port_from_cpu.receive(); 
      let resp_from_memory <- port_from_memory.receive();
      
      case (resp_from_memory) matches 
	 // if main memory is not supplying any data
	 tagged Invalid:
	    case (req_from_cpu) matches
	       // if NoMessage accepted from CPU
	       tagged Invalid:
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu.send(tagged Invalid);
		  end
	       // if message accepted from CPU
	       tagged Valid {.tok_from_cpu, .addr_from_cpu}:
		  begin
		     //let idx = get_idx(addr_from_cpu);  
		     //req_icache_tag <= get_tag(addr_from_cpu);
		     case (addr_from_cpu) matches
			tagged Valid .addr:
			   begin
			      Tuple3#(ICACHE_TAG, ICACHE_INDEX, ICACHE_LINE_OFFSET) address_tup = unpack(addr);
			      match {.tag, .idx, .line_off} = address_tup;
			      req_icache_tag <= tag;
			      req_icache_index <= idx;
			      req_tok <= tok_from_cpu;
			      req_addr <= addr_from_cpu;
			      icache_tag_store.readReq(idx);
			   end
		     endcase
		  end  
	    endcase
	 
	 // if main memory is supplying data
	 tagged Valid {.tok_from_memory, .inst_from_memory}:
	    begin
	       port_to_memory.send(tagged Invalid);
	       port_to_cpu.send(tagged Valid tuple2(tok_from_memory, inst_from_memory));
	    end
      endcase
   endrule
   
     
   rule tagcompare;
      let tagmatch <- icache_tag_store.readResp();
      case (tagmatch) matches
	 tagged Invalid:	 // cache miss       
   	    begin			
	       port_to_memory.send(tagged Valid tuple2(req_tok, req_addr));
	       port_to_cpu.send(tagged Invalid);			  
	       $display("Cache miss tag: %x, index: %d", req_icache_tag, req_icache_index);
	       icache_tag_store.write(req_icache_index, tagged Valid req_icache_tag);
	    end
	 tagged Valid .tag_stored:
	    begin
	       if (tag_stored == req_icache_tag)  // cache hit
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu.send(tagged Valid tuple2(req_tok, tagged Valid 0));
		     $display("Cache hit tag: %x, index: %d", req_icache_tag, req_icache_index);
		  end
	       else // cache miss
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok, req_addr));
		     port_to_cpu.send(tagged Invalid);			  
		     $display("Cache miss tag: %x, index: %d", req_icache_tag, req_icache_index);    
		     icache_tag_store.write(req_icache_index, tagged Valid req_icache_tag);
		  end
	    end
      endcase
   endrule
endmodule

	 
	 