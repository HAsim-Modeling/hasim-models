import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import hasim_cache_memory::*;
import fpga_components::*;

`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/hasim_cache_memory.bsh"

typedef enum {HandleReq, TagCompare} State deriving (Eq, Bits);

typedef union tagged {
		      ISA_ADDRESS Inst_mem_ref;              // Instruction fetch from ICache
		      ISA_ADDRESS Inst_prefetch_ref;         // Instruction prefetch from ICache
		      ISA_ADDRESS Data_read_mem_ref;         // Data read from DCache
		      ISA_ADDRESS Data_write_mem_ref;        // Data write to DCache
		      ISA_ADDRESS Data_read_prefetch_ref;    // Data read prefetch
		      ISA_ADDRESS Invalidate_line;           // Message to invalidate specific cache line
		      Bool Invalidate_all;                   // Message to entire cache
		      ISA_ADDRESS Flush_line;                // Flush specific cache line            
		      Bool Flush_all;                        // Flush entire cache
		      Bool Kill_all;                         // Kill all current operations of the cache      
		      } 
CacheInput deriving (Eq, Bits);

typedef union tagged{
   ISA_ADDRESS Hit;
   ISA_ADDRESS Miss_servicing;
   ISA_ADDRESS Miss_retry;
   } 
CacheOutputImmediate deriving (Eq, Bits);

typedef union tagged{
   ISA_ADDRESS Miss_response;
   }
CacheOutputDelayed deriving (Eq, Bits);

typedef Bit#(TSub#(`FUNCP_ISA_ADDR_SIZE, TAdd#(`ICACHE_LINE_BITS, `ICACHE_IDX_BITS))) ICACHE_TAG;
typedef Bit#(`ICACHE_LINE_BITS) ICACHE_LINE_OFFSET;
typedef Bit#(`ICACHE_IDX_BITS) ICACHE_INDEX;

module [HASIM_MODULE] mkICache();
   
   // initialize cache memory
   let cachememory <- mkCacheMemory();
   
   // tag store BRAM
   // produce a BRAM containing vectors holding all the tags of each way
   BRAM#(`ICACHE_IDX_BITS, Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_TAG))) icache_tag_store <- mkBramInitialized(Vector::replicate(tagged Invalid));

   // register to hold state
   Reg#(State) state <- mkReg(HandleReq);
   Reg#(Bit#(4)) replace <- mkReg(0);
   
   // register to hold cache request fields
   Reg#(ICACHE_TAG) req_icache_tag <- mkReg(0);
   Reg#(ICACHE_INDEX) req_icache_index <- mkReg(0);
   Reg#(TOKEN) req_tok <- mkRegU();
   Reg#(ISA_ADDRESS) req_addr <- mkReg(0);
   
   // incoming port from CPU (latency 0)
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu <- mkPort_Receive("cpu_to_icache", 0);
   
   // incoming port from memory (latency 10 cycles)
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_icache", 10);
   
   // outgoing port to CPU (latency 0)
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Send("icache_to_cpu_immediate");
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Send("icache_to_cpu_delayed");
   
   // outgoing port to memory (latency 10)
   Port_Send#(Tuple2#(TOKEN, MemInput)) port_to_memory <- mkPort_Send("icache_to_memory");
   
   // communication with local controller
   Vector#(2, Port_Control) inports  = newVector();
   Vector#(3, Port_Control) outports = newVector();
   inports[0]  = port_from_cpu.ctrl;
   inports[1] = port_from_memory.ctrl;
   outports[0] = port_to_cpu_imm.ctrl;
   outports[1] = port_to_cpu_del.ctrl;
   outports[2] = port_to_memory.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports); 
   
   // rules
   rule readreq (state == HandleReq);
      
      // read request from cpu
      let req_from_cpu <- port_from_cpu.receive();
      
      // read response from memory
      let resp_from_memory <- port_from_memory.receive();
      
      // blocking cache implementation
      case (resp_from_memory) matches
	 // if main memory is not supplying data
	 tagged Invalid:
	    case(req_from_cpu) matches
	       // NoMessage so CPU is not sending a request
	       tagged Invalid:
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Invalid);
		     port_to_cpu_del.send(tagged Invalid);
		  end
	       // CPU is making a request
	       tagged Valid {.tok_from_cpu, .req_from_fetch}:
		  begin
		     case (req_from_fetch) matches
			// CPU is making standard memory reference
			tagged Inst_mem_ref .addr_from_cpu:
			   begin
			      Tuple3#(ICACHE_TAG, ICACHE_INDEX, ICACHE_LINE_OFFSET) address_tup = unpack(addr_from_cpu);
			      match {.tag, .idx, .line_offset} = address_tup;
			      req_icache_tag <= tag;
			      req_icache_index <= idx;
			      req_tok <= tok_from_cpu;
			      req_addr <= addr_from_cpu;
			      icache_tag_store.readReq(idx);
			      state <= TagCompare;
			   end
			// No current implementation of other request types
		     endcase
		  end
	    endcase
	 
	 // if main memory is supplying data
	 tagged Valid {.tok_from_memory, .inst_from_memory}:
	    begin
	       case (inst_from_memory) matches
		  tagged ValueRet .servicedpc:
		     begin
			port_to_memory.send(tagged Invalid);	       
			port_to_cpu_imm.send(tagged Invalid);
			port_to_cpu_del.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response servicedpc));
		     end
	       endcase
	    end
      endcase
   endrule
   
   rule checktag (state == TagCompare);
      let tag_from_bram <- icache_tag_store.readResp();
      Bool hit = False;

      // check if anything in the set is valid
      for (Integer w = 0; w < `ICACHE_ASSOC; w = w + 1)
	 begin
	    if (tag_from_bram[w] matches tagged Valid .ret_icache_tag &&& ret_icache_tag == req_icache_tag)
	       begin
		  hit = True;
		  //$display("Cache hit tag: %x, index: %d, way: %d", req_icache_tag, req_icache_index, w);
	       end
	 end
      
      // react to hit or miss
      
      if (hit) 
	 begin 
	    port_to_memory.send(tagged Invalid);
	    port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit req_addr));
	    port_to_cpu_del.send(tagged Invalid);
	 end
      else
	 begin 
	    port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch req_addr));
	    port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing req_addr));			  
	    port_to_cpu_del.send(tagged Invalid);
	    //$display("Cache miss tag: %x, index: %d, victim way: %d, victim tag: %x", req_icache_tag, req_icache_index, replace, fromMaybe(24, tag_from_bram[replace]));
	    
	    if (replace == `ICACHE_ASSOC)
	       begin
		  replace <= 0;
	       end
	    else
	       begin
		  //replace <= replace + 1;
	       end
	    
	    tag_from_bram[replace] = tagged Valid req_icache_tag;
	    icache_tag_store.write(req_icache_index, tag_from_bram);	   
	 end
      state <= HandleReq;
   endrule
endmodule
   
   
	    
	    
	    
      
      
