import Vector::*;
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

import hasim_isa::*;
import hasim_dcache_memory::*;
import fpga_components::*;

`include "asim/provides/hasim_dcache_types.bsh"
`include "asim/provides/hasim_dcache_memory.bsh"
`include "asim/provides/hasim_icache.bsh"

typedef enum {HandleReq, HandleRead, HandleWrite, ReadStall, WriteStall} State deriving (Eq, Bits);
typedef enum {CLEAN, DIRTY} DIRTY_BIT deriving (Eq, Bits);

module [HASim_Module] mkDCache();

   // initialize cache memory
   let cachememory <- mkDCacheMemory();
   
   // state register
   Reg#(State) state <- mkReg(HandleReq);
   
   // BRAM for cache tag store
   BRAM#(`DCACHE_IDX_BITS, Maybe#(Tuple2#(DIRTY_BIT, DCACHE_TAG))) dcache_tag_store <- mkBramInitialized(tagged Invalid);
   
   // registers to hold cache request fields
   Reg#(DCACHE_TAG) req_dcache_tag <- mkReg(0);
   Reg#(DCACHE_INDEX) req_dcache_index <- mkReg(0);
   Reg#(TOKEN) req_tok <- mkRegU();
   Reg#(DATA_ADDRESS) req_dcache_addr <- mkReg(0);
   Reg#(INST_ADDRESS) inst_addr <- mkReg(0);
   
   /* incoming ports */
   // incoming port from CPU
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu <- mkPort_Receive("cpu_to_dcache", 0);
   
   // incoming port from memory
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_dcache", 10);
   
   /* outgoing ports */
   // outgoing port to cpu with immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Send("dcache_to_cpu_immediate");
   
   // outgoing port to cpu with delayed response
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Send("dcache_to_cpu_delayed");
   
   // outgoing port to memory
   Port_Send#(Tuple2#(TOKEN, MemInput)) port_to_memory <- mkPort_Send("dcache_to_memory");
   
   // communication with local controller
   Vector#(2, Port_Control) inports = newVector();
   Vector#(3, Port_Control) outports = newVector();
   inports[0] = port_from_cpu.ctrl;
   inports[1] = port_from_memory.ctrl;
   outports[0] = port_to_cpu_imm.ctrl;
   outports[1] = port_to_cpu_del.ctrl;
   outports[2] = port_to_memory.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // rules
   rule handlereq (state == HandleReq);
      
      // read port from cpu
      let msg_from_cpu <- port_from_cpu.receive();
      
      // read port from memory
      let msg_from_mem <- port_from_memory.receive();
      
      case (tuple2(msg_from_cpu, msg_from_mem)) matches
	 // if there is no action taking place on the cache
	 {tagged Invalid, tagged Invalid}:
			     begin
				port_to_cpu_imm.send(tagged Invalid);
				port_to_cpu_del.send(tagged Invalid);
				port_to_memory.send(tagged Invalid);
			     end
	 // memory returning value while cpu is not making request
	 // current not handled in this rule
	 {tagged Invalid, tagged Valid {.tok_from_mem, .resp_from_memory}}:
			     begin
				// not implemented
			     end
	 
	 // cpu making request while memory is not returning value
	 {tagged Valid {.tok_from_cpu, .req_from_cpu}, tagged Invalid}:
							 begin
							 // check the request type
							    case (req_from_cpu) matches
							       // standard memory read
							       tagged Data_read_mem_ref {.cpu_addr, .ref_addr}:
								  begin
								     Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr);
								     match {.tag, .idx, .line_offset} = address_tup;
								     req_dcache_tag <= tag;
								     req_dcache_index <= idx;
								     req_dcache_addr <= ref_addr;
								     req_tok <= tok_from_cpu;
								     inst_addr <= cpu_addr;
								     dcache_tag_store.readReq(idx);
								     state <= HandleRead;
								     //$display ("Read request %x", cpu_addr);
								  end
							       // standard memory write
							       tagged Data_write_mem_ref {.cpu_addr, .ref_addr}:
								  begin
								     Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr);
								     match {.tag, .idx, .line_offset} = address_tup;
								     req_dcache_tag <= tag;
								     req_dcache_index <= idx;
								     req_dcache_addr <= ref_addr;
								     req_tok <= tok_from_cpu;
								     inst_addr <= cpu_addr;
								     dcache_tag_store.readReq(idx);
								     state <= HandleWrite;
								     //$display ("Write request %x", inst_addr);     
								  end
							       // data prefetch read
							       tagged Data_read_prefetch_ref .ref_addr:
								  begin
								     // not currently implemented
								  end
							       // invalidate line
							       tagged Invalidate_line .ref_addr:
								  begin
								     Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr);
								     match {.tag, .idx, .line_offset} = address_tup;
								     dcache_tag_store.write(idx, tagged Invalid);
								  end
							       // invalidate entire cache
							       tagged Invalidate_all .ref_addr:
								  begin
								     // not currently implemented
								  end
							       // flush line
							       tagged Flush_line .ref_addr:
								  begin
								     // implement this
								  end
							       // flush all
							       tagged Flush_all .ref_addr:
								  begin
								     // implement this
								  end
							       // kill all
							       tagged Kill_all .ref_addr:
								  begin
								     // implement this
								  end					       
							    endcase
							 end
	 // cpu making a request while memory is returning a value
	 // currently not implemented since this is a blocking cache
	 {tagged Valid {.tok_from_cpu, .req_from_cpu}, tagged Valid {.tok_from_mem, .resp_from_mem}}:
							  begin
							     // not implemented
							  end
      endcase
   endrule
							       
   rule handleread (state == HandleRead);
      // read the tag from bram
      let tagstore <- dcache_tag_store.readResp();
      case (tagstore) matches
	 // cold read miss
	 tagged Invalid: 
	    begin
	       port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch inst_addr));
	       port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing inst_addr));
	       port_to_cpu_del.send(tagged Invalid);
	       dcache_tag_store.write(req_dcache_index, tagged Valid tuple2(CLEAN, req_dcache_tag));
	       state <= ReadStall;
	       //$display ("Read Miss!");
	    end
	 
	 tagged Valid {.ret_dirty, .ret_tag}:
	    begin
	       // read hit
	       if (ret_tag == req_dcache_tag)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit inst_addr));
		     port_to_cpu_del.send(tagged Invalid);
		     state <= HandleReq;
		     //$display ("Read Hit!");
		  end
	       // read miss
	       else
		  begin
		     if (ret_dirty == CLEAN)
			begin
			   port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch inst_addr));
			   port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing inst_addr));
			   port_to_cpu_del.send(tagged Invalid);
			   dcache_tag_store.write(req_dcache_index, tagged Valid tuple2(CLEAN, req_dcache_tag));
			   state <= ReadStall;
			   //$display ("Read Mss!");
			end
		     else
			begin
			   
			  // flush dirty line and then read new line in
			   
			end
		  end
	    end
      endcase
   endrule


   rule readstall (state == ReadStall);
   
      // read incoming memory port
      let msg_from_mem <- port_from_memory.receive();
      
      // read incoming cpu port
      let msg_from_cpu <- port_from_cpu.receive();
      
      // check what memory is sending
      case (msg_from_mem) matches
	 // memory is servicing previous read request
	 tagged Invalid:
	    begin
	       port_to_cpu_imm.send(tagged Invalid);
	       port_to_cpu_del.send(tagged Invalid);
	       port_to_memory.send(tagged Invalid);
	       //$display ("Stall");
	    end
	 // memory is returning a response
	 tagged Valid {.tok_from_memory, .resp_from_memory}:
	    begin
	       if(resp_from_memory matches tagged ValueRet .pc_from_mem)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Invalid);
		     port_to_cpu_del.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response pc_from_mem));
		     state <= HandleReq;
		     //$display ("Return data");
		  end
	    end
      endcase
   endrule
      
   rule handlewrite (state == HandleWrite);
      
      // read the tag from bram
      let tagstore <- dcache_tag_store.readResp();
      
      case (tagstore) matches 
	 // cold write miss
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Invalid);
	       port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit));  // although not strictly a hit, in a WB cache, has this behavior
	       port_to_cpu_del.send(tagged Invalid);				   
	       dcache_tag_store.write(req_dcache_index, tagged Valid tuple2(DIRTY, req_dcache_tag));
	    end
	 tagged Valid {.ret_dirty, .ret_tag}:
	    begin
	       // write hit, writeback
	       if (ret_tag == req_dcache_tag)
		  begin
		     port_to_memory.send(tagged Invalid));
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit inst_addr));
		     port_to_cpu_del.send(tagged Invalid);
		     dcache_tag_store.write(req_dcache_index, tagged Valid tuple2(DIRTY, req_dcache_tag));
		     
		  end
	       // write miss
	       else
		  begin
		     if (ret_dirty == DIRTY)
			begin
			   port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch inst_addr));
			   port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing inst_addr));
			   port_to_cpu_del.send(tagged Invalid);
			   dcache_tag_store.write(req_dcache_index, tagged Valid req_dcache_tag);
			   state <= WriteStall;
			end
		     else
			begin
			   // flush dirty line and then write
			end
		  end
	    end
      endcase
   endrule
   
   rule writestall (state == WriteStall);
      // read incoming memory port
      let msg_from_mem <- port_from_memory.receive();
      
      // read incoming cpu port
      let msg_from_cpu <- port_from_cpu.receive();
      
      // check what memory is sending
      case (msg_from_mem) matches
	 // memory is servicing previous write request
	 tagged Invalid:
	    begin
	       port_to_cpu_imm.send(tagged Invalid);
	       port_to_cpu_del.send(tagged Invalid);
	       port_to_memory.send(tagged Invalid);
	    end
	 // memory is returning a response
	 tagged Valid {.tok_from_memory, .resp_from_memory}:
	    begin
	       if(resp_from_memory matches tagged ValueRet .pc_from_mem)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Invalid);
		     port_to_cpu_del.send(tagged Valid tuple2(tok_from_memory, tagged Hit_response pc_from_mem));
		     state <= HandleReq;
		  end
	    end
      endcase
   endrule  
endmodule

	    
	 
  
   
	 