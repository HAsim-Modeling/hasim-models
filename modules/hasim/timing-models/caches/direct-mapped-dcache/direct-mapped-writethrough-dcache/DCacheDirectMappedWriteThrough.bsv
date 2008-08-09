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
`include "asim/dict/STATS_DIRECT_MAPPED_DCACHE_WRITETHROUGH.bsh"

typedef enum {HandleReq, HandleRead, HandleWrite, ReadStall, WriteStall, HandleReadWrite} State deriving (Eq, Bits);


module [HASim_Module] mkDCache();

   // initialize cache memory
   let cachememory <- mkDCacheMemory();
   
   // state register
   Reg#(State) state <- mkReg(HandleReq);
   
   // BRAM for cache tag store
   BRAM_MULTI_READ#(2, `DCACHE_IDX_BITS, Maybe#(DCACHE_TAG)) dcache_tag_store <- mkMultiReadBramInitialized(tagged Invalid);

   // registers to hold cache request fields
   Reg#(DCACHE_TAG) req_dcache_tag_spec <- mkReg(0);
   Reg#(DCACHE_INDEX) req_dcache_index_spec <- mkReg(0);
   Reg#(TOKEN) req_tok_spec <- mkRegU();
   Reg#(DATA_ADDRESS) req_dcache_addr_spec <- mkReg(0);
   Reg#(INST_ADDRESS) inst_addr_spec <- mkReg(0);
   
   Reg#(DCACHE_TAG) req_dcache_tag_comm <- mkReg(0);
   Reg#(DCACHE_INDEX) req_dcache_index_comm <- mkReg(0);
   Reg#(TOKEN) req_tok_comm <- mkRegU();
   Reg#(DATA_ADDRESS) req_dcache_addr_comm <- mkReg(0);
   Reg#(INST_ADDRESS) inst_addr_comm <- mkReg(0); 
   
   Reg#(Bool) hit <- mkReg(False);
   
   // incoming ports
   // incoming port from CPU with speculative stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_spec <- mkPort_Receive("cpu_to_dcache_speculative", 0);
   
   // incoming port from CPU with commited stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_comm <- mkPort_Receive("cpu_to_dcache_committed", 0);
   
   // incoming port from memory
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_dcache", valueOf(TSub#(`DCACHE_MISS_PENALTY, 1)));
   
   // outgoing ports
   // port to CPU with speculative request immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm_spec <- mkPort_Send("dcache_to_cpu_immediate_speculative");
   
   // port to CPU with speculative request delayed response 
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del_spec <- mkPort_Send("dcache_to_cpu_delayed_speculative");
   
   // port to CPU with commit request immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm_comm <- mkPort_Send("dcache_to_cpu_immediate_committed");
   
   // port to CPU with commit request delayed response
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del_comm <- mkPort_Send("dcache_to_cpu_delayed_committed");
   
   // outgoing port to memory
   Port_Send#(Tuple2#(TOKEN, MemInput)) port_to_memory <- mkPort_Send("dcache_to_memory");
   
   // communication with local controller
   Vector#(3, Port_Control) inports = newVector();
   Vector#(5, Port_Control) outports = newVector();
   inports[0] = port_from_cpu_spec.ctrl;
   inports[1] = port_from_cpu_comm.ctrl;
   inports[2] = port_from_memory.ctrl;
   outports[0] = port_to_cpu_imm_spec.ctrl;
   outports[1] = port_to_cpu_del_spec.ctrl;
   outports[2] = port_to_cpu_imm_comm.ctrl;
   outports[3] = port_to_cpu_del_comm.ctrl;
   outports[4] = port_to_memory.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // Stats
   Stat stat_dcache_read_hits <- mkStatCounter(`STATS_DIRECT_MAPPED_DCACHE_WRITETHROUGH_DCACHE_READ_HITS);
   Stat stat_dcache_read_misses <- mkStatCounter(`STATS_DIRECT_MAPPED_DCACHE_WRITETHROUGH_DCACHE_READ_MISSES);
   Stat stat_dcache_write_hits <- mkStatCounter(`STATS_DIRECT_MAPPED_DCACHE_WRITETHROUGH_DCACHE_WRITE_HITS);
   Stat stat_dcache_write_misses <- mkStatCounter(`STATS_DIRECT_MAPPED_DCACHE_WRITETHROUGH_DCACHE_WRITE_MISSES);
   
   
   // rules
   rule handlereq (state == HandleReq);
      
      // read message from CPU speculative port
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      
      // read message from CPU commit port
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      // read message from memory port
      let msg_from_mem <- port_from_memory.receive();     
      
      // check request type
      case (tuple2(msg_from_cpu_spec, msg_from_cpu_comm)) matches
	 // activity on only speculative port
	 {tagged Invalid, tagged Invalid}:
			     begin
				port_to_cpu_imm_spec.send(tagged Invalid);
				port_to_cpu_del_spec.send(tagged Invalid);
				port_to_cpu_imm_comm.send(tagged Invalid);
				port_to_cpu_del_comm.send(tagged Invalid);
				port_to_memory.send(tagged Invalid);
			     end
	 
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec},
	  tagged Invalid}:
	     begin
		// check memory reference type
		case (req_from_cpu_spec) matches
		   // standard memory read
		   tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec}:
		      begin
			 Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr_spec);
			 match {.tag, .idx, .line_offset} = address_tup;
			 req_dcache_tag_spec <= tag;
			 req_dcache_index_spec <= idx;
			 req_tok_spec <= tok_from_cpu_spec;
			 req_dcache_addr_spec <= ref_addr_spec;
			 inst_addr_spec <= cpu_addr_spec;
			 dcache_tag_store.req[0].read(idx);
			 state <= HandleRead;
		      end
		endcase
	     end
	 // activity on only committed port
	 {tagged Invalid, 
	  tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
	     begin
		// standard memory write
		case (req_from_cpu_comm) matches
		   // standard memory write
		   tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm}:
		      begin
			 Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr_comm);
			 match {.tag, .idx, .line_offset} = address_tup;
			 req_dcache_tag_comm <= tag;
			 req_dcache_index_comm <= idx;
			 req_tok_comm <= tok_from_cpu_comm;
			 req_dcache_addr_comm <= ref_addr_comm;
			 inst_addr_comm <= cpu_addr_comm;
			 dcache_tag_store.req[0].read(idx);
			 state <= HandleWrite;
		      end
		endcase
		// activity on both speculative and commit ports
	     end
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec}, 
	     tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
		begin
		   // read on speculative port and write on commit port
		   case (tuple2(req_from_cpu_spec, req_from_cpu_comm)) matches
		      {tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec}, 
			  tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm}}:
			     begin
				Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup_spec = unpack(ref_addr_spec);
				match {.tag_spec, .idx_spec, .line_offset_spec} = address_tup_spec;
				req_dcache_tag_spec <= tag_spec;
				req_dcache_index_spec <= idx_spec;
				req_tok_spec <= tok_from_cpu_spec;
				req_dcache_addr_spec <= ref_addr_spec;
				inst_addr_spec <= cpu_addr_spec;
				dcache_tag_store.req[0].read(idx_spec);
				
				Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup_comm = unpack(ref_addr_comm);
				match {.tag_comm, .idx_comm, .line_offset_comm} = address_tup_comm;
				req_dcache_tag_comm <= tag_comm;
				req_dcache_index_comm <= idx_comm;
				req_tok_comm <= tok_from_cpu_comm;
				req_dcache_addr_comm <= ref_addr_comm;
				inst_addr_comm <= cpu_addr_comm;
				dcache_tag_store.req[1].read(idx_comm);
				
				state <= HandleReadWrite;
			     end
		   endcase
		end
	 
      endcase
   endrule
   
   rule handleread (state == HandleRead);

      // read tag from BRAM
      let tagstore <- dcache_tag_store.resp[0].read();
      
      // check retrieved tag
      case (tagstore) matches
	 // cold read miss
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_spec));
	       port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_spec));
	       port_to_cpu_del_spec.send(tagged Invalid);
	       port_to_cpu_imm_comm.send(tagged Invalid);
	       port_to_cpu_del_comm.send(tagged Invalid);
	       dcache_tag_store.write(req_dcache_index_spec, tagged Valid req_dcache_tag_spec);
	       state <= ReadStall;
	       stat_dcache_read_misses.incr();
	    end
	 tagged Valid .dcache_tag:
	    begin
	       // read hit
	       if (dcache_tag == req_dcache_tag_spec) 
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Hit inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Invalid);
		     port_to_cpu_del_comm.send(tagged Invalid);
		     state <= HandleReq;
		     stat_dcache_read_hits.incr();
		  end
	       // cache miss
	       else
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_spec));
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Invalid);
		     port_to_cpu_del_comm.send(tagged Invalid);
		     dcache_tag_store.write(req_dcache_index_spec, tagged Valid req_dcache_tag_spec);
		     state <= ReadStall;
		     stat_dcache_read_misses.incr();
		  end
	    end
      endcase
   endrule
   
      
   rule readstall (state == ReadStall);
      
      // read incoming memory port
      let msg_from_mem <- port_from_memory.receive();
      
      // read incoming cpu speculative port
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      
      // read incoming cpu commit port
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      // check what memory is sending
      case (msg_from_mem) matches
	 // memory is servicing previous read request
	 tagged Invalid:
	    begin
	       if (msg_from_cpu_spec matches tagged Invalid)
		  port_to_cpu_imm_spec.send(tagged Invalid);
	       if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec})
		  begin
		     case (req_from_cpu_spec) matches
			tagged Data_read_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
			   begin
			      port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
			   end
		     endcase
		  end
 	       port_to_cpu_del_spec.send(tagged Invalid);
	       
	       
	       if (msg_from_cpu_comm matches tagged Invalid)
		  port_to_cpu_imm_comm.send(tagged Invalid);
	       if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
		  begin
		     case (req_from_cpu_comm) matches
			tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
			   begin
			      port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
			   end
		     endcase
		  end
	       port_to_cpu_del_comm.send(tagged Invalid); 
	       
	       port_to_memory.send(tagged Invalid);
	    end
	 // memory is returning value
	 tagged Valid {.tok_from_memory, .resp_from_memory}:
	    begin
	       if (resp_from_memory matches tagged ValueRet .pc_from_mem)
		  begin
		     if (msg_from_cpu_spec matches tagged Invalid)
			port_to_cpu_imm_spec.send(tagged Invalid);
		     if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec})
			begin
			   case (req_from_cpu_spec) matches
			      tagged Data_read_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 begin
				    port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_mem_addr));
				 end
			   endcase
			end
		     port_to_cpu_del_spec.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response pc_from_mem));
		     		     
		     if (msg_from_cpu_comm matches tagged Invalid)
			port_to_cpu_imm_comm.send(tagged Invalid);
		     if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			begin
			   case (req_from_cpu_comm) matches
			      tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 begin
				    port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
				 end
			   endcase
			end
			   
		     port_to_cpu_del_comm.send(tagged Invalid); 
		     
		     port_to_memory.send(tagged Invalid);    
		     
		     state <= HandleReq;
		  end
	    end
      endcase
   endrule
   
   
   rule handlewrite (state == HandleWrite);
      
      // read rag from BRAM
      let tagstore <- dcache_tag_store.resp[0].read();
      
      // check retrieved tag
      case (tagstore) matches
	 // cold write miss
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_comm));
	       port_to_cpu_imm_spec.send(tagged Invalid);
	       port_to_cpu_del_spec.send(tagged Invalid);
	       port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_comm));
	       port_to_cpu_del_comm.send(tagged Invalid);
	       dcache_tag_store.write(req_dcache_index_comm, tagged Valid req_dcache_tag_comm);
	       state <= WriteStall;
	       hit <= False;
	       stat_dcache_write_misses.incr();
	    end
	 tagged Valid .dcache_tag:
	    begin
	       // write hit
	       if (dcache_tag == req_dcache_tag_comm)
		  begin
		     // note that this is a write-through cache so even on a hit, we stall till write to memory
		     port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
		     port_to_cpu_imm_spec.send(tagged Invalid);
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Hit_servicing inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     state <= WriteStall;
		     hit <= True;
		     stat_dcache_write_hits.incr();
		  end
	       // cache miss
	       else
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
		     port_to_cpu_imm_spec.send(tagged Invalid);
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     dcache_tag_store.write(req_dcache_index_comm, tagged Valid req_dcache_tag_comm);
		     state <= WriteStall;
		     hit <= False;
		     stat_dcache_write_misses.incr();
		  end
	    end
      endcase
   endrule
   
   rule writestall (state == WriteStall);
      
      // read incoming memory port
      let msg_from_mem <- port_from_memory.receive();
      
      // read incoming cpu speculative request
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      
      // read incoming cpu commit request
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      // check what memory is sending
      case (msg_from_mem) matches
	 // memory is servicing previous write request
	 tagged Invalid:
	    begin
	       if (msg_from_cpu_spec matches tagged Invalid)
		  port_to_cpu_imm_spec.send(tagged Invalid);
	       if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec})
		  begin
		     case (req_from_cpu_spec) matches
			tagged Data_read_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
			   begin
			      port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_mem_addr));
			   end
		     endcase
		  end
	       port_to_cpu_del_spec.send(tagged Invalid);
	       
	       if (msg_from_cpu_comm matches tagged Invalid)
		  port_to_cpu_imm_comm.send(tagged Invalid);
	       if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
		  begin
		     case (req_from_cpu_comm) matches
			tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
			   begin
			      port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
			   end
		     endcase
		  end
	       port_to_cpu_del_comm.send(tagged Invalid);	       
	       
	       port_to_memory.send(tagged Invalid);
	    end
	 // memory has returned previous request
	 tagged Valid {.tok_from_memory, .resp_from_memory}:
	    begin
               if (resp_from_memory matches tagged ValueRet .pc_from_mem)
		  begin
		     if (msg_from_cpu_spec matches tagged Invalid)
			port_to_cpu_imm_spec.send(tagged Invalid);
		     if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec})
			begin
			   case (req_from_cpu_spec) matches
			      tagged Data_read_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 begin
				    port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_mem_addr));
				 end
			   endcase
			end
		     port_to_cpu_del_spec.send(tagged Invalid);
					  
		     if (msg_from_cpu_comm matches tagged Invalid)
			port_to_cpu_imm_comm.send(tagged Invalid);
		     if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			begin
			   case (req_from_cpu_comm) matches
			      tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 begin
				    port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
				 end
			   endcase
			end
			      
		     if (hit)
			port_to_cpu_del_comm.send(tagged Valid tuple2(tok_from_memory, tagged Hit_response pc_from_mem)); 
		     else
			port_to_cpu_del_comm.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response pc_from_mem)); 
		     
		     port_to_memory.send(tagged Invalid);    
		     
		     state <= HandleReq;
		  end
	    end
      endcase
   endrule      
					       
   rule handlereadwrite (state == HandleReadWrite);
					       
      // get stored tag for read request
      let tagstore_read <- dcache_tag_store.resp[0].read();
      
      // get stored tag for write request
      let tagstore_write <- dcache_tag_store.resp[1].read();
      
      // cache is unable to distinguish conflicting read/write values
      // the commit port has priority so we send a miss retry back to pipeline for the speculative case
      case (tagstore_write) matches
	 // cold write miss
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
	       port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
	       port_to_cpu_del_spec.send(tagged Invalid);
	       port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
	       port_to_cpu_del_comm.send(tagged Invalid);
	       dcache_tag_store.write(req_dcache_index_comm, tagged Valid req_dcache_tag_comm);
	       state <= WriteStall;
	       stat_dcache_write_misses.incr();
	    end
	 tagged Valid .dcache_tag:
	    begin
	       // write hit
	       // write through
	       if (dcache_tag == req_dcache_tag_comm)
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Hit_servicing inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     dcache_tag_store.write(req_dcache_index_comm,tagged Valid req_dcache_tag_comm);
		     state <= WriteStall;
		     stat_dcache_write_hits.incr();
		  end
	       // write miss
	       else
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     dcache_tag_store.write(req_dcache_index_comm,tagged Valid req_dcache_tag_comm);
		     state <= WriteStall;
		     stat_dcache_write_misses.incr();
		  end
	    end
      endcase
   endrule
endmodule

	       
		     
		     
	       
	 
	       
	       
 
		     
	 
		     
	    
							      

								       
															       
																	       
									  
																		    
												
												
												
							     
   
  
   
	 