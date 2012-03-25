//
// Copyright (C) 2012 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/hasim_dcache_memory.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/hasim_dcache_base_types.bsh"
`include "asim/provides/hasim_dcache_memory.bsh"
`include "asim/provides/hasim_icache.bsh"
`include "asim/provides/hasim_dcache_replacement_algorithm.bsh"


typedef enum {HandleReq, NullReq, CheckTag, HandleRead, HandleWrite, ReadStall, WriteStall, HandleReadWrite, Flush} State deriving (Eq, Bits);
typedef enum {Null, Read, Write, ReadWrite} RequestType deriving (Eq, Bits);

module [HASIM_MODULE] mkDCache();
   
   // initialize cache memory
   let cachememory <- mkDCacheMemory();
   
   // state register
   Reg#(State) state <- mkReg(HandleReq);
      
   // BRAM for cache tag store
   BRAM_MULTI_READ#(2, Bit#(`DCACHE_IDX_BITS), Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE))) dcache_tag_store <- mkBRAMMultiReadInitialized(Vector::replicate(tagged Invalid));

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
   
   Reg#(Bool) req_hit <- mkReg(False);
   Reg#(DCACHE_WAY) req_hit_way <- mkReg(0);
   Reg#(Bool) read <- mkReg(False);
   Reg#(RequestType) req_type <- mkReg(Null);
   Reg#(Bool) stall_phase <- mkReg(False);
   
   // instantiate replacement algorithm module
   let replacementmodule <- mkDCacheReplacementAlgorithm();
   
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
   
   // incoming port from replacement algorithm
   Port_Receive#(Tuple2#(TOKEN, DCacheReplacementAlgorithmOutput)) port_from_replacement_alg <- mkPort_Receive("replacement_algorithm_to_dcache_core", 0);
   
   // outgoing port to replacement algorithm
   Port_Send#(Tuple2#(TOKEN, DCacheReplacementAlgorithmInput)) port_to_replacement_alg <- mkPort_Send("dcache_core_to_replacement_algorithm");
   
   // communication with local controller
   Vector#(4, Port_Control) inports = newVector();
   Vector#(6, Port_Control) outports = newVector();
   inports[0] = port_from_cpu_spec.ctrl;
   inports[1] = port_from_cpu_comm.ctrl;
   inports[2] = port_from_memory.ctrl;
   inports[3] = port_from_replacement_alg.ctrl;
   outports[0] = port_to_cpu_imm_spec.ctrl;
   outports[1] = port_to_cpu_del_spec.ctrl;
   outports[2] = port_to_cpu_imm_comm.ctrl;
   outports[3] = port_to_cpu_del_comm.ctrl;
   outports[4] = port_to_memory.ctrl;
   outports[5] = port_to_replacement_alg.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // Stats
   STAT_ID statIDs[4] = {
       statName("SETASSOC_DCACHE_WRITEBACK_READ_HITS", "DCache Read Hits"),
       statName("SETASSOC_DCACHE_WRITEBACK_READ_MISSES", "DCache Read Misses"),
       statName("SETASSOC_DCACHE_WRITEBACK_WRITE_HITS", "DCache Write Hits"),
       statName("SETASSOC_DCACHE_WRITEBACK_WRITE_MISSES", "DCache Write Misses")
   };

   STAT_VECTOR#(4) stats <- mkStatCounter_Vector(statIDs);
   let stat_dcache_read_hits = 0;
   let stat_dcache_read_misses = 1;
   let stat_dcache_write_hits = 2;
   let stat_dcache_write_misses = 3;
   
   // rules
   rule handlereq (state == HandleReq);
      
      // read cpu speculative port
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      
      // read cpu commit port
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      // read memory port
      let msg_from_mem <- port_from_memory.receive();
      
      // check request type
      case (tuple2(msg_from_cpu_spec, msg_from_cpu_comm)) matches
	 // no activity on port
	 {tagged Invalid, tagged Invalid}:
			     begin
				port_to_replacement_alg.send(tagged Invalid);
				state <= NullReq;		
			     end
	 // activity on only speculative port
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec},
	  tagged Invalid}:
	     begin
		// check request type
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
			 dcache_tag_store.readPorts[0].readReq(idx);
			 state <= CheckTag;
			 req_type <= Read;
		      end
		endcase
	     end
	 // activity on only commit port
	 {tagged Invalid, 
	  tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
	     begin
		// check request type
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
			 dcache_tag_store.readPorts[0].readReq(idx);
			 state <= CheckTag;
			 req_type <= Write;
		      end
		endcase
	     end
	 // activity on both speculative and commit ports
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec},
	  tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm}}:
	     begin
		// check request type
		case (tuple2(req_from_cpu_spec, req_from_cpu_comm)) matches
		   {tagged Data_read_mem_ref {.cpu_addr_spec, .ref_addr_spec},
		    tagged Data_write_mem_ref {.cpu_addr_comm, .ref_addr_comm}}:
		       begin
			  Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup_spec = unpack(ref_addr_spec);
			  match {.tag_spec, .idx_spec, .line_offset_spec} = address_tup_spec;
			  req_dcache_tag_spec <= tag_spec;
			  req_dcache_index_spec <= idx_spec;
			  req_tok_spec <= tok_from_cpu_spec;
			  req_dcache_addr_spec <= cpu_addr_spec;
			  dcache_tag_store.readPorts[0].readReq(idx_spec);
			  
			  Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup_comm = unpack(ref_addr_comm);
			  match {.tag_comm, .idx_comm, .line_offset_comm} = address_tup_comm;
			  req_dcache_tag_comm <= tag_comm;
			  req_dcache_index_comm <= idx_comm;
			  req_tok_comm <= tok_from_cpu_comm;
			  req_dcache_addr_comm <= cpu_addr_comm;
			  dcache_tag_store.readPorts[1].readReq(idx_comm);
			  
			  state <= CheckTag;
			  req_type <= ReadWrite;
		       end
		endcase
	     end      
      endcase
   endrule
   
   rule nullreq (state == NullReq);
      
      // read response from replacement algorithm
      let msg_from_rep_alg <- port_from_replacement_alg.receive();
      
      // send invalid messages to pipeline
      port_to_cpu_imm_spec.send(tagged Invalid);
      port_to_cpu_del_spec.send(tagged Invalid);
      port_to_cpu_imm_comm.send(tagged Invalid);
      port_to_cpu_del_comm.send(tagged Invalid);
      
      // send invalid message to memory
      port_to_memory.send(tagged Invalid);
      
      state <= HandleReq;
   endrule
   
   
   rule checktag (state == CheckTag);
      
      Bool hit = False;
      DCACHE_WAY hit_way = 0;
      
      if (req_type == ReadWrite)
	 begin
	    
	    // read tags from BRAM
	    let tagstore_read <- dcache_tag_store.readPorts[0].readRsp();
	    let tagstore_write <- dcache_tag_store.readPorts[1].readRsp();
	    
	    // check all entries of the requested set
	    for (Integer w = 0; w < `DCACHE_ASSOC; w = w + 1)
	       begin 
		  if (tagstore_write[w] matches tagged Valid {.ret_dirty, .ret_lru, .ret_tag} &&& ret_tag == req_dcache_tag_comm)
		     begin
			hit = True;
			hit_way = fromInteger(w);
		     end
	       end
	    
	    req_hit <= hit;
	    req_hit_way <= hit_way;
	    
	    // react to hit or miss
	    if (hit)
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_comm, DCacheReplacementAlgorithmInput{accessed_hit: True, accessed_set: tagstore_write, accessed_way: hit_way}));
		  state <= HandleReadWrite;
		  stats.incr(stat_dcache_write_hits);
	       end
	    else
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_comm, DCacheReplacementAlgorithmInput{accessed_hit: False, accessed_set: tagstore_write, accessed_way: hit_way}));
		  state <= HandleReadWrite;
		  stats.incr(stat_dcache_write_misses);
	       end
	 end
      else if (req_type == Read)
	 begin
	    
	    // read tag from BRAM
	    let tagstore_read <- dcache_tag_store.readPorts[0].readRsp();
	    
	    // check all entries of the requested set
	    for (Integer w = 0; w < `DCACHE_ASSOC; w = w + 1) 
	       begin
		  if (tagstore_read[w] matches tagged Valid {.ret_dirty, .ret_lru, .ret_tag} &&& ret_tag == req_dcache_tag_spec)
		     begin
			hit = True;
			hit_way = fromInteger(w);
		     end
	       end
	    
	    req_hit <= hit;
	    req_hit_way <= hit_way;
	    
	    // react to hit or miss
	    if (hit)
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_spec, DCacheReplacementAlgorithmInput{accessed_hit: True, accessed_set: tagstore_read, accessed_way: hit_way}));
		  state <= HandleRead;
		  stats.incr(stat_dcache_read_hits);
	       end
	    else
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_spec, DCacheReplacementAlgorithmInput{accessed_hit: False, accessed_set: tagstore_read, accessed_way: hit_way}));
		  state <= HandleRead;
		  stats.incr(stat_dcache_read_misses);
	       end
	 end
      else if (req_type == Write)
	 begin
	    
	    // read tag from BRAM
	    let tagstore_write <- dcache_tag_store.readPorts[0].readRsp();
	    
	    // check all entries of the requested set
	    for (Integer w = 0; w < `DCACHE_ASSOC; w = w + 1) 
	       begin
		  if (tagstore_write[w] matches tagged Valid {.ret_dirty, .ret_lru, .ret_tag} &&& ret_tag == req_dcache_tag_comm)
		     begin
			hit = True;
			hit_way = fromInteger(w);
		     end
	       end
	    
	    req_hit <= hit;
	    req_hit_way <= hit_way;
	    
	    // react to hit or miss
	    if (hit) 
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_comm, DCacheReplacementAlgorithmInput{accessed_hit: True, accessed_set: tagstore_write, accessed_way: hit_way}));
		  state <= HandleWrite;
		  stats.incr(stat_dcache_write_hits);
	       end
	    else
	       begin
		  port_to_replacement_alg.send(tagged Valid tuple2(req_tok_comm, DCacheReplacementAlgorithmInput{accessed_hit: False, accessed_set: tagstore_write, accessed_way: hit_way}));
		  state <= HandleWrite;
		  stats.incr(stat_dcache_write_misses);
	       end
	 end
   endrule
   
   rule handleread (state == HandleRead);
      
      Bool dirty_line = False;
      
      // get response from replacement algorithm
      let rep_alg_resp <- port_from_replacement_alg.receive();
      
      // check replacement algorithm response
      case (rep_alg_resp) matches
	 tagged Valid {.resp_tok, .replace_out}:
	    begin
	       let updated_set = replace_out.updated_set;
	       let updated_way = replace_out.replaced_way;
	       let conflict_miss = replace_out.conflict_miss;
	       
	       // for read hit case
	       if (req_hit)
		  begin
		     // write to BRAM
		     dcache_tag_store.write(req_dcache_index_spec, updated_set);
		     
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Hit inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Invalid);
		     port_to_cpu_del_comm.send(tagged Invalid);
		     state <= HandleReq;
		     read <= True;
		  end
	       // for read miss case
	       else
		  begin
		     case (updated_set[updated_way]) matches
			tagged Valid {.dirty, .lru, .tag}:
			   begin
			      // track if cache line should be flushed
			      if (dirty == 1) 
				 dirty_line = True;
			      
			      updated_set[updated_way] = tagged Valid tuple3(0, lru, req_dcache_tag_spec);
			   end
		     endcase
		     
		     // write to BRAM
		     dcache_tag_store.write(req_dcache_index_spec, updated_set);
		     
		     // cold read miss
		     if (!conflict_miss)
			begin
			   port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_spec));
			   port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_spec));
			   port_to_cpu_del_spec.send(tagged Invalid);
			   port_to_cpu_imm_comm.send(tagged Invalid);
			   port_to_cpu_del_comm.send(tagged Invalid);
			   state <= ReadStall;
			   read <= True;
			end
		     // conflict read miss
		     else
			begin
			   // clean cache line
			   if (!dirty_line)
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_spec));
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_spec));
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Invalid);
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= ReadStall;
				 read <= True;
			      end
			   // dirty cache line
			   else
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_spec));
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_servicing inst_addr_spec));
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Invalid);
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= Flush;
				 read <= True;
			      end
			end
		  end
	    end
      endcase
   endrule
   
   rule readstall (state == ReadStall);
      
      if (!stall_phase)
	 begin
	    stall_phase <= True;
	    // send invalid to replacement algorithm
	    port_to_replacement_alg.send(tagged Invalid);
	 end
      else
	 begin
	    stall_phase <= False;
	    
	    // read incoming replacement algorithm port
	    let msg_from_rep_alg <- port_from_replacement_alg.receive();
	    
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
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
			   endcase
			end
		     port_to_cpu_del_spec.send(tagged Invalid);
		     
		     if (msg_from_cpu_comm matches tagged Invalid)
			port_to_cpu_imm_comm.send(tagged Invalid);
		     if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			begin
			   case (req_from_cpu_comm) matches
			      tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_inst_addr));
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
				       port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
				 endcase
			      end
			   port_to_cpu_del_spec.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response pc_from_mem));
			
			   if (msg_from_cpu_comm matches tagged Invalid)
			      port_to_cpu_imm_comm.send(tagged Invalid);
			   if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			      begin
				 case (req_from_cpu_comm) matches
				    tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				       port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_inst_addr));
				 endcase
			      end
			   port_to_cpu_del_comm.send(tagged Invalid);
			   port_to_memory.send(tagged Invalid);
			   state <= HandleReq;
			end
		  end
	    endcase
	 end
   endrule
   
   rule flush (state == Flush);
      
      if (!stall_phase)
	 begin
	    stall_phase <= True;
	    // send invalid to replacement algorithm
	    port_to_replacement_alg.send(tagged Invalid);
	 end
      else
	 begin
	    stall_phase <= False;	    
	    
	    let msg_from_rep_alg <- port_from_replacement_alg.receive();
	    
	    // read incoming memory port
	    let msg_from_mem <- port_from_memory.receive();
	    
	    // read incoming cpu speculative port
	    let msg_from_cpu_spec <- port_from_cpu_spec.receive();
	    
	    // read incoming cpu commit port
	    let msg_from_cpu_comm <- port_from_cpu_comm.receive();
	    
	    // check what memory is sending
	    case (msg_from_mem) matches
	       // memory is servicing flush
	       tagged Invalid:	 
		  begin
		     if (msg_from_cpu_spec matches tagged Invalid)
			port_to_cpu_imm_spec.send(tagged Invalid);
		     if (msg_from_cpu_spec matches tagged Valid {.tok_from_cpu_spec, .req_from_cpu_spec})
			begin
			   case (req_from_cpu_spec) matches
			      tagged Data_read_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
			   endcase
			end
		     port_to_cpu_del_spec.send(tagged Invalid);
		     
		     if (msg_from_cpu_comm matches tagged Invalid)
			port_to_cpu_imm_comm.send(tagged Invalid);
		     if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			begin
			   case (req_from_cpu_comm) matches
			      tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
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
				       port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
		     
				 endcase
			      end
			   port_to_cpu_del_spec.send(tagged Invalid);
			   
			   if (msg_from_cpu_comm matches tagged Invalid)
			      port_to_cpu_imm_comm.send(tagged Invalid);
			   if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			      begin
				 case (req_from_cpu_comm) matches
				    tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				       port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_mem_addr));
				 endcase
			      end
			   port_to_cpu_del_comm.send(tagged Invalid);
			   
			   if (read)
			      begin
				 state <= ReadStall;
				 port_to_memory.send(tagged Valid tuple2(req_tok_spec, tagged Mem_fetch inst_addr_spec));
			      end
			   else
			      begin
				 state <= WriteStall;
				 port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
			      end
			end
		  end
	    endcase
	 end
   endrule
   
	
   rule handlewrite (state == HandleWrite);

      Bool dirty_line = False;
      
      // get response from replacement algorithm
      let rep_alg_resp <- port_from_replacement_alg.receive();
      
      // check replacement algorithm response
      case (rep_alg_resp) matches
	 tagged Valid {.resp_tok, .replace_out}:
	    begin
	       let updated_set = replace_out.updated_set;
	       let updated_way = replace_out.replaced_way;
	       let conflict_miss = replace_out.conflict_miss;
	       
	       // update dirty bit
	       case (updated_set[updated_way]) matches
		  tagged Valid {.dirty, .lru, .tag}:
		     begin
			// track if the cache line should be flushed
			if (dirty == 1)
			   dirty_line = True;
			updated_set[updated_way] = tagged Valid tuple3(1, lru, req_dcache_tag_comm);
		     end
	       endcase
	       
	       // write to BRAM
	       dcache_tag_store.write(req_dcache_index_comm, updated_set);
	       
	       // for write hit case
	       if (req_hit)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm_spec.send(tagged Invalid);
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Hit inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     state <= HandleReq;
		     read <= False;
		  end
	       // write miss
	       else
		  begin
		     // cold write miss
		     if (!conflict_miss)
			begin
			   port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
			   port_to_cpu_imm_spec.send(tagged Invalid);
			   port_to_cpu_del_spec.send(tagged Invalid);
			   port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
			   port_to_cpu_del_comm.send(tagged Invalid);
			   state <= WriteStall;
			   read <= False;
			end
		     // conflict miss
		     else
			begin
			   // clean cache line
			   if (!dirty_line)
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
				 port_to_cpu_imm_spec.send(tagged Invalid);
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= WriteStall;
				 read <= False;
			      end
			   // dirty cache line
			   else
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
				 port_to_cpu_imm_spec.send(tagged Invalid);
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= Flush;
				 read <= False;
			      end
			end
		  end
	    end
      endcase
   endrule
   
   
   rule writestall (state == WriteStall);
      
      if (!stall_phase)
	 begin
	    stall_phase <= True;
	    // send invalid to replacement algorithm
	    port_to_replacement_alg.send(tagged Invalid);
	 end
      else
	 begin
	    stall_phase <= False;
	    
	    let msg_from_rep_alg <- port_from_replacement_alg.receive();
			   
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
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
			   endcase
			end
		     port_to_cpu_del_spec.send(tagged Invalid);
		     
		     if (msg_from_cpu_comm matches tagged Invalid)
			port_to_cpu_imm_comm.send(tagged Invalid);
		     if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			begin
			   case (req_from_cpu_comm) matches
			      tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_inst_addr));
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
				       port_to_cpu_imm_spec.send(tagged Valid tuple2(tok_from_cpu_spec, tagged Miss_retry cpu_inst_addr));
				 endcase
			      end
			   port_to_cpu_del_spec.send(tagged Invalid);
			   
			   if (msg_from_cpu_comm matches tagged Invalid)
			      port_to_cpu_imm_comm.send(tagged Invalid);
			   if (msg_from_cpu_comm matches tagged Valid {.tok_from_cpu_comm, .req_from_cpu_comm})
			      begin
				 case (req_from_cpu_comm) matches
				    tagged Data_write_mem_ref {.cpu_inst_addr, .cpu_mem_addr}:
				       port_to_cpu_imm_comm.send(tagged Valid tuple2(tok_from_cpu_comm, tagged Miss_retry cpu_inst_addr));
				 endcase
			      end
			   port_to_cpu_del_comm.send(tagged Valid tuple2(tok_from_memory, tagged Miss_response pc_from_mem));
			   port_to_memory.send(tagged Invalid);
			   state <= HandleReq;
			end
		  end
	    endcase
	 end
   endrule
   
   
   rule handlereadwrite (state == HandleReadWrite);
      
      Bool dirty_line = False;
      
      // get response from replacement algorithm
      let rep_alg_resp <- port_from_replacement_alg.receive();
      
      // check replacement algorithm response
      case (rep_alg_resp) matches
	 tagged Valid {.resp_tok, .replace_out}:
	    begin
	       let updated_set = replace_out.updated_set;
	       let updated_way = replace_out.replaced_way;
	       let conflict_miss = replace_out.conflict_miss;
	       
	       // update dirty bit
	       case (updated_set[updated_way]) matches
		  tagged Valid {.dirty, .lru, .tag}:
		     begin
			// track if the cache line should be flushed
			if (dirty == 1)
			   dirty_line = True;
			updated_set[updated_way] = tagged Valid tuple3(1, lru, req_dcache_tag_comm);
		     end
	       endcase
	       
	       // write to BRAM
	       dcache_tag_store.write(req_dcache_index_comm, updated_set);
	       
	       // for write hit case
	       if (req_hit) 
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
		     port_to_cpu_del_spec.send(tagged Invalid);
		     port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Hit inst_addr_comm));
		     port_to_cpu_del_comm.send(tagged Invalid);
		     state <= HandleReq;
		     read <= False;
		  end
	       // write miss
	       else
		  begin
		     // cold write miss
		     if (!conflict_miss)
			begin
			   port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
			   port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
			   port_to_cpu_del_spec.send(tagged Invalid);
			   port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
			   port_to_cpu_del_comm.send(tagged Invalid);
			   state <= WriteStall;
			   read <= False;
			end
		     // conflict miss
		     else
			begin
			   // clean cache line
			   if (!dirty_line)
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= WriteStall;
				 read <= False;
			      end
			   // dirty cache line
			   else
			      begin
				 port_to_memory.send(tagged Valid tuple2(req_tok_comm, tagged Mem_fetch inst_addr_comm));
				 port_to_cpu_imm_spec.send(tagged Valid tuple2(req_tok_spec, tagged Miss_retry inst_addr_spec));
				 port_to_cpu_del_spec.send(tagged Invalid);
				 port_to_cpu_imm_comm.send(tagged Valid tuple2(req_tok_comm, tagged Miss_servicing inst_addr_comm));
				 port_to_cpu_del_comm.send(tagged Invalid);
				 state <= Flush;
				 read <= False;
			      end
			end
		  end
	    end
      endcase
   endrule
   
endmodule

	      
		  
		     
			
	       

	       
	       
	    
						  
	    

 
      



		     
	       
	 
	       
	       
 
		     
	 
		     
	    
							      

								       
															       
																	       
									  
																		    
												
												
												
							     
   
  
   
	 
