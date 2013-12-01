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
`include "asim/provides/hasim_icache_memory.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/hasim_icache_memory.bsh"
`include "asim/provides/hasim_icache_base_types.bsh"
`include "asim/provides/hasim_icache_replacement_algorithm.bsh"

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

typedef Bit#(`ICACHE_IDX_BITS) ICACHE_IDX;

module [HASIM_MODULE] mkICache();
   
   // instantiate cache memory
   let cachememory <- mkICacheMemory();
   
   // tag store BRAM
   // produce a BRAM containing vectors holding all the tags of each way
   BRAM#(ICACHE_IDX, Vector#(`ICACHE_ASSOC, Maybe#(ICACHE_LINE))) icache_tag_store <- mkBRAMInitialized(Vector::replicate(tagged Invalid));

   // register to hold state
   Reg#(State) state <- mkReg(HandleReq);
   Reg#(Bit#(4)) replace <- mkReg(0);
   
   // register to hold cache request fields
   Reg#(ICACHE_TAG) req_icache_tag <- mkReg(0);
   Reg#(ICACHE_INDEX) req_icache_index <- mkReg(0);
   Reg#(TOKEN) req_tok <- mkRegU();
   Reg#(ISA_ADDRESS) req_addr <- mkReg(0);
   
   // instatiate replacement algorithm
   let replacementmodule <- mkICacheReplacementAlgorithm();
   
   // incoming port from CPU (latency 0)
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu <- mkPort_Receive("cpu_to_icache", 0);
   
   // incoming port from memory 
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_icache", valueOf(TSub#(`ICACHE_MISS_PENALTY, 1)));
   
   // outgoing port to CPU (latency 0)
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Send("icache_to_cpu_immediate");
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Send("icache_to_cpu_delayed");
   
   // outgoing port to memory (latency 10)
   Port_Send#(Tuple2#(TOKEN, MemInput)) port_to_memory <- mkPort_Send("icache_to_memory");
   
   // incoming port from replacement algorithm (latency 0)
   Port_Receive#(Tuple2#(TOKEN, ReplacementAlgorithmOutput)) port_from_replacement_alg <- mkPort_Receive("replacement_algorithm_to_icache_core", 0);
   
   // outgoing port to replacement algorithm (latency 0)
   Port_Send#(Tuple2#(TOKEN, ReplacementAlgorithmInput)) port_to_replacement_alg <- mkPort_Send("icache_core_to_replacement_algorithm");
   
   // communication with local controller
   Vector#(3, Port_Control) inports  = newVector();
   Vector#(4, Port_Control) outports = newVector();
   inports[0]  = port_from_cpu.ctrl;
   inports[1] = port_from_memory.ctrl;
   inports[2] = port_from_replacement_alg.ctrl;
   outports[0] = port_to_cpu_imm.ctrl;
   outports[1] = port_to_cpu_del.ctrl;
   outports[2] = port_to_memory.ctrl;
   outports[3] = port_to_replacement_alg.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports); 
   
   // Stats
   STAT_ID statIDs[2] = {
       statName("MODEL_SET_ASSOC_ICACHE_HITS", "ICache Hits"),
       statName("MODEL_SET_ASSOC_ICACHE_MISSES", "ICache Misses")
   };

   STAT_VECTOR#(2) stats <- mkStatCounter_Vector(statIDs);
   let stat_icache_hits = 0;
   let stat_icache_misses = 1;
   
   Reg#(Bool) noRequest <- mkReg(False);
   Reg#(Bool) hit  <- mkReg(False);      
   Reg#(ICACHE_WAY) hit_way <- mkReg(0);
   Reg#(Bool) missretry <- mkReg(False);
   Reg#(Bool) memret <- mkReg(False);
   Reg#(TOKEN) resp_tok <- mkRegU();
   Reg#(ISA_ADDRESS) resp_addr <- mkReg(0);
   
   // rules
   rule handlereq (state == HandleReq);
      // read request from CPU
      let req_from_cpu <- port_from_cpu.receive();
      
      // read response from memory
      let resp_from_mem <- port_from_memory.receive();
      
      // check request type
      case (req_from_cpu) matches
	 // no request
	 tagged Invalid:
	    begin
	       port_to_replacement_alg.send(tagged Invalid);
	       state <= HandleReq2;	     
	    end
	 
	 // request
	 tagged Valid {.tok_from_cpu, .msg_from_cpu}:
	    begin
	       // check request type
	       case (msg_from_cpu) matches
		  // standard read request
		  tagged Inst_mem_ref .addr:
		     begin
			Tuple3#(ICACHE_TAG, ICACHE_INDEX, ICACHE_LINE_OFFSET) address_tup = unpack(addr);
			match {.tag, .idx, .line_offset} = address_tup;
			req_icache_tag <= tag;
			req_icache_index <= idx;
			req_tok <= tok_from_cpu;
			req_addr <= addr;
			// make the read request from BRAM tag store
			icache_tag_store.readReq(idx);
			state <= HandleTag;
		     end
		  // no current implementation of other request types
	       endcase
	    end
      endcase
   endrule
   
   rule handlereq2 (state == HandleReq2);
      port_to_memory.send(tagged Invalid);
      port_to_cpu_imm.send(tagged Invalid);
      port_to_cpu_del.send(tagged Invalid);
      let resp_from_rep <- port_from_replacement_alg.receive();
      state <= HandleReq;   
   endrule
      
   rule handletag (state == HandleTag);
      
      Bool hit_rec = False;
      ICACHE_WAY hit_way_rec = 0;
     
      begin
	 // get response from BRAM
	 let tag_from_bram <- icache_tag_store.readRsp();
	 
	 // check if anything in the set is valid
	 for (Integer w = 0; w <`ICACHE_ASSOC; w = w + 1)
	    begin
	       if (tag_from_bram[w] matches tagged Valid {.ret_icache_lru_bits, .ret_icache_tag} &&& ret_icache_tag == req_icache_tag)
		  begin
		     hit_rec = True;
		     hit_way_rec = fromInteger(w);
		  end
	    end
	    
	 hit <= hit_rec;
	 hit_way <= hit_way_rec;
	 
	 // react to hit or miss
	 if (hit_rec)
	    begin
	       port_to_replacement_alg.send(tagged Valid tuple2(req_tok, ReplacementAlgorithmInput{accessed_hit: True, accessed_set: tag_from_bram, accessed_way: hit_way_rec}));
	       state <= HandleRead;
	       stats.incr(stat_icache_hits);
	    end
	 else
	    begin
	       port_to_replacement_alg.send(tagged Valid tuple2(req_tok, ReplacementAlgorithmInput{accessed_hit: False, accessed_set: tag_from_bram, accessed_way: ?}));
	       state <= HandleRead;
	       stats.incr(stat_icache_misses);
	    end
      end
     
   endrule
   
   
   rule handleread (state == HandleRead);
      
      // get response from replacement algorithm 
      let replacement_resp <- port_from_replacement_alg.receive();
      
      // check replacement algorithm response
      case (replacement_resp) matches
	 tagged Valid {.resp_tok, .replace_out}:
	    begin
	       let updated_set = replace_out.updated_set;
	       let updated_way = replace_out.replaced_way;
	       
	       // update set
	       case (updated_set[updated_way]) matches
		  tagged Valid {.lru_bits, .tag}:
		     begin
			updated_set[updated_way] = tagged Valid tuple2(lru_bits, req_icache_tag);
		     end
	       endcase
	       
	       // write updated set into BRAM
	       icache_tag_store.write(req_icache_index, updated_set);
	       
	       // send messages to cpu and memory
	       if (hit)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit req_addr));
		     port_to_cpu_del.send(tagged Invalid);
		     state <= HandleReq;
		  end
	       else
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch req_addr));
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing req_addr));
		     port_to_cpu_del.send(tagged Invalid); 
		     state <= HandleStall1;
		  end
	    end
      endcase
   endrule
   
      
   rule handlestall1 (state == HandleStall1);
      
      // receive CPU request
      let req_from_cpu <- port_from_cpu.receive();
      
      // receive memory response
      let resp_from_mem <- port_from_memory.receive();
      
      // check if memory has returned value
      case (resp_from_mem) matches
	 // not yet responded with data
	 tagged Invalid:
	    begin
	       // check CPU request type
	       case (req_from_cpu) matches
		  // no request
		  tagged Invalid:
		     begin
			//port_to_memory.send(tagged Invalid);
			//port_to_cpu_imm.send(tagged Invalid);
			//port_to_cpu_del.send(tagged Invalid);
			port_to_replacement_alg.send(tagged Invalid);
			state <= HandleStall2;
			missretry <= False;
			memret <= False;
		     end
		  // CPU is making a request
		  tagged Valid {.tok_from_cpu, .msg_from_cpu}:
		     begin
			case (msg_from_cpu) matches
			   tagged Inst_mem_ref .addr_from_cpu:
			      begin
				 // port_to_memory.send(tagged Invalid);
				 // port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_retry addr_from_cpu));
				 // port_to_cpu_del.send(tagged Invalid);
				 port_to_replacement_alg.send(tagged Invalid);
				 state <= HandleStall2;
				 missretry <= True;
				 memret <= False;
			      end
			endcase
		     end
	       endcase
	    end
	 // memory responds with data
	 tagged Valid {.tok_from_mem, .msg_from_mem}:
	    begin
	       case (msg_from_mem) matches
		  tagged ValueRet .resp_pc:
		     begin
			//port_to_memory.send(tagged Invalid);
			//port_to_cpu_imm.send(tagged Invalid);
			//port_to_cpu_del.send(tagged Valid tuple2(tok_from_mem, tagged Miss_response resp_pc));
			resp_tok <= tok_from_mem;
			resp_addr <= resp_pc;
			port_to_replacement_alg.send(tagged Invalid);
			state <= HandleStall2;
			missretry <= False;
			memret <= True;
		     end
	       endcase
	    end
      endcase
   endrule
   
   rule handlestall2 (state == HandleStall2);
      if (!memret)
	 begin
	    if (missretry)
	       begin
		  port_to_memory.send(tagged Invalid);
		  port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_retry req_addr));
		  port_to_cpu_del.send(tagged Invalid);  
		  let msg_from_rep <- port_from_replacement_alg.receive();
	       end
	    else
	       begin
		  port_to_memory.send(tagged Invalid);
		  port_to_cpu_imm.send(tagged Invalid);
		  port_to_cpu_del.send(tagged Invalid);
		  let msg_from_rep <- port_from_replacement_alg.receive();
	       end
	    state <= HandleStall1;
	 end
      else
	 begin
	    let msg_from_rep <- port_from_replacement_alg.receive();
	    port_to_memory.send(tagged Invalid);
	    port_to_cpu_imm.send(tagged Invalid);
	    port_to_cpu_del.send(tagged Valid tuple2(resp_tok, tagged Miss_response resp_addr));	    
	    state <= HandleReq;
	 end
   endrule
endmodule

 
	       
      
