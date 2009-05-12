import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_icache_memory.bsh"
`include "asim/provides/hasim_icache_base_types.bsh"
`include "asim/provides/hasim_icache_memory.bsh"

`include "asim/dict/STATS_ICACHE.bsh"
`include "asim/dict/PARAMS_HASIM_ICACHE.bsh"

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
   INST_ADDRESS Miss;
   INST_ADDRESS Miss_servicing;
   INST_ADDRESS Miss_retry;
   } 
CacheOutputImmediate deriving (Eq, Bits);

typedef union tagged{
   INST_ADDRESS Miss_response;
   INST_ADDRESS Hit_response;
   }
CacheOutputDelayed deriving (Eq, Bits);

typedef enum {HandleReq, HandleRead, HandleStall} State deriving (Eq, Bits);

 
module [HASIM_MODULE] mkICache();
   
   // ***** Dynamic parameters *****
   PARAMETER_NODE paramNode <- mkDynamicParameterNode();

   Param#(1) alwaysHitParam <- mkDynamicParameter(`PARAMS_HASIM_ICACHE_ICACHE_ALWAYS_HIT, paramNode);
   function Bool alwaysClaimHit() = (alwaysHitParam == 1);

   // FSM register
   Reg#(State) state <- mkReg(HandleReq);
   
   // initialize cache memory
   let cachememory <- mkICacheMemory();
   
   // BRAM for the cache tag store
   BRAM#(ICACHE_INDEX, Maybe#(ICACHE_TAG)) icache_tag_store <- mkBRAMInitialized(tagged Invalid);     
   // register to hold cache tag/index
   Reg#(ICACHE_TAG) req_icache_tag<- mkReg(0);
   Reg#(ICACHE_INDEX) req_icache_index <- mkReg(0);
   Reg#(TOKEN) req_tok <- mkRegU;
   Reg#(ISA_ADDRESS) req_addr <- mkReg(0);
   
   /* incoming ports */
   // incoming port from the CPU with 0 latency
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu <- mkPort_Receive("cpu_to_icache", 0);
   
   // incoming port from memory with 10 latency
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_icache", valueOf(TSub#(`ICACHE_MISS_PENALTY, 1)));
   
   /* outgoing ports */
   // outgoing port to the CPU with 0 latency
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Send("icache_to_cpu_immediate");
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Send("icache_to_cpu_delayed");
    
   // outgoing port to memory with 10 latency
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
   
   // Stats
   STAT stat_icache_hits <- mkStatCounter(`STATS_ICACHE_ICACHE_HITS);
   STAT stat_icache_misses <- mkStatCounter(`STATS_ICACHE_ICACHE_MISSES);
  
   // rules
   rule handlereq (state == HandleReq);
      // read request from cpu
      let req_from_cpu <- port_from_cpu.receive();
      
      // read response from memory
      let resp_from_mem <- port_from_memory.receive();
      
      // check request type
      case (req_from_cpu) matches
	 // no request
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Invalid);
	       port_to_cpu_imm.send(tagged Invalid);
	       port_to_cpu_del.send(tagged Invalid);
	    end
	 // request
	 tagged Valid {.tok_from_cpu, .addr_from_cpu}:
	    begin
	       // check request type
	       case (addr_from_cpu) matches
		  // standard read request
		  tagged Inst_mem_ref .addr:
		     begin
                        if (! alwaysClaimHit())
                          begin
                              // Look up address in cache
			      Tuple3#(ICACHE_TAG, ICACHE_INDEX, ICACHE_LINE_OFFSET) address_tup = unpack(addr);
		              match {.tag, .idx, .line_offset} = address_tup;
			      req_icache_tag <= tag;
			      req_icache_index <= idx;
			      req_tok <= tok_from_cpu;
			      req_addr <= addr;
			      // make the read request from BRAM tag store
			      icache_tag_store.readReq(idx);
			      state <= HandleRead;
                          end
                          else
                          begin
                              // Always hit (requested by dynamic parameter)
		              port_to_memory.send(tagged Invalid);
		              port_to_cpu_imm.send(tagged Valid tuple2(tok_from_cpu, tagged Hit addr));
		              port_to_cpu_del.send(tagged Invalid);
		              stat_icache_hits.incr();
                          end
		     end
		  // no current implementation of other request types
	       endcase
	    end
      endcase
   endrule
   
   rule handleread (state == HandleRead);
      
      // read response from BRAM
      let readtag <- icache_tag_store.readRsp();
      
      // check if there is a tag match
      case (readtag) matches
	 // cache miss because of invalid cache line
	 tagged Invalid:
	    begin
	       port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch req_addr));
	       port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing req_addr));
	       port_to_cpu_del.send(tagged Invalid);
	       //update BRAM tag store
	       icache_tag_store.write(req_icache_index, tagged Valid req_icache_tag);
	       state <= HandleStall;
	       stat_icache_misses.incr();
	    end
	 // valid cache line
	 tagged Valid .storedtag:
	    begin
	       // cache hit
	       if (storedtag == req_icache_tag)
		  begin
		     port_to_memory.send(tagged Invalid);
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Hit req_addr));
		     port_to_cpu_del.send(tagged Invalid);
		     state <= HandleReq;
		     stat_icache_hits.incr();
		  end
	       // cache miss
	       else
		  begin
		     port_to_memory.send(tagged Valid tuple2(req_tok, tagged Mem_fetch req_addr));
		     port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_servicing req_addr));
		     port_to_cpu_del.send(tagged Invalid);
		     // update BRAM tag store
		     icache_tag_store.write(req_icache_index, tagged Valid req_icache_tag);
		     state <= HandleStall;
		     stat_icache_misses.incr();
		  end
	    end
      endcase
   endrule
   
   
   rule readstall(state == HandleStall);
	
      // read request from CPU
      let req_from_cpu <- port_from_cpu.receive();
      // read response from memory
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
			port_to_memory.send(tagged Invalid);
			port_to_cpu_imm.send(tagged Invalid);
			port_to_cpu_del.send(tagged Invalid);
		     end
		  // CPU is making a request
		  tagged Valid {.tok_from_cpu, .msg_from_cpu}:
		     begin
			case (msg_from_cpu) matches
			   tagged Inst_mem_ref .addr_from_cpu:
			      begin
				 port_to_memory.send(tagged Invalid);
				 port_to_cpu_imm.send(tagged Valid tuple2(req_tok, tagged Miss_retry addr_from_cpu));
				 port_to_cpu_del.send(tagged Invalid);
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
			port_to_memory.send(tagged Invalid);
			port_to_cpu_imm.send(tagged Invalid);
			port_to_cpu_del.send(tagged Valid tuple2(tok_from_mem, tagged Miss_response resp_pc));
			state <= HandleReq;
		     end
	       endcase
	    end
      endcase
   endrule
   
endmodule
