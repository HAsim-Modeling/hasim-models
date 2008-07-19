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


module [HASim_Module] mkDCache();

   // initialize cache memory
   let cachememory <- mkDCacheMemory();
   
   // state register
   Reg#(State) state <- mkReg(HandleReq);
   
   // BRAM for cache tag store
   BRAM#(`DCACHE_IDX_BITS, Maybe#(DCACHE_TAG)) dcache_tag_store <- mkBramInitialized(tagged Invalid);

   // registers to hold cache request fields
   Reg#(DCACHE_TAG) spec_req_dcache_tag <- mkReg(0);
   Reg#(DCACHE_INDEX) spec_req_dcache_index <- mkReg(0);
   Reg#(TOKEN) spec_req_tok <- mkRegU();
   Reg#(DATA_ADDRESS) spec_req_dcache_addr <- mkReg(0);
   Reg#(INST_ADDRESS) spec_inst_addr <- mkReg(0);
   
   Reg#(DCACHE_TAG) comm_req_dcache_tag <- mkReg(0);
   Reg#(DCACHE_INDEX) comm_req_dcache_index <- mkReg(0);
   Reg#(TOKEN) comm_req_tok <- mkRegU();
   Reg#(DATA_ADDRESS) comm_req_dcache_addr <- mkReg(0);
   Reg#(INST_ADDRESS) comm_inst_addr <- mkReg(0); 
   
   // incoming ports
   // incoming port from CPU with speculative stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_spec <- mkPort_Receive("cpu_to_dcache_speculative", 0);
   
   // incoming port from CPU with commited stores
   Port_Receive#(Tuple2#(TOKEN, CacheInput)) port_from_cpu_comm <- mkPort_Receive("cpu_to_dcache_committed", 0);
   
   // incoming port from memory
   Port_Receive#(Tuple2#(TOKEN, MemOutput)) port_from_memory <- mkPort_Receive("memory_to_dcache", 0);
   
   // outgoing ports
   // outgoing port to CPU with immediate response
   Port_Send#(Tuple2#(TOKEN, CacheOutputImmediate)) port_to_cpu_imm <- mkPort_Receive("dcache_to_cpu_immediate");
   
   // outgoing port to CPU with delayed response
   Port_Send#(Tuple2#(TOKEN, CacheOutputDelayed)) port_to_cpu_del <- mkPort_Receive("dcache_to_cpu_delayed");
   
   // outgoing port to memory
   Port_Send#(Tuple2#(TOKEN, MemInput)) port_to_memory <- mkPort_Send("dcache_to_memory");
   
   // communication with local controller
   Vector#(3, Port_Control) inports = newVector();
   Vector#(3, Port_Control) outports = newVector();
   inports[0] = port_from_cpu_spec.ctrl;
   inports[1] = port_from_cpu_comm.ctrl;
   inports[2] = port_from_memory.ctrl;
   outports[0] = port_to_cpu_imm.ctrl;
   outports[1] = port_to_cpu_del.ctrl;
   outports[2] = port_to_memory.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // rules
   rule handlereq (state == HandleReq);
      
      // read cpu input port (speculative)
      let msg_from_cpu_spec <- port_from_cpu_spec.receive();
      
      // read cpu input port (committed)
      let msg_from_cpu_comm <- port_from_cpu_comm.receive();
      
      // read memory input port
      let msg_from_mem <- port_from_memory.receive();
      
      // decide response based on input combination
      case (tuple3(msg_from_cpu_spec, msg_from_cpu_comm, msg_from_mem)) matches
	 // if there is no action taking place
	 {tagged Invalid, tagged Invalid, tagged Invalid}:
					     begin
						port_to_cpu_imm.send(tagged Invalid);
						port_to_cpu_del.send(tagged Invalid);
						port_to_memory.send(tagged Invalid);
					     end
	 // if only speculative request 
	 {tagged Valid {.tok_from_cpu_spec, .req_from_cpu_sepc}, tagged Invalid, tagged Invalid}:
										    begin
										       // check speculative request type
										       case (req_from_cpu_spec) matches
											  // standard read
											  tagged Data_read_mem_ref {.inst_addr, .ref_addr}:
											     begin
												Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr);
												match {.tag, .idx, .line_offset} = address_tup;
												spec_req_dcache_tag <= tag;
												spec_req_dcache_index <= idx;
												spec_req_dcache_addr <= ref_addr;
												spec_req_tok <= tok_from_cpu_spec;
												spec_inst_addr <= ref_addr;
												dcache_tag_store.readReq(idx);
												state <= HandleReadSpec;
											     end
											  // standard write
											  tagged Data_write_mem_ref {.inst_addr, .ref_addr}:
											     begin
												Tuple3#(DCACHE_TAG, DCACHE_INDEX, DCACHE_LINE_OFFSET) address_tup = unpack(ref_addr);
												match {.tag, .idx, .line_offset} = address_tup;
												spec_req_dcache_tag <= tag;
												spec_req_dcache_index <= idx;
												spec_req_dcache_addr <= ref_addr;
												spec_req_tok <= tok_from_cpu_spec;
												spec_inst_addr <= ref_addr;
												
												
												
							     
   
  
   
	 