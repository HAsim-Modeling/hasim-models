//
// INTEL CONFIDENTIAL
// Copyright (c) 2008 Intel Corp.  Recipient is granted a non-sublicensable 
// copyright license under Intel copyrights to copy and distribute this code 
// internally only. This code is provided "AS IS" with no support and with no 
// warranties of any kind, including warranties of MERCHANTABILITY,
// FITNESS FOR ANY PARTICULAR PURPOSE or INTELLECTUAL PROPERTY INFRINGEMENT. 
// By making any use of this code, Recipient agrees that no other licenses 
// to any Intel patents, trade secrets, copyrights or other intellectual 
// property rights are granted herein, and no other licenses shall arise by 
// estoppel, implication or by operation of law. Recipient accepts all risks 
// of use.
//

import Vector::*;

//HASim library imports
`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_controller.bsh"
`include "asim/provides/module_local_controller.bsh"

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"

`include "asim/dict/EVENTS_CPU.bsh"
`include "asim/dict/STATS_CPU.bsh"

`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/hasim_icache.bsh"
`include "asim/provides/hasim_dcache.bsh"

//************************* Simple Timing Partition ***********************//
//                                                                         //
// This is about the simplest timing partition you can conceive of. It     //
// simply fetches one instruction at a time, executes it, then moves to    //
// the next instruction. This can serve as a good mechanism to verify      //
// the functional partition and can serve as a "golden model" for more     //
// complex timing partitions.                                              //
//                                                                         //
//*************************************************************************//



typedef enum 
{ 
 TOK, FET, ICACHE, ICACHE_STALL, DEC, EXE, LOA, STO, DCACHE, DCACHE_STALL, LCO, GCO 
 } 
Stage deriving (Eq, Bits);

module [HASIM_MODULE] mkPipeline
   //interface:
   ();
   
   Reg#(File) debug_log <- mkReg(InvalidFile);
   
   //********* State Elements *********//
   
   //Have we made a req to FP and are waiting for a response?
   Reg#(Bool) madeReq <- mkReg(False);
  
  //The current stage
   Reg#(Stage) stage <- mkReg(TOK);
   Reg#(Stage) req_stage <- mkRegU();
  
  //Current TOKEN (response from TOK stage)
  Reg#(TOKEN) cur_tok <- mkRegU();
  
  //Current instruction (response from FET stage)
  Reg#(ISA_INSTRUCTION)  cur_inst <- mkRegU();
  
  //The Program Counter
  Reg#(ISA_ADDRESS) pc <- mkReg(`PROGRAM_START_ADDR);
  
  //The actual Clock Cycle, for debugging messages
  Reg#(Bit#(32)) hostCC <- mkReg(0);
  
  //The simulation Clock Cycle, or "tick"
  Reg#(Bit#(32)) baseTick <- mkReg(0);
  
  //********* Connections *********//
  
  Connection_Send#(CONTROL_MODEL_CYCLE_MSG)
  //...
  link_model_cycle <- mkConnection_Send("model_cycle");

  Connection_Send#(CONTROL_MODEL_COMMIT_MSG)
  //...
  link_model_commit <- mkConnection_Send("model_commits");

  Connection_Client#(Void, TOKEN)
  //...
  link_to_tok <- mkConnection_Client("funcp_newInFlight");
  
  Connection_Client#(Tuple2#(TOKEN, ISA_ADDRESS),
                     Tuple2#(TOKEN, ISA_INSTRUCTION))
  //...
  link_to_fet <- mkConnection_Client("funcp_getInstruction");
  
  Connection_Client#(TOKEN,
                     Tuple2#(TOKEN, ISA_DEPENDENCY_INFO))
  //...
  link_to_dec <- mkConnection_Client("funcp_getDependencies");
  
  Connection_Client#(TOKEN,
                     //Tuple2#(TOKEN, ISA_EXECUTION_RESULT))
		     FUNCP_GET_RESULTS_MSG)
  //...
  link_to_exe <- mkConnection_Client("funcp_getResults");
  
  Connection_Client#(TOKEN,
                     TOKEN)
  //...
  link_to_load <- mkConnection_Client("funcp_doLoads");
  
  Connection_Client#(TOKEN,
                     TOKEN)
  //...
  link_to_store <- mkConnection_Client("funcp_doSpeculativeStores");

  Connection_Client#(TOKEN,
                     TOKEN)
  //...
  link_to_lco <- mkConnection_Client("funcp_commitResults");
  
  Connection_Client#(TOKEN,
                     TOKEN)
  //...
  link_to_gco <- mkConnection_Client("funcp_commitStores");

  //For killing. UNUSED
  
  Connection_Client#(TOKEN, Void) 
  //...
        link_rewindToToken <- mkConnection_Client("funcp_rewindToToken");

 
  //Events
  EventRecorder event_com <- mkEventRecorder(`EVENTS_CPU_INSTRUCTION_COMMIT);
  
  //Stats
  Stat stat_com <- mkStatCounter(`STATS_CPU_INSTRUCTION_COMMIT);
   
   
   // Create ICache
   let inst_cache <- mkICache();
   
   // Ports communicating with ICache
   Port_Send#(Tuple2#(TOKEN, CacheInput)) port_to_icache <- mkPort_Send("cpu_to_icache"); // port to the instruction cache
  
   Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) port_from_icache_imm <- mkPort_Receive("icache_to_cpu_immediate", 0); // port from icache
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) port_from_icache_del <- mkPort_Receive("icache_to_cpu_delayed", 0); // port from icache with miss response
  
   
   // Create DCache
   let data_cache <- mkDCache();
   
   // Ports communicating with DCache
   Port_Send#(Tuple2#(TOKEN, CacheInput)) port_to_dcache_spec <- mkPort_Send("cpu_to_dcache_speculative");  // speculative port to the data cache
   
   Port_Send#(Tuple2#(TOKEN, CacheInput)) port_to_dcache_comm <- mkPort_Send("cpu_to_dcache_committed"); // committed port to the data cache
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) port_from_dcache_imm_spec <- mkPort_Receive("dcache_to_cpu_immediate_speculative", 0);  // immediate speculative response
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) port_from_dcache_del_spec <- mkPort_Receive("dcache_to_cpu_delayed_speculative", 0); // delayed speculative response
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) port_from_dcache_imm_comm <- mkPort_Receive("dcache_to_cpu_immediate_committed", 0); // immediate committed response
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed)) port_from_dcache_del_comm <- mkPort_Receive("dcache_to_cpu_delayed_committed", 0); // delayed committed response
   
   
   // state for communication with ICache
   Reg#(Bool) waitForICache <- mkReg(True);
   Reg#(TOKEN) icache_tok <- mkRegU();
   
   // state for communication with DCache
   Reg#(Bool) waitForDCache <- mkReg(True);
   Reg#(TOKEN) dcache_tok <- mkRegU();
   
   // register to control port states
   Reg#(Bool) stall <- mkReg(False);
   
   
   /********* Communication with local controller for icache ports ******/
   Vector#(6, Port_Control) inports  = newVector();
   Vector#(3, Port_Control) outports = newVector();
   inports[0]  = port_from_icache_imm.ctrl;
   inports[1]  = port_from_icache_del.ctrl;
   inports[2] = port_from_dcache_imm_spec.ctrl;
   inports[3] = port_from_dcache_del_spec.ctrl;
   inports[4] = port_from_dcache_imm_comm.ctrl;
   inports[5] = port_from_dcache_del_comm.ctrl;
   outports[0] = port_to_icache.ctrl;
   outports[1] = port_to_dcache_spec.ctrl;
   outports[2] = port_to_dcache_comm.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);     
  
  //********* Rules *********//

  //count
  rule count (True);
    
    if (hostCC == 0)
    begin
      local_ctrl.startModelCC();

      let fd <- $fopen("hasim_cpu.out");
      if (fd == InvalidFile)
      begin
        $display("Error opening logfile!");
	$finish(1);
      end
      debug_log <= fd;
    end
    
    hostCC <= hostCC + 1;
  
  endrule
  
  //process
  
  rule process (local_ctrl.running());
     debug_rule("process");
        
     Bool iCacheRead = False;
 
    case (stage)
      TOK:
       begin
	    
        debug_case("stage", "TOK");
	
        if (!madeReq)
	  begin
	    debug_then("!madeReq");
	    
	    //Request a TOKEN
	    debug(2, $fdisplay(debug_log, "[%d] Requesting a new TOKEN on model cycle %0d.", hostCC, baseTick));
	    link_to_tok.makeReq(?);
            link_model_cycle.send(0);
	    
	    madeReq <= True;
	    
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
	    //Get the response
	    let tok = link_to_tok.getResp();
	    link_to_tok.deq();
	    
            tok.timep_info = TOKEN_TIMEP_INFO{epoch: 0, scratchpad: 0};

	    debug(2, $fdisplay(debug_log, "[%d] TOK Responded with TOKEN %0d.", hostCC, tok.index));
	    
	    cur_tok <= tok;
	    
	    stage <= FET;
	    madeReq <= False;
	  end
      end
      FET:
       begin
	   
        debug_case("stage", "FET");
	
        if (!madeReq)
	   begin	            
	    debug_then("!madeReq");
	    
	    //Fetch next instruction
	    debug(2, $fdisplay(debug_log, "[%d] Fetching TOKEN %0d at address 0x%h.", hostCC, cur_tok.index, pc));
             link_to_fet.makeReq(tuple2(cur_tok, pc));
	         	     
	      madeReq <= True;
	      
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
	    //Get the response
            match {.tok, .inst} = link_to_fet.getResp();
	    link_to_fet.deq();

	    debug(2, $fdisplay(debug_log, "[%d] FET Responded with TOKEN %0d.", hostCC, tok.index));
	    
            cur_inst <= inst;
	    if (tok.index != cur_tok.index) $display ("FET ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
	    
	     stage <= ICACHE;
	     madeReq <= False;
	     
	     // Make a request to the instruction cache
	     // Standard memory reference for now (no prefetches etc.)
	     port_to_icache.send(tagged Valid tuple2(cur_tok, tagged Inst_mem_ref pc));
	     iCacheRead = True;
//	     waitForICache <= True;	   
	     
	  end
       end
       
       ICACHE:
       begin
	  debug_case("stage", "ICACHE");
	  
	  // capture icache messages
	  let icache_ret_imm <- port_from_icache_imm.receive();
	  let icache_ret_del <- port_from_icache_del.receive();
	  
	  case (icache_ret_del) matches
	     // if there is no delayed response 
	     tagged Invalid:
		begin
		   case (icache_ret_imm) matches
		      // if there is message from icache
		      tagged Invalid:
			 begin
			    port_to_dcache_spec.send(tagged Invalid);
			    port_to_dcache_comm.send(tagged Invalid);
			    
			    stage <= ICACHE_STALL;
			 end
		      // if there is an immediate message from icache
		      tagged  Valid {.icachetok, .icachemsg}:
			 begin
			    case (icachemsg) matches
			       tagged Hit .icacheaddr:
				  begin 
			             stage <= DEC;
				  end
			       tagged Miss_servicing .icacheaddr:
				  begin
				     port_to_dcache_spec.send(tagged Invalid);
				     port_to_dcache_comm.send(tagged Invalid);				     
				     
				     stage <= ICACHE_STALL;
				  end
			       tagged Miss_retry .icacheaddr:
				  begin
				     port_to_dcache_spec.send(tagged Invalid);
				     port_to_dcache_comm.send(tagged Invalid);
				     
				     stage <= ICACHE_STALL;
				  end
			    endcase
			 end
		   endcase
		end
	     tagged Valid .icachedelresp:
		begin
		   stage <= DEC;		
		end
	  endcase
       end 

       ICACHE_STALL:
       begin
	  let msg_from_dcache_imm_spec <- port_from_dcache_imm_spec.receive();
	  let msg_from_dcache_del_spec <- port_from_dcache_del_spec.receive();
	  let msg_from_dcache_imm_comm <- port_from_dcache_imm_comm.receive();
	  let msg_from_dcache_del_comm <- port_from_dcache_del_comm.receive();
	  
	  port_to_icache.send(tagged Invalid);
	  iCacheRead = True;
	  baseTick <= baseTick + 1;
	  link_model_cycle.send(0);
	  stage <= ICACHE;
       end
       
       
       DEC:
       begin
          debug_case("stage", "DEC");
	  
          if (!madeReq)
	     begin
		debug_then("!madeReq");
		
		//Decode current inst
		debug(2, $fdisplay(debug_log, "[%d] Decoding TOKEN %0d.", hostCC, cur_tok.index));
		link_to_dec.makeReq(cur_tok);
		
		madeReq <= True;
	     end
	  else
	     begin
		debug_else("!madeReq");
		
 		//Get the response
		match {.tok, .deps} = link_to_dec.getResp();
		link_to_dec.deq();
		
		debug(2, $fdisplay(debug_log, "[%d] DEC Responded with TOKEN %0d.", hostCC, tok.index));
		
		if (tok.index != cur_tok.index) $display ("DEC ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
		
		stage <= EXE;
		madeReq <= False;
	     end
       end 
       
       EXE:
       begin
	  
	  ISA_ADDRESS effMemAddr = 0;
	   
        debug_case("stage", "EXE");
        if (!madeReq)
	  begin
	    debug_then("!madeReq");
	    //Execute instruction
	    debug(2, $fdisplay(debug_log, "[%d] Executing TOKEN %0d", hostCC, cur_tok.index));
            link_to_exe.makeReq(cur_tok);
	    madeReq <= True;
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
 	    //Get the response
            //match {.tok, .res} = link_to_exe.getResp();
	     let exe_resp = link_to_exe.getResp();
	     link_to_exe.deq();
	     
	     let tok = exe_resp.token;
	     let res = exe_resp.result;

	    debug(2, $fdisplay(debug_log, "[%d] EXE Responded with TOKEN %0d.", hostCC, tok.index));
	    
	    if (tok.index != cur_tok.index) $display ("EXE ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
	   	
	    case (res) matches
	      tagged RBranchTaken .addr:
	      begin
	        debug(2, $fdisplay(debug_log, "Branch taken to address %h", addr));
	   	pc <= addr;
	      end
              tagged RBranchNotTaken .addr:
	      begin
	        debug(2, $fdisplay(debug_log, "Branch not taken"));
	   	pc <= pc + 4;
	      end
              tagged RTerminate .pf:
	      begin
	        debug(2, $fdisplay(debug_log, "Terminating Execution"));
                local_ctrl.endProgram(pf);
	      end
	       tagged REffectiveAddr .ea:
		  begin
		     debug(2, $fdisplay(debug_log, "Load/Store"));
		     effMemAddr = ea;
		     $display ("ea is %x", ea);
		  end
              default:
	      begin
	   	pc <= pc + 4;
	      end
	    endcase
	    
             if (isaIsLoad(cur_inst))
		begin
  		   //stage <= LOA;
		   
		   stage <= DCACHE;
		   req_stage <= LOA;
		   // make read request to data cache
		   port_to_dcache_spec.send(tagged Valid tuple2(tok, tagged Data_read_mem_ref tuple2(pc, pc)));
		   port_to_dcache_comm.send(tagged Invalid);
		   waitForDCache <= True;
		   //$display("Model PC: %x, Load Instruction", baseTick, pc);
		    
		end
             else if (isaIsStore(cur_inst))
		begin
		  // stage <= STO;
		   
		   stage <= DCACHE;
		   req_stage <= STO;
		   // make write request to data cache
		   port_to_dcache_spec.send(tagged Invalid);
		   port_to_dcache_comm.send(tagged Valid tuple2(tok, tagged Data_write_mem_ref tuple2(pc, effMemAddr)));
		  // $display("Model cycle: %d, Store Instruction", baseTick);
		   waitForDCache <= True;
		    
		end
            else
              stage <= LCO;

	    madeReq <= False;
	  end
       end
       
       DCACHE:
       begin
	  // read output ports from data cache
	  let dcache_ret_imm_spec <- port_from_dcache_imm_spec.receive();
	  let dcache_ret_del_spec <- port_from_dcache_del_spec.receive();
	  let dcache_ret_imm_comm <- port_from_dcache_imm_comm.receive();
	  let dcache_ret_del_comm <- port_from_dcache_del_comm.receive();
	  
	  if (req_stage == LOA) 
	     begin
		case (dcache_ret_del_spec) matches
		   tagged Invalid:
		      begin
			 case (dcache_ret_imm_spec) matches
			    tagged Invalid:
			       begin
				  if (!iCacheRead) 
				     port_to_icache.send(tagged Invalid);
				  stage <= DCACHE_STALL;
				  //$display ("Model cycle %d, stall", baseTick);
			       end
			    tagged Valid {.dcachetok, .dcachemsg}:
			       begin
				  case (dcachemsg) matches
				     tagged Hit .ref_addr:
					begin
					   stage <= req_stage;
					   //$display ("Model cycle %d, hit address %x", baseTick, ref_addr);
					end
				     tagged Miss_servicing .ref_addr:
					begin
					   if (!iCacheRead)
					      port_to_icache.send(tagged Invalid);
					   stage <= DCACHE_STALL;
					   //$display ("Model cycle %d, miss address %x", baseTick, ref_addr);
					end
				     tagged Hit_servicing .ref_addr:
					begin
					   if (!iCacheRead)
					      port_to_icache.send(tagged Invalid);
					   stage <= DCACHE_STALL;
					   //$display ("Model cycle %d, hit service address %x", baseTick, ref_addr);
					end
				     tagged Miss_retry .ref_addr:
					begin
					   // currently not implemented
					end
				  endcase
			       end
			 endcase      
		      end
		   tagged Valid .dcachemsg:
		      begin
			 stage <= LOA;
		      end
		endcase
	     end
	  else
	     begin
		case (dcache_ret_del_comm) matches
		   tagged Invalid:
		      begin
			 case (dcache_ret_imm_comm) matches
			    tagged Invalid:
			       begin
				  if (!iCacheRead) 
				     port_to_icache.send(tagged Invalid);
				  stage <= DCACHE_STALL;
				  //$display ("Model cycle %d, stall", baseTick);
			       end
			    tagged Valid {.dcachetok, .dcachemsg}:
			       begin
				  case (dcachemsg) matches
				     tagged Hit .ref_addr:
					begin
					   stage <= req_stage;
					   //$display ("Model cycle %d, hit address %x", baseTick, ref_addr);
					end
				     tagged Miss_servicing .ref_addr:
					begin
					   if (!iCacheRead)
					      port_to_icache.send(tagged Invalid);
					   stage <= DCACHE_STALL;
					   //$display ("Model cycle %d, miss address %x", baseTick, ref_addr);
					end
				     tagged Hit_servicing .ref_addr:
					begin
					   if (!iCacheRead)
					      port_to_icache.send(tagged Invalid);
					   stage <= DCACHE_STALL;
					   //$display ("Model cycle %d, hit service address %x", baseTick, ref_addr);
					end
				     tagged Miss_retry .ref_addr:
					begin
					   // currently not implemented
					end
				  endcase
			       end
			 endcase      
		      end
		   tagged Valid .dcachemsg:
		      begin
			 stage <= STO;
		      end
		endcase
	     end
       end
		
       
       DCACHE_STALL:
       begin
	  let msg_icache_imm <- port_from_icache_imm.receive();
	  let msg_icache_del <- port_from_icache_del.receive();
	  port_to_dcache_spec.send(tagged Invalid);
	  port_to_dcache_comm.send(tagged Invalid);
	  iCacheRead = False;
	  baseTick <= baseTick + 1;
	  link_model_cycle.send(0);
	  stage <= DCACHE;
       end
    
	      
      LOA:
       begin

	  debug_case("stage", "LOA");
	  if (!madeReq)
	     begin
		debug_then("!madeReq");
		
		//Request load
		debug(2, $fdisplay(debug_log, "[%d] Load for TOKEN %0d", hostCC, cur_tok.index));
		link_to_load.makeReq(cur_tok);
		madeReq <= True;
		
	     end
	  else
	     begin
		debug_else("!madeReq");
		
 		//Get the response
		let tok = link_to_load.getResp();
		link_to_load.deq();
		
		debug(2, $fdisplay(debug_log, "[%d] Load ops responded with TOKEN %0d.", hostCC, tok.index));
		
		if (tok.index != cur_tok.index) $display ("LOA ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
		
		stage <= LCO;
		madeReq <= False;
	     end
       end
       
       STO:
       begin
	  debug_case("stage", "STO");
	  if (!madeReq)
	     begin
		debug_then("!madeReq");
		
		//Request store
		debug(2, $fdisplay(debug_log, "[%d] Store for TOKEN %0d", hostCC, cur_tok.index));
		link_to_store.makeReq(cur_tok);
		madeReq <= True;
		
	     end
	  else
	     begin
		debug_else("!madeReq");
		
 		//Get the response
		let tok = link_to_store.getResp();
		link_to_store.deq();
		
		debug(2, $fdisplay(debug_log, "[%d] Store ops responded with TOKEN %0d.", hostCC, tok.index));
		
		if (tok.index != cur_tok.index) $display ("STO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
		
		stage <= LCO;
		madeReq <= False;
	     end
       end
       
      LCO:
       begin

	  
        debug_case("stage", "LCO");
        if (!madeReq)
	  begin
	    debug_then("!madeReq");
	    
	    //Request memory ops
	    debug(2, $fdisplay(debug_log, "[%d] Locally committing TOKEN %0d.", hostCC, cur_tok.index));
            link_to_lco.makeReq(cur_tok);
	    
	    madeReq <= True;
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
 	    //Get the response
  
            let tok = link_to_lco.getResp();
	    link_to_lco.deq();

	    debug(2, $fdisplay(debug_log, "[%d] LCO Responded with TOKEN %0d.", hostCC, tok.index));
	    
	    if (tok.index != cur_tok.index) $display ("LCO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
	    if (isaIsStore(cur_inst))
	      stage <= GCO;
            else
            begin
              stage <= TOK;
              baseTick <= baseTick + 1;
	      debug(1, $fdisplay(debug_log, "Committed TOKEN %0d on model cycle %0d.", cur_tok.index, baseTick));
	      event_com.recordEvent(tagged Valid zeroExtend(cur_tok.index));
              link_model_commit.send(tuple2(0, 1));
              stat_com.incr();
            end
	    madeReq <= False;
	  end
      end
      GCO:
       begin
	    
        debug_case("stage", "GCO");
        if (!madeReq)
	  begin
	    debug_then("!madeReq");
	    
	    //Request memory ops
	    debug(2, $fdisplay(debug_log, "[%d] Globally committing TOKEN %0d", hostCC, cur_tok.index));
            link_to_gco.makeReq(cur_tok);
	    
	    madeReq <= True;
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
 	    //Get the response
            let tok = link_to_gco.getResp();
	    link_to_gco.deq();

	    debug(2, $fdisplay(debug_log, "[%d] GCO Responded with TOKEN %0d.", hostCC, tok.index));
	    
	    if (tok.index != cur_tok.index) $display ("GCO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);
	    
	    debug(1, $fdisplay(debug_log, "Committed TOKEN %0d on model cycle %0d.", cur_tok.index, baseTick));
	    event_com.recordEvent(tagged Valid zeroExtend(cur_tok.index));
            link_model_commit.send(tuple2(0, 1));
            stat_com.incr();
	    
	    stage <= TOK;
	    madeReq <= False;
	    baseTick <= baseTick + 1;
	  end
      end
    endcase    
  endrule
  
endmodule
`undef MODULE_NAME
