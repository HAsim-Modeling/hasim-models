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
import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import module_local_controller::*;

//Model-specific imports
import hasim_isa::*;

`include "asim/dict/EVENTS_CPU.bsh"
`include "asim/dict/STATS_CPU.bsh"

`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/hasim_icache.bsh"

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
 TOK, FET, DEC, EXE, LOA, STO, LCO, GCO 
 } 
Stage deriving (Eq, Bits);

module [HASim_Module] mkCPU
     //interface:
                 ();

  Reg#(File) debug_log <- mkReg(InvalidFile);
  
  //********* State Elements *********//
  
  //Have we made a req to FP and are waiting for a response?
  Reg#(Bool) madeReq <- mkReg(False);
  
  //The current stage
  Reg#(Stage) stage <- mkReg(TOK);
  
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
  
  Connection_Send#(Bool)
  //...
  link_model_cycle <- mkConnection_Send("model_cycle");

  Connection_Client#(Bit#(1), TOKEN)
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
                     Tuple2#(TOKEN, ISA_EXECUTION_RESULT))
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
  
  Connection_Client#(TOKEN, Bit#(1)) 
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
  
   Port_Receive#(Tuple2#(TOKEN, CacheOutput)) port_from_icache <- mkPort_Receive("icache_to_cpu", 0); // port from icache       

   // state for communication with ICache
   Reg#(Bool) waitForICache <- mkReg(True);
   Reg#(TOKEN) icache_tok <- mkRegU();
   
   /********* Communication with local controller for icache ports ******/
   Vector#(1, Port_Control) inports  = newVector();
   Vector#(1, Port_Control) outports = newVector();
   inports[0]  = port_from_icache.ctrl;
   outports[0] = port_to_icache.ctrl;
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
            link_model_cycle.send(?);
	    
	    madeReq <= True;
	    
	  end
	else
	  begin
	    debug_else("!madeReq");
	    
	    //Get the response
	    let tok = link_to_tok.getResp();
	    link_to_tok.deq();
	    
            tok.timep_info = TIMEP_TokInfo{epoch: 0, scratchpad: 0};

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
	    
	    stage <= DEC;
	     madeReq <= False;
	     
	     // Make a request to the instruction cache
	     // Standard memory reference for now (no prefetches etc.)
	     port_to_icache.send(tagged Valid tuple2(cur_tok, tagged Inst_mem_ref pc));
	     waitForICache <= True;	   
	     
	  end
      end
      DEC:
      begin
         debug_case("stage", "DEC");
	 /* Loop in this state until ICache responds with data */
	 if (waitForICache)
	    begin
	       // Check if the instruction cache has returned anything //
	       let icache_ret <- port_from_icache.receive();
	       case (icache_ret) matches
		  tagged Invalid: // go to an intermediate stall state
		     begin
			waitForICache <= True;
			port_to_icache.send(tagged Invalid);  // this is a NoMessage
		        baseTick <= baseTick + 1;      // stalling so increment model cycle
		     end
		  tagged Valid {.icachetok, .icachemsg}:   // message received from ICache
		     begin
			case(icachemsg) matches
			   tagged Hit:   // ICache hit
			      begin
				 waitForICache <= False;
			      end
			   tagged Miss_servicing: // ICache miss being serviced by memory
			      begin 
				 port_to_icache.send(tagged Invalid);  // this is a NoMessage
				 baseTick <= baseTick + 1;
			      end
			   tagged Miss_retry:   // ICache miss, retry because of lack of buffer space
			      begin
				 /* Currently not implemented */
			      end
			   tagged Miss_response:  // ICache miss has been serviced
			      begin
				 waitForICache <= False;
			      end
			endcase
		     end
	       endcase  
	    end 	 
	 
	 else 
	    begin		       
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
      end
       


      EXE:
       begin

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
            match {.tok, .res} = link_to_exe.getResp();
	    link_to_exe.deq();

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
              default:
	      begin
	   	pc <= pc + 4;
	      end
	    endcase
	    
            if (isaIsLoad(cur_inst))
  	      stage <= LOA;
            else if (isaIsStore(cur_inst))
              stage <= STO;
            else
              stage <= LCO;

	    madeReq <= False;
	  end
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
