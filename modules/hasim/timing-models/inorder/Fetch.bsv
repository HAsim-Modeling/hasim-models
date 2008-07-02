import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

`include "asim/provides/funcp_simulated_memory.bsh"

// ABHISHEK add 
`include "asim/provides/hasim_icache.bsh"
// ABHISHEK end 

import FShow::*;
import Vector::*;

typedef enum { FETCH_STATE_REWIND, FETCH_STATE_REW_RESP, FETCH_STATE_TOKEN, FETCH_STATE_ICACHE_REQ, FETCH_STATE_INST, FETCH_STATE_SEND } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch ();

    DebugFile debug <- mkDebugFile("pipe_fetch.out", "PIPE: FETCH:\t");

    Reg#(ISA_ADDRESS) pc <- mkReg(`PROGRAM_START_ADDR);

    StallPort_Send#(Tuple2#(TOKEN,ISA_INSTRUCTION))   outQ    <- mkStallPort_Send("fet2dec");
    Port_Receive#(Tuple2#(TOKEN,Maybe#(ISA_ADDRESS))) rewindQ <- mkPort_Receive  ("rewind", 1);

    Connection_Send#(Bool) model_cycle <- mkConnection_Send("model_cycle");

    Connection_Client#(UNIT,TOKEN)                     newInFlight    <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(Tuple2#(TOKEN,ISA_ADDRESS),
                       Tuple2#(TOKEN,ISA_INSTRUCTION)) getInstruction <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(TOKEN,UNIT)                     rewindToToken  <- mkConnection_Client("funcp_rewindToToken");

    Reg#(FETCH_STATE) state <- mkReg(FETCH_STATE_REWIND);

   Reg#(TOKEN_TIMEP_EPOCH) epoch <- mkReg(0);
   
   // ABHISHEK add
   Port_Send#(Tuple2#(TOKEN, CacheInput)) port_to_icache <- mkPort_Send("cpu_to_icache");
   
   Port_Receive#(Tuple2#(TOKEN, CacheOutput)) port_from_icache <- mkPort_Receive("icache_to_cpu", 0);
   
   Reg#(Bool) waitForICache <- mkReg(False);
   // ABHISHEK end

    //Local Controller
   /*
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    */
    //inports[0]  = inQ.ctrl;
    //outports[0] = outQ.ctrl;
   
   // ABHISHEK add
   Vector#(0, Port_Control) inports = newVector();
   Vector#(0, Port_Control) outports  = newVector();
   inports[0] = port_from_icache.ctrl;
   outports[0] = port_to_icache.ctrl;
   // ABHISHEK end 
   
   
    LocalController local_ctrl <- mkLocalController(inports, outports);
   
    rule rewind (state == FETCH_STATE_REWIND);
        local_ctrl.startModelCC();
        model_cycle.send(?);
        let x <- rewindQ.receive();
        if (x matches tagged Valid { .tok, .ma })
        begin
            if (ma matches tagged Valid .a)
            begin
                debug <= fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", a);
                rewindToToken.makeReq(tok);
                pc <= a;
                epoch <= epoch + 1;
                state <= FETCH_STATE_REW_RESP;
            end
        end
        else
            state <= FETCH_STATE_TOKEN;
    endrule

    rule rewindResp (state == FETCH_STATE_REW_RESP);
        rewindToToken.deq();
        state <= FETCH_STATE_TOKEN;
    endrule

   rule token (state == FETCH_STATE_TOKEN && outQ.canSend);
      if(waitForICache)
	 begin
	    state <= FETCH_STATE_INST;
	    port_to_icache.send(tagged Invalid);
	 end
      else
	  begin  
             newInFlight.makeReq(?);
             //state <= FETCH_STATE_INST;
	     state <= FETCH_STATE_ICACHE_REQ;
	  end
    endrule

    rule pass (state == FETCH_STATE_TOKEN);
        debug <= fshow("PASS");
        outQ.pass();
       state <= FETCH_STATE_REWIND;
       
       port_to_icache.send(tagged Invalid);
       let icache_resp <- port_from_icache.receive();       
    endrule
   
   rule icachefetch (state == FETCH_STATE_ICACHE_REQ);
      newInFlight.deq();
      let tok = newInFlight.getResp();
      tok.timep_info = TIMEP_TokInfo { epoch: epoch, scratchpad: 0 };
      port_to_icache.send(tagged Valid tuple2(tok, tagged Inst_mem_ref pc));
      state <= FETCH_STATE_INST;
   endrule
   
   rule inst (state == FETCH_STATE_INST);
      //newInFlight.deq();
      //let tok = newInFlight.getResp();
      //tok.timep_info = TIMEP_TokInfo { epoch: epoch, scratchpad: 0 };
      let icache_ret <- port_from_icache.receive();   
      case (icache_ret) matches
	 tagged Invalid:  // miss is still begin serviced
	    begin
	       waitForICache <= True;
	       //port_to_icache.send(tagged Invalid);
	       $display ("Invalid");
	    end
	 
	 tagged Valid {.icachetok, .icachemsg}:
	    begin
	       case (icachemsg) matches
		  tagged Hit:
		     begin
			waitForICache <= False;
			getInstruction.makeReq(tuple2(icachetok,pc));
			debug <= fshow("FETCHINST: ") + fshow(icachetok) + $format(" ADDR:0x%h", pc);
			pc <= pc + 4;
			$display ("Hit");
		     end
		  
		  tagged Miss_servicing:
		     begin
			waitForICache <= True;
			//port_to_icache.send(tagged Invalid);
			$display ("Miss servicing");
		     end
		  
		  tagged Miss_retry:
		     begin
			waitForICache <= True;
			// not currently implemented
		     end 
		  
		  tagged Miss_response:
		     begin
			waitForICache <= False;
			getInstruction.makeReq(tuple2(icachetok,pc));
			debug <= fshow("FETCHINST: ") + fshow(icachetok) + $format(" ADDR:0x%h", pc);
			pc <= pc + 4;
			$display ("Miss response");
		     end
	       endcase
	    end
      endcase      
      /*
      getInstruction.makeReq(tuple2(tok,pc));
      debug <= fshow("FETCHINST: ") + fshow(tok) + $format(" ADDR:0x%h", pc);
      pc <= pc + 4;
       */
      state <= FETCH_STATE_SEND;
   endrule
   
   rule done (state == FETCH_STATE_SEND);
      if (waitForICache)
	 begin
	    outQ.send(tagged Invalid);
	    state <= FETCH_STATE_REWIND;
	 end
      else
	 begin
            getInstruction.deq();
	    let inst = getInstruction.getResp();
	    outQ.send(Valid(inst));
	    state <= FETCH_STATE_REWIND;
	 end
    endrule

endmodule
