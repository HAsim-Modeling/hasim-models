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

typedef enum { FETCH_STATE_REWIND, FETCH_STATE_REW_RESP, FETCH_STATE_TOKEN, FETCH_STATE_ICACHE_REQ, FETCH_STATE_INST, FETCH_STATE_SEND, FETCH_STATE_PASS } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch ();

    DebugFile debug <- mkDebugFile("pipe_fetch.out");

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

    Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) port_from_icache_immediate <- mkPort_Receive("icache_to_cpu_immediate", 0);
    Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed))   port_from_icache_delayed   <- mkPort_Receive("icache_to_cpu_delayed", 0);

    Reg#(Bool) waitForICache <- mkReg(False);
    // ABHISHEK end

    //Local Controller
    Vector#(3, Port_Control) inports  = newVector();
    Vector#(2, Port_Control) outports = newVector();
    inports[0]  = rewindQ.ctrl;
    inports[1]  = port_from_icache_immediate.ctrl;
    inports[2]  = port_from_icache_delayed.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = port_to_icache.ctrl;

    LocalController local_ctrl <- mkLocalController(inports, outports);
   
    rule rewind (state == FETCH_STATE_REWIND);
        local_ctrl.startModelCC();
        debug.startModelCC();
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

    rule pass (state == FETCH_STATE_TOKEN && !outQ.canSend);
        outQ.pass();
        port_to_icache.send(tagged Invalid);
        state <= FETCH_STATE_PASS;
    endrule
    rule pass2 (state == FETCH_STATE_PASS);
        debug <= fshow("PASS");
        state <= FETCH_STATE_REWIND;
        let icache_resp_imm <- port_from_icache_immediate.receive();
        let icache_resp_del <- port_from_icache_delayed.receive();
        // assert icache_resp == Invalid.
    endrule

   rule icachefetch (state == FETCH_STATE_ICACHE_REQ);
      newInFlight.deq();
      let tok = newInFlight.getResp();
      tok.timep_info = TIMEP_TokInfo { epoch: epoch, scratchpad: 0 };
      port_to_icache.send(tagged Valid tuple2(tok, tagged Inst_mem_ref pc));
      pc <= pc + 4;
      state <= FETCH_STATE_INST;
   endrule
   
   rule inst (state == FETCH_STATE_INST);
      let icache_ret_imm <- port_from_icache_immediate.receive();   
      let icache_ret_del <- port_from_icache_delayed.receive();   
      case (icache_ret_del) matches
        tagged Invalid:
          case (icache_ret_imm) matches
            tagged Invalid:  // miss is still begin serviced
              begin
                  debug <= fshow("MISS: STALL");
                  waitForICache <= True;
              end

            tagged Valid {.icachetok, .icachemsg}:
              begin
                  case (icachemsg) matches
                    tagged Hit .reqpc:
                      begin
                          waitForICache <= False;
                          getInstruction.makeReq(tuple2(icachetok,reqpc));
                          debug <= fshow("HIT: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc);
                      end

                    tagged Miss_servicing .reqpc:
                      begin
                          debug <= fshow("MISS: STALL");
                          waitForICache <= True;
                      end

                    tagged Miss_retry .reqpc:
                      begin
                          debug <= fshow("MISS: RETRY");
                          waitForICache <= True;
                          // not currently implemented
                      end 

                  endcase
              end
          endcase
        tagged Valid { .icachetok, .icachemsg }:
          case (icachemsg) matches
            tagged Miss_response .reqpc:
              begin
                  waitForICache <= False;
                  getInstruction.makeReq(tuple2(icachetok,reqpc));
                  debug <= fshow("MISS: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc);
              end
          endcase
      endcase
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
