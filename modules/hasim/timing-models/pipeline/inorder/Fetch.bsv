import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/hasim_icache.bsh"
`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"

import FShow::*;
import FIFO::*;
import Vector::*;

typedef enum { FETCH_STATE_NEXTPC, FETCH_STATE_REWIND, FETCH_STATE_REW_RESP, FETCH_STATE_TOKEN, FETCH_STATE_ICACHE_REQ, FETCH_STATE_ITRANS_REQ, FETCH_STATE_INST_REQ, FETCH_STATE_SEND, FETCH_STATE_PASS } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch ();

    ModelDebugFile debug <- mkModelDebugFile("pipe_fetch.out");

    Reg#(ISA_ADDRESS)          pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(TOKEN_TIMEP_EPOCH) epoch <- mkReg(0);
    FIFO#(BRANCH_ATTR)  pred_fifo <- mkFIFO1;
    FIFO#(ISA_ADDRESS)    pc_fifo <- mkFIFO1;

    StallPort_Send#(Tuple2#(TOKEN,FETCH_BUNDLE))   outQ <- mkStallPort_Send("fet2dec");
    Port_Receive#(Tuple2#(TOKEN,ISA_ADDRESS))      rewindQ <- mkPort_Receive  ("rewind", 1);

    Port_Send#(ISA_ADDRESS)    bpQ     <- mkPort_Send("bp_req");
    Port_Receive#(ISA_ADDRESS) nextpcQ <- mkPort_Receive("bp_reply_pc", 1);
    Port_Receive#(BRANCH_ATTR) predQ   <- mkPort_Receive("bp_reply_pred", 0);

    Connection_Send#(Bool) model_cycle <- mkConnection_Send("model_cycle");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT,
                       FUNCP_RSP_NEW_IN_FLIGHT)      newInFlight    <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE,
                       FUNCP_RSP_DO_ITRANSLATE)      doITranslate   <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)    getInstruction <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)    rewindToToken  <- mkConnection_Client("funcp_rewindToToken");

    Reg#(FETCH_STATE) state <- mkReg(FETCH_STATE_NEXTPC);

    Port_Send#(Tuple2#(TOKEN, CacheInput)) port_to_icache <- mkPort_Send("cpu_to_icache");

    Port_Receive#(Tuple2#(TOKEN, CacheOutputImmediate)) port_from_icache_immediate <- mkPort_Receive("icache_to_cpu_immediate", 0);
    Port_Receive#(Tuple2#(TOKEN, CacheOutputDelayed))   port_from_icache_delayed   <- mkPort_Receive("icache_to_cpu_delayed", 0);

    Reg#(Bool) waitForICache <- mkReg(False);

    //Local Controller
    Vector#(5, Port_Control) inports  = newVector();
    Vector#(3, Port_Control) outports = newVector();
    inports[0]  = rewindQ.ctrl;
    inports[1]  = port_from_icache_immediate.ctrl;
    inports[2]  = port_from_icache_delayed.ctrl;
    inports[3]  = nextpcQ.ctrl;
    inports[4]  = predQ.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = port_to_icache.ctrl;
    outports[2] = bpQ.ctrl;

    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_fet <- mkEventRecorder(`EVENTS_FETCH_INSTRUCTION_FET);

    //Stats
    Stat stat_cycles   <- mkStatCounter(`STATS_FETCH_TOTAL_CYCLES);
    Stat stat_fet      <- mkStatCounter(`STATS_FETCH_INSTS_FETCHED);
    Stat stat_imisses  <- mkStatCounter(`STATS_FETCH_ICACHE_MISSES);

    rule nextpc (state == FETCH_STATE_NEXTPC);
        local_ctrl.startModelCC();
        debug.startModelCC();
        stat_cycles.incr();
        model_cycle.send(?);
        let x <- nextpcQ.receive();
        if (x matches tagged Valid .a)
            pc <= a;
        state <= FETCH_STATE_REWIND;
    endrule

    rule rewind (state == FETCH_STATE_REWIND);
        let x <- rewindQ.receive();
        if (x matches tagged Valid { .tok, .a })
        begin
            debug <= fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", a);
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            pc <= a;
            epoch <= epoch + 1;
            state <= FETCH_STATE_REW_RESP;
        end
        else
            state <= FETCH_STATE_TOKEN;
    endrule

    rule rewindResp (state == FETCH_STATE_REW_RESP);
        rewindToToken.deq();
        state <= FETCH_STATE_TOKEN;
    endrule

    rule token (state == FETCH_STATE_TOKEN && outQ.canSend);
        if (waitForICache)
        begin
            state <= FETCH_STATE_ITRANS_REQ;
            port_to_icache.send(tagged Invalid);
            bpQ.send(Invalid);
        end
        else
        begin
            newInFlight.makeReq(initFuncpReqNewInFlight());
            state <= FETCH_STATE_ICACHE_REQ;
	end
    endrule

    rule pass (state == FETCH_STATE_TOKEN && !outQ.canSend);
        outQ.pass();
        event_fet.recordEvent(Invalid);
        port_to_icache.send(tagged Invalid);
        bpQ.send(Invalid);
        state <= FETCH_STATE_PASS;
    endrule
    rule pass2 (state == FETCH_STATE_PASS);
        debug <= fshow("PASS");
        let icache_resp_imm <- port_from_icache_immediate.receive();
        let icache_resp_del <- port_from_icache_delayed.receive();
        // assert icache_resp == Invalid.
        state <= FETCH_STATE_NEXTPC;
    endrule

   rule icachefetch (state == FETCH_STATE_ICACHE_REQ);
      newInFlight.deq();
      let rsp = newInFlight.getResp();
      let tok = rsp.newToken;
      tok.timep_info = TOKEN_TIMEP_INFO { epoch: epoch, scratchpad: 0 };
      port_to_icache.send(tagged Valid tuple2(tok, tagged Inst_mem_ref pc));
      bpQ.send(Valid(pc));
      state <= FETCH_STATE_ITRANS_REQ;
   endrule

   rule inst (state == FETCH_STATE_ITRANS_REQ);
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
                          doITranslate.makeReq(initFuncpReqDoITranslate(icachetok,reqpc));
                          pc_fifo.enq(reqpc);
                          debug <= fshow("HIT: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc);
                          stat_fet.incr();
                      end

                    tagged Miss_servicing .reqpc:
                      begin
                          debug <= fshow("MISS: STALL");
                          stat_imisses.incr();
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
                  doITranslate.makeReq(initFuncpReqDoITranslate(icachetok,reqpc));
                  pc_fifo.enq(reqpc);
                  debug <= fshow("MISS: RESP: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc);
                  stat_fet.incr();
              end
          endcase
      endcase
      state <= FETCH_STATE_INST_REQ;
   endrule

   rule getInst (state == FETCH_STATE_INST_REQ);
      if (waitForICache)
	 begin
	    outQ.send(tagged Invalid);
            event_fet.recordEvent(Invalid);
	    state <= FETCH_STATE_NEXTPC;
	 end
      else
	 begin
            doITranslate.deq();
            let rsp = doITranslate.getResp();
            
            if (!rsp.hasMore)
            begin
            
                state <= FETCH_STATE_SEND;
                getInstruction.makeReq(initFuncpReqGetInstruction(rsp.token));
            
            end
     end

    endrule
    
    rule done (state == FETCH_STATE_SEND);
 
        let rsp = getInstruction.getResp();
        getInstruction.deq();
        let tok = rsp.token;
        let bundle = FETCH_BUNDLE { pc: pc_fifo.first(), inst: rsp.instruction, branchAttr: pred_fifo.first() };
        outQ.send(Valid(tuple2(tok,bundle)));
        pc_fifo.deq();
        pred_fifo.deq();
        event_fet.recordEvent(Valid(zeroExtend(tok.index)));
        state <= FETCH_STATE_NEXTPC;

    endrule

    rule get_pred; // any state
        let x <- predQ.receive();
        if (x matches tagged Valid .pred)
            pred_fifo.enq(pred);
    endrule

endmodule
