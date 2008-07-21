
import LFSR::*;
import RegFile::*;
import Vector::*;
 
`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_modellib.bsh"

`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_branch_pred.bsh"

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"
`include "asim/dict/PARAMS_HASIM_CPU.bsh"

`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/hasim_5stage_interfaces.bsh"

//AWB Parameters            default:
//FET_ICACHE_HIT_CHANCE       125   (dynamic)
//FET_ICACHE_MISS_PENALTY     10    (dynamic)
//FET_BTB_HASH_BITS           8


typedef enum 
{
  FET_Init,
  FET_Rewind,
  FET_Ready,
  FET_Ready2,
  FET_GetInst,
  FET_Finish
}
  FET_STATE
    deriving (Eq, Bits);


typedef Bit#(`FET_BTB_HASH_BITS) ISA_ADDRESS_HASH;

function ISA_ADDRESS_HASH btbHash(ISA_ADDRESS a);

    return truncate(hashTo32(a[31:2]));

endfunction

module [HASIM_MODULE] mkPipe_Fetch#(File debug_file, Bit#(32) curTick)
    //interface:
                ()
    provisos
            (Bits#(TOKEN_INDEX, idx_SZ));

  //Local State

  Reg#(ISA_ADDRESS)           pc <- mkReg(`PROGRAM_START_ADDR);
  Reg#(TOKEN_TIMEP_EPOCH)     epoch <- mkReg(0);
  Reg#(TOKEN)          stall_tok <- mkRegU;
  Reg#(ISA_ADDRESS)   stall_addr <- mkRegU;
  Reg#(ISA_INSTRUCTION)    stall_inst <- mkRegU;
  Reg#(Bit#(16))     stall_count <- mkReg(0);
  Reg#(Bool)                 stalling <- mkReg(False);
  Reg#(FET_STATE)               state <- mkReg(FET_Init);
  
  Reg#(Bit#(64)) counter <- mkReg(0);

  //For branch prediction
  
  Param#(1)  predOnlyBranches  <- mkDynamicParameter(`PARAMS_HASIM_CPU_FET_PRED_ONLY_BRANCHES);
  Param#(7)  icacheHitChance   <- mkDynamicParameter(`PARAMS_HASIM_CPU_FET_ICACHE_HIT_CHANCE);
  Param#(16) icacheMissPenalty <- mkDynamicParameter(`PARAMS_HASIM_CPU_FET_ICACHE_MISS_PENALTY);
  BranchPred branch_pred <- mkBranchPred();
  BRAM#(`FET_BTB_HASH_BITS, Maybe#(ISA_ADDRESS)) btb <- mkBramInitialized(tagged Invalid);
  
  //Pseudo-randomness
  LFSR#(Bit#(7)) lfsr <- mkFeedLFSR(7'b1001110);

  //Connections to controller
  Connection_Send#(Bool) link_model_cycle <- mkConnection_Send("model_cycle");

  //Connections to FP
  Connection_Send#(Bit#(1))   fp_tok_req  <- mkConnection_Send("funcp_newInFlight_req");
  Connection_Receive#(TOKEN)  fp_tok_resp <- mkConnection_Receive("funcp_newInFlight_resp");
  
  Connection_Send#(Tuple2#(TOKEN, ISA_ADDRESS))         fp_fet_req  <- mkConnection_Send("funcp_getInstruction_req");
  Connection_Receive#(Tuple2#(TOKEN, ISA_INSTRUCTION))  fp_fet_resp <- mkConnection_Receive("funcp_getInstruction_resp");
      
  Connection_Send#(Token)     rewindToToken <- mkConnection_Send("funcp_rewindToToken_req");

  //Events
  EventRecorder event_fet <- mkEventRecorder(`EVENTS_FETCH_INSTRUCTION_FET);
  
  //Stats
  Stat stat_cycles   <- mkStatCounter(`STATS_FETCH_TOTAL_CYCLES);
  Stat stat_fet      <- mkStatCounter(`STATS_FETCH_INSTS_FETCHED);
  Stat stat_imisses  <- mkStatCounter(`STATS_FETCH_ICACHE_MISSES);

    
  //Incoming Ports
  Port_Receive#(EXE_TO_FET_MSG) port_from_exe <- mkPort_Receive("fet_branchResolve", 1);

  Connection_Receive#(Bit#(1)) rewind <- mkConnection_Receive("funcp_rewindToToken_resp");

  //Outgoing Ports
  Port_Send#(Tuple3#(TOKEN, ISA_ADDRESS, ISA_INSTRUCTION)) port_to_dec <- mkPort_Send("fet_to_dec");

  //Local Controller
  Vector#(1, Port_Control) inports  = newVector();
  Vector#(1, Port_Control) outports = newVector();
  inports[0]  = port_from_exe.ctrl;
  outports[0] = port_to_dec.ctrl;
  LocalController local_ctrl <- mkLocalController(inports, outports);

    rule initialize (state == FET_Init);
        
        lfsr.seed(1);
        state <= FET_Ready;

    endrule

    rule beginFetch (state == FET_Ready);
  
        local_ctrl.startModelCC();

        counter <= counter + 1;

        $fdisplay(debug_file, "[%d]:FET: ****** Begin Model Cycle: %0d ******", curTick, counter);

        let exe_resp <- port_from_exe.receive();
        stat_cycles.incr();

        //Note new model cycle
        link_model_cycle.send(?);

        case (exe_resp) matches
            tagged Invalid: //No Re-steer
                state <= FET_Ready2;

            tagged Valid .pinfo: //Re-steer
            begin
                // Look up this token
                Bool pred_taken = pinfo.token.timep_info.scratchpad[0] == 1; //The prediction is stored in the scratchpad
                let cur_pc = pinfo.instrPC;    // Address of the current instr
                let new_pc = pinfo.newPC;      // Correct next PC

                let hash = btbHash(cur_pc);    // Hash the address
        
                if (pinfo.correctPrediction)
                begin 
                    //
                    // Correct prediction.  Just keep going...
                    //
                    if (pinfo.updatePredictor)
                        branch_pred.upd(pinfo.token, cur_pc, pred_taken, pred_taken);
                    state <= FET_Ready2;
                end
                else
                begin
                    //
                    // Incorrect prediction.  Resteer to new PC.
                    //
                    if (pinfo.updatePredictor)
                    begin
                        branch_pred.upd(pinfo.token, cur_pc, pred_taken, !pred_taken);

                        // Update BTB if branch is taken.  (Mispredicted here, so
                        // test !pred_taken
                        if (! pred_taken)
                        begin
                            btb.write(hash, tagged Valid new_pc);
                        end
                    end

                    epoch <= epoch + 1;
                    state <= FET_Rewind;
                    rewindToToken.send(pinfo.token);
                    $fdisplay(debug_file, "[%d]:Fetch Counter when rewinding: %0d", curTick, counter);
                    $fdisplay(debug_file, "[%d]:Fetch resuming from PC 0x%0x", curTick, new_pc);

                    pc <= new_pc;
                end
            end
        endcase

    endrule
  
  rule beginFetch2 (state == FET_Ready2);
    if (!stalling)
      begin
        $fdisplay(debug_file, "[%d]:TOK:REQ", curTick);
        fp_tok_req.send(?);
        state <= FET_GetInst;

      end
    else
      begin
        state <= FET_Ready;
        if (stall_count == 0)
          begin
            port_to_dec.send(tagged Valid tuple3(stall_tok, stall_addr, stall_inst));
            event_fet.recordEvent(tagged Valid zeroExtend(stall_tok.index));
            stat_fet.incr();
            stalling <= False;
          end
        else
          begin
            port_to_dec.send(tagged Invalid);
            event_fet.recordEvent(tagged Invalid);
            stall_count <= stall_count - 1;
          end
      end

   endrule
   
   rule fetchInst (state == FET_GetInst);

     let tok = fp_tok_resp.receive();
     fp_tok_resp.deq();

     $fdisplay(debug_file, "[%d]:TOK:RSP: %0d", curTick, tok.index);
     
     let inf = TIMEP_TokInfo {epoch: epoch, scratchpad: 0};
     tok.timep_info = inf;

     $fdisplay(debug_file, "[%d]:FET:REQ: %0d:0x%h", curTick, tok.index, pc);
     fp_fet_req.send(tuple2(tok, pc));
     branch_pred.getPredReq(tok, pc);
     btb.readReq(btbHash(pc));

     state <= FET_Finish;
     
   endrule


   rule finishFetch (state == FET_Finish);
   
     match {.tok, .inst} = fp_fet_resp.receive();
     fp_fet_resp.deq();
     
     $fdisplay(debug_file, "[%d]:FET:RSP: %0d:0x%h", curTick, tok.index, inst);
     
     let pred_taken <- branch_pred.getPredResp();
     let btb_resp <- btb.readResp();
     
     // Only use branch predictor if instruction is a branch
     if ((predOnlyBranches == 1) && ! isaIsBranch(inst))
     begin
         pred_taken = False;
     end

     if (! isValid(btb_resp))
     begin
         pred_taken = False;
     end

     tok.timep_info.scratchpad[0] = pack(pred_taken);

     let pred_addr = pred_taken ? validValue(btb_resp) : pc + 4;
       
     $fdisplay(debug_file, "[%d]:FET:BR: Taken=%d BTB=0x%0x", curTick, pred_taken, isValid(btb_resp) ? validValue(btb_resp) : -1);

     pc <= pred_addr;

     let isHit = lfsr.value <= icacheHitChance;
     lfsr.next();

     if (isHit)
     begin
     
       port_to_dec.send(tagged Valid tuple3(tok, pred_addr, inst));
       event_fet.recordEvent(tagged Valid zeroExtend(tok.index));
       stat_fet.incr();

     end
     else
     begin
       port_to_dec.send(tagged Invalid);
       event_fet.recordEvent(tagged Invalid);
       stat_imisses.incr();
       stall_count <= icacheMissPenalty;
       stall_tok   <= tok;
       stall_addr  <= pred_addr;
       stall_inst  <= inst;
       stalling    <= True;
     end
     
     state       <= FET_Ready;
     
   endrule

  rule fetchFinishRewind(state == FET_Rewind);

      rewind.deq();
      state <= FET_Ready2;
 
  endrule

endmodule
