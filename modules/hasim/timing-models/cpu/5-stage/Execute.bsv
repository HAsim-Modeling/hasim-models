import FIFO::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_5stage_interfaces.bsh"

`include "asim/dict/EVENTS_EXECUTE.bsh"
`include "asim/dict/STATS_EXECUTE.bsh"


typedef enum
{
    PRED_CORRECT,                  // Branch prediction is right
    PRED_WRONG_BRANCH,             // Branch is wrong
    PRED_NONBRANCH,                // Nobranch predicted as branch
    PRED_DRAIN                     // Drain resteer after emulation
 }
    PRED_TYPE
        deriving (Eq, Bits);


module [HASIM_MODULE] mkPipe_Execute#(File debug_file, Bit#(32) curTick)
    //interface:
                ();
  
  //Local State
  Reg#(TOKEN_TIMEP_EPOCH)  epoch     <- mkReg(0);
  Reg#(Bool)   in_flight <- mkReg(False);
  FIFO#(Tuple4#(ISA_ADDRESS, Bool, Bool, Bool)) addrQ <- mkFIFO();

  //Connections to FP
  
  Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS) fp_exe <- mkConnection_Client("funcp_getResults");

  //Events
  EventRecorder event_exe <- mkEventRecorder(`EVENTS_EXECUTE_INSTRUCTION_EXECUTE);
  
  //Stats
  Stat stat_mpred <- mkStatCounter(`STATS_EXECUTE_BPRED_MISPREDS);
  
  //Incoming Ports
  Port_Receive#(Tuple5#(TOKEN, ISA_ADDRESS, Bool, Bool, Bool)) port_from_dec <- mkPort_Receive("dec_to_exe", 1);

  //Outgoing Ports
  Port_Send#(Tuple3#(TOKEN, Bool, Bool))                        port_to_mem <- mkPort_Send("exe_to_mem");
  Port_Send#(EXE_TO_FET_MSG) port_to_fet <- mkPort_Send("fet_branchResolve");

    Reg#(Bit#(64)) counter <- mkReg(0);

  //Local Controller
  Vector#(1, Port_Control) inports  = newVector();
  Vector#(2, Port_Control) outports = newVector();
  inports[0]  = port_from_dec.ctrl;
  outports[0] = port_to_mem.ctrl;
  outports[1] = port_to_fet.ctrl;
  LocalController local_ctrl <- mkLocalController(inports, outports);

    //
    // msgToFetch converts internal state to a resteer request in the front end.
    //
    function EXE_TO_FET_MSG msgToFetch(TOKEN tok, PRED_TYPE pType, ISA_ADDRESS newPC, ISA_ADDRESS curPC);
    
        if (pType == PRED_DRAIN)
        begin
            //
            // Drain following non-branch emulation.
            //
            return EXE_TO_FET_MSG {token: tok,
                                   updatePredictor: False,
                                   correctPrediction: False,
                                   instrPC: curPC,
                                   newPC: newPC};
        end
        else if (pType == PRED_NONBRANCH)
        begin
            //
            // Not a branch instruction, but front end predicted it was
            //
            return EXE_TO_FET_MSG {token: tok,
                                   updatePredictor: True,
                                   correctPrediction: False,
                                   instrPC: curPC,
                                   newPC: newPC};
        end
        else if (pType == PRED_WRONG_BRANCH)
        begin
            //
            // Normal mispredicted branch instruction
            //
            return EXE_TO_FET_MSG {token: tok,
                                   updatePredictor: True,
                                   correctPrediction: False,
                                   instrPC: curPC,
                                   newPC: newPC};
        end
        else
        begin
            //
            // Normal correct prediction
            //
            return EXE_TO_FET_MSG {token: tok,
                                   updatePredictor: True,
                                   correctPrediction: True,
                                   instrPC: curPC,
                                   newPC: newPC};
        end

    endfunction
    

    function Action nonBranchInstr(TOKEN tok, Bool drainAfter, Bool pred_taken, ISA_ADDRESS nextSeqPC, ISA_ADDRESS curPC);
    action
        if (drainAfter)
        begin
            $fdisplay(debug_file, "[%d]:EXE: Emulation nonBranch resteer to 0x%h!", curTick, nextSeqPC);
            port_to_fet.send(tagged Valid msgToFetch(tok, PRED_DRAIN, nextSeqPC, curPC));
            epoch <= epoch + 1;
        end
        else if (pred_taken)
        begin
            $fdisplay(debug_file, "[%d]:EXE: Predicted taken on non-branch!  Resteer to 0x%h", curTick, nextSeqPC);
            stat_mpred.incr();
            port_to_fet.send(tagged Valid msgToFetch(tok, PRED_NONBRANCH, nextSeqPC, curPC));
            epoch <= epoch + 1;
        end
        else
        begin
            port_to_fet.send(tagged Invalid);
        end
    endaction
    endfunction


  rule executeReq (!in_flight);
  
    local_ctrl.startModelCC();

    counter <= counter + 1;
    $fdisplay(debug_file, "[%d]:Exe Counter : %0d", curTick, counter);
    
    
    let mtup <- port_from_dec.receive();
    
    case (mtup) matches
      tagged Invalid:
      begin
            port_to_mem.send(tagged Invalid);
            event_exe.recordEvent(tagged Invalid);
            port_to_fet.send(tagged Invalid);
      end
      tagged Valid {.tok, .branchPredAddr, .isLoad, .isStore, .drainAfter}:
      begin
            if (tok.timep_info.epoch != epoch) //kill it
            begin
              event_exe.recordEvent(tagged Invalid);
              port_to_mem.send(tagged Invalid);
              port_to_fet.send(tagged Invalid);
            end
            else //continue to execute it
            begin
              $fdisplay(debug_file, "[%d]:EXE:REQ: %0d", curTick, tok.index);
              fp_exe.makeReq(initFuncpReqGetResults(tok));
              addrQ.enq(tuple4(branchPredAddr, isLoad, isStore, drainAfter));
              in_flight <= True;
        end
      end
    endcase
  
  endrule

  rule executeResp (in_flight);
  
    let exe_resp = fp_exe.getResp();
    fp_exe.deq();
    
    let tok = exe_resp.token;
    let cur_pc = exe_resp.instructionAddress;
    let next_seq_pc =  cur_pc + zeroExtend(exe_resp.instructionSize);

    $fdisplay(debug_file, "[%d]:EXE:RSP: %0d", curTick, tok.index);
    
    Bool pred_taken = unpack(tok.timep_info.scratchpad[0]);

    match {.predAddr, .isLoad, .isStore, .drainAfter} = addrQ.first();
    addrQ.deq();
    
    $fdisplay(debug_file, "[%d]:Exe Counter when sent : %0d", curTick, counter);
    
    Bool mispredict = False;
    
    case (exe_resp.result) matches
      tagged RBranchTaken .addr:
          begin
            $fdisplay(debug_file, "[%d]:EXE: Branch taken", curTick);
            if (predAddr != addr)
            begin
              $fdisplay(debug_file, "[%d]:EXE: Branch mispredicted!", curTick);
              stat_mpred.incr();
              epoch <= epoch + 1;
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_WRONG_BRANCH, addr, cur_pc));
            end
            else if (drainAfter)
            begin
              $fdisplay(debug_file, "[%d]:EXE: Emulation resteer to 0x%h!", curTick, predAddr);
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_DRAIN, next_seq_pc, cur_pc));
              epoch <= epoch + 1;
            end
            else
            begin
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_CORRECT, addr, cur_pc));
            end
          end
      tagged RBranchNotTaken .addr:
          begin
          
            $fdisplay(debug_file, "[%d]:EXE: Branch not taken", curTick);
            if (pred_taken)
            begin
              $fdisplay(debug_file, "[%d]:EXE: Branch mispredicted!", curTick);
              stat_mpred.incr();
              epoch <= epoch + 1;
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_WRONG_BRANCH, addr, cur_pc));
            end
            else if (drainAfter)
            begin
              $fdisplay(debug_file, "[%d]:EXE: Emulation resteer to 0x%h!", curTick, predAddr);
              epoch <= epoch + 1;
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_DRAIN, next_seq_pc, cur_pc));
            end
            else
            begin
              port_to_fet.send(tagged Valid msgToFetch(tok, PRED_CORRECT, addr, cur_pc));
            end
          end
      tagged RNop:
      begin
          nonBranchInstr(tok, drainAfter, pred_taken, next_seq_pc, predAddr);
      end
      tagged REffectiveAddr .ea:
      begin
          nonBranchInstr(tok, drainAfter, pred_taken, next_seq_pc, predAddr);
      end
      tagged RTerminate .pf:
      begin
            if (drainAfter)
            begin
                $fdisplay(debug_file, "[%d]:EXE: Emulation resteer to 0x%h!", curTick, predAddr);
                port_to_fet.send(tagged Valid msgToFetch(tok, PRED_DRAIN, next_seq_pc, cur_pc));
                epoch <= epoch + 1;
            end
            else
            begin
                port_to_fet.send(tagged Invalid);
            end
            $fdisplay(debug_file, "[%d]:EXE: Setting Termination!", curTick);
            tok.timep_info.scratchpad[1] = 1; //[1] is termination
            tok.timep_info.scratchpad[2] = pack(pf); //[2] is passfail
      end
    endcase
    port_to_mem.send(tagged Valid tuple3(tok, isLoad, isStore));
    event_exe.recordEvent(tagged Valid zeroExtend(tok.index));
    in_flight <= False;
    
  endrule


endmodule
