import LFSR::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/module_local_controller.bsh"

`include "asim/dict/EVENTS_MEMORY.bsh"

//AWB Parameters          default:
//MEM_DCACHE_HIT_CHANCE     50
//MEM_DCACHE_MISS_PENALTY   10


typedef enum 
{
  MEM_Ready,
  MEM_Finish_Load,
  MEM_Finish_Store
}
  MEM_STATE
    deriving (Eq, Bits);

Integer mem_hit_chance = (`MEM_DCACHE_HIT_CHANCE * 127) / 100;

module [HASIM_MODULE] mkPipe_Mem#(File debug_file, Bit#(32) curTick)
    //interface:
                ();
  
  //Local State
  Reg#(TOKEN)     stall_tok   <- mkRegU;
  Reg#(Bit#(16))  stall_count <- mkReg(0);
  Reg#(Bool)      stalling    <- mkReg(False);
  Reg#(MEM_STATE) state       <- mkReg(MEM_Ready);
  
  //Pseudo-randomness
  LFSR#(Bit#(7)) lfsr <- mkFeedLFSR(7'b1001110);

  //Connections to FP
  Connection_Send#(TOKEN)     fp_loads_req  <- mkConnection_Send("funcp_doLoads_req");
  Connection_Receive#(TOKEN)  fp_loads_resp <- mkConnection_Receive("funcp_doLoads_resp");
  Connection_Send#(TOKEN)     fp_stores_req  <- mkConnection_Send("funcp_doSpeculativeStores_req");
  Connection_Receive#(TOKEN)  fp_stores_resp <- mkConnection_Receive("funcp_doSpeculativeStores_resp");

  //Events
  EventRecorder event_mem <- mkEventRecorder(`EVENTS_MEMORY_INSTRUCTION_MEM);
    
  //Incoming Ports
  Port_Receive#(Tuple3#(TOKEN, Bool, Bool)) port_from_exe <- mkPort_Receive("exe_to_mem", 1);

  //Outgoing Ports
  Port_Send#(Tuple2#(TOKEN, Bool)) port_to_wb <- mkPort_Send("mem_to_wb");

  //Local Controller
  Vector#(1, Port_Control) inports  = newVector();
  Vector#(1, Port_Control) outports = newVector();
  inports[0]  = port_from_exe.ctrl;
  outports[0] = port_to_wb.ctrl;
  LocalController local_ctrl <- mkLocalController(inports, outports);


  rule beginMem (state == MEM_Ready);
    
    local_ctrl.startModelCC();
     
    let mtok <- port_from_exe.receive();

    case (mtok) matches
      tagged Invalid:
      begin
        port_to_wb.send(tagged Invalid);
        event_mem.recordEvent(tagged Invalid);
      end
      tagged Valid {.tok, .isLoad, .isStore}:
      begin
        if (isLoad)
        begin
          $fdisplay(debug_file, "[%d]:REQ:LOA: %0d", curTick, tok.index);
          fp_loads_req.send(tok);
          state <= MEM_Finish_Load;
        end
        else if (isStore)
        begin
          $fdisplay(debug_file, "[%d]:REQ:STO: %0d", curTick, tok.index);
          fp_stores_req.send(tok);
          state <= MEM_Finish_Store;
        end
        else
        begin
          $fdisplay(debug_file, "[%d]:no mem: %0d", curTick, tok.index);
          port_to_wb.send(tagged Valid tuple2(tok, False));
          event_mem.recordEvent(tagged Invalid);
          state <= MEM_Ready;
        end
      end
    endcase

  endrule

  rule finishLoad (state == MEM_Finish_Load);
  
    let tok = fp_loads_resp.receive();
    fp_loads_resp.deq();
    
    $fdisplay(debug_file, "[%d]:RSP:LOA: %0d", curTick, tok.index);
    
    port_to_wb.send(tagged Valid tuple2(tok, False));
    event_mem.recordEvent(tagged Valid zeroExtend(tok.index));

    state <= MEM_Ready;
    
  endrule

  rule finishStore (state == MEM_Finish_Store);
  
    let tok = fp_stores_resp.receive();
    fp_stores_resp.deq();
    
    $fdisplay(debug_file, "[%d]:RSP:STO: %0d", curTick, tok.index);
    
    port_to_wb.send(tagged Valid tuple2(tok, True));
    event_mem.recordEvent(tagged Valid zeroExtend(tok.index));

    state <= MEM_Ready;
    
  endrule

endmodule
