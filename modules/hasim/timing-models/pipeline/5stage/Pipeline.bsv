
import hasim_common::*;
import hasim_isa::*;


module [HASIM_MODULE] mkPipeline
    //interface:
                ();
   
  let debug_file <- mkReg(InvalidFile);
  Reg#(Bit#(32)) curTick <- mkReg(0);

  Reg#(Bool) ran <- mkReg(False);

  let fet <- mkPipe_Fetch(debug_file, curTick);
  let dec <- mkPipe_Decode(debug_file, curTick);
  let exe <- mkPipe_Execute(debug_file, curTick);
  let mem <- mkPipe_Mem(debug_file, curTick);
  let wb  <- mkPipe_Writeback(debug_file, curTick);

  rule openFile (debug_file == InvalidFile);
  
    let fd <- $fopen("hasim_cpu.out", "w");
    
    if (fd == InvalidFile)
    begin
      $display("CPU: ERROR: Could not open file hasim_cpu.out");
      $finish(1);
    end
    
    debug_file <= fd;
  
  endrule

  rule countCC (True);
  
    curTick <= curTick + 1;
  
  endrule

endmodule
