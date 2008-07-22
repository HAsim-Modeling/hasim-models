`ifndef DEBUG_FILE_BSV
`define DEBUG_FILE_BSV

import FShow::*;

interface DebugFile;
    method Action startModelCC ();
    method Action _write (Fmt fmt);
    method Action endModelCC();
endinterface

module mkDebugFile#(String fname)
        (DebugFile);

    Reg#(Bit#(32))  cycle      <- mkReg(0);
    PulseWire       incr_w     <- mkPulseWire;

    Reg#(Maybe#(File)) fileHandle <- mkReg(Invalid);

    rule open (fileHandle == Invalid);
        let fh <- $fopen(fname, "w");
        fileHandle <= Valid(fh);
    endrule

    method Action startModelCC();
        incr_w.send();
        cycle <= cycle + 1;
    endmethod

    method Action _write (Fmt fmt) if (fileHandle matches tagged Valid .fh);
        let cyc = incr_w ? cycle + 1 : cycle;
        $fdisplay(fh, $format("[%d]: ", cyc) + fmt);
    endmethod

    method Action endModelCC();
        cycle <= cycle + 1;
    endmethod
endmodule

`endif
