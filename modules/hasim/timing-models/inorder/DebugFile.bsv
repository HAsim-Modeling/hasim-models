import FShow::*;

interface DebugFile;
    method Action startModelCC ();
    method Action _write (Fmt fmt);
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
        $fwrite(fh, $format("[%d]: ", cyc) + fmt + fshow("\n"));
    endmethod
endmodule
