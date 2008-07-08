import FShow::*;

interface DebugFile;
    method Action startModelCC ();
    method Action _write (Fmt fmt);
endinterface

module mkDebugFile#(String fname)
        (DebugFile);

    Reg#(Bit#(32))  cycle      <- mkReg(0);
    PulseWire       incr_w     <- mkPulseWire;

    Reg#(File)      fileHandle <- mkReg(InvalidFile);

    rule open (fileHandle == InvalidFile);
        let fh <- $fopen(fname, "w");
        fileHandle <= fh;
    endrule
    method Action startModelCC();
        incr_w.send();
        cycle <= cycle + 1;
    endmethod
    method Action _write (Fmt fmt) if (fileHandle != InvalidFile);
        let cyc = incr_w ? cycle + 1 : cycle;
        $fwrite(fileHandle, $format("[%d]: ", cyc) + fmt + fshow("\n"));
    endmethod
endmodule
