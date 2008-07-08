import FShow::*;

interface DebugFile;
    method Action _write (Fmt fmt);
endinterface

module mkDebugFile#(String fname)
    (DebugFile);

    Reg#(File) fileHandle <- mkReg(InvalidFile);

    rule open (fileHandle == InvalidFile);
        let fh <- $fopen(fname, "w");
        fileHandle <= fh;
    endrule

    method Action _write (Fmt fmt) if (fileHandle != InvalidFile);
        $fwrite(fileHandle, fmt + fshow("\n"));
    endmethod
endmodule
