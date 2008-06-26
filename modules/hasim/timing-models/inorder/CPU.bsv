import hasim_common::*;

module [HASIM_MODULE] mkCPU ();
    let fetch   <- mkFetch();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let mem     <- mkMem();
    let wb      <- mkWriteBack();
endmodule

