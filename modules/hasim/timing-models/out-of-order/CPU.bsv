import hasim_common::*;

module [HASIM_MODULE] mkCPU ();
    let  fetch <- mkFetch();
    let decode <- mkDecode();
    let    rob <- mkRob();
    let    alu <- mkAlu();
    let   mem1 <- mkMemAddress();
    let  merge <- mkGetResults();
    let   mem2 <- mkMem();
    let commit <- mkCommit();
    /*
    let fetch   <- mkFetch();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let mem     <- mkMem();
    let wb      <- mkWriteBack();
    */
endmodule

