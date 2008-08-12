`include "hasim_common.bsh"

module [HASIM_MODULE] mkPipeline ();
    let fetch   <- mkFetch();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let mem     <- mkMem();
    let wb      <- mkWriteBack();

    let bp     <- mkBranchPredictor();
    let sb     <- mkStoreBuffer();
endmodule

