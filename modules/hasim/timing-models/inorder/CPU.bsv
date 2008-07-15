import hasim_common::*;
import hasim_icache::*;

module [HASIM_MODULE] mkCPU ();
    let fetch   <- mkFetch();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let mem     <- mkMem();
    let wb      <- mkWriteBack();

    let bp <- mkBranchPredictor();
    let icache <- mkICache();
endmodule

