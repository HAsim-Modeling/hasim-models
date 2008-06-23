import hasim_common::*;

import Fetch::*;
import Decode::*;
import Execute::*;
import Mem::*;
import WriteBack::*;

module [HASIM_MODULE] mkCPU ();
    let fetch   <- mkFetch();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let mem     <- mkMem();
    let wb      <- mkWriteBack();
endmodule

