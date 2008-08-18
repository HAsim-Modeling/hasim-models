`include "hasim_common.bsh"

module [HASIM_MODULE] mkPipeline();
    let  fetch <- mkFetch;
    let decode <- mkDecode;
    let    rob <- mkRob;
    let    alu <- mkAlu;
    let   mem1 <- mkMemAddress;
    let  merge <- mkGetResults;
    let   mem2 <- mkMem;
    let commit <- mkCommit;
endmodule

