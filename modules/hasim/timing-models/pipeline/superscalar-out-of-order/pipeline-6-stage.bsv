`include "hasim_common.bsh"

`include "hasim_fetch.bsh"
`include "hasim_decode.bsh"
`include "hasim_issue.bsh"
`include "hasim_alu.bsh"
`include "hasim_mem.bsh"
`include "hasim_commit.bsh"

module [HASIM_MODULE] mkPipeline();
    let  fetch <- mkFetch;
    let decode <- mkDecode;
    let  issue <- mkIssue;
    let    alu <- mkAlu;
    let   mem1 <- mkMemAddress;
    let  merge <- mkGetResults;
    let   mem2 <- mkMem;
    let commit <- mkCommit;
endmodule

