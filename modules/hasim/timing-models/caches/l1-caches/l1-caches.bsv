`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_itlb.bsh"
`include "asim/provides/hasim_icache.bsh"
`include "asim/provides/hasim_dcache.bsh"

module [HASIM_MODULE] mkL1Cache();
    let itlb   <- mkITLB();
    let icache <- mkICache();
    let dcache <- mkDCache();
endmodule
