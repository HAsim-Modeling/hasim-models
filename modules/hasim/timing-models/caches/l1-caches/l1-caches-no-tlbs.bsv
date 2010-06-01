`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_itlb.bsh"
`include "asim/provides/hasim_dtlb.bsh"
`include "asim/provides/hasim_l1_icache.bsh"
`include "asim/provides/hasim_l1_dcache.bsh"
`include "asim/provides/hasim_l1_arbiter.bsh"

module [HASIM_MODULE] mkL1Cache();
    let icache  <- mkL1ICache();
    let dcache  <- mkL1DCache();
    let arbiter <- mkL1CacheArbiter();
endmodule
