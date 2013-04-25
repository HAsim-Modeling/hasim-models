`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_l1_icache.bsh"
`include "asim/provides/hasim_l1_dcache.bsh"
`include "asim/provides/hasim_l1_arbiter.bsh"

module [HASIM_MODULE] mkPrivateCaches();
    let icache  <- mkL1ICache();
    let dcache  <- mkL1DCache();
    let arbiter <- mkL1CacheArbiter("CorePvtCache_to_UncoreQ",
                                    "Uncore_to_CorePvtCacheQ");
endmodule
