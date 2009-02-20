`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_icache.bsh"
`include "asim/provides/hasim_dcache.bsh"

module [HASIM_MODULE] mkL1Cache();
    let icache <- mkICache;
    let dcache <- mkDCache;
endmodule
