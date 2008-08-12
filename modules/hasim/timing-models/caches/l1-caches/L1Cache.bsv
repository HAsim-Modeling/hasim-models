`include "hasim_common.bsh"
`include "hasim_icache.bsh"
`include "hasim_dcache.bsh"

module [HASIM_MODULE] mkL1Cache();
    let icache <- mkICache;
    let dcache <- mkDCache;
endmodule
