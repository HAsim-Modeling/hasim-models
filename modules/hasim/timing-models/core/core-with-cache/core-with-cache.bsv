`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_pipeline.bsh"
`include "asim/provides/hasim_l1_caches.bsh"

module [HASIM_MODULE] mkCore();
    let pipeline <- mkPipeline;
    let l1Caches <- mkL1Cache;
endmodule
