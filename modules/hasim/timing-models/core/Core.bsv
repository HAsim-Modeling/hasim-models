`include "hasim_common.bsh"
`include "hasim_pipeline.bsh"
`include "hasim_l1_caches.bsh"

module [HASIM_MODULE] mkCore();
    let pipeline <- mkPipeline;
    let l1Caches <- mkL1Cache;
endmodule
