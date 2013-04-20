`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_pipeline.bsh"
`include "asim/provides/hasim_private_caches.bsh"

module [HASIM_MODULE] mkCore();
    let pipeline <- mkPipeline;
    let l1Caches <- mkPrivateCaches;
endmodule
