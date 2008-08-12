`include "hasim_common.bsh"
`include "hasim_core.bsh"
`include "hasim_shared_cache.bsh"

module [HASIM_MODULE] mkChip();
    let core <- mkCore;
    let sharedCache <- mkSharedCache;
endmodule
