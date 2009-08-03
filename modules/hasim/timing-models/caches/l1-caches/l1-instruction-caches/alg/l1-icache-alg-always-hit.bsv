
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/platform_interface.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_memory.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"


// mkL1ICacheAlg

// Always-Hit L1ICache algorithm.

// Instantiate an always-hit algorithm. This module is brain-dead simple.

module [HASIM_MODULE] mkL1ICacheAlg
    // interface:
        (CACHE_ALG#(NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8));

    CACHE_ALG#(NUM_CPUS, t_OPAQUE) alg <- mkCacheAlgAlwaysHit();

    return alg;
        
endmodule

