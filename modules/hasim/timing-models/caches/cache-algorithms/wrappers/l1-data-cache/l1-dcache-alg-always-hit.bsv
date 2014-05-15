
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/mem_services.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"


// mkL1DCacheAlg

// Always-Hit L1DCache algorithm.

// Instantiate an always-hit algorithm.

// Although the always-hit algorithm does not actually use indexing,
// other algorithms do, so we still need a dummy index size.

module [HASIM_MODULE] mkL1DCacheAlg
    // interface:
        (CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8));

    CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE) alg <- mkCacheAlgAlwaysHit();

    return alg;
        
endmodule

