
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

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L1_DCACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY#(void, 0, 1)
    L1_DCACHE_ENTRY#(type t_OPAQUE);

module [HASIM_MODULE] mkL1DCacheAlg
    // interface:
    (L1_DCACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    let _alg <- mkCacheAlgAlwaysHit();
    return _alg;
endmodule

