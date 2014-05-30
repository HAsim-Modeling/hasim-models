
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/soft_connections.bsh"
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


// mkL1ICacheAlg

// Always-Hit L1ICache algorithm.

// Instantiate an always-hit algorithm. This module is brain-dead simple.

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L1_ICACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY_IDX#(0, 1)
    L1_ICACHE_IDX;

typedef CACHE_LOOKUP_RSP#(void, 0, 1)
    L1_ICACHE_LOOKUP_RSP#(type t_OPAQUE);

module [HASIM_MODULE] mkL1ICacheAlg#(function Bool mayEvict(void opaque))
    // interface:
    (L1_ICACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    let _alg <- mkCacheAlgAlwaysHit(mayEvict);
    return _alg;
endmodule

