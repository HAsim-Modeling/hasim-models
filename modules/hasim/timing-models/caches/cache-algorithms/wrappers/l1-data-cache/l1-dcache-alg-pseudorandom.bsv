
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/platform_services.bsh"
`include "asim/provides/common_services.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"

// ******* Generated File Imports *******

`include "asim/dict/PARAMS_HASIM_L1_DCACHE_ALG.bsh"

// mkL1DCache

// Pseudo-random L1DCache.

// Instantiate a pseudo-random algorithm using our dynamic parameters.
// Instantiate a cache controller to interact with it.

// Although the pseudo-random algorithm does not actually use tagging,
// the entries returned to the controller must have some tag data.
// We arbitrarily choose 7 bits for this tag size.

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L1_DCACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY_IDX#(0, 1)
    L1_DCACHE_IDX;

typedef CACHE_LOOKUP_RSP#(void, 0, 1)
    L1_DCACHE_LOOKUP_RSP#(type t_OPAQUE);

module [HASIM_MODULE] mkL1DCacheAlg#(function Bool mayEvict(void opaque))
    // interface:
    (L1_DCACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) seedParam     <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_L1D_SEED, paramNode);
    Param#(8) missChanceParam   <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_L1D_MISS_CHANCE, paramNode);
    Param#(8) cleanEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_L1D_CLEAN_EVICT_CHANCE, paramNode);
    Param#(8) dirtyEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_L1D_DIRTY_EVICT_CHANCE, paramNode);

    let _alg <- mkCacheAlgPseudoRandom
    (
        mayEvict,
        seedParam,
        missChanceParam,
        cleanEvictChanceParam,
        dirtyEvictChanceParam
    );

    return _alg;
endmodule

