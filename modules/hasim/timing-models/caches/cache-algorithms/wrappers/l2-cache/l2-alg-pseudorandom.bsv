
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

`include "asim/dict/PARAMS_HASIM_L2_CACHE_ALG.bsh"

// mkL2CacheAlg

// Pseudo-random L2 cache algorithm.

// Instantiate a pseudo-random algorithm using our dynamic parameters.
// Instantiate a cache controller to interact with it.

// Although the pseudo-random algorithm does not actually use tagging,
// the entries returned to the controller must have some tag data.
// We arbitrarily choose 7 bits for this tag size.

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L2_CACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY#(void, 0, 1)
    L2_CACHE_ENTRY#(type t_OPAQUE);

module [HASIM_MODULE] mkL2CacheAlg
    // interface:
    (L2_CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) loadSeedParam     <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_LOAD_SEED, paramNode);
    Param#(8) storeSeedParam    <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_STORE_SEED, paramNode);
    Param#(8) evictionSeedParam <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_EVICT_SEED, paramNode);
    
    Param#(8) loadMissChanceParam   <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_LOAD_MISS_CHANCE, paramNode);
    Param#(8) storeMissChanceParam  <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_STORE_MISS_CHANCE, paramNode);
    Param#(8) cleanEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_CLEAN_EVICT_CHANCE, paramNode);
    Param#(8) dirtyEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L2_CACHE_ALG_L2_DIRTY_EVICT_CHANCE, paramNode);

    let _alg <- mkCacheAlgPseudoRandom
    (
        loadSeedParam,
        storeSeedParam,
        evictionSeedParam,
        loadMissChanceParam,
        storeMissChanceParam,
        cleanEvictChanceParam,
        dirtyEvictChanceParam
    );

    return _alg;
endmodule
