
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

module [HASIM_MODULE] mkL1DCacheAlg
    // interface:
        (CACHE_ALG#(NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8));


    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) loadSeedParam     <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_LOAD_SEED, paramNode);
    Param#(8) storeSeedParam    <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_STORE_SEED, paramNode);
    Param#(8) evictionSeedParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_EVICT_SEED, paramNode);
    
    Param#(8) loadMissChanceParam   <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_LOAD_MISS_CHANCE, paramNode);
    Param#(8) storeMissChanceParam  <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_STORE_MISS_CHANCE, paramNode);
    Param#(8) cleanEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_CLEAN_EVICT_CHANCE, paramNode);
    Param#(8) dirtyEvictChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L1_DCACHE_ALG_DIRTY_EVICT_CHANCE, paramNode);

    CACHE_ALG#(NUM_CPUS, t_OPAQUE) alg <- mkCacheAlgPseudoRandom
    (
        loadSeedParam,
        storeSeedParam,
        evictionSeedParam,
        loadMissChanceParam,
        storeMissChanceParam,
        cleanEvictChanceParam,
        dirtyEvictChanceParam
    );

    return alg;
        
endmodule

