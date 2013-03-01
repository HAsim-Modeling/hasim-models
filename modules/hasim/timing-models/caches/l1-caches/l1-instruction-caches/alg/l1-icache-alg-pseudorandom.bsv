
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

`include "asim/dict/PARAMS_HASIM_L1_ICACHE_ALG.bsh"

// mkL1ICacheAlg

// Pseudo-random L1ICache algorithm.

// Instantiate a pseudo-random algorithm using our dynamic parameters.

module [HASIM_MODULE] mkL1ICacheAlg
    // interface:
        (CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8));


    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) seedParam     <- mkDynamicParameter(`PARAMS_HASIM_L1_ICACHE_ALG_SEED, paramNode);
    
    Param#(8) missChanceParam   <- mkDynamicParameter(`PARAMS_HASIM_L1_ICACHE_ALG_MISS_CHANCE, paramNode);

    // All unused parameters are set to zero.
    CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE) alg <- mkCacheAlgPseudoRandom
    (
        seedParam,
        0,
        0,
        missChanceParam,
        0,
        0,
        0
    );

    return alg;
        
endmodule

