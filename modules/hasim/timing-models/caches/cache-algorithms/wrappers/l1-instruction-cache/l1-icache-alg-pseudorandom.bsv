
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

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L1_ICACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY_IDX#(0, 1)
    L1_ICACHE_IDX;

typedef CACHE_LOOKUP_RSP#(void, 0, 1)
    L1_ICACHE_LOOKUP_RSP#(type t_OPAQUE);

module [HASIM_MODULE] mkL1ICacheAlg#(function Bool mayEvict(void opaque))
    // interface:
    (L1_ICACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    // ****** Dynamic Parameters ******

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();
    Param#(8) seedParam <- mkDynamicParameter(`PARAMS_HASIM_L1_ICACHE_ALG_L1I_SEED, paramNode);
    Param#(8) missChanceParam <- mkDynamicParameter(`PARAMS_HASIM_L1_ICACHE_ALG_L1I_MISS_CHANCE, paramNode);

    // All unused parameters are set to zero.
    let _alg <- mkCacheAlgPseudoRandom
    (
        mayEvict,
        seedParam,
        missChanceParam,
        0,
        0
    );

    return _alg;
endmodule

