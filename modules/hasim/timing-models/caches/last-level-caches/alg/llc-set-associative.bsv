// ******* Application Imports *******

`include "asim/provides/soft_connections.bsh"
`include "asim/provides/common_services.bsh"

// ******* HAsim Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/hasim_modellib.bsh"

`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"

// ******* Timing Model Imports *******

`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"

// ******* Generated File Imports *******

`include "asim/dict/VDEV_SCRATCH.bsh"

// mkLastLevelCacheAlg

// Set-Associative Last Level Cache algorithm.

// Instantiate a set-associative algorithm using our specific scratchpad name.

// The set-associative algorithm will use the indexing size and number of ways 
// set by our parameter.

typedef `LLC_ALG_INDEX_SIZE IDX_SIZE;

module [HASIM_MODULE] mkLastLevelCacheAlg
    // interface:
        (CACHE_ALG#(NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8),
         Add#(IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         // The following is brought to you courtesy of proviso hell:
         Add#(t_TMP3, TAdd#(TSub#(TAdd#(TLog#(NUM_CPUS), IDX_SIZE), 
         TLog#(TDiv#(64, TExp#(TLog#(TAdd#(1,
         TAdd#(t_OPAQUE_SZ, t_TAG_SIZE))))))), TLog#(TDiv#(TExp#(TLog#(TAdd#(1,
         TAdd#(t_OPAQUE_SZ, t_TAG_SIZE)))), 64))), 32));

    // NUM_WAYS is passed as a pseudo-numeric parameter.
    NumTypeParam#(`LLC_ALG_NUM_WAYS) numWays = ?;

    CACHE_ALG_INDEXED#(NUM_CPUS, t_OPAQUE, IDX_SIZE) alg <- mkCacheAlgSetAssociative(`VDEV_SCRATCH_HASIM_LAST_LEVEL_CACHE_ALG_SCRATCHPAD, numWays);

    return toCacheAlg(alg);
        
endmodule

