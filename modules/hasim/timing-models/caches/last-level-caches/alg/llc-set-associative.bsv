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
         Add#(IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc_alg.out");

    // NUM_WAYS is passed as a pseudo-numeric parameter.
    NumTypeParam#(`LLC_ALG_NUM_WAYS) numWays = ?;

    CACHE_ALG_INDEXED#(NUM_CPUS, t_OPAQUE, IDX_SIZE) alg <- mkCacheAlgSetAssociative(debugLog,
                                                                                     `VDEV_SCRATCH_HASIM_LAST_LEVEL_CACHE_ALG_SCRATCHPAD,
                                                                                     `LLC_ALG_TAGS_USE_SCRATCHPAD != 0,
                                                                                     numWays);

    return toCacheAlg(alg);
        
endmodule

