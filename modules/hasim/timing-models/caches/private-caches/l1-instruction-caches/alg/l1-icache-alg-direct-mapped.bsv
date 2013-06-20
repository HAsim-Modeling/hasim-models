
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

// mkL1ICacheAlg

// Direct-Mapped L1ICache algorithm.

// Instantiate a direct-mapped algorithm using our specific scratchpad name.

// The direct-mapped algorithm will use our index size parameter.

typedef `L1_ICACHE_ALG_INDEX_SIZE IDX_SIZE;

module [HASIM_MODULE] mkL1ICacheAlg
    // interface:
        (CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8),
         Add#(IDX_SIZE, t_TMP2, LINE_ADDRESS_SIZE),
         // The following is brought to you courtesy of proviso hell:
         Add#(t_TMP3, TAdd#(TSub#(TAdd#(TLog#(MAX_NUM_CPUS), IDX_SIZE), TLog#(TDiv#(64,
         TExp#(TLog#(TAdd#(1, TAdd#(1, TAdd#(t_OPAQUE_SZ, t_TMP2)))))))),
         TLog#(TDiv#(TExp#(TLog#(TAdd#(1, TAdd#(1, TAdd#(t_OPAQUE_SZ, t_TMP2))))),
         64))), 32));

    CACHE_ALG_INDEXED#(MAX_NUM_CPUS, t_OPAQUE, IDX_SIZE) alg <- mkCacheAlgDirectMapped(`VDEV_SCRATCH_HASIM_L1_ICACHE_ALG_SCRATCHPAD,
                                                                                   `L1_ICACHE_ALG_TAGS_USE_SCRATCHPAD != 0);

    return toCacheAlg(alg);
        
endmodule
