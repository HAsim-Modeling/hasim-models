
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/platform_interface.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_memory.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"

// ******* Generated File Imports *******

`include "asim/dict/VDEV_SCRATCH.bsh"

// mkL1DCacheAlg

// Direct-Mapped L1DCache algorithm.

// Instantiate a direct-mapped algorithm using our specific scratchpad name.

// The direct-mapped algorithm will use the indexing size set by our parameter.

typedef `L1_DCACHE_ALG_INDEX_SIZE IDX_SIZE;

module [HASIM_MODULE] mkL1DCacheAlg
    // interface:
        (CACHE_ALG#(NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8),
         Add#(IDX_SIZE, t_TMP2, LINE_ADDRESS_SIZE),
         // The following is brought to you courtesy of proviso hell:
         Add#(t_TMP3, TAdd#(TSub#(TAdd#(1, IDX_SIZE), TLog#(TDiv#(64,
         TExp#(TLog#(TAdd#(1, TAdd#(1, TAdd#(t_OPAQUE_SZ, t_TMP2)))))))),
         TLog#(TDiv#(TExp#(TLog#(TAdd#(1, TAdd#(1, TAdd#(t_OPAQUE_SZ, t_TMP2))))),
         64))), 32));

    CACHE_ALG_INDEXED#(NUM_CPUS, t_OPAQUE, IDX_SIZE) alg <- mkCacheAlgDirectMapped(`VDEV_SCRATCH_HASIM_L1_DCACHE_ALG_SCRATCHPAD);

    return toCacheAlg(alg);
        
endmodule
