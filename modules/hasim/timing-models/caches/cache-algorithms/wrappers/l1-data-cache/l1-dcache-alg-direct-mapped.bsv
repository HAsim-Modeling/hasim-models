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

// mkL1DCacheAlg

// Direct-Mapped L1DCache algorithm.

// Instantiate a direct-mapped algorithm using our specific scratchpad name.

// The direct-mapped algorithm will use the indexing size set by our parameter.

typedef `L1_DCACHE_ALG_INDEX_SIZE IDX_SIZE;

typedef CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, IDX_SIZE, 1)
    L1_DCACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY_IDX#(IDX_SIZE, 1)
    L1_DCACHE_IDX;

typedef CACHE_LOOKUP_RSP#(t_OPAQUE, IDX_SIZE, 1)
    L1_DCACHE_LOOKUP_RSP#(type t_OPAQUE);

module [HASIM_MODULE] mkL1DCacheAlg#(function Bool mayEvict(t_OPAQUE opaque))
    // interface:
    (L1_DCACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ));

    let _alg <- mkCacheAlgDirectMapped(mayEvict,
                                       `VDEV_SCRATCH_HASIM_L1_DCACHE_ALG_SCRATCHPAD,
                                       `L1_DCACHE_ALG_TAGS_USE_SCRATCHPAD != 0);
    return _alg;
endmodule
