
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

typedef CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, IDX_SIZE, 1)
    L1_ICACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY#(void, IDX_SIZE, 1)
    L1_ICACHE_ENTRY#(type t_OPAQUE);

module [HASIM_MODULE] mkL1ICacheAlg
    // interface:
    (L1_ICACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ));

    let _alg <- mkCacheAlgDirectMapped(`VDEV_SCRATCH_HASIM_L1_ICACHE_ALG_SCRATCHPAD,
                                       `L1_ICACHE_ALG_TAGS_USE_SCRATCHPAD != 0);
    return _alg;
endmodule

