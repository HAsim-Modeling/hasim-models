`include "asim/provides/funcp_base_types.bsh"

typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE, TAdd#(`DCACHE_LINE_BITS, `DCACHE_IDX_BITS))) DCACHE_TAG;
typedef Bit#(`DCACHE_LINE_BITS) DCACHE_LINE_OFFSET;
typedef Bit#(`DCACHE_IDX_BITS) DCACHE_INDEX;
typedef Bit#(TLog#(`DCACHE_ASSOC)) DCACHE_WAY;
