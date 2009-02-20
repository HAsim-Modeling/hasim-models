`include "asim/provides/funcp_base_types.bsh"

typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE, TAdd#(`ICACHE_LINE_BITS, `ICACHE_IDX_BITS))) ICACHE_TAG;
typedef Bit#(`ICACHE_LINE_BITS) ICACHE_LINE_OFFSET;
typedef Bit#(`ICACHE_IDX_BITS) ICACHE_INDEX;
typedef Bit#(TLog#(`ICACHE_ASSOC)) ICACHE_WAY;
