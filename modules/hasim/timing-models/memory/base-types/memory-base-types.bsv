// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"

typedef 5 MEM_WORD_OFFSET_SIZE;
typedef Bit#(MEM_WORD_OFFSET_SIZE) MEM_WORD_OFFSET;
typedef TSub#(MEM_ADDRESS_SIZE, MEM_WORD_OFFSET_SIZE) LINE_ADDRESS_SIZE;
typedef Bit#(LINE_ADDRESS_SIZE) LINE_ADDRESS;

function LINE_ADDRESS toLineAddress(MEM_ADDRESS mem_addr);
    return truncateLSB(mem_addr);
endfunction

function MEM_ADDRESS fromLineAddress(LINE_ADDRESS line_addr);
    return {line_addr, 0};
endfunction

typedef 8 MEM_OPAQUE_SIZE;
typedef Bit#(MEM_OPAQUE_SIZE) MEM_OPAQUE;

typedef struct
{
    LINE_ADDRESS physicalAddress;
    Bool isStore;
    MEM_OPAQUE opaque;
}
CORE_MEMORY_REQ deriving (Eq, Bits);

function CORE_MEMORY_REQ initMemLoad(LINE_ADDRESS addr);

    return CORE_MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: False,
        opaque: 0
    };

endfunction

function CORE_MEMORY_REQ initMemStore(LINE_ADDRESS addr);

    return CORE_MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: True,
        opaque: 0
    };

endfunction

function CORE_MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);

    return CORE_MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };

endfunction

typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
}
CORE_MEMORY_RSP deriving (Eq, Bits);

typedef CORE_MEMORY_REQ MEMORY_REQ;
typedef CORE_MEMORY_RSP MEMORY_RSP;

typedef CORE_MEMORY_REQ CORE_IC_REQ;
typedef CORE_MEMORY_RSP CORE_IC_RSP;


function MEM_OPAQUE toMemOpaque(t_ANY x)
    provisos
        (Bits#(t_ANY, t_SZ),
         Add#(t_SZ, t_TMP, MEM_OPAQUE_SIZE));
    
    return zeroExtend(pack(x));

endfunction

function t_ANY fromMemOpaque(MEM_OPAQUE x)
    provisos
        (Bits#(t_ANY, t_SZ),
         Add#(t_SZ, t_TMP, MEM_OPAQUE_SIZE));
        
    return unpack(truncate(x));

endfunction
