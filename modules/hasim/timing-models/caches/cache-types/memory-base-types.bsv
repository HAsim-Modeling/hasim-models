import FShow::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_base_types.bsh"


`define IMEM_ITLB_EPOCH_BITS 2
`define IMEM_ICACHE_EPOCH_BITS 2

typedef Bit#(`IMEM_ITLB_EPOCH_BITS) IMEM_ITLB_EPOCH;
typedef Bit#(`IMEM_ICACHE_EPOCH_BITS) IMEM_ICACHE_EPOCH;

typedef struct
{
    TOKEN token;
    IMEM_ITLB_EPOCH iTLBEpoch;
    IMEM_ICACHE_EPOCH iCacheEpoch;
    ISA_ADDRESS virtualAddress;
    ISA_ADDRESS physicalAddress;
    ISA_INSTRUCTION instruction;
}
IMEM_BUNDLE deriving (Eq, Bits);

function IMEM_BUNDLE initIMemBundle(TOKEN tok, IMEM_ITLB_EPOCH tlb_epoch, IMEM_ICACHE_EPOCH cache_epoch, ISA_ADDRESS pc);

    return IMEM_BUNDLE {token: tok, iTLBEpoch: tlb_epoch, iCacheEpoch: cache_epoch, virtualAddress: pc, instruction: ?, physicalAddress: ?};

endfunction

typedef IMEM_BUNDLE ITLB_INPUT;

typedef enum
{
    ITLB_hit,
    ITLB_pageFault
}
ITLB_RESPONSE_TYPE deriving (Eq, Bits);

typedef struct
{
    IMEM_BUNDLE bundle;
    ITLB_RESPONSE_TYPE rspType;
}
ITLB_OUTPUT deriving (Eq, Bits);

function ITLB_OUTPUT initITLBHit(IMEM_BUNDLE bundle, FUNCP_PADDR phys_addr);

    
    let out = ITLB_OUTPUT {bundle: bundle, rspType: tagged ITLB_hit};
    out.bundle.physicalAddress = zeroExtend(phys_addr);
    return out;

endfunction

typedef IMEM_BUNDLE ICACHE_INPUT;

function ICACHE_INPUT initICacheLoad(IMEM_BUNDLE bundle);

    return bundle;

endfunction

typedef enum
{
    ICACHE_hit,
    ICACHE_miss,
    ICACHE_retry
}
ICACHE_RESPONSE deriving (Eq, Bits);

typedef struct
{
    IMEM_BUNDLE bundle;
    ICACHE_RESPONSE rspType;
} 
ICACHE_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef IMEM_BUNDLE ICACHE_OUTPUT_DELAYED;

function ICACHE_OUTPUT_IMMEDIATE initICacheHit(IMEM_BUNDLE bundle, ISA_INSTRUCTION inst);

    bundle.instruction = inst;
    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: ICACHE_hit};

endfunction

function ICACHE_OUTPUT_IMMEDIATE initICacheMiss(IMEM_BUNDLE bundle);

    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: ICACHE_miss};

endfunction

function ICACHE_OUTPUT_IMMEDIATE initICacheRetry(IMEM_BUNDLE bundle);

    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: ICACHE_retry};

endfunction

function ICACHE_OUTPUT_DELAYED initICacheMissRsp(IMEM_BUNDLE bundle);

    return bundle;

endfunction


typedef struct
{
    TOKEN token;
    ISA_ADDRESS virtualAddress;
    ISA_ADDRESS physicalAddress;
    Bool isJunk;
    Bool isLoad;
    Bool isStore;
    Maybe#(Bool) isTerminate;
    ISA_INST_DSTS dests;
}
DMEM_BUNDLE deriving (Eq, Bits);

instance FShow#(DMEM_BUNDLE);
    function Fmt fshow(DMEM_BUNDLE x);
        Fmt s = fshow("DMEM_BUNDLE: va = ") + fshow(x.virtualAddress);
        s = s + fshow(" pa = ") + fshow(x.physicalAddress);
        if (x.isJunk)
            s = s + fshow(" JUNK");
        if (x.isLoad)
            s = s + fshow(" LOAD");
        if (x.isStore)
            s = s + fshow(" STORE");
        if (x.isTerminate matches tagged Valid .b)
            s = s + $format(" TERMINATE(%b)", b);
        return s;
    endfunction
endinstance

function DMEM_BUNDLE initDMemBundle(TOKEN tok, ISA_ADDRESS va, Bool isJ, Bool isL, Bool isS, Maybe#(Bool) isT, ISA_INST_DSTS dests);

    return DMEM_BUNDLE {token: tok, virtualAddress: va, physicalAddress: ?, isJunk: isJ, isLoad: isL, isStore: isS, isTerminate: isT, dests: dests};

endfunction

typedef DMEM_BUNDLE DTLB_INPUT;

typedef enum
{
    DTLB_hit,
    DTLB_pageFault
}
DTLB_RESPONSE_TYPE deriving (Eq, Bits);

typedef DMEM_BUNDLE DTLB_OUTPUT; // We poison the instruction on a TLB miss.

function DTLB_OUTPUT initDTLBHit(DMEM_BUNDLE bundle, FUNCP_PADDR phys_addr);

    
    let out = bundle;
    out.physicalAddress = zeroExtend(phys_addr);
    return out;

endfunction


function DTLB_OUTPUT initDTLBMiss(DMEM_BUNDLE bundle);

    
    let out = bundle;
    out.token.poison = True;

    return out;

endfunction



typedef DMEM_BUNDLE DCACHE_LOAD_INPUT;
typedef Tuple2#(TOKEN, ISA_ADDRESS) DCACHE_STORE_INPUT;

function DCACHE_LOAD_INPUT initDCacheLoad(DMEM_BUNDLE bundle);

    return bundle;

endfunction

typedef enum
{
    DCACHE_hit,
    DCACHE_miss,
    DCACHE_retry
}
DCACHE_LOAD_RESPONSE deriving (Eq, Bits);

typedef struct
{
    DMEM_BUNDLE bundle;
    DCACHE_LOAD_RESPONSE rspType;
} 
DCACHE_LOAD_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef DMEM_BUNDLE DCACHE_LOAD_OUTPUT_DELAYED;

function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadHit(DMEM_BUNDLE bundle);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: DCACHE_hit};

endfunction

function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadMiss(DMEM_BUNDLE bundle);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: DCACHE_miss};

endfunction

function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadRetry(DMEM_BUNDLE bundle);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: DCACHE_retry};

endfunction

function DCACHE_LOAD_OUTPUT_DELAYED initDCacheLoadMissRsp(DMEM_BUNDLE bundle);

    return bundle;

endfunction


function DCACHE_STORE_INPUT initDCacheStore(TOKEN tok, ISA_ADDRESS addr);

    return tuple2(tok, addr);

endfunction

typedef enum
{
    DCACHE_ok,
    DCACHE_delay,
    DCACHE_retryStore
}
DCACHE_STORE_RESPONSE deriving (Eq, Bits);

typedef struct
{
    TOKEN token;
    DCACHE_STORE_RESPONSE rspType;
} 
DCACHE_STORE_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef TOKEN DCACHE_STORE_OUTPUT_DELAYED;

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreOk(TOKEN tok);

    return DCACHE_STORE_OUTPUT_IMMEDIATE {token: tok, rspType: DCACHE_ok};

endfunction

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreDelay(TOKEN tok);

    return DCACHE_STORE_OUTPUT_IMMEDIATE {token: tok, rspType: DCACHE_delay};

endfunction

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreRetry(TOKEN tok);

    return DCACHE_STORE_OUTPUT_IMMEDIATE {token: tok, rspType: DCACHE_retryStore};

endfunction

function DCACHE_STORE_OUTPUT_DELAYED initDCacheStoreDelayOk(TOKEN tok);

    return tok;

endfunction

typedef enum
{
    SB_search,
    SB_complete
}
SB_REQUEST deriving (Eq, Bits);

typedef struct
{
    DMEM_BUNDLE bundle;
    SB_REQUEST reqType;
}
SB_INPUT deriving (Eq, Bits);

function SB_INPUT initSBSearch(DMEM_BUNDLE bundle);

    return SB_INPUT {bundle: bundle, reqType: tagged SB_search};
    
endfunction

function SB_INPUT initSBComplete(DMEM_BUNDLE bundle);

    return SB_INPUT {bundle: bundle, reqType: tagged SB_complete};
    
endfunction

typedef enum
{
    SB_writeback,
    SB_drop
}
SB_DEALLOC_REQUEST deriving (Eq, Bits);

typedef struct
{
    TOKEN token;
    SB_DEALLOC_REQUEST reqType;
}
SB_DEALLOC_INPUT deriving (Eq, Bits);

function SB_DEALLOC_INPUT initSBWriteback(TOKEN tok);

    return SB_DEALLOC_INPUT {token: tok, reqType: tagged SB_writeback};

endfunction

function SB_DEALLOC_INPUT initSBDrop(TOKEN tok);

    return SB_DEALLOC_INPUT {token: tok, reqType: tagged SB_drop};

endfunction

typedef enum
{
    WB_miss,
    WB_hit
}
WB_SEARCH_RESPONSE deriving (Eq, Bits);

typedef struct
{
    DMEM_BUNDLE bundle;
    WB_SEARCH_RESPONSE rspType;
}
WB_SEARCH_OUTPUT deriving (Eq, Bits);

function WB_SEARCH_OUTPUT initWBHit(DMEM_BUNDLE bundle);

    return WB_SEARCH_OUTPUT {bundle: bundle, rspType: tagged WB_hit};

endfunction

function WB_SEARCH_OUTPUT initWBMiss(DMEM_BUNDLE bundle);

    return WB_SEARCH_OUTPUT {bundle: bundle, rspType: tagged WB_miss};

endfunction


typedef DMEM_BUNDLE WB_SEARCH_INPUT;

function WB_SEARCH_INPUT initWBSearch(DMEM_BUNDLE bundle);

    return bundle;
    
endfunction

typedef enum
{
    SB_miss,
    SB_hit
}
SB_RESPONSE deriving (Eq, Bits);

typedef struct
{
    DMEM_BUNDLE bundle;
    SB_RESPONSE rspType;
}
SB_OUTPUT deriving (Eq, Bits);

function SB_OUTPUT initSBHit(DMEM_BUNDLE bundle);

    return SB_OUTPUT {bundle: bundle, rspType: tagged SB_hit};

endfunction

function SB_OUTPUT initSBMiss(DMEM_BUNDLE bundle);

    return SB_OUTPUT {bundle: bundle, rspType: tagged SB_miss};

endfunction

typedef Tuple2#(TOKEN, ISA_ADDRESS) WB_ENTRY;
