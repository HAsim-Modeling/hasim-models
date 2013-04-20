import Vector::*;
import FShow::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/hasim_miss_tracker.bsh"
`include "asim/provides/memory_base_types.bsh"


`define IMEM_ITLB_EPOCH_BITS 2
`define IMEM_ICACHE_EPOCH_BITS 2
`define IMEM_PREDICTION_EPOCH_BITS 2


typedef Bit#(`IMEM_ITLB_EPOCH_BITS) IMEM_ITLB_EPOCH;
typedef Bit#(`IMEM_ICACHE_EPOCH_BITS) IMEM_ICACHE_EPOCH;
typedef Bit#(`IMEM_PREDICTION_EPOCH_BITS) IMEM_PREDICTION_EPOCH;

typedef struct
{
    IMEM_ITLB_EPOCH iTLB;
    IMEM_ICACHE_EPOCH iCache;
    IMEM_PREDICTION_EPOCH prediction;
    TOKEN_BRANCH_EPOCH branch;
    TOKEN_FAULT_EPOCH fault;
}
IMEM_EPOCH deriving (Eq, Bits);

function IMEM_EPOCH initIMemEpoch(IMEM_ITLB_EPOCH iT, IMEM_ICACHE_EPOCH iC, IMEM_PREDICTION_EPOCH iP, TOKEN_BRANCH_EPOCH b,TOKEN_FAULT_EPOCH f);

    return IMEM_EPOCH {iTLB: iT, iCache: iC, prediction: iP, branch: b, fault: f};

endfunction

IMEM_EPOCH initialIMemEpoch = initIMemEpoch(0, 0, 0, 0, 0);


typedef struct
{
    IMEM_EPOCH epoch;
    ISA_ADDRESS virtualAddress;
    MEM_OFFSET offset;
    MEM_ADDRESS physicalAddress;
    ISA_INSTRUCTION instruction;
    ISA_ADDRESS linePrediction;
    CONTEXT_ID ctx_id;
}
IMEM_BUNDLE deriving (Eq, Bits);

function IMEM_BUNDLE initIMemBundle(IMEM_EPOCH ep, ISA_ADDRESS pc, ISA_ADDRESS lp, CONTEXT_ID ctx_id);

    return IMEM_BUNDLE {epoch: ep, virtualAddress: pc, offset: ?, physicalAddress: ?, instruction: ?, linePrediction: lp, ctx_id: ctx_id};

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

function ITLB_OUTPUT initITLBHit(IMEM_BUNDLE bundle, FUNCP_PADDR phys_addr, MEM_OFFSET offs);

    
    let out = ITLB_OUTPUT {bundle: bundle, rspType: tagged ITLB_hit};
    out.bundle.physicalAddress = zeroExtend(phys_addr);
    out.bundle.offset = offs;
    return out;

endfunction

typedef IMEM_BUNDLE ICACHE_INPUT;

function ICACHE_INPUT initICacheLoad(IMEM_BUNDLE bundle);

    return bundle;

endfunction

typedef `ICACHE_MISS_ID_SIZE ICACHE_MISS_ID_SIZE;

typedef CACHE_MISS_INDEX#(ICACHE_MISS_ID_SIZE) L1_ICACHE_MISS_ID;
typedef CACHE_MISS_TOKEN#(ICACHE_MISS_ID_SIZE) L1_ICACHE_MISS_TOKEN;
typedef TExp#(ICACHE_MISS_ID_SIZE) NUM_ICACHE_MISS_IDS;

typedef union tagged
{
    void ICACHE_hit;
    L1_ICACHE_MISS_ID ICACHE_miss;
    void ICACHE_retry;
}
ICACHE_RESPONSE deriving (Eq, Bits);

typedef struct
{
    ICACHE_INPUT bundle;
    ICACHE_RESPONSE rspType;
} 
ICACHE_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef struct
{
    L1_ICACHE_MISS_ID missID;
}
ICACHE_OUTPUT_DELAYED deriving (Eq, Bits);

function ICACHE_OUTPUT_IMMEDIATE initICacheHit(ICACHE_INPUT bundle);

    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: ICACHE_hit};

endfunction

function ICACHE_OUTPUT_IMMEDIATE initICacheMiss(ICACHE_INPUT bundle, L1_ICACHE_MISS_ID miss_id);

    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: tagged ICACHE_miss miss_id};

endfunction

function ICACHE_OUTPUT_IMMEDIATE initICacheRetry(ICACHE_INPUT bundle);

    return ICACHE_OUTPUT_IMMEDIATE {bundle: bundle, rspType: ICACHE_retry};

endfunction

function ICACHE_OUTPUT_DELAYED initICacheMissRsp(L1_ICACHE_MISS_ID miss_id);

    return ICACHE_OUTPUT_DELAYED {missID: miss_id};

endfunction


typedef struct
{
    TOKEN token;
    ISA_ADDRESS virtualAddress;
    MEM_ADDRESS physicalAddress;
    TOKEN_FAULT_EPOCH faultEpoch;
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
        if (x.isLoad)
            s = s + fshow(" LOAD");
        if (x.isStore)
            s = s + fshow(" STORE");
        if (x.isTerminate matches tagged Valid .b)
            s = s + $format(" TERMINATE(%b)", b);
        return s;
    endfunction
endinstance

function DMEM_BUNDLE initDMemBundle(TOKEN tok,
                                    ISA_ADDRESS va,
                                    TOKEN_FAULT_EPOCH epoch,
                                    Bool isL,
                                    Bool isS,
                                    Maybe#(Bool) isT,
                                    ISA_INST_DSTS destRegs,
                                    ISA_INST_DSTS_MASK writeMask);

    function Maybe#(FUNCP_PHYSICAL_REG_INDEX) destRegIfEnabled(Maybe#(FUNCP_PHYSICAL_REG_INDEX) dst, Bool mask) =
        (mask ? dst : tagged Invalid);
       
    let dests = zipWith(destRegIfEnabled, destRegs, writeMask);

    return DMEM_BUNDLE { token: tok,
                         virtualAddress: va,
                         physicalAddress: ?,
                         faultEpoch: epoch,
                         isLoad: isL,
                         isStore: isS,
                         isTerminate: isT,
                         dests: dests };
 
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


typedef `DCACHE_LOAD_MISS_ID_SIZE DCACHE_MISS_ID_SIZE;

typedef CACHE_MISS_INDEX#(DCACHE_MISS_ID_SIZE) L1_DCACHE_MISS_ID;
typedef CACHE_MISS_TOKEN#(DCACHE_MISS_ID_SIZE) L1_DCACHE_MISS_TOKEN;
typedef TExp#(DCACHE_MISS_ID_SIZE) NUM_DCACHE_MISS_IDS;

typedef DMEM_BUNDLE DCACHE_LOAD_INPUT;

function DCACHE_LOAD_INPUT initDCacheLoad(DMEM_BUNDLE bundle);

    return bundle;

endfunction

typedef union tagged
{
    void DCACHE_hit;
    L1_DCACHE_MISS_ID DCACHE_miss;
    void DCACHE_retry;
}
DCACHE_LOAD_RESPONSE deriving (Eq, Bits);

typedef struct
{
    DMEM_BUNDLE bundle;
    DCACHE_LOAD_RESPONSE rspType;
} 
DCACHE_LOAD_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef struct
{
    L1_DCACHE_MISS_ID missID;
}
DCACHE_LOAD_OUTPUT_DELAYED deriving (Eq, Bits);


function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadHit(DMEM_BUNDLE bundle);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: tagged DCACHE_hit};

endfunction

function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadMiss(DMEM_BUNDLE bundle, L1_DCACHE_MISS_ID miss_id);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: tagged DCACHE_miss miss_id};

endfunction

function DCACHE_LOAD_OUTPUT_IMMEDIATE initDCacheLoadRetry(DMEM_BUNDLE bundle);

    return DCACHE_LOAD_OUTPUT_IMMEDIATE {bundle: bundle, rspType: tagged DCACHE_retry};

endfunction

function DCACHE_LOAD_OUTPUT_DELAYED initDCacheLoadMissRsp(L1_DCACHE_MISS_ID miss_id);

    return DCACHE_LOAD_OUTPUT_DELAYED {missID: miss_id};

endfunction


typedef struct
{
    MEM_ADDRESS physicalAddress;
}
DCACHE_STORE_INPUT deriving (Eq, Bits);


function DCACHE_STORE_INPUT initDCacheStore(MEM_ADDRESS phy_addr);

    return DCACHE_STORE_INPUT {physicalAddress: phy_addr};

endfunction

typedef union tagged
{
    void              DCACHE_ok;
    L1_DCACHE_MISS_ID DCACHE_delay;
    void              DCACHE_retryStore;
}
DCACHE_STORE_OUTPUT_IMMEDIATE deriving (Eq, Bits);

typedef struct
{
    L1_ICACHE_MISS_ID missID;
}
DCACHE_STORE_OUTPUT_DELAYED deriving (Eq, Bits);

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreOk();

    return tagged DCACHE_ok;

endfunction

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreDelay(L1_DCACHE_MISS_ID miss_id);

    return tagged DCACHE_delay miss_id;

endfunction

function DCACHE_STORE_OUTPUT_IMMEDIATE initDCacheStoreRetry();

    return tagged DCACHE_retryStore;

endfunction

function DCACHE_STORE_OUTPUT_DELAYED initDCacheStoreDelayOk(L1_DCACHE_MISS_ID miss_id);

    return DCACHE_STORE_OUTPUT_DELAYED {missID: miss_id};

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
    STORE_TOKEN storeToken;
    SB_DEALLOC_REQUEST reqType;
}
SB_DEALLOC_INPUT deriving (Eq, Bits);

function SB_DEALLOC_INPUT initSBWriteback(TOKEN tok, STORE_TOKEN store_tok);

    return SB_DEALLOC_INPUT {token: tok, storeToken: store_tok, reqType: tagged SB_writeback};

endfunction

function SB_DEALLOC_INPUT initSBDrop(TOKEN tok);

    return SB_DEALLOC_INPUT {token: tok, storeToken: ?, reqType: tagged SB_drop};

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

typedef Tuple2#(STORE_TOKEN, MEM_ADDRESS) WB_ENTRY;
