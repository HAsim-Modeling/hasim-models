`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_base_types.bsh"


typedef ISA_ADDRESS INST_ADDRESS;
typedef ISA_ADDRESS DATA_ADDRESS;

typedef union tagged 
{
    ISA_ADDRESS                         CACHE_loadInstruction;      // Instruction fetch from ICache
    ISA_ADDRESS                         CACHE_prefetchInstruction;  // Instruction prefetch from ICache
    Tuple2#(INST_ADDRESS, DATA_ADDRESS) CACHE_loadData;             // Data read from DCache
    Tuple2#(INST_ADDRESS, DATA_ADDRESS) CACHE_writeData;            // Data write to DCache
    Tuple2#(INST_ADDRESS, DATA_ADDRESS) CACHE_prefetchData;         // Data read prefetch
    ISA_ADDRESS                         CACHE_invalidateLine;       // Message to invalidate specific cache line
    Bool                                CACHE_invalidateAll;        // Message to entire cache
    ISA_ADDRESS                         CACHE_flushLine;            // Flush specific cache line            
    Bool                                CACHE_flushAll;             // Flush entire cache
    Bool                                CACHE_killAll;              // Kill all current operations of the cache 
} 
    CACHE_INPUT_REQUEST
        deriving (Eq, Bits);

typedef struct
{
    TOKEN token;
    CACHE_INPUT_REQUEST reqType;
} 
    CACHE_INPUT
        deriving (Eq, Bits);

typedef struct
{
    TOKEN token;
    ISA_ADDRESS address;
} 
    CACHE_OUTPUT 
        deriving (Eq, Bits);

typedef union tagged
{
    CACHE_OUTPUT CACHE_hit;            
    CACHE_OUTPUT CACHE_hitServicing;   
    CACHE_OUTPUT CACHE_miss;           
    CACHE_OUTPUT CACHE_missServicing;  
    CACHE_OUTPUT CACHE_missRetry;      
} 
    CACHE_OUTPUT_IMMEDIATE 
        deriving (Eq, Bits);

typedef CACHE_OUTPUT CACHE_OUTPUT_DELAYED;

function CACHE_OUTPUT initCacheOutput(TOKEN tok, ISA_ADDRESS addr);

    return CACHE_OUTPUT {token: tok, address: addr};

endfunction

function CACHE_OUTPUT_IMMEDIATE initCacheHit(TOKEN tok, ISA_ADDRESS addr);

    return tagged CACHE_hit initCacheOutput(tok, addr);

endfunction

function CACHE_OUTPUT_IMMEDIATE initCacheMissServicing(TOKEN tok, ISA_ADDRESS addr);

    return tagged CACHE_missServicing initCacheOutput(tok, addr);

endfunction

function CACHE_OUTPUT_DELAYED initCacheOutputDelayed(TOKEN tok, ISA_ADDRESS addr);

    return initCacheOutput(tok, addr);

endfunction

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


