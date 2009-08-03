
// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_memory.bsh"


interface CACHE_ALG#(parameter numeric type t_NUM_INSTANCES,
                     parameter type t_OPAQUE);

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE sc);

endinterface

interface CACHE_ALG_INDEXED#(parameter numeric type t_NUM_INSTANCES,
                             parameter type t_OPAQUE,
                             parameter numeric type t_IDX_SIZE);

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE sc);

endinterface


function Bit#(t_IDX_SIZE) getCacheIndex(LINE_ADDRESS addr)
    provisos 
        (Add#(t_IDX_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return truncateLSB(addr);

endfunction

function Bit#(t_TAG_SIZE) getCacheTag(LINE_ADDRESS addr)
    provisos 
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return truncate(addr);

endfunction

function CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE) toCacheAlg(CACHE_ALG_INDEXED#(t_NUM_INSTANCES, t_OPAQUE, t_IDX_SIZE) alg);

    return (interface CACHE_ALG
        method loadLookupReq    = alg.loadLookupReq;
        method loadLookupRsp    = alg.loadLookupRsp;
        method storeLookupReq   = alg.storeLookupReq;
        method storeLookupRsp   = alg.storeLookupRsp;
        method evictionCheckReq = alg.evictionCheckReq;
        method evictionCheckRsp = alg.evictionCheckRsp;
        method allocate         = alg.allocate;
    endinterface);

endfunction

typedef struct
{
    Bool dirty;
    t_OPAQUE opaque;
    Bit#(t_TAG_SIZE) tag;
}
CACHE_ENTRY_INTERNAL#(parameter type t_OPAQUE, 
                      parameter numeric type t_TAG_SIZE) deriving (Eq, Bits);

typedef struct
{
    Bool dirty;
    t_OPAQUE opaque;
    LINE_ADDRESS physicalAddress;
}
CACHE_ENTRY#(parameter type t_OPAQUE) deriving (Eq, Bits);


function CACHE_ENTRY_INTERNAL#(t_OPAQUE, t_TAG_SIZE) initInternalCacheEntryClean(LINE_ADDRESS addr)
    provisos
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return 
        CACHE_ENTRY_INTERNAL
        {
            dirty: False,
            opaque: ?,
            tag: getCacheTag(addr)
        };

endfunction

function CACHE_ENTRY_INTERNAL#(t_OPAQUE, t_TAG_SIZE) initInternalCacheEntryDirty(LINE_ADDRESS addr)
    provisos
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return 
        CACHE_ENTRY_INTERNAL
        {
            dirty: True,
            opaque: ?,
            tag: getCacheTag(addr)
        };

endfunction

function CACHE_ENTRY#(t_OPAQUE) initCacheEntryClean(LINE_ADDRESS addr);

    return 
        CACHE_ENTRY
        {
            dirty: False,
            opaque: ?,
            physicalAddress: addr
        };

endfunction

function CACHE_ENTRY#(t_OPAQUE) initCacheEntryDirty(LINE_ADDRESS addr);

    return 
        CACHE_ENTRY
        {
            dirty: True,
            opaque: ?,
            physicalAddress: addr
        };

endfunction

function CACHE_ENTRY#(t_OPAQUE) toCacheEntry(CACHE_ENTRY_INTERNAL#(t_OPAQUE, t_TAG_SIZE) entry, Bit#(t_IDX_SIZE) idx)
    provisos
        (Add#(t_IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE));

    let phys_addr = {idx, entry.tag};
    return CACHE_ENTRY 
    {
        dirty: entry.dirty, 
        opaque: entry.opaque, 
        physicalAddress: phys_addr
    };

endfunction
