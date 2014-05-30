//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

import FShow::*;
import DefaultValue::*;


// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/hasim_modellib.bsh"


//
// CACHE_ALG --
//   Cache algorithm interface.  Not all of the parameters make sense for
//   all flavors.  For example, the number of ways may be set to 0 for
//   direct mapped caches.  For an always hit cache, the index size may
//   be 0.
//
interface CACHE_ALG#(numeric type t_NUM_INSTANCES,
                     type t_OPAQUE,
                     numeric type t_SET_SIZE,
                     numeric type t_NUM_WAYS);

    //
    // lookupByAddrReq/Rsp --
    //   Is addr present in the cache?  Returns a Valid response if present.
    //
    //   If updateReplacement is True then the replacement policy state is
    //   updated.
    //
    //   isLoad is a hint to the replacement policy, which may or may not
    //   be interested.  It has no effect on the dirty bit.
    //
    method Action lookupByAddrReq(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                                  LINE_ADDRESS addr,
                                  Bool updateReplacement,
                                  Bool isLoad);

    method ActionValue#(CACHE_LOOKUP_RSP#(t_OPAQUE,
                                          t_SET_SIZE,
                                          t_NUM_WAYS)) lookupByAddrRsp(
        INSTANCE_ID#(t_NUM_INSTANCES) iid);


    //
    // lookupByIndex --
    //   Return the current state of a cache entry.  No state is updated,
    //   including replacement policy state.
    //
    method Action lookupByIndexReq(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);

    method ActionValue#(Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) lookupByIndexRsp(
        INSTANCE_ID#(t_NUM_INSTANCES) iid);


    //
    // allocate --
    //   Write a new line to the cache at entry idx, overwriting whatever
    //   was there.
    //
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                           CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                           CACHE_ENTRY_STATE#(t_OPAQUE) state,
                           CACHE_ALLOC_SOURCE source);

    //
    // update --
    //   Similar to allocate(), but updates an existing entry for a line
    //   already present in the cache.  No update to the replacement policy.
    //
    method Action update(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                         CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                         CACHE_ENTRY_STATE#(t_OPAQUE) state);
endinterface


//
// CACHE_LOOKUP_RSP is a combination of the set/way and the state.
//
typedef struct
{
    CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx;
    CACHE_LOOKUP_STATE#(t_OPAQUE) state;
}
CACHE_LOOKUP_RSP#(type t_OPAQUE, numeric type t_SET_SIZE, numeric type t_NUM_WAYS)
    deriving (Eq, Bits);

//
// CACHE_LOOKUP_STATE is returned as part of a lookupRsp().
//
//   Invalid:
//     Line not present and the cache entry is currently invalid.
//   Valid:
//     Line present.
//   MustEvict:
//     Line not present.  In order to add the new line, the returned entry
//     must be evicted.
//   Blocked:
//     Line not present.  Current entries are in transition and none may be
//     evicted.
//
typedef union tagged
{
    void                         Invalid;
    CACHE_ENTRY_STATE#(t_OPAQUE) Valid;
    CACHE_ENTRY_STATE#(t_OPAQUE) MustEvict;
    void                         Blocked;
}
CACHE_LOOKUP_STATE#(type t_OPAQUE)
    deriving (Eq, Bits);


//
// CACHE_ENTRY_IDX is the index of the cache bucket holding a line.
//
typedef struct
{
    Bit#(t_SET_SIZE) set;
    UInt#(TLog#(t_NUM_WAYS)) way;
}
CACHE_ENTRY_IDX#(numeric type t_SET_SIZE, numeric type t_NUM_WAYS)
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS));
    defaultValue = CACHE_ENTRY_IDX { set: 0, way: 0 };
endinstance

function CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) initCacheEntryIdx(
                                                       Bit#(t_SET_SIZE) set,
                                                       UInt#(TLog#(t_NUM_WAYS)) way);
    return CACHE_ENTRY_IDX { set: set, way: way };
endfunction
                                                                     

instance FShow#(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS));
    function Fmt fshow(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        return $format("set 0x%h, way %0d", idx.set, idx.way);
    endfunction
endinstance


//
// CACHE_ENTRY_STATE is the cache state of a single entry.
//
typedef struct
{
    Bool dirty;
    t_OPAQUE opaque;
    LINE_ADDRESS linePAddr;
}
CACHE_ENTRY_STATE#(type t_OPAQUE)
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_ENTRY_STATE#(t_OPAQUE));
    defaultValue = CACHE_ENTRY_STATE { dirty: False, opaque: ?, linePAddr: 0 };
endinstance

function CACHE_ENTRY_STATE#(t_OPAQUE) initCacheEntryState(LINE_ADDRESS linePAddr,
                                                          Bool dirty,
                                                          t_OPAQUE opaque);
    return CACHE_ENTRY_STATE { dirty: dirty,
                               opaque: opaque,
                               linePAddr: linePAddr };
endfunction

instance FShow#(CACHE_ENTRY_STATE#(t_OPAQUE)) provisos (Bits#(t_OPAQUE, t_OPAQUE_SZ));
    function Fmt fshow(CACHE_ENTRY_STATE#(t_OPAQUE) state);
        return $format("line 0x%h, dirty %0d, opaque 0x%h",
                       state.linePAddr, state.dirty, pack(state.opaque));
    endfunction
endinstance


//
// CACHE_ALLOC_SOURCE --
//   Source of the allocated line.
//
typedef enum
{
    CACHE_ALLOC_FILL,
    CACHE_ALLOC_STORE
}
CACHE_ALLOC_SOURCE
    deriving (Eq, Bits);
