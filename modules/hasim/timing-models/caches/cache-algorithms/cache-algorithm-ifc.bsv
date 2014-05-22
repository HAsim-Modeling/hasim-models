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
                     numeric type t_IDX_SIZE,
                     numeric type t_NUM_WAYS);

    method Action loadLookupReq(
        INSTANCE_ID#(t_NUM_INSTANCES) iid,
        LINE_ADDRESS addr);

    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE,
                                            t_IDX_SIZE,
                                            t_NUM_WAYS))) loadLookupRsp(
        INSTANCE_ID#(t_NUM_INSTANCES) iid);


    method Action storeLookupReq(
        INSTANCE_ID#(t_NUM_INSTANCES) iid,
        LINE_ADDRESS addr);

    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE,
                                            t_IDX_SIZE,
                                            t_NUM_WAYS))) storeLookupRsp(
        INSTANCE_ID#(t_NUM_INSTANCES) iid);


    method Action evictionCheckReq(
        INSTANCE_ID#(t_NUM_INSTANCES) iid,
        LINE_ADDRESS addr);

    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE,
                                            t_IDX_SIZE,
                                            t_NUM_WAYS))) evictionCheckRsp(
        INSTANCE_ID#(t_NUM_INSTANCES) iid);


    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                           LINE_ADDRESS addr,
                           Bool dirty,
                           t_OPAQUE sc);
endinterface


//
// CACHE_ENTRY is a combination of the set/way and the state.  A
// tagged Invalid state indicates the entry is currently invalid.
//
typedef struct
{
    CACHE_ENTRY_IDX#(t_IDX_SIZE, t_NUM_WAYS) idx;
    Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE)) state;
}
CACHE_ENTRY#(type t_OPAQUE, numeric type t_IDX_SIZE, numeric type t_NUM_WAYS)
    deriving (Eq, Bits);

//
// CACHE_ENTRY_IDX is the index of the cache bucket holding a line.
//
typedef struct
{
    Bit#(t_IDX_SIZE) set;
    Bit#(TLog#(t_NUM_WAYS)) way;
}
CACHE_ENTRY_IDX#(numeric type t_IDX_SIZE, numeric type t_NUM_WAYS)
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_ENTRY_IDX#(t_IDX_SIZE, t_NUM_WAYS));
    defaultValue = CACHE_ENTRY_IDX { set: 0, way: 0 };
endinstance

//
// CACHEN_ENTRY_STATE is the cache state of a single entry.
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
