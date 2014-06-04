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


// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/hasim_modellib.bsh"


// ========================================================================
// 
//   Common utilities used internally by cache algorithms. 
// 
// ========================================================================

function Bit#(t_SET_SIZE) getCacheSet(LINE_ADDRESS addr)
    provisos 
        (Add#(t_SET_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    // Purposely use the inverse hash as the hash, and the hash as the inverse.
    // this should help keep things different from the actual scratchpad.

    LINE_ADDRESS h = hashBits_inv(addr);
    return truncate(h);
endfunction

function Bit#(t_TAG_SIZE) getCacheTag(LINE_ADDRESS addr)
    provisos 
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    LINE_ADDRESS h = hashBits_inv(addr);
    return truncateLSB(h);
endfunction

function LINE_ADDRESS unhashAddress(Bit#(t_SET_SIZE) set, Bit#(t_TAG_SIZE) tag)
    provisos
        (Add#(t_SET_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE));

    LINE_ADDRESS hashed_addr = {tag, set};
    return hashBits(hashed_addr);
endfunction

//
// CACHE_ENTRY_STATE_INTERNAL is similar to CACHE_ENTRY_STATE but holds a tag
// instead of a line's physical address.
//
typedef struct
{
    Bool dirty;
    t_OPAQUE opaque;
    Bit#(t_TAG_SIZE) tag;
}
CACHE_ENTRY_STATE_INTERNAL#(type t_OPAQUE, numeric type t_TAG_SIZE)
    deriving (Eq, Bits);


function CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE,
                                     t_TAG_SIZE) initInternalCacheEntry(
                                                     LINE_ADDRESS addr,
                                                     Bool dirty,
                                                     t_OPAQUE opaque)
    provisos
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return 
        CACHE_ENTRY_STATE_INTERNAL
        {
            dirty: dirty,
            opaque: opaque,
            tag: getCacheTag(addr)
        };
endfunction

function CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE,
                                     t_TAG_SIZE)
    initInternalCacheEntryFromState(CACHE_ENTRY_STATE#(t_OPAQUE) state)
    provisos
        (Add#(t_TAG_SIZE, t_TMP, LINE_ADDRESS_SIZE));

    return initInternalCacheEntry(state.linePAddr, state.dirty, state.opaque);
endfunction


function CACHE_ENTRY_STATE#(t_OPAQUE) initCacheEntryClean(LINE_ADDRESS addr);
    return
        CACHE_ENTRY_STATE
        {
            dirty: False,
            opaque: ?,
            linePAddr: addr
        };
endfunction

function CACHE_ENTRY_STATE#(t_OPAQUE) initCacheEntryDirty(LINE_ADDRESS addr);
    return
        CACHE_ENTRY_STATE
        {
            dirty: True,
            opaque: ?,
            linePAddr: addr
        };
endfunction

//
// toCacheEntryState --
//   Convert an internal cache entry state (with tag) to a standard cache entry
//   state (with physical address).
//
function CACHE_ENTRY_STATE#(t_OPAQUE) toCacheEntryState(
            CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE, t_TAG_SIZE) entry,
            Bit#(t_SET_SIZE) set)
    provisos
    (Add#(t_SET_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE));

    let phys_addr = unhashAddress(set, entry.tag);

    return CACHE_ENTRY_STATE
    {
        dirty: entry.dirty, 
        opaque: entry.opaque, 
        linePAddr: phys_addr
    };
endfunction
