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
//

import FShow::*;


// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/chip_base_types.bsh"
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
MEMORY_REQ deriving (Eq, Bits);

function MEMORY_REQ initMemLoad(LINE_ADDRESS addr);
    return MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: False,
        opaque: 0
    };
endfunction

function MEMORY_REQ initMemStore(LINE_ADDRESS addr);
    return MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: True,
        opaque: 0
    };
endfunction


typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
}
MEMORY_RSP deriving (Eq, Bits);

function MEMORY_RSP initMemRspFromReq(MEMORY_REQ req);
    return MEMORY_RSP
    {
        physicalAddress: req.physicalAddress,
        opaque: req.opaque
    };
endfunction

function MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);
    return MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };
endfunction


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


//
// updateMemOpaque --
//   Similar to toMemOpaque but preserves the original value of bits outside
//   the new portion.  (I.e. outside the size of t_ANY.)  Preserving bits
//   in a memory hierarchy allows a cache model to reduce the RAM needed
//   to store the original state of a token.  The original state is typically
//   stored in a memory in order to restore the token before returning a
//   result up the cache hierarchy.
//
function MEM_OPAQUE updateMemOpaque(MEM_OPAQUE orig, t_ANY x)
    provisos
        (Bits#(t_ANY, t_SZ),
         Add#(t_SZ, t_TMP, MEM_OPAQUE_SIZE));
    
    return unpack({ truncateLSB(orig), pack(x) });
endfunction


instance FShow#(MEMORY_REQ);
    function Fmt fshow(MEMORY_REQ req);
        if (req.isStore)
        begin
            return $format("STORE line=0x%x", req.physicalAddress);
        end
        else
        begin
            return $format("LOAD line=0x%x, opaque=%0d",
                           req.physicalAddress,
                           req.opaque);
        end
    endfunction
endinstance

instance FShow#(MEMORY_RSP);
    function Fmt fshow(MEMORY_RSP rsp);
        return $format("RSP line=0x%x, opaque=%0d",
                       rsp.physicalAddress,
                       rsp.opaque);
    endfunction
endinstance
