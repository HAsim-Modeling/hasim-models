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
// Global definitions for traffic between cache levels.
//

import FShow::*;

`include "awb/provides/memory_base_types.bsh"


//
// Number of L1 channels in each direction (needed to instantiate the L1
// cache arbiter).
//
typedef 1 CACHE_PROTOCOL_CHANNELS_FROM_L1;
typedef 1 CACHE_PROTOCOL_CHANNELS_TO_L1;


//
// All coherence messages are represented in CACHE_PROTOCOL_MSG.
//
typedef struct
{
    LINE_ADDRESS linePAddr;
    MEM_OPAQUE opaque;
    CACHE_PROTOCOL_MSG_KIND kind;
}
CACHE_PROTOCOL_MSG
    deriving (Eq, Bits);

//
// Flavors of cache protocol messages, stored as a sub-type in CACHE_PROTOCOL_MSG.
//
typedef union tagged
{
    void REQ_LOAD;
    void REQ_STORE;
    void RSP_LOAD;
}
CACHE_PROTOCOL_MSG_KIND
    deriving (Eq, Bits);


function Bool cacheMsg_IsReqLoad(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged REQ_LOAD)
        return True;
    else
        return False;
endfunction

function Bool cacheMsg_IsReqStore(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged REQ_STORE)
        return True;
    else
        return False;
endfunction

function Bool cacheMsg_IsRspLoad(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged RSP_LOAD)
        return True;
    else
        return False;
endfunction


//
// cacheMsg_ReqLoad --
//   Generate a load request CACHE_PROTOCOL_MSG.
//
function CACHE_PROTOCOL_MSG cacheMsg_ReqLoad(LINE_ADDRESS linePAddr,
                                             MEM_OPAQUE opaque);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: linePAddr,
        opaque: opaque,
        kind: tagged REQ_LOAD
    };
endfunction

//
// cacheMsg_ReqStore --
//   Generate a store request CACHE_PROTOCOL_MSG.
//
function CACHE_PROTOCOL_MSG cacheMsg_ReqStore(LINE_ADDRESS linePAddr,
                                              MEM_OPAQUE opaque);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: linePAddr,
        opaque: opaque,
        kind: tagged REQ_STORE
    };
endfunction


//
// cacheMsg_RspLoad --
//   Generate a load response CACHE_PROTOCOL_MSG.
//
function CACHE_PROTOCOL_MSG cacheMsg_RspLoad(LINE_ADDRESS linePAddr,
                                             MEM_OPAQUE opaque);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: linePAddr,
        opaque: opaque,
        kind: tagged RSP_LOAD
    };
endfunction


//
// cacheMsgFromMemReq --
//   Convert a MEMORY_REQ to a cache protocol message.
//
function CACHE_PROTOCOL_MSG cacheMsgFromMemReq(MEMORY_REQ mreq);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: mreq.linePAddr,
        opaque: mreq.opaque,
        kind: (mreq.isStore ? tagged REQ_STORE : tagged REQ_LOAD)
    };
endfunction

//
// cacheMsgFromMemRsp --
//   Convert a MEMORY_RSP to a cache protocol message.
//
function CACHE_PROTOCOL_MSG cacheMsgFromMemRsp(MEMORY_RSP mrsp);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: mrsp.linePAddr,
        opaque: mrsp.opaque,
        kind: tagged RSP_LOAD
    };
endfunction



//
// Debugging
//

instance FShow#(CACHE_PROTOCOL_MSG);
    function Fmt fshow(CACHE_PROTOCOL_MSG msg);
        return fshow(msg.kind) + $format(" line=0x%x, opaque=%0d",
                                         msg.linePAddr, msg.opaque);
    endfunction
endinstance

instance FShow#(CACHE_PROTOCOL_MSG_KIND);
    function Fmt fshow(CACHE_PROTOCOL_MSG_KIND kind);
        Fmt fs = $format("ERROR:  Unexpected CACHE_PROTOCOL_MSG_KIND");

        case (kind) matches
            tagged REQ_LOAD:
            begin
                fs = $format("REQ_LOAD");
            end

            tagged REQ_STORE:
            begin
                fs = $format("REQ_STORE");
            end

            tagged RSP_LOAD:
            begin
                fs = $format("RSP_LOAD");
            end
        endcase

        return fs;
    endfunction
endinstance
