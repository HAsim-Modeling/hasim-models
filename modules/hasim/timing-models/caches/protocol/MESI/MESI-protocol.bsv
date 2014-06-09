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

import DefaultValue::*;
import FShow::*;


`include "awb/provides/memory_base_types.bsh"


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
    CACHE_PROTOCOL_REQ_LOAD REQ_LOAD;
    CACHE_PROTOCOL_RSP_LOAD RSP_LOAD;
    CACHE_PROTOCOL_WB_INVAL WB_INVAL;
    CACHE_PROTOCOL_FORCE_WB FORCE_WB;
}
CACHE_PROTOCOL_MSG_KIND
    deriving (Eq, Bits);


typedef struct
{
    // Shared if False, write-intent if True
    Bool exclusive;
}
CACHE_PROTOCOL_REQ_LOAD
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_PROTOCOL_REQ_LOAD);
    defaultValue = CACHE_PROTOCOL_REQ_LOAD {
        exclusive: False
        };
endinstance


typedef struct
{
    // Shared if False, write-intent if True
    Bool exclusive;
}
CACHE_PROTOCOL_RSP_LOAD
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_PROTOCOL_RSP_LOAD);
    defaultValue = CACHE_PROTOCOL_RSP_LOAD {
        exclusive: False
        };
endinstance


typedef struct
{
    // Line remains in shared state if False. Line no longer present if True.
    Bool inval;
    // Does the payload include dirty state needing writeback?
    Bool isWriteback;
    // Is the message a response to a FORCE_WB request from the directory?
    Bool toDir;
}
CACHE_PROTOCOL_WB_INVAL
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_PROTOCOL_WB_INVAL);
    defaultValue = CACHE_PROTOCOL_WB_INVAL {
        inval: False,
        isWriteback: False,
        toDir: False
        };
endinstance


typedef struct
{
    // Drop the line from the cache if True.
    Bool inval;
    // Is the request from the directory controller?
    Bool fromDir;
}
CACHE_PROTOCOL_FORCE_WB
    deriving (Eq, Bits);

instance DefaultValue#(CACHE_PROTOCOL_FORCE_WB);
    defaultValue = CACHE_PROTOCOL_FORCE_WB {
        inval: False,
        fromDir: False
        };
endinstance



function Bool cacheMsg_IsReqLoad(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged REQ_LOAD .req)
        return True;
    else
        return False;
endfunction

function Bool cacheMsg_IsRspLoad(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged RSP_LOAD .rsp)
        return True;
    else
        return False;
endfunction

function Bool cacheMsg_IsWBInval(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged WB_INVAL .wb)
        return True;
    else
        return False;
endfunction

function Bool cacheMsg_IsForceWB(CACHE_PROTOCOL_MSG msg);
    if (msg.kind matches tagged FORCE_WB .force_wb)
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
        kind: tagged REQ_LOAD defaultValue
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
        kind: tagged RSP_LOAD defaultValue
    };
endfunction

//
// cacheMsg_WBInval --
//   Generate a WB_INVAL response CACHE_PROTOCOL_MSG.
//
function CACHE_PROTOCOL_MSG cacheMsg_WBInval(LINE_ADDRESS linePAddr,
                                             MEM_OPAQUE opaque);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: linePAddr,
        opaque: opaque,
        kind: tagged WB_INVAL defaultValue
    };
endfunction

//
// cacheMsg_ForceWB --
//   Generate a FORCE_WB response CACHE_PROTOCOL_MSG.
//
function CACHE_PROTOCOL_MSG cacheMsg_ForceWB(LINE_ADDRESS linePAddr,
                                             MEM_OPAQUE opaque);
    return CACHE_PROTOCOL_MSG
    {
        linePAddr: linePAddr,
        opaque: opaque,
        kind: tagged FORCE_WB defaultValue
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
            tagged REQ_LOAD .req:
            begin
                fs = $format("REQ_LOAD excl=%0d", req.exclusive);
            end

            tagged RSP_LOAD .rsp:
            begin
                fs = $format("RSP_LOAD excl=%0d", rsp.exclusive);
            end

            tagged WB_INVAL .wb:
            begin
                fs = $format("WB_INVAL inval=%0d, isWB=%0d, toDir=%0d",
                             wb.inval, wb.isWriteback, wb.toDir);
            end

            tagged FORCE_WB .force_wb:
            begin
                fs = $format("FORCE_WB inval=%0d, fromDir=%0d",
                             force_wb.inval, force_wb.fromDir);
            end
        endcase

        return fs;
    endfunction
endinstance