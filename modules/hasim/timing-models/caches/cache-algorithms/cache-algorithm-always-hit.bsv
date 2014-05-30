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
// Always hit cache algorithm.
//
// NOTE:  Use of this algorithm in hierarchies that depend on consistent
//        addresses or opaque values is NOT recommended, since the algorithm
//        is stateless.  Returned values will be inconsistent!
//

import FIFO::*;


module [HASIM_MODULE] mkCacheAlgAlwaysHit#(function Bool mayEvict(t_OPAQUE opaque))
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1))
    provisos
        (Alias#(t_IID, INSTANCE_ID#(t_NUM_INSTANCES)),
         Alias#(t_OPAQUE, void),
         NumAlias#(t_SET_SIZE, 0),
         NumAlias#(t_NUM_WAYS, 1));

    let buffering = 2;

    FIFO#(LINE_ADDRESS) lookupByAddrQ <- mkSizedFIFO(buffering);
    FIFO#(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS)) lookupByIdxQ <- mkSizedFIFO(buffering);


    method Action lookupByAddrReq(t_IID iid,
                                  LINE_ADDRESS addr,
                                  Bool updateReplacement,
                                  Bool isLoad);
        lookupByAddrQ.enq(addr);
    endmethod

    method ActionValue#(CACHE_LOOKUP_RSP#(t_OPAQUE,
                                          t_SET_SIZE,
                                          t_NUM_WAYS)) lookupByAddrRsp(t_IID iid);
        let addr = lookupByAddrQ.first();
        lookupByAddrQ.deq();
        
        // Always hit.
        return
            CACHE_LOOKUP_RSP { idx: defaultValue,
                               state: tagged Valid initCacheEntryClean(addr) };
    endmethod

    method Action lookupByIndexReq(t_IID iid,
                                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        lookupByIdxQ.enq(idx);
    endmethod

    method ActionValue#(Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) lookupByIndexRsp(t_IID iid);
        let idx = lookupByIdxQ.first();
        lookupByIdxQ.deq();
        
        // Always hit.  Lacking storage, there is no known address or opaque
        // state to return.
        return tagged Valid initCacheEntryClean(?);
    endmethod

    method Action allocate(t_IID iid,
                           CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                           CACHE_ENTRY_STATE#(t_OPAQUE) state,
                           CACHE_ALLOC_SOURCE source);
        noAction;
    endmethod
    
    method Action update(t_IID iid,
                         CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                         CACHE_ENTRY_STATE#(t_OPAQUE) state);
        noAction;
    endmethod
endmodule
