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
// Pseudo-random cache algorithm.
//
// NOTE:  Use of this algorithm in hierarchies that depend on consistent
//        addresses or opaque values is NOT recommended, since the algorithm
//        is stateless.  Returned values will be inconsistent!
//


import FIFO::*;
import LFSR::*;

module [HASIM_MODULE] mkCacheAlgPseudoRandom
    // parameters:
    #(function Bool mayEvict(t_OPAQUE opaque),
      Bit#(8) seed,
      Bit#(8) missChance,
      Bit#(8) cleanEvictionChance,
      Bit#(8) dirtyEvictionChance)
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

    MULTIPLEXED#(t_NUM_INSTANCES, LFSR#(Bit#(16))) addrLFSRPool  <- mkMultiplexed(mkLFSR_16());
    MULTIPLEXED#(t_NUM_INSTANCES, LFSR#(Bit#(16))) idxLFSRPool  <- mkMultiplexed(mkLFSR_16());
    
    Reg#(Bool) initializingLFSRs <- mkReg(False);


    //
    // genRsp --
    //   Pick a random response based on LFSR and weights.
    //
    function CACHE_LOOKUP_RSP#(t_OPAQUE, t_SET_SIZE, t_NUM_WAYS) genRsp(
        LINE_ADDRESS addr,
        Bit#(16) rnd);

        if (rnd[7:0] < missChance)
        begin
            // A miss.  Should there be an eviction?
            if (rnd[15:8] < cleanEvictionChance)
            begin
                // Miss and clean eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged MustEvict initCacheEntryClean(addr) };
            end
            else if (rnd[15:8] < dirtyEvictionChance)
            begin
                // Miss and dirty eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged MustEvict initCacheEntryDirty(addr) };
            end
            else
            begin
                // Miss and no eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged Invalid };
            end
        end
        else
        begin
            // A hit!
            return
                CACHE_LOOKUP_RSP { idx: defaultValue,
                                   state: tagged Valid initCacheEntryClean(addr) };
        end
    endfunction


    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSRs (initializingLFSRs);
        // Reset all the LFSRs to the seed parameter.
        for (Integer x = 0; x < valueof(t_NUM_INSTANCES); x = x + 1)
        begin
            addrLFSRPool[x].seed(zeroExtend(seed));
            idxLFSRPool[x].seed(zeroExtend(seed) + 1);
        end

        initializingLFSRs <= False;
    endrule


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
        
        // Use a pseudo-random number to see if we hit or not.
        let addrLFSR = addrLFSRPool[iid];
        let rnd = addrLFSR.value();
        addrLFSR.next();

        if (rnd[7:0] < missChance)
        begin
            // A miss.  Should there be an eviction?
            if (rnd[15:8] < cleanEvictionChance)
            begin
                // Miss and clean eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged MustEvict initCacheEntryClean(addr) };
            end
            else if (rnd[15:8] < dirtyEvictionChance)
            begin
                // Miss and dirty eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged MustEvict initCacheEntryDirty(addr) };
            end
            else
            begin
                // Miss and no eviction.
                return CACHE_LOOKUP_RSP { idx: defaultValue,
                                          state: tagged Invalid };
            end
        end
        else
        begin
            // A hit!
            return
                CACHE_LOOKUP_RSP { idx: defaultValue,
                                   state: tagged Valid initCacheEntryClean(addr) };
        end
    endmethod

    method Action lookupByIndexReq(t_IID iid,
                                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        lookupByIdxQ.enq(idx);
    endmethod

    method ActionValue#(Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) lookupByIndexRsp(t_IID iid);
        let idx = lookupByIdxQ.first();
        lookupByIdxQ.deq();

        // Use a pseudo-random number to see if we hit or not.
        let idxLFSR = idxLFSRPool[iid];
        let rnd = idxLFSR.value();
        idxLFSR.next();

        if (rnd[7:0] < missChance)
        begin
            // A miss.  Should there be an eviction?
            if (rnd[15:8] < cleanEvictionChance)
            begin
                // Miss and clean eviction.  Because of random addresses it is
                // impossible to tell the difference between a needed eviction
                // and the desired line.  Don't use pseudo random if this is
                // important to you.
                return tagged Valid initCacheEntryClean(?);
            end
            else if (rnd[15:8] < dirtyEvictionChance)
            begin
                // Miss and dirty eviction.
                return tagged Valid initCacheEntryDirty(?);
            end
            else
            begin
                // Miss and no eviction.
                return tagged Invalid;
            end
        end
        else
        begin
            // A hit!
            return tagged Valid initCacheEntryClean(?);
        end
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
    
    method Action invalidate(t_IID iid,
                             CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        noAction;
    endmethod
endmodule
