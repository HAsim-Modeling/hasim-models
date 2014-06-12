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

import DefaultValue::*;


`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/librl_bsv_storage.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/mem_services.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/scratchpad_memory_common.bsh"

`define PORT_ADDR 0
`define PORT_IDX  1

module [HASIM_MODULE] mkCacheAlgDirectMapped#(function Bool mayEvict(t_OPAQUE opaque),
                                              Integer opaque_name,
                                              Bool storeTagsInScratchpad)
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, t_SET_SIZE, 1))
    provisos
        (Alias#(t_IID, INSTANCE_ID#(t_NUM_INSTANCES)),
         Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_SET_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(t_INTERNAL_ENTRY, CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE, t_TAG_SIZE)),
         NumAlias#(t_NUM_WAYS, 1));

    let buffering = valueof(t_NUM_INSTANCES) + 1;

    FIFO#(Tuple2#(Bit#(t_SET_SIZE), Bit#(t_TAG_SIZE))) lookupByAddrQ <- mkSizedFIFO(buffering);
    FIFO#(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS)) lookupByIdxQ <- mkSizedFIFO(buffering);

    let assertIdxOk <- mkAssertionSimOnly("cache-algorithm-direct-mapped.bsv: Incorrect ADDR for IDX!",
                                          ASSERT_ERROR);

    // Initialize a opaque memory to store our tags in.   
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       2,
                                       Bit#(t_SET_SIZE),
                                       Maybe#(t_INTERNAL_ENTRY))
        tagStore <- (storeTagsInScratchpad ?
                         mkMultiReadScratchpad_Multiplexed(opaque_name, defaultValue) :
                         mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, tagged Invalid)));


    method Action lookupByAddrReq(t_IID iid,
                                  LINE_ADDRESS addr,
                                  Bool updateReplacement,
                                  Bool isLoad);
        // Look up the set in the tag store.
        let set = getCacheSet(addr);
        tagStore.readPorts[`PORT_ADDR].readReq(iid, set);

        // Pass the request on to the next stage.
        lookupByAddrQ.enq(tuple2(set, getCacheTag(addr)));
    endmethod
    
    method ActionValue#(CACHE_LOOKUP_RSP#(t_OPAQUE,
                                          t_SET_SIZE,
                                          t_NUM_WAYS)) lookupByAddrRsp(t_IID iid);
        match {.set, .target_tag} = lookupByAddrQ.first();
        lookupByAddrQ.deq();
        
        let m_entry <- tagStore.readPorts[`PORT_ADDR].readRsp(iid);

        let entry_idx = CACHE_ENTRY_IDX { set: set, way: 0 };

        if (m_entry matches tagged Valid .entry)
        begin
            // Check if the tags match.
            if (entry.tag == target_tag)
            begin
                // A hit!
                return
                    CACHE_LOOKUP_RSP { idx: entry_idx,
                                       state: tagged Valid toCacheEntryState(entry, set) };
            end
            else if (mayEvict(entry.opaque))
            begin
                // A miss and the current entry may be evicted.
                return
                    CACHE_LOOKUP_RSP { idx: entry_idx,
                                       state: tagged MustEvict toCacheEntryState(entry, set) };
            end
            else
            begin
                // A miss and the current entry may NOT be evicted.
                return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged Blocked };
            end
        end
        else
        begin
            // No line at this entry.
            return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged Invalid };
        end
    endmethod

    
    method Action lookupByIndexReq(t_IID iid,
                                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        // Look up the index in the tag store.
        tagStore.readPorts[`PORT_IDX].readReq(iid, idx.set);

        lookupByIdxQ.enq(idx);
    endmethod

    method ActionValue#(Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) lookupByIndexRsp(t_IID iid);
        let idx = lookupByIdxQ.first();
        lookupByIdxQ.deq();
        
        let m_entry <- tagStore.readPorts[`PORT_IDX].readRsp(iid);
        if (m_entry matches tagged Valid .entry)
        begin
            return tagged Valid toCacheEntryState(entry, idx.set);
        end
        else
        begin
            return tagged Invalid;
        end
    endmethod


    method Action allocate(t_IID iid,
                           CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                           CACHE_ENTRY_STATE#(t_OPAQUE) state,
                           CACHE_ALLOC_SOURCE source);

        assertIdxOk(idx.set == getCacheSet(state.linePAddr));

        let entry = initInternalCacheEntryFromState(state);
        tagStore.write(iid, idx.set, tagged Valid entry);
    endmethod
    
    method Action update(t_IID iid,
                         CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                         CACHE_ENTRY_STATE#(t_OPAQUE) state);

        assertIdxOk(idx.set == getCacheSet(state.linePAddr));

        let entry = initInternalCacheEntryFromState(state);
        tagStore.write(iid, idx.set, tagged Valid entry);
    endmethod
    
    method Action invalidate(t_IID iid,
                             CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);

        tagStore.write(iid, idx.set, tagged Invalid);
    endmethod
endmodule
