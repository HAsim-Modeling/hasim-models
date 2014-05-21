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

`define PORT_LOAD 0
`define PORT_STORE 1
`define PORT_EVICT 2

module [HASIM_MODULE] mkCacheAlgDirectMapped#(Integer opaque_name,
                                              Bool storeTagsInScratchpad)
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, t_IDX_SIZE, 1))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(t_ENTRY, CACHE_ENTRY#(t_OPAQUE, t_IDX_SIZE, 1)),
         Alias#(CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE, t_TAG_SIZE), t_INTERNAL_ENTRY));

    let buffering = valueof(t_NUM_INSTANCES) + 1;

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(Bit#(t_IDX_SIZE)) evictionQ <- mkSizedFIFO(buffering);

    // Initialize a opaque memory to store our tags in.   
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_IDX_SIZE),
                                       Maybe#(t_INTERNAL_ENTRY))
        tagStore <- (storeTagsInScratchpad ?
                         mkMultiReadScratchpad_Multiplexed(opaque_name, defaultValue) :
                         mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, tagged Invalid)));

    function Maybe#(t_ENTRY) entryTagCheck(LINE_ADDRESS addr, Maybe#(t_INTERNAL_ENTRY) m_entry);
        if (m_entry matches tagged Valid .entry)
        begin
            // Check if the tags match.
            let existing_tag = entry.tag;
            let idx = getCacheIndex(addr);
            let target_tag = getCacheTag(addr);

            if (existing_tag == target_tag)
            begin
                // A hit!
                let entry_idx = CACHE_ENTRY_IDX { set: idx, way: 0 };
                return tagged Valid CACHE_ENTRY { idx: entry_idx,
                                                  state: toCacheEntryState(entry, idx) };
            end
            else
            begin
                // A miss.
                return tagged Invalid;
            end
        end
        else
        begin
            // No line at this entry.
            return tagged Invalid;
        end
    endfunction

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        // Look up the index in the tag store.
        let idx = getCacheIndex(addr);
        tagStore.readPorts[`PORT_LOAD].readReq(iid, idx);

        // Pass the request on to the next stage.
        loadLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        
        let m_entry <- tagStore.readPorts[`PORT_LOAD].readRsp(iid);
        
        return entryTagCheck(addr, m_entry);
    endmethod
    
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        // Look up the index in the tag store.    
        let idx = getCacheIndex(addr);
        tagStore.readPorts[`PORT_STORE].readReq(iid, idx);

        // Pass the request on to the next stage.
        storeLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let addr = storeLookupQ.first();
        storeLookupQ.deq();

        let m_entry <- tagStore.readPorts[`PORT_STORE].readRsp(iid);
        
        return entryTagCheck(addr, m_entry);
    endmethod

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        // Look up the index in the tag store.    
        let idx = getCacheIndex(addr);
        tagStore.readPorts[`PORT_EVICT].readReq(iid, idx);
        evictionQ.enq(idx);
    endmethod

    
    method ActionValue#(Maybe#(t_ENTRY)) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        // Since we're direct-mapped this is the same as a lookup.
        // A set-associative cache would do something here to see which way it should use.
        let m_entry <- tagStore.readPorts[`PORT_EVICT].readRsp(iid);
        
        let idx = evictionQ.first();
        evictionQ.deq();
        
        if (m_entry matches tagged Valid .entry)
        begin
            let entry_idx = CACHE_ENTRY_IDX { set: idx, way: 0 };
            return tagged Valid CACHE_ENTRY { idx: entry_idx,
                                              state: toCacheEntryState(entry, idx) };
        end
        else
        begin
            return tagged Invalid;
        end
    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE opaque);
        let entry = dirty ? initInternalCacheEntryDirty(addr) : initInternalCacheEntryClean(addr);
        entry.opaque = opaque;
        let idx = getCacheIndex(addr);
        tagStore.write(iid, idx, tagged Valid entry);
    endmethod

endmodule
