//
// Copyright (C) 2009 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/librl_bsv_storage.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/mem_services.bsh"
`include "asim/provides/common_services.bsh"

`define PORT_LOAD 0
`define PORT_STORE 1
`define PORT_EVICT 2

module [HASIM_MODULE] mkCacheAlgDirectMapped#(Integer opaque_name)
    // interface:
        (CACHE_ALG_INDEXED#(t_NUM_INSTANCES, t_OPAQUE, t_IDX_SIZE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(CACHE_ENTRY_INTERNAL#(t_OPAQUE, t_TAG_SIZE), t_INTERNAL_ENTRY),
         // The following is brought to you courtesy of proviso hell:
         Add#(t_TMP, TAdd#(TSub#(TAdd#(TLog#(t_NUM_INSTANCES), t_IDX_SIZE),
         TLog#(TDiv#(64, TExp#(TLog#(TAdd#(1, TAdd#(1, TAdd#(t_OPAQUE_SIZE,
         t_TAG_SIZE)))))))), TLog#(TDiv#(TExp#(TLog#(TAdd#(1, TAdd#(1,
         TAdd#(t_OPAQUE_SIZE, t_TAG_SIZE))))), 64))), 32));



    let buffering = valueof(t_NUM_INSTANCES) + 1;

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(Bit#(t_IDX_SIZE)) evictionQ <- mkSizedFIFO(buffering);

    // Initialize a opaque memory to store our tags in.   
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES, 3, Bit#(t_IDX_SIZE), Maybe#(t_INTERNAL_ENTRY)) tagStore <- mkMultiReadScratchpad_Multiplexed(opaque_name, SCRATCHPAD_CACHED);

    function Maybe#(CACHE_ENTRY#(t_OPAQUE)) entryTagCheck(LINE_ADDRESS addr, Maybe#(t_INTERNAL_ENTRY) m_entry);
    
        if (m_entry matches tagged Valid .entry)
        begin

            // Check if the tags match.
            let existing_tag = entry.tag;
            let idx = getCacheIndex(addr);
            let target_tag = getCacheTag(addr);

            if (existing_tag == target_tag)
            begin
                
                // A hit!
                return tagged Valid toCacheEntry(entry, idx);
                
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
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
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
        storeLookupQ.enq(getCacheIndex(addr));
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
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

    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        // Since we're direct-mapped this is the same as a lookup.
        // A set-associative cache would do something here to see which way it should use.
        let m_entry <- tagStore.readPorts[`PORT_EVICT].readRsp(iid);
        
        let idx = evictionQ.first();
        evictionQ.deq();
        
        if (m_entry matches tagged Valid .entry)
            return tagged Valid toCacheEntry(entry, idx);
        else
            return tagged Invalid;
        
    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE opaque);
    
        let entry = dirty ? initInternalCacheEntryDirty(addr) : initInternalCacheEntryClean(addr);
        entry.opaque = opaque;
        let idx = getCacheIndex(addr);
        tagStore.write(iid, idx, tagged Valid entry);

    endmethod

    
endmodule
