//
// Copyright (C) 2010 Intel Corporation
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

import Vector::*;

`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/librl_bsv_storage.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/mem_services.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"

`define PORT_LOAD 0
`define PORT_EVICT 1
`define PORT_ALLOC 2

module [HASIM_MODULE] mkCacheAlgSetAssociative#(Integer opaque_name, NumTypeParam#(t_NUM_WAYS) dummy)
    // interface:
        (CACHE_ALG_INDEXED#(t_NUM_INSTANCES, t_OPAQUE, t_IDX_SIZE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(CACHE_ENTRY_INTERNAL#(t_OPAQUE, t_TAG_SIZE), t_INTERNAL_ENTRY),
         // The following is brought to you courtesy of proviso hell:
         Add#(t_TMP, TAdd#(TSub#(TAdd#(TLog#(t_NUM_INSTANCES), t_IDX_SIZE),
             TLog#(TDiv#(64, TExp#(TLog#(TAdd#(1, TAdd#(t_OPAQUE_SIZE,
             t_TAG_SIZE))))))), TLog#(TDiv#(TExp#(TLog#(TAdd#(1, TAdd#(t_OPAQUE_SIZE, t_TAG_SIZE)))), 64))), 32));


    let buffering = valueof(t_NUM_INSTANCES) + 1;
    Integer numWays = valueof(t_NUM_WAYS);

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(Bit#(t_IDX_SIZE)) evictionQ <- mkSizedFIFO(buffering);

    // Initialize a opaque memory to store our tags in.   
    Vector#(t_NUM_WAYS, MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES, 2, Bit#(t_IDX_SIZE), t_INTERNAL_ENTRY)) tagStoreBanks = newVector();
    
    MULTIPLEXED_LUTRAM_MULTI_WRITE#(t_NUM_INSTANCES, 3, Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessedPool <- mkMultiplexedLUTRAMMultiWrite(replicate(False));
    MULTIPLEXED_LUTRAM#(t_NUM_INSTANCES, Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) validsPool <- mkMultiplexedLUTRAM(replicate(False));

    for (Integer x = 0; x < numWays; x = x + 1)
    begin
        tagStoreBanks[x] <- mkMultiReadScratchpad_Multiplexed(opaque_name + x, SCRATCHPAD_CACHED);
    end

    function Maybe#(CACHE_ENTRY#(t_OPAQUE)) entryTagCheck(LINE_ADDRESS addr, Bool valid, t_INTERNAL_ENTRY entry);
    
        if (valid)
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
        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            tagStoreBanks[x].readPorts[`PORT_LOAD].readReq(iid, idx);
        end

        // Pass the request on to the next stage.
        loadLookupQ.enq(addr);
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
    
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessed = accessedPool.getRAMWithWritePort(iid, `PORT_LOAD);
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) valids   = validsPool.getRAM(iid);

        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        let idx = getCacheIndex(addr);

        Maybe#(CACHE_ENTRY#(t_OPAQUE)) res = tagged Invalid;
        let validvec = valids.sub(idx);
        
        Maybe#(Bit#(TLog#(t_NUM_WAYS))) winner = tagged Invalid;

        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            let entry <- tagStoreBanks[x].readPorts[`PORT_LOAD].readRsp(iid);
        
            if (entryTagCheck(addr, validvec[x], entry) matches tagged Valid .entry2)
            begin
                winner = tagged Valid fromInteger(x);
                res = tagged Valid entry2;
            end
        end
        
        if (winner matches tagged Valid .way)
        begin
            let new_accessed = accessed.sub(idx);
            new_accessed[way] = True;
            new_accessed = all(id, new_accessed) ? replicate(False) : new_accessed;
            accessed.upd(idx, new_accessed);
        end
        
        return res;

    endmethod
    

    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
/*
        // Look up the index in the tag store.
        let idx = getCacheIndex(addr);
        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            tagStoreBanks[x].readPorts[`PORT_STORE].readReq(iid, idx);
        end

        // Pass the request on to the next stage.
        storeLookupQ.enq(addr);
*/
        noAction;
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        /*
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessed = accessedPool.getRAMWithWritePort(iid, `PORT_STORE);
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) valids   = validsPool.getRAM(iid);

        let addr = storeLookupQ.first();
        storeLookupQ.deq();
        let idx = getCacheIndex(addr);

        Maybe#(CACHE_ENTRY#(t_OPAQUE)) res = tagged Invalid;
        let validvec = valids.sub(idx);

        Maybe#(Bit#(TLog#(t_NUM_WAYS))) winner = tagged Invalid;

        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            let entry <- tagStoreBanks[x].readPorts[`PORT_STORE].readRsp(iid);
        
            if (entryTagCheck(addr, validvec[x], entry) matches tagged Valid .entry2)
            begin
                res = tagged Valid entry2;
                winner = tagged Valid fromInteger(x);
            end
        end
        
        if (winner matches tagged Valid .way)
        begin
            let new_accessed = accessed.sub(idx);
            new_accessed[way] = True;
            new_accessed = all(id, new_accessed) ? replicate(False) : new_accessed;
            accessed.upd(idx, new_accessed);
        end
        
        return res;
        */
        noAction;
        return tagged Invalid;
    endmethod

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);

        // Look up the index in the tag store.
        let idx = getCacheIndex(addr);
        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            tagStoreBanks[x].readPorts[`PORT_EVICT].readReq(iid, idx);
        end

        // Pass the request on to the next stage.
        evictionQ.enq(getCacheIndex(addr));
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        let idx = evictionQ.first();
        evictionQ.deq();

        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessed = accessedPool.getRAMWithWritePort(iid, `PORT_EVICT);
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) valids   = validsPool.getRAM(iid);

        
        // Since we're a set associative cache we need to figure out which bank to insert into for this address.
        // This is where the Accessing psuedo-LRU scheme comes in.
        
        Vector#(t_NUM_WAYS, t_INTERNAL_ENTRY) entries = newVector();
       
        let validvec = valids.sub(idx);
        let accessedvec = accessed.sub(idx);
        
        Bool allValid = True;

        for (Integer x = 0; x < numWays; x = x + 1)
        begin

            entries[x] <- tagStoreBanks[x].readPorts[`PORT_EVICT].readRsp(iid);
            allValid = allValid || validvec[x];

        end

        if (!allValid)
        begin
            // There's an unused way, so no one will be evicted.
            return tagged Invalid;
        end
        else
        begin
        
            Maybe#(CACHE_ENTRY#(t_OPAQUE)) res = tagged Valid toCacheEntry(entries[0], idx);

            // Everyone's valid, so figure out which was (pseudo) LRU.
            for (Integer x = 0; x < numWays; x = x + 1)
            begin
                if (!accessedvec[x])
                begin
                    res = tagged Valid toCacheEntry(entries[x], idx);
                end
            end

            return res;

        end


    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE opaque);
    
        let entry = dirty ? initInternalCacheEntryDirty(addr) : initInternalCacheEntryClean(addr);
        entry.opaque = opaque;
        let idx = getCacheIndex(addr);

        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessed = accessedPool.getRAMWithWritePort(iid, `PORT_ALLOC);
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) valids   = validsPool.getRAM(iid);

        Bit#(TLog#(t_NUM_WAYS)) winner = 0;
        let accessedvec = accessed.sub(idx);
        let validvec = valids.sub(idx);
        Bool allValid = all(id, validvec);

        if (!allValid)
        begin
        
            // Pick an invalid entry.
            for (Integer x = 0; x < numWays; x = x + 1)
            begin
                if (!validvec[x])
                begin
                    winner = fromInteger(x);
                end
            end

        end
        else
        begin

            // Figure out which was (pseudo) LRU. Assert that at least one entry accessed == 0.
            for (Integer x = 0; x < numWays; x = x + 1)
            begin
                if (!accessedvec[x])
                begin
                    winner = fromInteger(x);
                end
            end

        end
        
        let new_accessed = accessedvec;
        new_accessed[winner] = False;
        accessed.upd(idx, new_accessed);
        let new_valid = validvec;
        validvec[winner] = True;
        valids.upd(idx, validvec);
        tagStoreBanks[winner].write(iid, idx, entry);

    endmethod

endmodule
