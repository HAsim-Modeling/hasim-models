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

import Vector::*;
import FIFOF::*;
import DefaultValue::*;

`include "asim/provides/librl_bsv_base.bsh"
`include "asim/provides/librl_bsv_storage.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/mem_services.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/scratchpad_memory_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"

`define PORT_LOAD 0
`define PORT_EVICT 1
`define PORT_ALLOC 2

module [HASIM_MODULE] mkCacheAlgSetAssociative#(TIMEP_DEBUG_FILE_MULTIPLEXED#(t_NUM_INSTANCES) debugLog,
                                                Integer opaque_name,
                                                Bool storeTagsInScratchpad)
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, t_IDX_SIZE, t_NUM_WAYS))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_IDX_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(t_ENTRY, CACHE_ENTRY#(t_OPAQUE, t_IDX_SIZE, t_NUM_WAYS)),
         Alias#(CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE, t_TAG_SIZE), t_INTERNAL_ENTRY));

    let buffering = 2;
    Integer numWays = valueOf(t_NUM_WAYS);

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    //FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(Bit#(t_IDX_SIZE)) evictionQ <- mkSizedFIFO(buffering);
    FIFOF#(Tuple3#(INSTANCE_ID#(t_NUM_INSTANCES), Bit#(t_IDX_SIZE), t_INTERNAL_ENTRY)) allocQ <- mkSizedFIFOF(buffering);

    // Store tags in a scratchpad
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_IDX_SIZE),
                                       Vector#(t_NUM_WAYS, t_INTERNAL_ENTRY))
        tagStoreBanks <- (storeTagsInScratchpad ?
                              mkMultiReadScratchpad_Multiplexed(opaque_name, defaultValue) :
                              mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiRead(False)));
    
    // Smaller cache management meta-data fits in local BRAM
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_IDX_SIZE),
                                       Vector#(t_NUM_WAYS, Bool))
        accessedPool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, replicate(False)));

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_IDX_SIZE),
                                       Vector#(t_NUM_WAYS, Bool))
        validsPool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, replicate(False)));


    //
    // entryTagCheck --
    //     Check whether a single entry's valid bit is set and whether its
    //     address matches a request.
    //
    function Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE)) entryTagCheck(LINE_ADDRESS addr,
                                                                Bool valid,
                                                                t_INTERNAL_ENTRY entry);
        if (valid)
        begin
            // Check if the tags match.
            let existing_tag = entry.tag;
            let idx = getCacheIndex(addr);
            let target_tag = getCacheTag(addr);

            if (existing_tag == target_tag)
            begin
                // A hit!
                return tagged Valid toCacheEntryState(entry, idx);
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


    rule finishAllocate (True);
    
        match {.iid, .idx, .entry} = allocQ.first();
        allocQ.deq();
        let entryvec <- tagStoreBanks.readPorts[`PORT_ALLOC].readRsp(iid);
        let accessedvec <- accessedPool.readPorts[`PORT_ALLOC].readRsp(iid);
        let validsvec <- validsPool.readPorts[`PORT_ALLOC].readRsp(iid);

        UInt#(TLog#(t_NUM_WAYS)) winner = 0;

        if (findElem(False, validsvec) matches tagged Valid .e)
        begin
            // Pick an invalid entry.
            winner = e;
        end
        else if (findElem(False, accessedvec) matches tagged Valid .e)
        begin
            // Figure out which was (pseudo) LRU. Assert that at least one
            // entry accessed == 0.
            winner = e;
        end
        
        let new_accessed = accessedvec;
        new_accessed[winner] = False;
        accessedPool.write(iid, idx, new_accessed);

        let new_valid = validsvec;
        new_valid[winner] = True;
        validsPool.write(iid, idx, new_valid);

        let new_entry = entryvec;
        new_entry[winner]= entry;
        tagStoreBanks.write(iid, idx, new_entry);

        debugLog.record_simple_ctx(iid, $format("Alloc idx %d: way %d, tag 0x%0h, accessed 0x%0h, valids 0x%0h", idx, winner, entry, pack(new_accessed), pack(new_valid)));
    endrule

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr) if (!allocQ.notEmpty());
        // Look up the index in the tag store.
        let idx = getCacheIndex(addr);
        tagStoreBanks.readPorts[`PORT_LOAD].readReq(iid, idx);
        accessedPool.readPorts[`PORT_LOAD].readReq(iid, idx);
        validsPool.readPorts[`PORT_LOAD].readReq(iid, idx);

        // Pass the request on to the next stage.
        loadLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid) if (!allocQ.notEmpty());
        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        Bit#(t_IDX_SIZE) idx = getCacheIndex(addr);
        
        let entryvec <- tagStoreBanks.readPorts[`PORT_LOAD].readRsp(iid);
        let accessedvec <- accessedPool.readPorts[`PORT_LOAD].readRsp(iid);
        let validsvec <- validsPool.readPorts[`PORT_LOAD].readRsp(iid);

        let entryAddr = entryTagCheck(addr);
             
        Vector#(t_NUM_WAYS, Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) res =
            zipWith(entryAddr, validsvec, entryvec);
  
        if (findIndex(isValid, res) matches tagged Valid .way)
        begin
            // Hit -- update the pseudo-LRU accessed bits.
            let new_accessed = accessedvec;
            new_accessed[way] = True;
            new_accessed = all(id, new_accessed) ? replicate(False) : new_accessed;
            accessedPool.write(iid, idx, new_accessed);

            debugLog.record_simple_ctx(iid, $format("LD lookup HIT PA 0x%0h idx %d: hit way %d, accessed 0x%0h", addr, idx, way, pack(new_accessed)));

            let entry_idx = CACHE_ENTRY_IDX { set: idx, way: pack(way) };
            return tagged Valid
                CACHE_ENTRY { idx: entry_idx,
                              state: res[way] };
        end
        else
        begin
            debugLog.record_simple_ctx(iid, $format("LD lookup MISS PA 0x%0h idx %d", addr, idx));

            return tagged Invalid;
        end
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
    
    method ActionValue#(Maybe#(t_ENTRY)) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        /*
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) accessed = accessedPool.getRAMWithWritePort(iid, `PORT_STORE);
        LUTRAM#(Bit#(t_IDX_SIZE), Vector#(t_NUM_WAYS, Bool)) valids   = validsPool.getRAM(iid);

        let addr = storeLookupQ.first();
        storeLookupQ.deq();
        let idx = getCacheIndex(addr);

        Maybe#(t_ENTRY) res = tagged Invalid;
        let validsvec = valids.sub(idx);

        Maybe#(Bit#(TLog#(t_NUM_WAYS))) winner = tagged Invalid;

        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            let entry <- tagStoreBanks[x].readPorts[`PORT_STORE].readRsp(iid);
        
            if (entryTagCheck(addr, validsvec[x], entry) matches tagged Valid .entry2)
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

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr) if (!allocQ.notEmpty());
        // Look up the index in the tag store.
        let idx = getCacheIndex(addr);
        tagStoreBanks.readPorts[`PORT_EVICT].readReq(iid, idx);
        accessedPool.readPorts[`PORT_EVICT].readReq(iid, idx);
        validsPool.readPorts[`PORT_EVICT].readReq(iid, idx);

        // Pass the request on to the next stage.
        evictionQ.enq(idx);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Bit#(t_IDX_SIZE) idx = evictionQ.first();
        evictionQ.deq();

        // Since we're a set associative cache we need to figure out which bank
        // to insert into for this address.  This is where the accessing
        // psuedo-LRU scheme comes in.
       
        let entryvec <- tagStoreBanks.readPorts[`PORT_EVICT].readRsp(iid);
        let accessedvec <- accessedPool.readPorts[`PORT_EVICT].readRsp(iid);
        let validsvec <- validsPool.readPorts[`PORT_EVICT].readRsp(iid);
        
        Bool allValid = True;

        for (Integer x = 0; x < numWays; x = x + 1)
        begin
            allValid = allValid && validsvec[x];
        end

        if (!allValid)
        begin
            // There's an unused way, so no one will be evicted.
            debugLog.record_simple_ctx(iid, $format("EVICT NONE idx %d: accessed 0x%0h, valids 0x%0h", idx, pack(accessedvec), pack(validsvec)));
            return tagged Invalid;
        end
        else
        begin
            // Everyone's valid, so figure out which was (pseudo) LRU.
            let lru = 0;
            if (findElem(False, accessedvec) matches tagged Valid .x)
            begin
                lru = x;
            end

            debugLog.record_simple_ctx(iid, $format("EVICT LRU idx %d: way %d, tag 0x%0h, accessed 0x%0h, valids 0x%0h", idx, lru, entryvec[lru], pack(accessedvec), pack(validsvec)));

            let entry_idx = CACHE_ENTRY_IDX { set: idx, way: pack(lru) };
            return tagged Valid
                CACHE_ENTRY { idx: entry_idx,
                              state: tagged Valid toCacheEntryState(entryvec[lru], idx) };
        end
    endmethod

    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE opaque);
        let entry = dirty ? initInternalCacheEntryDirty(addr) : initInternalCacheEntryClean(addr);
        entry.opaque = opaque;
        let idx = getCacheIndex(addr);
        tagStoreBanks.readPorts[`PORT_ALLOC].readReq(iid, idx);
        accessedPool.readPorts[`PORT_ALLOC].readReq(iid, idx);
        validsPool.readPorts[`PORT_ALLOC].readReq(iid, idx);
        allocQ.enq(tuple3(iid, idx, entry));
    endmethod

endmodule
