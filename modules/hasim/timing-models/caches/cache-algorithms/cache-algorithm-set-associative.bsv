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

`define PORT_ADDR  0
`define PORT_IDX   1
`define PORT_ALLOC 2

module [HASIM_MODULE] mkCacheAlgSetAssociative#(function Bool mayEvict(t_OPAQUE opaque),
                                                TIMEP_DEBUG_FILE_MULTIPLEXED#(t_NUM_INSTANCES) debugLog,
                                                Integer opaque_name,
                                                Bool storeTagsInScratchpad)
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE, t_SET_SIZE, t_NUM_WAYS))
    provisos
        (Alias#(t_IID, INSTANCE_ID#(t_NUM_INSTANCES)),
         Bits#(t_OPAQUE, t_OPAQUE_SIZE),
         Add#(t_SET_SIZE, t_TAG_SIZE, LINE_ADDRESS_SIZE),
         Alias#(t_INTERNAL_ENTRY, CACHE_ENTRY_STATE_INTERNAL#(t_OPAQUE, t_TAG_SIZE)));

    let assertIdxOk <- mkAssertionSimOnly("cache-algorithm-set-associative.bsv: Incorrect ADDR for IDX!",
                                          ASSERT_ERROR);

    let buffering = valueof(t_NUM_INSTANCES) + 1;
    Integer numWays = valueOf(t_NUM_WAYS);

    FIFO#(Tuple3#(Bit#(t_SET_SIZE), Bit#(t_TAG_SIZE), Bool)) lookupByAddrQ <- mkSizedFIFO(buffering);
    FIFO#(CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS)) lookupByIdxQ <- mkSizedFIFO(buffering);
    FIFOF#(Tuple4#(t_IID,
                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS),
                   t_INTERNAL_ENTRY,
                   Bool)) updateQ <- mkSizedFIFOF(buffering);

    // Store tags in a scratchpad
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_SET_SIZE),
                                       Vector#(t_NUM_WAYS, Maybe#(t_INTERNAL_ENTRY)))
        tagStoreBanks <- (storeTagsInScratchpad ?
                              mkMultiReadScratchpad_Multiplexed(opaque_name, defaultValue) :
                              mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, replicate(tagged Invalid))));
    
    // Smaller cache management meta-data fits in local BRAM
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       3,
                                       Bit#(t_SET_SIZE),
                                       Vector#(t_NUM_WAYS, Bool))
        accessedPool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, replicate(False)));


    //
    // entryTagCheck --
    //   Check whether a single entry's valid bit is set and whether its
    //   address matches a request.
    //
    function Bool entryTagCheck(Bit#(t_TAG_SIZE) targetTag,
                                Maybe#(t_INTERNAL_ENTRY) m_entry);
        if (m_entry matches tagged Valid .entry)
            return (entry.tag == targetTag);
        else
            return False;
    endfunction

    //
    // isLRUCandidate --
    //   Is an entry a candidate for LRU eviction?  To be a candidate, the
    //   pseudo-LRU accessed bit must be False and mayEvict must be True.
    //
    function Bool isLRUCandidate(Bool accessed,
                                 Maybe#(t_INTERNAL_ENTRY) m_entry);
        // Assume the entry is valid.  Invalid entries are already high priority
        // candidates for eviction, and will be used with higher priority than
        // the result of this function.
        let entry = validValue(m_entry);
        return mayEvict(entry.opaque) && ! accessed;
    endfunction


    //
    // finishUpdate --
    //   Complete alloc or update request.
    //
    rule finishUpdate (True);
        match {.iid, .idx, .entry, .is_alloc} = updateQ.first();
        updateQ.deq();

        let entryvec <- tagStoreBanks.readPorts[`PORT_ALLOC].readRsp(iid);
        let accessedvec <- accessedPool.readPorts[`PORT_ALLOC].readRsp(iid);

        entryvec[idx.way] = tagged Valid entry;
        tagStoreBanks.write(iid, idx.set, entryvec);
        
        if (is_alloc)
        begin
            accessedvec[idx.way] = False;
            accessedPool.write(iid, idx.set, accessedvec);
        end

        debugLog.record_simple_ctx(iid, $format("Alloc set %0d: way %0d, tag 0x%0h, accessed 0x%0h", idx.set, idx.way, entry.tag, pack(accessedvec)));
    endrule


    method Action lookupByAddrReq(t_IID iid,
                                  LINE_ADDRESS addr,
                                  Bool updateReplacement,
                                  Bool isLoad);
        // Look up the set in the tag store.
        Bit#(t_SET_SIZE) set = getCacheSet(addr);
        Bit#(t_TAG_SIZE) tag = getCacheTag(addr);

        tagStoreBanks.readPorts[`PORT_ADDR].readReq(iid, set);
        accessedPool.readPorts[`PORT_ADDR].readReq(iid, set);

        debugLog.record_simple_ctx(iid, $format("LD lookup req addr 0x%0h, tag 0x%0h, set %0d", addr, tag, set));

        // Pass the request on to the next stage.
        lookupByAddrQ.enq(tuple3(set, tag, updateReplacement));
    endmethod
    
    method ActionValue#(CACHE_LOOKUP_RSP#(t_OPAQUE,
                                          t_SET_SIZE,
                                          t_NUM_WAYS)) lookupByAddrRsp(t_IID iid);
        match {.set, .target_tag, .update_replacement} = lookupByAddrQ.first();
        lookupByAddrQ.deq();
        
        let entryvec <- tagStoreBanks.readPorts[`PORT_ADDR].readRsp(iid);
        let accessedvec <- accessedPool.readPorts[`PORT_ADDR].readRsp(iid);

        // Does a way match the target?
        let m_match_way = findIndex(entryTagCheck(target_tag), entryvec);
  
        //
        // Pick a victim in case no way matches the target.
        //

        // Generate a vector of only the valid bits for current ways
        let valid_bits = map(isValid, entryvec);

        // Is one of the ways currently invalid?
        let m_invalid_way = findElem(False, valid_bits);

        // Is a pseudo LRU entry still False?
        let lru_candidates = zipWith(isLRUCandidate, accessedvec, entryvec);
        let m_plru = findElem(True, lru_candidates);

        //
        // Compute the response.
        //
        if (m_match_way matches tagged Valid .way)
        begin
            // Hit!

            // Update the pseudo-LRU accessed bits
            let new_accessed = accessedvec;
            new_accessed[way] = True;
            // Reset when all bits are set (this is the "pseudo" part)
            new_accessed = all(id, new_accessed) ? replicate(False) : new_accessed;

            if (update_replacement)
            begin
                accessedPool.write(iid, set, new_accessed);
            end

            let entry_idx = CACHE_ENTRY_IDX { set: set, way: way };
            let state = toCacheEntryState(validValue(entryvec[way]), set);

            debugLog.record_simple_ctx(iid, $format("LD lookup HIT PA 0x%0h set %0d: hit way %0d, accessed 0x%0h", state.linePAddr, set, way, pack(new_accessed)));

            return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged Valid state };
        end
        else if (m_invalid_way matches tagged Valid .way)
        begin
            // Miss and some way is currently invalid.  Indicate it as the way
            // to store the new entry.
            let entry_idx = CACHE_ENTRY_IDX { set: set, way: way };

            debugLog.record_simple_ctx(iid, $format("LD lookup miss tag 0x%0h set %0d: evict invalid way %0d", target_tag, set, way));

            return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged Invalid };
        end
        else if (m_plru matches tagged Valid .way)
        begin
            // Miss.  Indicate the LRU way as a victim.
            let entry_idx = CACHE_ENTRY_IDX { set: set, way: way };
            let state = toCacheEntryState(validValue(entryvec[way]), set);

            debugLog.record_simple_ctx(iid, $format("LD lookup miss tag 0x%0h set %0d: evict LRU way %0d, addr 0x%0h, dirty %0d", target_tag, set, way, state.linePAddr, state.dirty));

            return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged MustEvict state };
        end
        else
        begin
            // Miss and all ways are in transitional states and may not be
            // evicted.
            let entry_idx = CACHE_ENTRY_IDX { set: set, way: ? };

            debugLog.record_simple_ctx(iid, $format("LD lookup miss tag 0x%0h set %0d: BLOCKED", target_tag, set));

            return CACHE_LOOKUP_RSP { idx: entry_idx, state: tagged Blocked };
        end
    endmethod

    
    method Action lookupByIndexReq(t_IID iid,
                                   CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx);
        // Look up the index in the tag store.
        tagStoreBanks.readPorts[`PORT_IDX].readReq(iid, idx.set);

        debugLog.record_simple_ctx(iid, $format("LD lookup req idx set %0d, way %0d", idx.set, idx.way));

        lookupByIdxQ.enq(idx);
    endmethod

    method ActionValue#(Maybe#(CACHE_ENTRY_STATE#(t_OPAQUE))) lookupByIndexRsp(t_IID iid);
        let idx = lookupByIdxQ.first();
        lookupByIdxQ.deq();
        
        let entryvec <- tagStoreBanks.readPorts[`PORT_IDX].readRsp(iid);
        if (entryvec[idx.way] matches tagged Valid .entry)
        begin
            let rsp = toCacheEntryState(entry, idx.set);

            debugLog.record_simple_ctx(iid, $format("LD lookup rsp idx set %0d, way %0d: addr 0x%0h, dirty %0d", idx.set, idx.way, rsp.linePAddr, rsp.dirty));

            return tagged Valid rsp;
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

        tagStoreBanks.readPorts[`PORT_ALLOC].readReq(iid, idx.set);
        accessedPool.readPorts[`PORT_ALLOC].readReq(iid, idx.set);

        updateQ.enq(tuple4(iid, idx, entry, True));
    endmethod

    method Action update(t_IID iid,
                         CACHE_ENTRY_IDX#(t_SET_SIZE, t_NUM_WAYS) idx,
                         CACHE_ENTRY_STATE#(t_OPAQUE) state);
        assertIdxOk(idx.set == getCacheSet(state.linePAddr));

        let entry = initInternalCacheEntryFromState(state);

        tagStoreBanks.readPorts[`PORT_ALLOC].readReq(iid, idx.set);
        accessedPool.readPorts[`PORT_ALLOC].readReq(iid, idx.set);

        updateQ.enq(tuple4(iid, idx, entry, False));
    endmethod
endmodule
