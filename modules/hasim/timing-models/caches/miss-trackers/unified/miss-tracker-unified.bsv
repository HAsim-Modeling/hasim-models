//
// Copyright (C) 2011 Intel Corporation
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

import FIFO::*;
import FIFOF::*;

// ******* Project Includes *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"

// ******* Timing Model Includes *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

// ******* Local Includes *******

`include "miss-tracker-unified.bsh"


// CACHE_MISS_INDEX

// A sized index to track a cache miss.

typedef Bit#(t_MISS_ID_SZ) CACHE_MISS_INDEX#(parameter numeric type t_MISS_ID_SZ);


// CACHE_MISS_TOKEN

// A token is a cache index and some other internal values.

typedef struct
{
    CACHE_MISS_INDEX#(t_MISS_ID_SZ) index;
    Bool isStore;
}
CACHE_MISS_TOKEN#(parameter numeric type t_MISS_ID_SZ) deriving (Eq, Bits);


// initMissTokLoad

function CACHE_MISS_TOKEN#(t_MISS_ID_SZ) initMissTokLoad(CACHE_MISS_INDEX#(t_MISS_ID_SZ) idx);

    return
        CACHE_MISS_TOKEN
        {
            index: idx,
            isStore: False
        };

endfunction


// initMissTokStore

function CACHE_MISS_TOKEN#(t_MISS_ID_SZ) initMissTokStore(CACHE_MISS_INDEX#(t_MISS_ID_SZ) idx);

    return
        CACHE_MISS_TOKEN
        {
            index: idx,
            isStore: True
        };

endfunction


// missTokIsLoad

function Bool missTokIsLoad(CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);

    return !miss_tok.isStore;

endfunction


// missTokIsStore

function Bool missTokIsStore(CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);

    return miss_tok.isStore;

endfunction


// misTokIndex

function CACHE_MISS_INDEX#(t_MISS_ID_SZ) missTokIndex(CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);

    return miss_tok.index;

endfunction

//
// CACHE_MISS_TRACKER --
//
//   A structure to handle the allocation and freeing of cache miss tokens.
//   Multiplexing is handled internally.
//
interface CACHE_MISS_TRACKER#(parameter type t_NUM_INSTANCES, parameter type t_MISS_ID_SZ);
    method Bool canAllocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool canAllocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool loadOutstanding(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method Action reportLoadDone(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    
    method Bool noLoadsInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool noStoresInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
    method Action allocateLoadReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoadRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Action allocateStoreReq(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStoreRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok_to_free);
    
    method Maybe#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) fillToDeliver(INSTANCE_ID#(t_NUM_INSTANCES) iid);
endinterface

//
// mkCoalescingCacheMissTracker --
//
//   A cache miss tracker that uses a unified freelist for loads and stores.
//   This could mean that a section of heavy loads cause stores to stall
//   and vice versa which might be alleviated by having separate freelists
//   for both.
//
//   The coalescing variant of the miss tracker merges consecutive loads from
//   the same address into a single miss token.  This variant is typically
//   used by L1 caches.  Lower level caches assume that coalescing is managed
//   by the L1 and not required, making the code simpler and using less
//   FPGA storage.
//
module [HASIM_MODULE] mkCoalescingCacheMissTracker
    // Interface:
    (CACHE_MISS_TRACKER#(t_NUM_INSTANCES, t_MISS_ID_SZ));

    // ******* Model State *******

    // Free miss IDs. 
    function freeListInitFunc(t_IDX idx) = truncate(idx);
    let freemem <- mkBRAMBufferedPseudoMultiRead(False);
    let freemem_init = mkMultiMemInitializedWith(freemem, freeListInitFunc);
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                                       2,
                                       CACHE_MISS_INDEX#(t_MISS_ID_SZ),
                                       CACHE_MISS_INDEX#(t_MISS_ID_SZ))
        freelist <- mkMemoryMultiRead_Multiplexed(freemem_init);

    // A LUTRAM to store which other miss IDs a fill should be returned to,
    // represented as a linked list.
    MEMORY_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                            CACHE_MISS_INDEX#(t_MISS_ID_SZ),
                            CACHE_MISS_INDEX#(t_MISS_ID_SZ))
        multipleFillListPool <- mkMemory_Multiplexed(mkBRAM());

    MULTIPLEXED_LUTRAM_MULTI_WRITE#(t_NUM_INSTANCES, 2, CACHE_MISS_INDEX#(t_MISS_ID_SZ), Bool) multipleFillListValidsPool <- mkMultiplexedLUTRAMPseudoMultiWrite(False);
    
    // A register to store the current token that we are returning a fill to, beyond the first.
    MULTIPLEXED_REG#(t_NUM_INSTANCES, Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFillsPool <- mkMultiplexedReg(tagged Invalid);
    
    // A register to store the current token that is the tail of a "run".
    MULTIPLEXED_REG#(t_NUM_INSTANCES, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) runTailPool <- mkMultiplexedReg(0);

    // A register to store the last load we served. No more loads will be made to this address.
    MULTIPLEXED_REG#(t_NUM_INSTANCES, LINE_ADDRESS) lastServedAddrPool <- mkMultiplexedReg(?);
    MULTIPLEXED_REG_MULTI_WRITE#(t_NUM_INSTANCES, 2, Bool) lastServedAddrValidPool <- mkMultiplexedRegPseudoMultiWrite(False);

    // Track the state of the freelist. Initially the freelist is full and
    // every ID is on the list.
    MULTIPLEXED_REG_MULTI_WRITE#(t_NUM_INSTANCES, 2, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtrPool <- mkMultiplexedRegPseudoMultiWrite(minBound);
    MULTIPLEXED_REG#(t_NUM_INSTANCES, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtrPool <- mkMultiplexedReg(maxBound);
    
    FIFO#(Bool) loadRespQ <- mkFIFO();


    // ******* Local Functions *******
    

    // empty()
    
    // Return true if the freelist for this instance is out of IDs.
    // Note that we only allocate half of the IDs at a time for a given instance.
    // This acts as an epoch which stops collisions of "deallocate ID, reallocate the same ID"
    // We accomplish this by ignoring the high bit in the equality.
    
    function Bool empty(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);
        
        Bit#(TSub#(t_MISS_ID_SZ,1)) hp = truncate(headPtr);
        Bit#(TSub#(t_MISS_ID_SZ,1)) tp = truncate(tailPtr);

        return hp == tp;
    endfunction
        
    function Bool full(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);

        return (tailPtr + 1) == headPtr;
    endfunction


    // ******* Rules *******

    //
    // updateFillToDeliver --
    //     Deliver multiple fills for a single address?
    //
    FIFOF#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Bool)) updateFillToDeliverQ <- mkFIFOF();

    rule updateFillToDeliver (True);
        match {.iid, .is_valid} = updateFillToDeliverQ.first();
        updateFillToDeliverQ.deq();

        let fill_idx <- multipleFillListPool.readRsp(iid);

        Reg#(Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFills = servingMultipleFillsPool.getReg(iid);
        servingMultipleFills <= is_valid ? tagged Valid fill_idx : tagged Invalid;
    endrule


    // ******* Methods *******
    

    // canAllocateLoad/Store
    //
    // Since we're unified, we return true for these if the freelist is non-empty.
    
    method Bool canAllocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return !empty(iid);
    endmethod
    
    method Bool canAllocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return !empty(iid);
    endmethod
    

    method Bool loadOutstanding(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        Reg#(LINE_ADDRESS) lastServedAddr = lastServedAddrPool.getReg(iid);
        Reg#(Bool)    lastServedAddrValid = lastServedAddrValidPool.getRegWithWritePort(iid, 0);
        
        return (lastServedAddrValid) ? (lastServedAddr == addr) : False;
    endmethod
    

    method Bool noLoadsInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return full(iid);
    endmethod
    
    method Bool noStoresInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return full(iid);
    endmethod
    

    // allocateLoad/Store
    //
    // Pop the freelist and return the head, coloring the token as appropriate.
        
    method Action allocateLoadReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(LINE_ADDRESS) lastServedAddr = lastServedAddrPool.getReg(iid);
        Reg#(Bool) lastServedAddrValid = lastServedAddrValidPool.getRegWithWritePort(iid, 0);

        freelist.readPorts[0].readReq(iid, headPtr);

        headPtr <= headPtr + 1;

        loadRespQ.enq(lastServedAddrValid && lastServedAddr == addr);

        lastServedAddr <= addr;
        lastServedAddrValid <= True;
    endmethod
    
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoadRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let matched_last_addr = loadRespQ.first();
        loadRespQ.deq();

        let idx <- freelist.readPorts[0].readRsp(iid);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) runTail = runTailPool.getReg(iid);
        LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), Bool) multipleFillListValids = multipleFillListValidsPool.getRAMWithWritePort(iid, 0);

        if (matched_last_addr)
        begin
            multipleFillListPool.write(iid, runTail, idx);
            multipleFillListValids.upd(runTail, True);
        end

        runTail <= idx;

        return initMissTokLoad(idx);
    endmethod

    method Action reportLoadDone(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        Reg#(LINE_ADDRESS) lastServedAddr = lastServedAddrPool.getReg(iid);
        Reg#(Bool) lastServedAddrValid = lastServedAddrValidPool.getRegWithWritePort(iid, 1);

        if (lastServedAddrValid && lastServedAddr == addr)
        begin
            lastServedAddrValid <= False;
        end
    endmethod

    method Action allocateStoreReq(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 1);

        freelist.readPorts[1].readReq(iid, headPtr);

        headPtr <= headPtr + 1;
    endmethod

    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStoreRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let idx <- freelist.readPorts[1].readRsp(iid);

        return initMissTokStore(idx);
    endmethod

    // free
    //
    // Push the freed ID back onto the freelist.
    // Since we're unified this method is load/store-agnostic.

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);
        LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), Bool) multipleFillListValids = multipleFillListValidsPool.getRAMWithWritePort(iid, 1);

        let miss_idx = missTokIndex(miss_tok);
        freelist.write(iid, tailPtr, miss_idx);

        tailPtr <= tailPtr + 1;
        
        multipleFillListPool.readReq(iid, miss_idx);
        updateFillToDeliverQ.enq(tuple2(iid,
                                        multipleFillListValids.sub(miss_idx)));

        multipleFillListValids.upd(miss_idx, False);
    endmethod

    method Maybe#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) fillToDeliver(INSTANCE_ID#(t_NUM_INSTANCES) iid) if (! updateFillToDeliverQ.notEmpty());
        Reg#(Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFills = servingMultipleFillsPool.getReg(iid);
        if (servingMultipleFills matches tagged Valid .idx)
        begin
            return tagged Valid initMissTokLoad(idx);
        end
        else
        begin
            return tagged Invalid;
        end
    endmethod

endmodule


//
// mkCacheMissTracker --
//
//   Similar to mkCoalescingCacheMissTracker except that consecutive loads
//   from the same address are allocated unique tokens.  Lower cache levels
//   assume that coalescing has already happened in the L1 caches.  Adding
//   logic to merge loads at each level wastes FPGA resources.
//
module [HASIM_MODULE] mkCacheMissTracker
    // Interface:
    (CACHE_MISS_TRACKER#(t_NUM_INSTANCES, t_MISS_ID_SZ));

    // ******* Model State *******

    // Free miss IDs. 
    function freeListInitFunc(t_IDX idx) = truncate(idx);
    MEMORY_IFC_MULTIPLEXED#(t_NUM_INSTANCES,
                            CACHE_MISS_INDEX#(t_MISS_ID_SZ),
                            CACHE_MISS_INDEX#(t_MISS_ID_SZ))
        freelist <- mkMemory_Multiplexed(mkBRAMInitializedWith(freeListInitFunc));

    // A register to store the current token that is the tail of a "run".
    MULTIPLEXED_REG#(t_NUM_INSTANCES,
                     CACHE_MISS_INDEX#(t_MISS_ID_SZ)) runTailPool <-
        mkMultiplexedReg(0);

    // Track the state of the freelist. Initially the freelist is full and
    // every ID is on the list.
    MULTIPLEXED_REG_MULTI_WRITE#(t_NUM_INSTANCES, 2,
                                 CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtrPool <-
        mkMultiplexedRegPseudoMultiWrite(minBound);

    MULTIPLEXED_REG#(t_NUM_INSTANCES,
                     CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtrPool <-
        mkMultiplexedReg(maxBound);


    // ******* Local Functions *******
    

    // empty()
    
    // Return true if the freelist for this instance is out of IDs.
    // Note that we only allocate half of the IDs at a time for a given
    // instance.  This acts as an epoch which stops collisions of
    // "deallocate ID, reallocate the same ID."  We accomplish this by
    // ignoring the high bit in the equality.
    
    function Bool empty(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr =
            headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);
        
        Bit#(TSub#(t_MISS_ID_SZ,1)) hp = truncate(headPtr);
        Bit#(TSub#(t_MISS_ID_SZ,1)) tp = truncate(tailPtr);

        return hp == tp;
    endfunction
        
    function Bool full(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr =
            headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);

        return (tailPtr + 1) == headPtr;
    endfunction


    // ******* Methods *******
    

    // canAllocateLoad/Store
    //
    // Since we're unified, we return true for these if the freelist is non-empty.
    
    method Bool canAllocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return !empty(iid);
    endmethod
    
    method Bool canAllocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return !empty(iid);
    endmethod
    

    method Bool loadOutstanding(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        return False;
    endmethod
    

    method Bool noLoadsInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return full(iid);
    endmethod
    
    method Bool noStoresInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return full(iid);
    endmethod
    

    //
    // allocateLoad/Store
    //
    // Pop the freelist and return the head, coloring the token as appropriate.
    // Load and store use identical algorithms since this version of the tracker
    // does not coalesce consecutive loads to the same address.
    //

    method Action allocateLoadReq(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                                  LINE_ADDRESS addr);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr =
            headPtrPool.getRegWithWritePort(iid, 0);
        freelist.readReq(iid, headPtr);
        headPtr <= headPtr + 1;
    endmethod
    
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoadRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let idx <- freelist.readRsp(iid);
        return initMissTokLoad(idx);
    endmethod

    method Action reportLoadDone(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                                 LINE_ADDRESS addr);
        noAction;
    endmethod

    method Action allocateStoreReq(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr =
            headPtrPool.getRegWithWritePort(iid, 1);
        freelist.readReq(iid, headPtr);
        headPtr <= headPtr + 1;
    endmethod

    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStoreRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let idx <- freelist.readRsp(iid);
        return initMissTokStore(idx);
    endmethod


    // free
    //
    // Push the freed ID back onto the freelist.
    // Since we're unified this method is load/store-agnostic.
    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid,
                       CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);

        let miss_idx = missTokIndex(miss_tok);
        freelist.write(iid, tailPtr, miss_idx);

        tailPtr <= tailPtr + 1;
    endmethod

    method Maybe#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) fillToDeliver(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        return tagged Invalid;
    endmethod

endmodule
