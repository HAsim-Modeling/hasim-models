
// ******* Project Includes *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"

// ******* Timing Model Includes *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/chip_base_types.bsh"

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


// CACHE_MISS_TRACKER

// A structure to handle the allocation and freeing of cache miss tokens.

// Multiplexing is handled internally.

interface CACHE_MISS_TRACKER#(parameter type t_NUM_INSTANCES, parameter type t_MISS_ID_SZ);

    method Bool canAllocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool canAllocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok_to_free);

endinterface

// mkCacheMissTracker

// A cache miss tracker which uses a unified freelist for loads
// and stores.

// This could mean that a section of heavy loads cause stores to stall
// and vice versa which might be alleviated by having separate freelists
// for both.

module [HASIM_MODULE] mkCacheMissTracker 
    // interface:
        (CACHE_MISS_TRACKER#(t_NUM_INSTANCES, t_MISS_ID_SZ));

    // ******* Model State *******

    // A LUTRAM to store the free miss IDs. 
    // Initially each entry is initialized to be equal to its index.
    MULTIPLEXED#(t_NUM_INSTANCES, LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), CACHE_MISS_INDEX#(t_MISS_ID_SZ))) freelist <- mkMultiplexedLUTRAMInitializedWith(id);
    
    // Track the state of the freelist. Initially the freelist is full and
    // every ID is on the list.
    MULTIPLEXED#(t_NUM_INSTANCES, Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) headPtrPool <- mkMultiplexed(mkReg(minBound));
    MULTIPLEXED#(t_NUM_INSTANCES, Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) tailPtrPool <- mkMultiplexed(mkReg(maxBound));
    

    // ******* Local Functions *******
    

    // empty()
    
    // Return true if the freelist for this instance is out of IDs.
    
    function Bool empty(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool[iid];
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool[iid];

        return headPtr == tailPtr;
    
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
    

    // allocateLoad/Store
    //
    // Pop the freelist and return the head, coloring the token as appropriate.
        
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool[iid];

        let idx = freelist[iid].sub(headPtr);

        headPtr <= headPtr + 1;

        return initMissTokLoad(idx);
    
    endmethod

    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool[iid];

        let idx = freelist[iid].sub(headPtr);

        headPtr <= headPtr + 1;

        return initMissTokStore(idx);

    endmethod


    // free
    //
    // Push the freed ID back onto the freelist.
    // Since we're unified this method is load/store-agnostic.

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);
    
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool[iid];

        freelist[iid].upd(tailPtr, missTokIndex(miss_tok));

        tailPtr <= tailPtr + 1;
    
    endmethod

endmodule
