
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


// CACHE_MISS_TRACKER

// A structure to handle the allocation and freeing of cache miss tokens.

// Multiplexing is handled internally.

interface CACHE_MISS_TRACKER#(parameter type t_NUM_INSTANCES, parameter type t_MISS_ID_SZ);

    method Bool canAllocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool canAllocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool loadOutstanding(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method Action reportLoadDone(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    
    method Bool noLoadsInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method Bool noStoresInFlight(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok_to_free);
    
    method Bool fillToDeliver(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) freeNextDelivery(INSTANCE_ID#(t_NUM_INSTANCES) iid);

endinterface

// mkCacheMissTracker

// A cache miss tracker which uses a unified freelist for loads
// and stores.

// This could mean that a section of heavy loads cause stores to stall
// and vice versa which might be alleviated by having separate freelists
// for both.

module [HASIM_MODULE] mkCacheMissTracker 
    // interface:
        (CACHE_MISS_TRACKER#(t_NUM_INSTANCES, t_MISS_ID_SZ))
    provisos
        // Perhaps the dumbest proviso ever.
        (Add#(TSub#(t_MISS_ID_SZ, 1), t_TMP, t_MISS_ID_SZ));

    // ******* Model State *******

    // A LUTRAM to store the free miss IDs. 
    // Initially each entry is initialized to be equal to its index.
    MULTIPLEXED_LUTRAM#(t_NUM_INSTANCES, CACHE_MISS_INDEX#(t_MISS_ID_SZ), CACHE_MISS_INDEX#(t_MISS_ID_SZ)) freelist <- mkMultiplexedLUTRAMInitializedWith(id);
    
    // A LUTRAM to store which other miss IDs a fill should be returned to,
    // represented as a linked list.
    MULTIPLEXED_LUTRAM_MULTI_WRITE#(t_NUM_INSTANCES, 2, CACHE_MISS_INDEX#(t_MISS_ID_SZ), Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) multipleFillListPool <- mkMultiplexedLUTRAMMultiWrite(tagged Invalid);
    
    // A register to store the current token that we are returning a fill to, beyond the first.
    MULTIPLEXED_REG#(t_NUM_INSTANCES, Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFillsPool <- mkMultiplexedReg(tagged Invalid);
    
    // A register to store the current token that is the tail of a "run".
    MULTIPLEXED_REG#(t_NUM_INSTANCES, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) runTailPool <- mkMultiplexedReg(0);

    // A register to store the last load we served. No more loads will be made to this address.
    MULTIPLEXED_REG_MULTI_WRITE#(t_NUM_INSTANCES, 2, Maybe#(LINE_ADDRESS)) lastServedAddrPool <- mkMultiplexedRegMultiWrite(tagged Invalid);

    // Track the state of the freelist. Initially the freelist is full and
    // every ID is on the list.
    MULTIPLEXED_REG_MULTI_WRITE#(t_NUM_INSTANCES, 2, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtrPool <- mkMultiplexedRegMultiWrite(minBound);
    MULTIPLEXED_REG#(t_NUM_INSTANCES, CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtrPool <- mkMultiplexedReg(maxBound);
    

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

        Reg#(Maybe#(LINE_ADDRESS)) lastServedAddr = lastServedAddrPool.getRegWithWritePort(iid, 0);
        if (lastServedAddr matches tagged Valid .a)
            return (a == addr);
        else
            return False;

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
        
    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateLoad(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 0);
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) runTail = runTailPool.getReg(iid);
        Reg#(Maybe#(LINE_ADDRESS)) lastServedAddr = lastServedAddrPool.getRegWithWritePort(iid, 0);
        LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) multipleFillList = multipleFillListPool.getRAMWithWritePort(iid, 0);

        let idx = freelist.getRAM(iid).sub(headPtr);

        headPtr <= headPtr + 1;

        if (lastServedAddr matches tagged Valid .a &&& a == addr)
        begin
            multipleFillList.upd(runTail, tagged Valid idx);
        end

        runTail <= idx;
        lastServedAddr <= tagged Valid addr;

        return initMissTokLoad(idx);
    
    endmethod
    
    method Action reportLoadDone(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        
        Reg#(Maybe#(LINE_ADDRESS)) lastServedAddr = lastServedAddrPool.getRegWithWritePort(iid, 1);
        if (lastServedAddr matches tagged Valid .a &&& a == addr)
        begin
            lastServedAddr <= tagged Invalid;
        end

    endmethod

    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) allocateStore(INSTANCE_ID#(t_NUM_INSTANCES) iid);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) headPtr = headPtrPool.getRegWithWritePort(iid, 1);

        let idx = freelist.getRAM(iid).sub(headPtr);

        headPtr <= headPtr + 1;

        return initMissTokStore(idx);

    endmethod


    // free
    //
    // Push the freed ID back onto the freelist.
    // Since we're unified this method is load/store-agnostic.

    method Action free(INSTANCE_ID#(t_NUM_INSTANCES) iid, CACHE_MISS_TOKEN#(t_MISS_ID_SZ) miss_tok);
    
        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);
        Reg#(Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFills = servingMultipleFillsPool.getReg(iid);
        LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) multipleFillList = multipleFillListPool.getRAMWithWritePort(iid, 1);

        freelist.getRAM(iid).upd(tailPtr, missTokIndex(miss_tok));

        tailPtr <= tailPtr + 1;
        
        servingMultipleFills <= multipleFillList.sub(missTokIndex(miss_tok));
        multipleFillList.upd(missTokIndex(miss_tok), tagged Invalid);
    
    endmethod


    method Bool fillToDeliver(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        Reg#(Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFills = servingMultipleFillsPool.getReg(iid);
        return isValid(servingMultipleFills);
    endmethod

    method ActionValue#(CACHE_MISS_TOKEN#(t_MISS_ID_SZ)) freeNextDelivery(INSTANCE_ID#(t_NUM_INSTANCES) iid);

        Reg#(CACHE_MISS_INDEX#(t_MISS_ID_SZ)) tailPtr = tailPtrPool.getReg(iid);
        Reg#(Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) servingMultipleFills = servingMultipleFillsPool.getReg(iid);
        LUTRAM#(CACHE_MISS_INDEX#(t_MISS_ID_SZ), Maybe#(CACHE_MISS_INDEX#(t_MISS_ID_SZ))) multipleFillList = multipleFillListPool.getRAMWithWritePort(iid, 1);


        let cur = validValue(servingMultipleFills);

        freelist.getRAM(iid).upd(tailPtr, cur);
        tailPtr <= tailPtr + 1;        

        servingMultipleFills <= multipleFillList.sub(cur);
        multipleFillList.upd(cur, tagged Invalid);

        return initMissTokLoad(cur);

    endmethod


endmodule

