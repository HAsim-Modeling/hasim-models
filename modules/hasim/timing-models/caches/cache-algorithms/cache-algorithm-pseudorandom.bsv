import FIFO::*;
import LFSR::*;

module [HASIM_MODULE] mkCacheAlgPseudoRandom
    // parameters:
    #(Bit#(8) loadSeed,
      Bit#(8) storeSeed,
      Bit#(8) evictSeed,
      Bit#(8) loadMissChance,
      Bit#(8) storeMissChance,
      Bit#(8) cleanEvictionChance,
      Bit#(8) dirtyEvictionChance)
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1))
    provisos
        (Alias#(t_ENTRY, CACHE_ENTRY#(void, 0, 1)));

    let buffering = 2;

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) evictionQ <- mkSizedFIFO(buffering);

    MULTIPLEXED#(t_NUM_INSTANCES, LFSR#(Bit#(8))) loadLFSRPool  <- mkMultiplexed(mkLFSR_8());
    MULTIPLEXED#(t_NUM_INSTANCES, LFSR#(Bit#(8))) storeLFSRPool <- mkMultiplexed(mkLFSR_8());
    MULTIPLEXED#(t_NUM_INSTANCES, LFSR#(Bit#(8))) evictLFSRPool <- mkMultiplexed(mkLFSR_8());
    
    Reg#(Bool) initializingLFSRs <- mkReg(False);

    // ****** Rules ******

    // initializeLFSRs
    
    rule initializeLFSRs (initializingLFSRs);
        // Reset all the LFSRs to the seed parameter.
        for (Integer x = 0; x < valueof(t_NUM_INSTANCES); x = x + 1)
        begin
            loadLFSRPool[x].seed(loadSeed);
            storeLFSRPool[x].seed(storeSeed);
            evictLFSRPool[x].seed(evictSeed);
        end

        initializingLFSRs <= False;
    endrule


    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr) if (!initializingLFSRs);
        loadLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let loadLFSR = loadLFSRPool[iid];
    
        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        
        // Use a pseudo-random number to see if we hit or not.
        let rnd = loadLFSR.value();
        loadLFSR.next();

        if (rnd < loadMissChance)
        begin
            // A miss.
            return tagged Invalid;
        end
        else
        begin
            // A hit!
            return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                              state: initCacheEntryClean(addr) };
        end
    endmethod
    
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr) if (!initializingLFSRs);
        storeLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let storeLFSR = storeLFSRPool[iid];
    
        let addr = storeLookupQ.first();
        storeLookupQ.deq();
        
        // Use a pseudo-random number to see if we hit or not.
        let rnd = storeLFSR.value();
        storeLFSR.next();

        if (rnd < storeMissChance)
        begin
            // A miss.
            return tagged Invalid;
        end
        else
        begin
            // A hit!
            return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                              state: initCacheEntryClean(addr) };
        end
    endmethod

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr) if (!initializingLFSRs);
        evictionQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let evictLFSR = evictLFSRPool[iid];

        let addr = evictionQ.first();
        evictionQ.deq();

        let rnd = evictLFSR.value();
        evictLFSR.next();

        if (rnd < cleanEvictionChance)
        begin
            // A clean line to be evicted.
            return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                              state: initCacheEntryClean(addr) };
        end
        else if (rnd < dirtyEvictionChance)
        begin
            // A dirty line to be evicted.
            return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                              state: initCacheEntryDirty(addr) };
        end
        else
        begin
            // No eviction.
            return tagged Invalid;
        end

    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, void scratch);
        noAction;
    endmethod
    
endmodule
