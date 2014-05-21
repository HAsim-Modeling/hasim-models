

module [HASIM_MODULE] mkCacheAlgAlwaysHit
    // interface:
    (CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1))
    provisos
        (Alias#(t_ENTRY, CACHE_ENTRY#(void, 0, 1)));

    let buffering = 2;

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) evictionQ <- mkSizedFIFO(buffering);

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        loadLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        
        // Always hit.
        return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                          state: initCacheEntryClean(addr) };
    endmethod
    
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        storeLookupQ.enq(addr);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        let addr = storeLookupQ.first();
        storeLookupQ.deq();
        
        return tagged Valid CACHE_ENTRY { idx: defaultValue,
                                          state: initCacheEntryClean(addr) };
    endmethod

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
        evictionQ.enq(?);
    endmethod
    
    method ActionValue#(Maybe#(t_ENTRY)) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        evictionQ.deq();
        return tagged Invalid;
    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, void sc);
        noAction;
    endmethod
    
endmodule
