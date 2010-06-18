

module [HASIM_MODULE] mkCacheAlgAlwaysHit
    // interface:
        (CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SIZE));

    let buffering = 2;

    FIFO#(LINE_ADDRESS) loadLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) storeLookupQ <- mkSizedFIFO(buffering);
    FIFO#(LINE_ADDRESS) evictionQ <- mkSizedFIFO(buffering);

    method Action loadLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    
        loadLookupQ.enq(addr);
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) loadLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        let addr = loadLookupQ.first();
        loadLookupQ.deq();
        
        // Always hit.
        return tagged Valid initCacheEntryClean(addr);
    
    endmethod
    
    method Action storeLookupReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    
        storeLookupQ.enq(addr);
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) storeLookupRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        let addr = storeLookupQ.first();
        storeLookupQ.deq();
        
        return tagged Valid initCacheEntryClean(addr);
    
    endmethod

    method Action evictionCheckReq(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr);
    
        evictionQ.enq(?);
    
    endmethod
    
    method ActionValue#(Maybe#(CACHE_ENTRY#(t_OPAQUE))) evictionCheckRsp(INSTANCE_ID#(t_NUM_INSTANCES) iid);
    
        evictionQ.deq();
        return tagged Invalid;

    endmethod
    
    method Action allocate(INSTANCE_ID#(t_NUM_INSTANCES) iid, LINE_ADDRESS addr, Bool dirty, t_OPAQUE sc);
    
        noAction;
    
    endmethod
    
endmodule
