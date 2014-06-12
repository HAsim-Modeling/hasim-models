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

// ******* Library Imports *******

import Vector::*;
import DefaultValue::*;


// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/fpga_components.bsh"

// ******* HAsim Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/hasim_modellib.bsh"

`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/funcp_memstate_base_types.bsh"
`include "awb/provides/funcp_interface.bsh"

// ******* Timing Model Imports *******

`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/l1_cache_base_types.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_l1_icache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"

// ****** Local Definitions *******

typedef union tagged
{
    L1_ICACHE_MISS_TOKEN MULTI_LOAD;
    CACHE_PROTOCOL_MSG   FILL_RSP;
    ICACHE_INPUT         LOAD_REQ;
    void Invalid;
}
L1I_OPER
    deriving (Eq, Bits);

// newL1ICOper --
//   The compiler sometimes fails to infer the type when initialized.  This
//   function makes types clear.
function L1I_OPER newL1ICOper(L1I_OPER oper) = oper;


// IC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Maybe#(L1_ICACHE_MISS_TOKEN) missTokToFree;

    Bool toL2QNotFull;
    Bool toL2QUsed;
    CACHE_PROTOCOL_MSG toL2QData;

    Bool writePortUsed;
    L1_ICACHE_IDX writePortIdx;
    LINE_ADDRESS writePortData;

    Maybe#(ICACHE_OUTPUT_IMMEDIATE) loadRspImm;
    Maybe#(ICACHE_OUTPUT_DELAYED) loadRsp;
}
IC_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(IC_LOCAL_STATE);
    defaultValue = IC_LOCAL_STATE { 
        missTokToFree: tagged Invalid,
        toL2QNotFull: True,
        toL2QUsed: False,
        toL2QData: ?,
        writePortUsed: False,
        writePortIdx: ?,
        writePortData: ?,
        loadRspImm: tagged Invalid,
        loadRsp: tagged Invalid
        };
endinstance



// mkL11Cache

// A model of a straightforward L1 ICache.
// There is no victim buffer.
//
// Note that the module itself is implmented as a pipeline, though the target
// model carries out all actions in one model cycle.

module [HASIM_MODULE] mkL1ICache ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_instruction.out");

 
    // ****** Submodules ******

    // The cache algorithm which determines hits, misses, and evictions.
    L1_ICACHE_ALG#(MAX_NUM_CPUS, void) iCacheAlg <-
        mkL1ICacheAlg(constFn(True));

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, ICACHE_MISS_ID_SIZE) outstandingMisses <- mkCoalescingCacheMissTracker();


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_ICache_load", 0);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_immediate");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_delayed");

    // Queues to and from the memory hierarchy, encapsulated as StallPorts.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqToMemQ <- mkPortStallSend_Multiplexed("L1_ICache_OutQ_0");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) fillFromMemory <- mkPortStallRecv_Multiplexed("L1_ICache_InQ_0");


    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = fillFromMemory.ctrl.in;
    inports[2] = reqToMemQ.ctrl.in;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = reqToMemQ.ctrl.out;
    outports[3] = fillFromMemory.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L1 ICache", inports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple3#(L1I_OPER,
                                            IC_LOCAL_STATE,
                                            Bool)) stage4Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage5Ctrl <-
        mkStageController();


    // A RAM to record the future homes of outstanding fills.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS,
                            L1_ICACHE_MISS_ID,
                            L1_ICACHE_IDX) fillIdxPool <-
        mkMemory_Multiplexed(mkBRAM);

    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_HIT",
                                           "L1 ICache Controller Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_MISS",
                                           "L1 ICache Controller Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_RETRY",
                                           "L1 ICache Controller Read Retries"));

    // ****** Assertions ******

    let assertRspOk <- mkAssertionSimOnly("l1-instruction-cache.bsv: Unexpected response kind",
                                          ASSERT_ERROR);


    // ****** Rules ******

    //
    // canReplaceLine --
    //   Is the cache in a state that permits a line to be replaced to service
    //   a miss?
    //
    function Bool canReplaceLine(L1_ICACHE_LOOKUP_RSP#(void) rsp);
        if (rsp.state matches tagged Blocked)
            return False;
        else
            return True;
    endfunction


    // stage1_fill
    
    // See if there are any new fill responses from memory.

    // Ports read:
    // * fillFromMemory
    
    // Ports written:
    // * loadRspDelayedtoCPU
    
    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        IC_LOCAL_STATE local_state = defaultValue;

        // Check if the toL2Q has room for any new requests.
        let toL2Q_not_full <- reqToMemQ.canEnq(cpu_iid);
        local_state.toL2QNotFull = toL2Q_not_full;

        // Consume incoming messages
        let m_fill <- fillFromMemory.receive(cpu_iid);
        let m_cpu_req_load <- loadReqFromCPU.receive(cpu_iid);

        //
        // Pick an action for the model cycle.
        //

        L1I_OPER oper = tagged Invalid;

        // Are any previously returned fills going to multiple loads?
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin
            oper = tagged MULTI_LOAD miss_tok;
            debugLog.record(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
        end
        else if (m_fill matches tagged Valid .fill)
        begin
            assertRspOk(cacheMsg_IsRspLoad(fill));

            oper = tagged FILL_RSP fill;

            L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
            debugLog.record(cpu_iid, $format("1: FILL RSP: idx %0d, line 0x%h", miss_tok.index, fill.linePAddr));
        end
        else if (m_cpu_req_load matches tagged Valid .req)
        begin
            oper = tagged LOAD_REQ req;

            let line_addr = toLineAddress(req.physicalAddress);
            debugLog.record(cpu_iid, $format("1: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        // If there is a load request then some immediate response
        // is required.  Initialize a retry response by default.
        if (m_cpu_req_load matches tagged Valid .req)
        begin
            local_state.loadRspImm = tagged Valid initICacheRetry(req);
        end

        stage2Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule

    rule stage2 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                // A fill that came in previously is going to multiple miss tokens.
                local_state.missTokToFree = tagged Valid miss_tok;

                // Now send the fill to CPU
                debugLog.record(cpu_iid, $format("2: FILL MULTIPLE RSP LOAD: %0d", miss_tok.index));
                local_state.loadRsp = tagged Valid initICacheMissRsp(miss_tok.index);
            end

            tagged FILL_RSP .fill:
            begin
                // Record fill meta data that will be written to the cache.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.linePAddr;

                // Deallocate the Miss ID.
                L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = tagged Valid miss_tok;

                // Now send the fill to the CPU
                debugLog.record(cpu_iid, $format("2: MEM RSP LOAD: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
                local_state.loadRsp = tagged Valid initICacheMissRsp(miss_tok.index);

                // Pick the victim
                iCacheAlg.lookupByAddrReq(cpu_iid,
                                          fill.linePAddr,
                                          False,
                                          True);
            end

            tagged LOAD_REQ .req:
            begin
                let line_addr = toLineAddress(req.physicalAddress);

                debugLog.record(cpu_iid, $format("2: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

                // Look up the address in the cache.
                iCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, True);
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("2: Bubble"));
            end
        endcase

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage3 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        Bool new_miss_tok_req = False;

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble MULTI_LOAD"));
            end

            tagged FILL_RSP .fill:
            begin
                //
                // Does the cache entry being replaced require a writeback?
                //
                let evict <- iCacheAlg.lookupByAddrRsp(cpu_iid);

                local_state.writePortIdx = evict.idx;

                if (evict.state matches tagged Blocked)
                begin
                    // No available victim due to temporary state of the old
                    // line.  Wait for it to settle.
                    oper = newL1ICOper(tagged Invalid);
                    local_state.writePortUsed = False;
                    local_state.missTokToFree = tagged Invalid;

                    debugLog.record(cpu_iid, $format("3: BLOCKED EVICTION: new line 0x%h", fill.linePAddr));
                end
                else
                begin
                    outstandingMisses.reportLoadDone(cpu_iid, fill.linePAddr);
                    debugLog.record(cpu_iid, $format("3: CLEAN EVICTION: line 0x%h", fill.linePAddr));
                end
            end

            tagged LOAD_REQ .req:
            begin
                //
                // Did the load hit in the cache?
                //
                let entry <- iCacheAlg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // A hit, so give the data back.
                    local_state.loadRspImm = tagged Valid initICacheHit(req);
                    statHit.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD HIT: line 0x%h", state.linePAddr));
                end
                else
                begin
                    // A miss.
                    let line_addr = toLineAddress(req.physicalAddress);

                    // Is there space in the miss tracker for a new L2 request?
                    let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);

                    if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) &&
                        can_allocate)
                    begin
                        // A fill of this address is already in flight.  Latch
                        // on to it and don't generate a new request.
                        new_miss_tok_req = True;
                        outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                        statMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS (ALREADY OUTSTANDING): line 0x%h", line_addr));
                    end
                    else if (can_allocate && local_state.toL2QNotFull)
                    begin
                        // Allocate the next miss ID
                        new_miss_tok_req = True;
                        outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                        // Record that we are using the memory queue.
                        local_state.toL2QUsed = True;

                        statMiss.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS: line 0x%h", line_addr));
                    end
                    else
                    begin
                        // Miss, but miss tracker is full or L2 is busy.
                        local_state.loadRspImm = tagged Valid initICacheRetry(req);

                        debugLog.record(cpu_iid, $format("3: LOAD RETRY: line 0x%h, can alloc %0d, l2 notFull %0d", line_addr, can_allocate, local_state.toL2QNotFull));
                    end
                end
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble"));
            end
        endcase

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage4 (True);
        match {.cpu_iid, {.oper,
                          .local_state,
                          .new_miss_tok_req}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble MULTI_LOAD"));
            end

            tagged FILL_RSP .fill:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble FILL"));
            end

            tagged LOAD_REQ .req:
            begin
                if (new_miss_tok_req)
                begin
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);
                    local_state.loadRspImm = tagged Valid initICacheMiss(req, miss_tok.index);

                    if (! local_state.toL2QUsed)
                    begin
                        debugLog.record(cpu_iid, $format("4: LOAD MISS (ALREADY OUTSTANDING): %0d", miss_tok.index));
                    end
                    else
                    begin
                        // Use the opaque bits to store the miss token.
                        let line_addr = toLineAddress(req.physicalAddress);
                        let l2_req = cacheMsg_ReqLoad(line_addr,
                                                      toMemOpaque(miss_tok));
                        local_state.toL2QData = l2_req;

                        debugLog.record(cpu_iid, $format("4: LOAD MISS: line 0x%h, tok %0d", line_addr, miss_tok.index));
                    end
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("4: Bubble LOAD_REQ"));
                end
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble"));
            end
        endcase


        // Send immediate response
        loadRspImmToCPU.send(cpu_iid, local_state.loadRspImm);

        // Update retry statistics.
        if (local_state.loadRspImm matches tagged Valid .rsp &&&
            rsp.rspType matches tagged ICACHE_retry)
        begin
            statRetry.incr(cpu_iid);
        end


        stage5Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    (* conservative_implicit_conditions *)
    rule stage5 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        loadRspDelToCPU.send(cpu_iid, local_state.loadRsp);

        if (oper matches tagged FILL_RSP .rsp)
        begin
            fillFromMemory.doDeq(cpu_iid);
        end
        else
        begin
            fillFromMemory.noDeq(cpu_iid);
        end

        // Take care of the memory queue.
        if (local_state.toL2QUsed)
        begin
            reqToMemQ.doEnq(cpu_iid, local_state.toL2QData);
        end
        else
        begin
            reqToMemQ.noEnq(cpu_iid);
        end
        
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
            let state = initCacheEntryState(local_state.writePortData,
                                            False,
                                            ?);

            // Fill allocates a new line.
            iCacheAlg.allocate(cpu_iid,
                               local_state.writePortIdx,
                               state,
                               CACHE_ALLOC_FILL);

            debugLog.record(cpu_iid, $format("5: ALLOC: line 0x%h", state.linePAddr));
        end
        
        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        debugLog.record(cpu_iid, $format("5: Done"));

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
