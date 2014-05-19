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
import FIFO::*;

// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"

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


// IC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Bool writePortUsed;
    LINE_ADDRESS writePortData;
    Maybe#(ICACHE_INPUT) loadReq;
    Bool loadBypass;
    
}
IC_LOCAL_STATE deriving (Eq, Bits);


// initLocalState
//
// A fresh local state for the first stage.

function IC_LOCAL_STATE initLocalState();

    return 
        IC_LOCAL_STATE 
        { 
            writePortUsed: False,
            writePortData: 0,
            loadReq: tagged Invalid,
            loadBypass: False
        };

endfunction


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
    CACHE_ALG#(MAX_NUM_CPUS, VOID) iCacheAlg <- mkL1ICacheAlg();

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, ICACHE_MISS_ID_SIZE) outstandingMisses <- mkCoalescingCacheMissTracker();


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_ICache_load", 0);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_immediate");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_delayed");

    // Queues to and from the memory hierarchy, encapsulated as StallPorts.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqToMemQ <- mkPortStallSend_Multiplexed("L1_ICache_OutQ");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) fillFromMemory <- mkPortStallRecv_Multiplexed("L1_ICache_InQ");


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

    STAGE_CONTROLLER#(MAX_NUM_CPUS, IC_LOCAL_STATE) stage2Ctrl <- mkBufferedStageController();

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple4#(IC_LOCAL_STATE,
                                            Maybe#(ICACHE_OUTPUT_IMMEDIATE),
                                            Maybe#(ICACHE_INPUT),
                                            Bool))
       stage3Ctrl <- mkStageController();

    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statHits <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_HIT",
                                           "L1 ICache Controller Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statMisses <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_MISS",
                                           "L1 ICache Controller Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statRetries <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_RETRY",
                                           "L1 ICache Controller Read Retries"));

    // ****** Assertions ******

    let assertRspOk <- mkAssertionSimOnly("l1-instruction-cache.bsv: Unexpected response kind",
                                          ASSERT_ERROR);


    // ****** Rules ******

    // stage1_fill
    
    // See if there are any new fill responses from memory.

    // Ports read:
    // * fillFromMemory
    
    // Ports written:
    // * loadRspDelayedtoCPU
    
    (* conservative_implicit_conditions *)
    rule stage1_fill (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = initLocalState();
        
        // Check for fills.
        let m_fill <- fillFromMemory.receive(cpu_iid);
        
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin
            // A fill that came in previously is going to multiple miss tokens.
            outstandingMisses.free(cpu_iid, miss_tok);
            
            // Return it to the CPU.
            debugLog.record(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
            let rsp = initICacheMissRsp(miss_tok.index);
            loadRspDelToCPU.send(cpu_iid, tagged Valid rsp);
            // We must ignore any fills this cycle.
            fillFromMemory.noDeq(cpu_iid);
        end
        else if (m_fill matches tagged Valid .fill)
        begin
            // Note that the actual cache update will be done later, so that
            // any lookups this model cycle don't see it accidentally.

            assertRspOk(cacheMsg_IsRspLoad(fill));

            local_state.writePortUsed = True;
            local_state.writePortData = fill.linePAddr;

            // Deallocate the Miss ID.
            L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);
            outstandingMisses.free(cpu_iid, miss_tok);
            outstandingMisses.reportLoadDone(cpu_iid, fill.linePAddr);
            

            // Return it to the CPU.
            debugLog.record(cpu_iid, $format("1: MEM RSP: %0d LINE: 0x%h", miss_tok.index, fill.linePAddr));
            let rsp = initICacheMissRsp(miss_tok.index);
            loadRspDelToCPU.send(cpu_iid, tagged Valid rsp);            

            // We finished the fill succesfully, so dequeue it.
            fillFromMemory.doDeq(cpu_iid);
        end
        else
        begin
            // Tell the CPU there's no delayed response.
            debugLog.record(cpu_iid, $format("1: NO RSP"));
            loadRspDelToCPU.send(cpu_iid, tagged Invalid);
            
            // No dequeue.
            fillFromMemory.noDeq(cpu_iid);
        end

        // Now read the load input port.
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);

        // Deal with any load requests.
        if (msg_from_cpu matches tagged Valid .req)
        begin
            let line_addr = toLineAddress(req.physicalAddress);

            // See if it's served by the fill from this cycle
            if (local_state.writePortUsed && line_addr == local_state.writePortData)
            begin
                
                // The fill was for this address, so we'll serve it, bypassing the cache.
                debugLog.record(cpu_iid, $format("1: LOAD BYPASS: 0x%h LINE: 0x%h", req.physicalAddress, line_addr));

                // Pass the bypass to the next stage.
                local_state.loadBypass = True;
                local_state.loadReq = tagged Valid req;
            end
            else
            begin
                // See if the cache algorithm hit or missed.
                iCacheAlg.loadLookupReq(cpu_iid, line_addr);
                debugLog.record(cpu_iid, $format("1: LOAD REQ: 0x%h LINE: 0x%h", req.physicalAddress, line_addr));

                // Finish the request in the next stage.
                local_state.loadReq = tagged Valid req;
            end
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: NO LOAD"));
        end
        
        // Pass our information to the next stage.
        stage2Ctrl.ready(cpu_iid, local_state);
    endrule

    
    // stage2_loadRsp
    
    // Finish up any loads to see if they hit or miss.
    // Begin handling any store requests.
    
    // Ports Read:
    // * storeReqFromCPU

    rule stage2_loadRsp (True);

        // Get the local state from the previous stage.
        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        Maybe#(ICACHE_OUTPUT_IMMEDIATE) load_rsp_imm = tagged Invalid;
        Maybe#(ICACHE_INPUT) new_miss_tok_req = tagged Invalid;
        Bool new_miss_used_memq = False;

        // Check if the memQ has room for any new requests.
        let memQ_not_full <- reqToMemQ.canEnq(cpu_iid);

        // See if we need to finish any load responses
        if (local_state.loadBypass)
        begin
            // A bypass, which is as good as a hit, so give the data back. We won't need the memory queue.
            load_rsp_imm = tagged Valid initICacheHit(validValue(local_state.loadReq));
            statHits.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("2: LOAD HIT (BYPASSED)"));
        end
        else if (local_state.loadReq matches tagged Valid .req)
        begin
            // Get the lookup response.
            let m_entry <- iCacheAlg.loadLookupRsp(cpu_iid);

            // Does the cache contain this addresss?
            if (m_entry matches tagged Valid .entry)
            begin
                // A hit, so give the data back. We won't need the memory queue.
                load_rsp_imm = tagged Valid initICacheHit(req);
                statHits.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("2: LOAD HIT"));
            end
            else
            begin
                // A miss. But is there already an outstanding miss to this address?
                // And do we have a free missID to track the fill with?
                // And is the memQ not full?

                let line_addr = toLineAddress(req.physicalAddress);
                let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);
                
                if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) && can_allocate)
                begin
                    // Allocate the next miss ID and give it back to the CPU.
                    new_miss_tok_req = tagged Valid req;
                    outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                    // Tell the CPU their load missed, but we're handling it.
                    statMisses.incr(cpu_iid);

                    debugLog.record(cpu_iid, $format("2: LOAD MISS (ALREADY OUTSTANDING)"));
                end
                else if (can_allocate && memQ_not_full)
                begin
                    // Allocate the next miss ID and give it back to the CPU.
                    new_miss_tok_req = tagged Valid req;
                    outstandingMisses.allocateLoadReq(cpu_iid, line_addr);
                    
                    // Send the fill request to memory
                    new_miss_used_memq = True;
                    statMisses.incr(cpu_iid);

                    debugLog.record(cpu_iid, $format("2: LOAD MISS"));
                end
                else
                begin
                    // The CPU must retry.
                    load_rsp_imm = tagged Valid initICacheRetry(req);
                    debugLog.record(cpu_iid, $format("2: LOAD MISS RETRY"));

                    // Record the retry.
                    statRetries.incr(cpu_iid);
                end
            end // cache load miss
        end
        else
        begin
            // Propogate the bubble.
            load_rsp_imm = tagged Invalid;
        end
        
        stage3Ctrl.ready(cpu_iid, tuple4(local_state,
                                         load_rsp_imm,
                                         new_miss_tok_req,
                                         new_miss_used_memq));
    endrule


    // Ports Written:
    // * loadRspImmToCPU

    rule stage3_end (True);
        match {.cpu_iid, {.local_state,
                          .load_rsp_imm,
                          .new_miss_tok_req,
                          .new_miss_used_memq}} <- stage3Ctrl.nextReadyInstance();
        
        if (new_miss_tok_req matches tagged Valid .req)
        begin
            let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);

            let line_addr = toLineAddress(req.physicalAddress);

            if (! new_miss_used_memq)
            begin
                // No request to memory.
                reqToMemQ.noEnq(cpu_iid);

                load_rsp_imm = tagged Valid initICacheMiss(req, miss_tok.index);
                debugLog.record(cpu_iid, $format("3: LOAD MISS (ALREADY OUTSTANDING): %0d", miss_tok.index));
            end
            else
            begin
                // Send the fill request to memory, using the opaque bits for the miss id.
                let mem_req = cacheMsg_LoadReq(line_addr, toMemOpaque(miss_tok));
                reqToMemQ.doEnq(cpu_iid, mem_req);

                // Tell the CPU their load missed, but we're handling it.
                load_rsp_imm = tagged Valid initICacheMiss(req, miss_tok.index);
                debugLog.record(cpu_iid, $format("3: LOAD MISS: %0d", miss_tok.index));
            end
        end
        else
        begin
            // No request to memory.
            reqToMemQ.noEnq(cpu_iid);
        end

        loadRspImmToCPU.send(cpu_iid, load_rsp_imm);

        debugLog.record(cpu_iid, $format("3: DONE"));
                    
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
        
            iCacheAlg.allocate(cpu_iid, local_state.writePortData, False, ?);
        
        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 

    endrule

endmodule
