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

// ****** Bluespec imports ******

import Vector::*;
import FShow::*;
import FIFO::*;


// ****** Project imports ******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/hasim_isa.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/funcp_simulated_memory.bsh"
`include "awb/provides/funcp_interface.bsh"


// ****** Timing Model imports ******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/pipeline_base_types.bsh"
`include "awb/provides/l1_cache_base_types.bsh"



typedef Bit#(TLog#(`WB_NUM_ENTRIES)) WB_INDEX;

// mkWriteBuffer

// A write buffer which commits store to the DCache.

// If the store attempt misses then this blocks until the miss comes back and then performs the write.
// This plays nicely with cache coherence protocols.

typedef struct
{
    Vector#(`WB_NUM_ENTRIES, Maybe#(WB_ENTRY)) buff;
    WB_INDEX head;
    WB_INDEX tail;
    Bool stalled;
    Bool didCreditInit;
}
WRITE_BUFF_STATE deriving (Eq, Bits);

WRITE_BUFF_STATE initWriteBuffState =
    WRITE_BUFF_STATE
    {
        buff: replicate(Invalid),
        head: 0,
        tail: 0,
        stalled: False,
        didCreditInit: False
    };

module [HASIM_MODULE] mkWriteBuffer ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_writebuffer.out");


    // ****** Model State (per instance) ******
    
    MULTIPLEXED_STATE_POOL#(MAX_NUM_CPUS, WRITE_BUFF_STATE) statePool <- mkMultiplexedStatePool(initWriteBuffState);
    
    function Bool empty(WRITE_BUFF_STATE s) = s.head == s.tail;
    function Bool full(WRITE_BUFF_STATE s)  = s.head == (s.tail + 1);

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, WB_ENTRY)      enqFromSB  <- mkPortRecv_Multiplexed("SB_to_WB_enq", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, WB_SEARCH_INPUT) loadReqFromDMem <- mkPortRecv_Multiplexed("DMem_to_WB_search", 0);

    // 
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, WB_INDEX) creditToSB <- mkPortSend_Multiplexed("WB_to_SB_credit");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_INPUT) storeReqToDCache <- mkPortSend_Multiplexed("CPU_to_DCache_store");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, WB_SEARCH_OUTPUT)   rspToDMem     <- mkPortSend_Multiplexed("WB_to_DMem_rsp");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) immediateRspFromDCache <- mkPortRecvDependent_Multiplexed("DCache_to_CPU_store_immediate");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED)   delayedRspFromDCache   <- mkPortRecv_Multiplexed("DCache_to_CPU_store_delayed", 1);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, STORE_TOKEN) writebackToDec <- mkPortSend_Multiplexed("MemWriteBuf_to_Dec_writeback");

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");


    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) depports = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = enqFromSB.ctrl;
    inports[1]  = loadReqFromDMem.ctrl;
    inports[2]  = delayedRspFromDCache.ctrl;
    inports[3]  = statePool.ctrl;
    depports[0] = immediateRspFromDCache.ctrl;
    outports[0] = creditToSB.ctrl;
    outports[1] = storeReqToDCache.ctrl;
    outports[2] = rspToDMem.ctrl;
    outports[3] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("Write Buffer", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, WRITE_BUFF_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, WRITE_BUFF_STATE) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(WRITE_BUFF_STATE, Bool)) stage4Ctrl <- mkBufferedStageController();

    // ****** Assertion ******
    let checkBufNotFull <- mkAssertionStrPvtChecker("write-buffer-blocking.bsv: request received but buffer is full!",
                                                    ASSERT_ERROR);

    // ****** Rules ******


    // stage1_search
    
    (* conservative_implicit_conditions *)
    rule stage1_search (True);

        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        let local_state <- statePool.extractState(cpu_iid);

        // See if the DMem is searching.
        let m_req <- loadReqFromDMem.receive(cpu_iid);

        case (m_req) matches
            tagged Invalid:
            begin
                // Propogate the bubble.
                debugLog.record(cpu_iid, fshow("NO SEARCH"));
                rspToDMem.send(cpu_iid, Invalid);
            end
            tagged Valid .bundle:
            begin
                // Luckily, since we're a simulation, we don't actually 
                // need to retrieve the value, which makes the hardware a LOT simpler
                // as we don't need to get the "youngest store older than this load"
                // Instead, just tell the DMem module that we have the value.

                let target_addr = bundle.physicalAddress;
                Bool hit = False;

                for (Integer x = 0; x < `WB_NUM_ENTRIES; x = x + 1)
                begin
                    // It's a hit if it's a store to the same address. (It must be older than the load.)
                    let addr_match = case (local_state.buff[x]) matches
                                        tagged Valid {.st_tok, .addr}: return addr == target_addr;
                                        tagged Invalid: return False;
                                     endcase;

                    hit = hit || addr_match;
                end

                if (hit)
                begin
                    // We've got that address in the store buffer.
                    debugLog.record(cpu_iid, fshow("LOAD HIT ") + fshow(bundle.token));

                    rspToDMem.send(cpu_iid, tagged Valid initWBHit(bundle));
                end
                else
                begin
                    // We don't have it.
                    debugLog.record(cpu_iid, fshow("LOAD MISS ") + fshow(bundle.token));
                    rspToDMem.send(cpu_iid, tagged Valid initWBMiss(bundle));
                end
            end
        endcase
        
        // Continue to the next stage.
        stage2Ctrl.ready(cpu_iid, local_state);
    endrule

    (* conservative_implicit_conditions *)
    rule stage2_alloc (True);

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        Bool stall_for_store_rsp = False;

        // Check if the store buffer is enq'ing a new write.
        let m_enq <- enqFromSB.receive(cpu_iid);

        if (m_enq matches tagged Valid {.st_tok, .addr})
        begin
            // Allocate a new slot.
            debugLog.record(cpu_iid, fshow("ALLOC ") + fshow(st_tok));
            local_state.buff[local_state.tail] = tagged Valid tuple2(st_tok, addr);

            local_state.tail = local_state.tail + 1;
            checkBufNotFull(local_state.tail != local_state.head);
            if (local_state.tail == local_state.head)
            begin
                debugLog.record(cpu_iid, $format("ALLOC: Request received but buffer is full!"));
            end
        end

        // If we were empty we're done. (The new allocation doesn't count.) 
        // Otherwise the next stage will try to deallocate the oldest write.
        if (empty(local_state) || local_state.stalled)
        begin
            // No request to the DCache.
            storeReqToDCache.send(cpu_iid, tagged Invalid);
        end
        else
        begin
            // Request a store of the oldest write.
            match {.st_tok, .phys_addr} = validValue(local_state.buff[local_state.head]);
            storeReqToDCache.send(cpu_iid, tagged Valid initDCacheStore(phys_addr));
        end

        // Continue to the next stage.
        stage3Ctrl.ready(cpu_iid, local_state);
    endrule


    rule stage3_storeRsp (True);
        // Get our context from the previous stage.
        match {.cpu_iid, .local_state} <- stage3Ctrl.nextReadyInstance();

        let st_tok = tpl_1(validValue(local_state.buff[local_state.head]));
        Bool did_commitStores = False;

        // Get the responses from the DCache.
        let m_imm_rsp <- immediateRspFromDCache.receive(cpu_iid);
        let m_del_rsp <- delayedRspFromDCache.receive(cpu_iid);
        
        if (local_state.stalled &&& m_del_rsp matches tagged Valid .rsp)
        begin
            debugLog.record(cpu_iid, fshow("STORE FILL ") + fshow(st_tok));
            // We're no longer stalled. We'll retry the store next cycle.
            local_state.stalled = False;
        end
        else if (m_imm_rsp matches tagged Valid .rsp)
        begin
            case (rsp) matches
                tagged DCACHE_ok:
                begin
                    debugLog.record(cpu_iid, fshow("STORE OK ") + fshow(st_tok));
                    // Dequeue the buffer.
                    local_state.buff[local_state.head] = tagged Invalid;
                    local_state.head = local_state.head + 1;

                    // Tell the functional partition to commit the store.
                    commitStores.makeReq(initFuncpReqCommitStores(st_tok));
                    did_commitStores = True;
                end

                tagged DCACHE_delay .miss_id:
                begin
                    debugLog.record(cpu_iid, fshow("STORE DELAY"));
                    // Stall on a response
                    local_state.stalled = True;
                end

                tagged DCACHE_retryStore:
                begin
                    debugLog.record(cpu_iid, fshow("STORE RETRY"));
                    // No change. Try again next cycle.
                    noAction;
                end
            endcase
        end

        // Continue to the next stage.
        stage4Ctrl.ready(cpu_iid, tuple2(local_state, did_commitStores));
    endrule


    rule stage4_funcpRsp (True);
        // Get our context from the previous stage.
        match {.cpu_iid, {.local_state, .did_commitStores}} <- stage4Ctrl.nextReadyInstance();

        // Was the functional model told to commit a store that is now in the
        // cache?
        if (did_commitStores)
        begin
            let rsp = commitStores.getResp();
            commitStores.deq();

            writebackToDec.send(cpu_iid, tagged Valid rsp.storeToken);

            // Tell the SB we still have room.
            creditToSB.send(cpu_iid, tagged Valid 1);
            debugLog.record(cpu_iid, $format("1 credit to SB"));
        end
        else
        begin
            writebackToDec.send(cpu_iid, tagged Invalid);

            // First pass must send full buffer credits to the store buffer
            if (! local_state.didCreditInit)
            begin
                WB_INDEX c = fromInteger(valueOf(TSub#(`WB_NUM_ENTRIES, 1)));
                creditToSB.send(cpu_iid, tagged Valid c);
                debugLog.record(cpu_iid, $format("%0d credits to SB", c));

                local_state.didCreditInit = True;
            end
            else
            begin
                creditToSB.send(cpu_iid, tagged Invalid);
                debugLog.record(cpu_iid, $format("No new credit to SB"));
            end
        end

        // End of model cycle. (Path 1)
        statePool.insertState(cpu_iid, local_state);
        debugLog.nextModelCycle(cpu_iid);
        localCtrl.endModelCycle(cpu_iid, 1);
    endrule
endmodule
