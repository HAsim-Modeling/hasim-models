//
// Copyright (C) 2008 Intel Corporation
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

// ****** Bluespec imports ******

import Vector::*;
import FShow::*;
import FIFO::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"


// ****** Timing Model imports ******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"



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
}
WRITE_BUFF_STATE deriving (Eq, Bits);

WRITE_BUFF_STATE initWriteBuffState =
    WRITE_BUFF_STATE
    {
        buff: replicate(Invalid),
        head: 0,
        tail: 0,
        stalled: False
    };

module [HASIM_MODULE] mkWriteBuffer ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_writebuffer.out");


    // ****** Model State (per instance) ******
    
    MULTIPLEXED_STATE_POOL#(NUM_CPUS, WRITE_BUFF_STATE) statePool <- mkMultiplexedStatePool(initWriteBuffState);
    
    function Bool empty(WRITE_BUFF_STATE s) = s.head == s.tail;
    function Bool full(WRITE_BUFF_STATE s)  = s.head == (s.tail + 1);

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, WB_ENTRY)      enqFromSB  <- mkPortRecv_Multiplexed("SB_to_WB_enq", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, WB_SEARCH_INPUT) loadReqFromDMem <- mkPortRecv_Multiplexed("DMem_to_WB_search", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VOID)          creditToSB <- mkPortSend_Multiplexed("WB_to_SB_credit");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_INPUT) storeReqToDCache <- mkPortSend_Multiplexed("CPU_to_DCache_store");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, WB_SEARCH_OUTPUT)   rspToDMem     <- mkPortSend_Multiplexed("WB_to_DMem_rsp");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) immediateRspFromDCache <- mkPortRecvDependent_Multiplexed("DCache_to_CPU_store_immediate");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED)   delayedRspFromDCache   <- mkPortRecv_Multiplexed("DCache_to_CPU_store_delayed", 1);

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");


    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS)) depports = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = enqFromSB.ctrl;
    inports[1]  = loadReqFromDMem.ctrl;
    inports[2]  = delayedRspFromDCache.ctrl;
    inports[3]  = statePool.ctrl;
    depports[0] = immediateRspFromDCache.ctrl;
    outports[0] = creditToSB.ctrl;
    outports[1] = storeReqToDCache.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalControllerWithUncontrolled(inports, depports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, WRITE_BUFF_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, Tuple2#(WRITE_BUFF_STATE, Bool)) stage3Ctrl <- mkStageController();

    // ****** Rules ******


    // stage1_search
    
    (* conservative_implicit_conditions *)
    rule stage1_search (True);

        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        let local_state <- statePool.extractState(cpu_iid);

        // See if the DMem is searching.
        let m_req <- loadReqFromDMem.receive(cpu_iid);

        case (m_req) matches
            tagged Invalid:
            begin

                // Propogate the bubble.
                debugLog.record_next_cycle(cpu_iid, fshow("NO SEARCH"));
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
                    debugLog.record_next_cycle(cpu_iid, fshow("LOAD HIT ") + fshow(bundle.token));

                    rspToDMem.send(cpu_iid, tagged Valid initWBHit(bundle));

                end
                else
                begin

                    // We don't have it.
                    debugLog.record_next_cycle(cpu_iid, fshow("LOAD MISS ") + fshow(bundle.token));
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
            // assert !full(cpu_iid)
            debugLog.record(cpu_iid, fshow("ALLOC ") + fshow(st_tok));
            local_state.buff[local_state.tail] = tagged Valid tuple2(st_tok, addr);

            local_state.tail = local_state.tail + 1;
            
            // Tell the functional partition to commit the store.
            commitStores.makeReq(initFuncpReqCommitStores(st_tok));
            stall_for_store_rsp = True;
        
        end
        
        // Calculate the credit for the SB.
        if ((local_state.tail + 1) != local_state.head)
        begin

            // Tell the SB we still have room.
            debugLog.record(cpu_iid, fshow("SEND CREDIT"));
            creditToSB.send(cpu_iid, tagged Valid (?));

        end
        else
        begin

            // Tell the SB we're full.
            debugLog.record(cpu_iid, fshow("NO CREDIT"));
            creditToSB.send(cpu_iid, tagged Invalid);
        
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
        stage3Ctrl.ready(cpu_iid, tuple2(local_state, stall_for_store_rsp));

    endrule


    rule stage3_storeRsp (True);
    
        // Get our context from the previous stage.
        match {.cpu_iid, {.local_state, .get_rsp}} <- stage3Ctrl.nextReadyInstance();

        if (get_rsp)
        begin
        
            let rsp = commitStores.getResp();
            commitStores.deq();

        end

        // Get the responses from the DCache.
        let m_imm_rsp <- immediateRspFromDCache.receive(cpu_iid);
        let m_del_rsp <- delayedRspFromDCache.receive(cpu_iid);
        
        if (local_state.stalled &&& m_del_rsp matches tagged Valid .rsp)
        begin
        
            debugLog.record(cpu_iid, fshow("STORE FILL"));
            // We're no longer stalled. We'll retry the store next cycle.
            local_state.stalled = False;
        
        end
        else if (m_imm_rsp matches tagged Valid .rsp)
        begin
        
            case (rsp) matches

                tagged DCACHE_ok:
                begin
                    
                    debugLog.record(cpu_iid, fshow("STORE OK"));
                    // Dequeue the buffer.
                    local_state.buff[local_state.head] = tagged Invalid;
                    local_state.head = local_state.head + 1;
                    
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


        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        statePool.insertState(cpu_iid, local_state);

    endrule

endmodule

