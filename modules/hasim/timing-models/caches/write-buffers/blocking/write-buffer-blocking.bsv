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
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"


// ****** Timing Model imports ******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"



typedef Bit#(TLog#(`WB_NUM_ENTRIES)) WB_INDEX;

// mkWriteBuffer

// A write buffer which commits store to the DCache.

// If the store attempt misses then this blocks until the miss comes back and then performs the write.
// This plays nicely with cache coherence protocols.

module [HASIM_MODULE] mkWriteBuffer ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_writebuffer.out");


    // ****** Model State (per Context) ******
    
    MULTICTX#(Reg#(Vector#(`WB_NUM_ENTRIES, Maybe#(WB_ENTRY))))    ctx_buff   <- mkMultiCtx(mkReg(replicate(Invalid)));

    MULTICTX#(Reg#(WB_INDEX)) ctx_head <- mkMultiCtx(mkReg(0));
    MULTICTX#(Reg#(WB_INDEX)) ctx_tail <- mkMultiCtx(mkReg(0));

    function Bool empty(CONTEXT_ID ctx) = ctx_head[ctx] == ctx_tail[ctx];
    function Bool full(CONTEXT_ID ctx)  = ctx_head[ctx] == ctx_tail[ctx] + 1;

    MULTICTX#(Reg#(Bool)) ctx_stalled <- mkMultiCtx(mkReg(False));
    
    
    // ****** UnModel Pipeline State ******

    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();
    
    Reg#(Vector#(NUM_CONTEXTS, Bool)) stallForStoreRsp <- mkReg(replicate(False));

    // ****** Ports ******

    PORT_RECV_MULTICTX#(WB_ENTRY)      enqFromSB  <- mkPortRecv_MultiCtx("SB_to_WB_enq", 1);
    PORT_RECV_MULTICTX#(WB_SEARCH_INPUT) loadReqFromDMem <- mkPortRecv_MultiCtx("DMem_to_WB_search", 0);

    PORT_SEND_MULTICTX#(VOID)          creditToSB <- mkPortSend_MultiCtx("WB_to_SB_credit");
    PORT_SEND_MULTICTX#(DCACHE_STORE_INPUT) storeReqToDCache <- mkPortSend_MultiCtx("CPU_to_DCache_store");
    PORT_SEND_MULTICTX#(WB_SEARCH_OUTPUT)   rspToDMem     <- mkPortSend_MultiCtx("WB_to_DMem_rsp");
    PORT_RECV_MULTICTX#(DCACHE_STORE_OUTPUT_IMMEDIATE) immediateRspFromDCache <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_store_immediate", 0);
    PORT_RECV_MULTICTX#(DCACHE_STORE_OUTPUT_DELAYED)   delayedRspFromDCache   <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_store_delayed", 0);

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0]  = enqFromSB.ctrl;
    inports[1]  = loadReqFromDMem.ctrl;
    outports[0] = creditToSB.ctrl;
    outports[1] = storeReqToDCache.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******


    // stage1_search
    
    rule stage1_search (True);

        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);

        Reg#(Vector#(`WB_NUM_ENTRIES, Maybe#(WB_ENTRY))) buff = ctx_buff[ctx];

        // See if the DMem is searching.
        let m_req <- loadReqFromDMem.receive(ctx);

        case (m_req) matches
            tagged Invalid:
            begin

                // Propogate the bubble.
                debugLog.record_next_cycle(ctx, fshow("NO SEARCH"));
                rspToDMem.send(ctx, Invalid);

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
                    let addr_match = case (buff[x]) matches
                                        tagged Valid {.tok, .addr}: return addr == target_addr;
                                        tagged Invalid: return False;
                                     endcase;

                    hit = hit || addr_match;
                end

                if (hit)
                begin

                    // We've got that address in the store buffer.
                    debugLog.record_next_cycle(ctx, fshow("LOAD HIT ") + fshow(bundle.token));

                    rspToDMem.send(ctx, tagged Valid initWBHit(bundle));

                end
                else
                begin

                    // We don't have it.
                    debugLog.record_next_cycle(ctx, fshow("LOAD MISS ") + fshow(bundle.token));
                    rspToDMem.send(ctx, tagged Valid initWBMiss(bundle));

                end

            end
        endcase
        
        // Continue to the next stage.
        stage2Q.enq(ctx);

    endrule

    rule stage2_alloc (True);
    

        let ctx = stage2Q.first();
        stage2Q.deq();

        // Get our local state based on the current context.
        Reg#(WB_INDEX) tail = ctx_tail[ctx];
        Reg#(Vector#(`WB_NUM_ENTRIES, Maybe#(WB_ENTRY))) buff = ctx_buff[ctx];
        Reg#(WB_INDEX) head = ctx_head[ctx];
        Reg#(Bool) stalled = ctx_stalled[ctx];

        // Check if the store buffer is enq'ing a new write.
        let m_enq <- enqFromSB.receive(ctx);
        
        let new_tail = tail;
        
        if (m_enq matches tagged Valid {.tok, .addr})
        begin
        
            // Allocate a new slot.
            // assert !full(ctx)
            debugLog.record(ctx, fshow("ALLOC ") + fshow(tok));
            buff[tail] <= tagged Valid tuple2(tok, addr);

            new_tail = tail + 1;
            
            // Tell the functional partition to commit the store.
            commitStores.makeReq(initFuncpReqCommitStores(tok));
            stallForStoreRsp[ctx] <= True;
        
        end
        
        // Calculate the credit for the SB.
        if ((new_tail + 1) != head)
        begin

            // Tell the SB we still have room.
            debugLog.record(ctx, fshow("SEND CREDIT"));
            creditToSB.send(ctx, tagged Valid (?));

        end
        else
        begin

            // Tell the SB we're full.
            debugLog.record(ctx, fshow("NO CREDIT"));
            creditToSB.send(ctx, tagged Invalid);
        
        end
        
        
        // Update the tail.        
        tail <= new_tail;
        
        // If we were empty we're done. (The new allocation doesn't count.) 
        // Otherwise the next stage will try to deallocate the oldest write.
        if (empty(ctx) || stalled)
        begin

            // No request to the DCache.
            storeReqToDCache.send(ctx, tagged Invalid);

        end
        else
        begin

            // Request a store of the oldest write.
            match {.tok, .phys_addr} = validValue(buff[head]);
            storeReqToDCache.send(ctx, tagged Valid initDCacheStore(tok, phys_addr));

        end

        // Continue to the next stage.
        stage3Q.enq(ctx);

    endrule

    
    rule stage3_storeRsp (!stallForStoreRsp[stage3Q.first()]);
    
        // Get our context from the previous stage.
        let ctx = stage3Q.first();
        stage3Q.deq();
    
        // Get our local state based on the current context.
        Reg#(WB_INDEX) head = ctx_head[ctx];
        Reg#(Vector#(`WB_NUM_ENTRIES, Maybe#(WB_ENTRY))) buff = ctx_buff[ctx];
        Reg#(Bool) stalled = ctx_stalled[ctx];


        // Get the responses from the DCache.
        let m_imm_rsp <- immediateRspFromDCache.receive(ctx);
        let m_del_rsp <- delayedRspFromDCache.receive(ctx);
        
        if (stalled &&& m_del_rsp matches tagged Valid .rsp)
        begin
        
            debugLog.record(ctx, fshow("STORE FILL"));
            // We're no longer stalled. We'll retry the store next cycle.
            stalled <= False;
        
        end
        else if (m_imm_rsp matches tagged Valid .rsp)
        begin
        
            case (rsp.rspType) matches

                tagged DCACHE_ok:
                begin
                    
                    debugLog.record(ctx, fshow("STORE OK"));
                    // Dequeue the buffer.
                    buff[head] <= tagged Invalid;
                    head <= head + 1;
                    
                end

                tagged DCACHE_delay:
                begin
                    
                    debugLog.record(ctx, fshow("STORE DELAY"));
                    // Stall on a response
                    stalled <= True;
                    
                end

                tagged DCACHE_retryStore:
                begin
                
                    debugLog.record(ctx, fshow("STORE RETRY"));
                    // No change. Try again next cycle.
                    noAction;
                
                end

            endcase
        
        end


        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(ctx, 1);

    endrule
    
    rule storeRsp (True);
    
        let rsp = commitStores.getResp();
        commitStores.deq();
        let tok = rsp.token;
        
        let ctx = tokContextId(tok);
        stallForStoreRsp[ctx] <= False;
    
    endrule

endmodule

