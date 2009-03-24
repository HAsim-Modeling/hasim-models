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
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model Imports ******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"


typedef Bit#(TLog#(`SB_NUM_ENTRIES)) SB_INDEX;

// mkStoreBuffer

// A simple head/tail circular buffer store buffer.

// This uses an associative memory. Therefore it is best for small sizes. 
// Larger sizes would want to use BRAM or LUTRAM and sequentially search the RAMs.

// This module is pipelined across contexts. Stages:
// Stage 1 -> Stage 2
// These stages will never stall.

// There is only one way that a model cycle can end.


module [HASIM_MODULE] mkStoreBuffer ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_storebuffer.out");


    // ****** Model State (per Context) ******
    
    MULTICTX#(Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))))       ctx_tokID       <- mkMultiCtx(mkReg(replicate(Invalid)));
    MULTICTX#(Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(ISA_ADDRESS)))) ctx_physAddress <- mkMultiCtx(mkReg(replicate(Invalid)));

    MULTICTX#(Reg#(SB_INDEX)) ctx_oldestCommitted   <- mkMultiCtx(mkReg(0));
    MULTICTX#(Reg#(SB_INDEX)) ctx_numToCommit       <- mkMultiCtx(mkReg(0));
    MULTICTX#(Reg#(SB_INDEX)) ctx_nextFreeSlot      <- mkMultiCtx(mkReg(0));

    function Bool empty(CONTEXT_ID ctx) = ctx_nextFreeSlot[ctx] == ctx_oldestCommitted[ctx];
    function Bool full(CONTEXT_ID ctx)  = ctx_oldestCommitted[ctx] == ctx_nextFreeSlot[ctx] + 1;


    // ****** UnModel Pipeline State ******

    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage4Q <- mkFIFO();
    
    Reg#(Vector#(NUM_CONTEXTS, Bool)) stallForStoreRsp <- mkReg(replicate(False));

    // ****** Ports ******

    PORT_RECV_MULTICTX#(TOKEN)             allocFromDec    <- mkPortRecv_MultiCtx("Dec_to_SB_alloc", 1);
    PORT_RECV_MULTICTX#(SB_INPUT)          reqFromDMem     <- mkPortRecv_MultiCtx("DMem_to_SB_req", 0);
    PORT_RECV_MULTICTX#(SB_DEALLOC_INPUT)  deallocFromCom  <- mkPortRecv_MultiCtx("Com_to_SB_dealloc", 1);
    PORT_RECV_MULTICTX#(VOID)            creditFromWriteQ  <- mkPortRecv_MultiCtx("WB_to_SB_credit", 1);

    PORT_SEND_MULTICTX#(SB_OUTPUT)      rspToDMem     <- mkPortSend_MultiCtx("SB_to_DMem_rsp");
    PORT_SEND_MULTICTX#(VOID)          creditToDecode <- mkPortSend_MultiCtx("SB_to_Dec_credit");
    PORT_SEND_MULTICTX#(WB_ENTRY)       storeToWriteQ <- mkPortSend_MultiCtx("SB_to_WB_enq");

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_DO_STORES, FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    // ****** Local Controller ******

    Vector#(4, PORT_CONTROLS) inports  = newVector();
    Vector#(3, PORT_CONTROLS) outports = newVector();
    inports[0]  = reqFromDMem.ctrl;
    inports[1]  = allocFromDec.ctrl;
    inports[2]  = deallocFromCom.ctrl;
    inports[3]  = creditFromWriteQ.ctrl;
    outports[0] = rspToDMem.ctrl;
    outports[1] = creditToDecode.ctrl;
    outports[2] = storeToWriteQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    rule stage1_alloc (True);
    
        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);

        // Get our local state based on the current context.
        Reg#(SB_INDEX) nextFreeSlot = ctx_nextFreeSlot[ctx];
        Reg#(SB_INDEX) oldestCommitted = ctx_oldestCommitted[ctx];

        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN)))             tokID = ctx_tokID[ctx];
        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(ISA_ADDRESS))) physAddress = ctx_physAddress[ctx];

        // Check if the decode is allocating a new slot.
        let m_alloc <- allocFromDec.receive(ctx);
        
        let new_free = nextFreeSlot;
        
        if (m_alloc matches tagged Valid .tok)
        begin
        
            // Allocate a new slot.
            // assert !full(ctx)
                        debugLog.record(ctx, fshow("ALLOC ") + fshow(tok));
            tokID[nextFreeSlot] <= tagged Valid tok;

            // We don't know its effective address yet.
            physAddress[nextFreeSlot] <= tagged Invalid;

            new_free = new_free + 1;
        
        end
        
        // Calculate the credit for decode.
        if (((new_free + 2) == oldestCommitted) || ((new_free + 1) == oldestCommitted)) // Plus 2 because we assume it takes a cycle for the credit to arrive. Should really be +L+1 where L is latency of credit.
        begin

            // Tell decode we're full.
            debugLog.record_next_cycle(ctx, fshow("NO CREDIT"));
            creditToDecode.send(ctx, tagged Invalid);

        end
        else
        begin

            // Tell decode still have room.
            debugLog.record_next_cycle(ctx, fshow("SEND CREDIT"));
            creditToDecode.send(ctx, tagged Valid (?));
        
        end
        
        
        // Update the tail.        
        nextFreeSlot <= new_free;


        // Continue to the next stage.
        stage2Q.enq(ctx);

    endrule


    // stage2_search
    
    rule stage2_search (True);

        let ctx = stage2Q.first();
        stage2Q.deq();

        // Get our local state based on the current context.
        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN)))             tokID = ctx_tokID[ctx];
        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(ISA_ADDRESS))) physAddress = ctx_physAddress[ctx];

        // See if the DMem is completing or searching.
        let m_req <- reqFromDMem.receive(ctx);

        case (m_req) matches
            tagged Invalid:
            begin

                // Propogate the bubble.
                debugLog.record(ctx, fshow("NO SEARCH"));
                rspToDMem.send(ctx, Invalid);

            end
            tagged Valid .req:
            case (req.reqType) matches
                tagged SB_search:
                begin


                    let target_addr = req.bundle.physicalAddress;

                    // Luckily, since we're a simulation, we don't actually 
                    // need to retrieve the value, which makes the hardware a LOT simpler
                    // as we don't need to get the "youngest store older than this load"
                    // Instead, just tell the DMem module that we have the value.
                
                    Bool hit = False;
                
                    for (Integer x = 0; x < `SB_NUM_ENTRIES; x = x + 1)
                    begin

                        // It's a hit if it's a store to the same address which is older than the load.
                        let addr_match = case (physAddress[x]) matches 
                                            tagged Valid .addr: return addr == target_addr;
                                            tagged Invalid: return False;
                                         endcase;

                        let older_store = case (tokID[x]) matches 
                                                tagged Valid .tok: return tokenIsOlderOrEq(tok.index, req.bundle.token.index);
                                                tagged Invalid: return False;
                                            endcase;

                        hit = hit || (addr_match && older_store);
                    end

                    if (hit)
                    begin

                        // We've got that address in the store buffer.
                        debugLog.record(ctx, fshow("LOAD HIT ") + fshow(req.bundle.token));

                        rspToDMem.send(ctx, tagged Valid initSBHit(req.bundle));

                    end
                    else
                    begin

                        // We don't have it.
                        debugLog.record(ctx, fshow("LOAD MISS ") + fshow(req.bundle.token));
                        rspToDMem.send(ctx, tagged Valid initSBMiss(req.bundle));

                    end

                end
                tagged SB_complete:
                begin

                    // A completion of a previously allocated store.
                    debugLog.record(ctx, fshow("COMPLETE STORE ") + fshow(req.bundle.token));

                    // Update with the actual physical address.
                    // (A real store buffer would also record the value.)
                    let tok_id = tokTokenId(req.bundle.token);
                    
                    // We find the index for this token using a CAM Write
                    
                    SB_INDEX sb_idx = 0;
                    
                    for (Integer x = 0; x < `SB_NUM_ENTRIES; x = x + 1)
                    begin
                    
                        if (tokID[x] matches tagged Valid .tok &&& tokTokenId(tok) == tok_id)
                            sb_idx = fromInteger(x);
                    
                    end
                    
                    physAddress[sb_idx] <= tagged Valid req.bundle.physicalAddress;

                    // Tell the functional partition to make the store locally visible.
                    doStores.makeReq(initFuncpReqDoStores(req.bundle.token));

                    // Don't end the model cycle until the store response has come in.
                    stallForStoreRsp[ctx] <= True;

                    // No need for a response.
                    rspToDMem.send(ctx, tagged Invalid);

                end

            endcase

        endcase
        
        // Continue to the next stage.
        stage3Q.enq(ctx);

    endrule
    
    rule stage3_dealloc (True);
    
        // Get our context from the previous stage.
        let ctx = stage3Q.first();
        stage3Q.deq();
    
        // Get our local state based on the current context.
        Reg#(SB_INDEX) nextFreeSlot = ctx_nextFreeSlot[ctx];
        Reg#(SB_INDEX) oldestCommitted = ctx_oldestCommitted[ctx];
        Reg#(SB_INDEX) numToCommit = ctx_numToCommit[ctx];

        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))) tokID = ctx_tokID[ctx];

        // See if we're getting a deallocation request.
        let m_dealloc <- deallocFromCom.receive(ctx);
        
        if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_drop)
        begin

            // Invalidate the requested entry. We assume drop/dealloc requests come in allocation order.
            debugLog.record(ctx, fshow("DROP REQ ") + fshow(req.token));
            tokID[oldestCommitted + numToCommit] <= tagged Invalid;

            // Record that the commit path has work to do.
            numToCommit <= numToCommit + 1;
        
        end
        else if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_writeback)
        begin

            // Update the token with the latest value.
            debugLog.record(ctx, fshow("DEALLOC REQ ") + fshow(req.token));
            tokID[oldestCommitted + numToCommit] <= tagged Valid req.token;

            // Record that the commit path has work to do.
            numToCommit <= numToCommit + 1;
        
        end
        
        // Finish up in the next stage.
        stage4Q.enq(ctx);
        
    endrule
    
    rule stage4_commit (!stallForStoreRsp[stage4Q.first()]);
    
        // Get our context from the previous stage.
        let ctx = stage4Q.first();
        stage4Q.deq();
    
        // Get our local state based on the current context.
        Reg#(SB_INDEX) nextFreeSlot = ctx_nextFreeSlot[ctx];
        Reg#(SB_INDEX) oldestCommitted = ctx_oldestCommitted[ctx];
        Reg#(SB_INDEX) numToCommit = ctx_numToCommit[ctx];

        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN)))       tokID = ctx_tokID[ctx];
        Reg#(Vector#(`SB_NUM_ENTRIES, Maybe#(ISA_ADDRESS))) physAddress = ctx_physAddress[ctx];

        // See if the Write Buffer has room.
        let m_credit <- creditFromWriteQ.receive(ctx);
        let write_buff_has_credit = isValid(m_credit);

        // We need to dealloc if we have pending commmits.
        if (numToCommit != 0)
        begin
            case (tokID[oldestCommitted]) matches
                tagged Invalid:
                begin
                
                    // If the oldest committed token is invalid then it was dropped. Just move over it.
                    debugLog.record(ctx, fshow("JUNK DROPPED"));
                    oldestCommitted <= oldestCommitted + 1;
                    numToCommit <= numToCommit - 1;
                    
                    // No guys to commit.
                    storeToWriteQ.send(ctx, tagged Invalid);

                end
                tagged Valid .tok:
                begin
                    // The oldest token has been committed. Let's see if we can send it to the write buffer.
                    if (physAddress[oldestCommitted] matches tagged Valid .phys_addr &&& write_buff_has_credit)
                    begin

                        // It's got room. Let's send the oldest store.
                        debugLog.record(ctx, fshow("DEALLOC ") + fshow(tok));

                        // Dequeue the old entry.
                        tokID[oldestCommitted] <= tagged Invalid;
                        oldestCommitted <= oldestCommitted + 1;
                        numToCommit <= numToCommit - 1;

                        // Send it to the writeBuffer.
                        storeToWriteQ.send(ctx, tagged Valid tuple2(tok, phys_addr));

                    end
                    else
                    begin
                    
                        // No room to commit this guy.
                        debugLog.record(ctx, fshow("DEALLOC STALL ") + fshow(tok));
                        storeToWriteQ.send(ctx, tagged Invalid);
                    
                    end
                    
                end
            
            endcase
        
        end
        else
        begin

            // No guys to commit.
            debugLog.record(ctx, fshow("NO DEALLOC"));
            storeToWriteQ.send(ctx, tagged Invalid);
        
        end
        
        localCtrl.endModelCycle(ctx, 1);

    endrule
    
    rule storeRsp (True);
    
        let rsp = doStores.getResp();
        doStores.deq();
        let tok = rsp.token;
        
        let ctx = tokContextId(tok);
        stallForStoreRsp[ctx] <= False;
    
    endrule

endmodule
