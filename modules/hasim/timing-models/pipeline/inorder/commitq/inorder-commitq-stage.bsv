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

import FShow::*;
import Vector::*;
import FIFO::*;
import FIFOF::*;

// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model Imports ******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

// mkCommitQueue

// A completion buffer which waits for load responses from cache misses. 
// This ensures that commits of faults occur at the proper time.

// Also reports writebacks of the loads to decode.

module [HASIM_MODULE] mkCommitQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_commitq.out");


    // ****** Model State (per Context) ******

    // Which tokens are allocated.
    MULTICTX#(Reg#(Vector#(NUM_TOKENS, Bool)))            ctx_allocated   <- mkMultiCtx(mkReg(replicate(False)));
    
    // Which tokens are complete? (Ready to issue)
    MULTICTX#(Reg#(Vector#(NUM_TOKENS, Bool)))            ctx_complete    <- mkMultiCtx(mkReg(replicate(False)));

    // Token destinations.
    // Should this be block ram?
    MULTICTX#(LUTRAM#(TOKEN_ID, Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)))) ctx_dests <- mkMultiCtx(mkLUTRAM(replicate(tagged Invalid)));

    // How many slots are taken?
    MULTICTX#(Reg#(TOKEN_ID))                 ctx_numElems    <- mkMultiCtx(mkReg(0));
    
    // What's the oldest token we know about? (The next one which should issue when it's ready.)
    MULTICTX#(Reg#(TOKEN_ID))                 ctx_oldestTok <- mkMultiCtx(mkReg(0));

    // Queues to rendezvous with instructions.
    MULTICTX#(FIFOF#(DMEM_BUNDLE))   ctx_bundleQ <- mkMultiCtx(mkUGSizedFIFOF(`CQ_NUM_SLOTS));

    // ****** UnModel State ******
    
    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();

    // ****** Ports ******

    PORT_RECV_MULTICTX#(Tuple2#(DMEM_BUNDLE, Bool)) allocateFromDMem <- mkPortRecv_MultiCtx("commitQ_alloc", 0);
    PORT_RECV_MULTICTX#(DCACHE_LOAD_OUTPUT_DELAYED)  rspFromDCacheDelayed <- mkPortRecv_MultiCtx("DCache_to_CPU_load_delayed", 0);

    PORT_STALL_SEND_MULTICTX#(DMEM_BUNDLE) bundleToRetireQ <- mkPortStallSend_MultiCtx("RetireQ");
    PORT_SEND_MULTICTX#(VOID)               creditToDMem <- mkPortSend_MultiCtx("commitQ_credit");
    PORT_SEND_MULTICTX#(BUS_MESSAGE)        writebackToDec <- mkPortSend_MultiCtx("DMem_to_Dec_miss_writeback");

    // ****** Local Controller ******

    Vector#(3, PORT_CONTROLS) inports  = newVector();
    Vector#(3, PORT_CONTROLS) outports = newVector();
    inports[0]  = allocateFromDMem.ctrl;
    inports[1]  = rspFromDCacheDelayed.ctrl;
    inports[2]  = bundleToRetireQ.ctrl;
    outports[0] = creditToDMem.ctrl;
    outports[1] = bundleToRetireQ.ctrl;
    outports[2] = writebackToDec.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    // stage1_deallocate
    
    // We begin by checking if the oldest slot is ready to go.
    // If so we send it to the decode queue.
    // We also check for miss responses from the ICache.

    rule stage1_deallocate (True);
    
        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
            
        // Get our local state based on the current context.
        Reg#(Vector#(NUM_TOKENS, Bool)) allocated = ctx_allocated[ctx];
        Reg#(Vector#(NUM_TOKENS, Bool)) complete  = ctx_complete[ctx];
        let dests = ctx_dests[ctx];

        Reg#(TOKEN_ID)  numElems = ctx_numElems[ctx];
        Reg#(TOKEN_ID) oldestTok = ctx_oldestTok[ctx];

        FIFOF#(DMEM_BUNDLE) bundleQ = ctx_bundleQ[ctx];
    
        // Let's see if the oldest token has been completed.
        // If so, send it to the retireQ.

        if (bundleToRetireQ.canEnq(ctx) && allocated[oldestTok] &&& complete[oldestTok])
        begin

            // It's ready to go.
            let bundle = bundleQ.first();
            debugLog.record_next_cycle(ctx, fshow("SEND READY: ") + fshow(bundle.token));
            
            // Send it to the retire queue.
            bundleToRetireQ.doEnq(ctx, bundle);
            
            // Deallocate this token.
            bundleQ.deq();
            allocated[oldestTok] <= False;
            numElems <= numElems - 1;
            oldestTok <= oldestTok + 1;

        end
        else
        begin
            
            // We're not ready to send anything to the retireQ.
            debugLog.record_next_cycle(ctx, fshow("NO SEND"));
            bundleToRetireQ.noEnq(ctx);

        end
        
        // Now let's check for miss responses from the ICache.
        let m_complete <- rspFromDCacheDelayed.receive(ctx);
        
        if (m_complete matches tagged Valid .rsp)
        begin
        
            // We've got a new response.
            debugLog.record_next_cycle(ctx, fshow("COMPLETE: ") + fshow(rsp.token));
            
            // Mark this token as complete.
            TOKEN_ID tok_id = tokTokenId(rsp.token);
            complete[tok_id] <= True; // Note: this is harmless if this was already true. In a real completion buffer we would NOT want to override an earlier completion. IE the SB hit, so drop the cache miss response when it finally comes.
            
            // Tell Decode to writeback the destination.
            writebackToDec.send(ctx, tagged Valid genBusMessage(rsp.token, dests.sub(tok_id)));

        end
        else
        begin

            // No writebacks to report.
            writebackToDec.send(ctx, tagged Invalid);

        end

        // Pass this context to the next stage.
        stage2Q.enq(ctx);

    endrule
    

    // stage2_allocate
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to Fetch.
    
    // If there was no gap in the tokens we are done. Otherwise we
    // proceed to stage3.

    rule stage2_allocate (True);

        // Get our context from the previous stage.
        let ctx = stage2Q.first();
        stage2Q.deq();
        
        // Get our local state based on the context.
        Reg#(Vector#(NUM_TOKENS, Bool)) allocated = ctx_allocated[ctx];
        Reg#(Vector#(NUM_TOKENS, Bool)) complete  = ctx_complete[ctx];
        let dests = ctx_dests[ctx];

        Reg#(TOKEN_ID)    numElems = ctx_numElems[ctx];
        Reg#(TOKEN_ID) oldestTok = ctx_oldestTok[ctx];

        FIFOF#(DMEM_BUNDLE) bundleQ = ctx_bundleQ[ctx];

        // Check for any new allocations.
        let m_alloc <- allocateFromDMem.receive(ctx);
        let new_num_elems = numElems;

        // If we just allocated the oldest guy, then this will tell us there's no need to search.
        Bool allocating_oldest = False;
        
        if (m_alloc matches tagged Valid {.bundle, .comp})
        begin

            // A new allocation.
            let tok = bundle.token;
            debugLog.record(ctx, fshow("ALLOC: ") + fshow(tok) + $format(" COMPLETE: %0b", pack(comp)));
            bundleQ.enq(bundle);
            complete[tokTokenId(tok)] <= comp;
            allocated[tokTokenId(tok)] <= True;
            dests.upd(tokTokenId(tok), bundle.dests);
            new_num_elems = numElems + 1;
            allocating_oldest = tokTokenId(tok) == oldestTok;

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        if (new_num_elems < `CQ_NUM_SLOTS)
        begin
        
            // We still have room.
            debugLog.record(ctx, fshow("SEND CREDIT"));
            creditToDMem.send(ctx, tagged Valid (?));

        end
        else
        begin

            // We're full.
            debugLog.record(ctx, fshow("NO CREDIT"));
            creditToDMem.send(ctx, tagged Invalid);

        end
        
        // Update the element count.
        numElems <= new_num_elems;

        // See if we know who the oldest token is, or if we have to search for it.
        if (new_num_elems != 0 && !(allocated[oldestTok] || allocating_oldest))
        begin
            // We have to search for it.
            stage3Q.enq(ctx);
        end
        else
        begin
            // We know it. 
            // End of Model Cycle. (path 1)
            localCtrl.endModelCycle(ctx, 1);
        end
            
    endrule
    
    // stage3_findOldest
    
    // This should only occur uncommonly.
    // Search for the oldest allocated token, beginning at the current oldest.
    
    // This rule recurs until we find the oldest and dequeue the FIFO.

    rule stage3_findOldest (True);
    
        // Get our context from the FIFO.
        let ctx = stage3Q.first();
        
        // Get our local state based on the current context.
        Reg#(TOKEN_ID) oldestTok = ctx_oldestTok[ctx];
        Reg#(Vector#(NUM_TOKENS, Bool)) allocated = ctx_allocated[ctx];
    
        // Check for the oldest allocated token.
        if (allocated[oldestTok])
        begin

            // We've found it.
            stage3Q.deq();
            // End of model cycle. (path 2)
            localCtrl.endModelCycle(ctx, 2);

        end
        else
        begin

            // Keep looking.
            oldestTok <= oldestTok + 1;

        end
    
    endrule

endmodule
