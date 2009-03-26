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
import FIFO::*;
import FIFOF::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/hasim_controller.bsh"
`include "asim/provides/fpga_components.bsh"


// ****** Timing Model imports *****
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"

// ****** Modules ******

// mkInstructionQueue

// A completion buffer which rendezvous ICache miss responses with
// the inorder instruction stream.

// Also accepts the branch predictor attributes and rendezvous them.

// This actually stores information per-token, but artificially limits
// the number of slots using a counter. Gaps between tokens (which can happen because of rewinds and faults)
// are handled using a sequential search.

// Normal flow: stage 1: perform deallocate and complete -> stage 2: perform allocate

// Ways a model cycle can end:
// Path 1: The next sequential token is the one we will try to issue next cycle.
// Path 2: There is a gap and we stalled to find the next oldest token.

// NOTE: This module makes an assumption that a Branch Predictor response is 
// faster than a cache miss response. This is probably a reasonable assumption.

module [HASIM_MODULE] mkInstructionQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_instq.out");


    // ****** Model State (per Context) ******

    // Which tokens are allocated.
    MULTIPLEXED#(NUM_CPUS, Reg#(Vector#(NUM_TOKENS, Bool)))            allocatedPool   <- mkMultiplexed(mkReg(replicate(False)));
    
    // Which tokens are complete? (Ready to issue)
    // Should this be block ram?
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(TOKEN_ID, Maybe#(ISA_INSTRUCTION))) completePool    <- mkMultiplexed(mkLUTRAM(tagged Invalid));

    // How many slots are taken?
    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_ID))                 numElemsPool    <- mkMultiplexed(mkReg(0));
    
    // What's the oldest token we know about? (The next one which should issue when it's ready.)
    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_ID))                 oldestTokPool <- mkMultiplexed(mkReg(0));

    // Queues to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, FIFOF#(BRANCH_ATTR))    attrQPool   <- mkMultiplexed(mkUGSizedFIFOF(`IQ_NUM_SLOTS));
    MULTIPLEXED#(NUM_CPUS, FIFOF#(FETCH_BUNDLE))   bundleQPool <- mkMultiplexed(mkUGSizedFIFOF(`IQ_NUM_SLOTS));

    // ****** UnModel State ******
    
    FIFO#(CPU_INSTANCE_ID) stage2Q <- mkFIFO();
    FIFO#(CPU_INSTANCE_ID) stage3Q <- mkFIFO();

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(TOKEN, ISA_ADDRESS, Maybe#(ISA_INSTRUCTION))) allocateFromFetch <- mkPortRecv_Multiplexed("Fet_to_InstQ_allocate", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED)                             rspFromICacheDelayed <- mkPortRecv_Multiplexed("ICache_to_CPU_delayed", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BRANCH_ATTR)                                                 attrFromBP <- mkPortRecv_Multiplexed("BP_to_Fet_attr", 0);

    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, FETCH_BUNDLE) bundleToDecQ  <- mkPortStallSend_Multiplexed("DecQ");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VOID)               creditToFetch <- mkPortSend_Multiplexed("InstQ_to_Fet_credit");

    // ****** Local Controller ******

    Vector#(4, PORT_CONTROLS#(NUM_CPUS)) inports  = newVector();
    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = allocateFromFetch.ctrl;
    inports[1]  = rspFromICacheDelayed.ctrl;
    inports[2]  = attrFromBP.ctrl;
    inports[3]  = bundleToDecQ.ctrl;
    outports[0] = creditToFetch.ctrl;
    outports[1] = bundleToDecQ.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    // stage1_deallocate
    
    // We begin by checking if the oldest slot is ready to go.
    // If so we send it to the decode queue.
    // We also check for miss responses from the ICache.

    rule stage1_deallocate (True);
    
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
            
        // Get our local state based on the current context.
        Reg#(Vector#(NUM_TOKENS, Bool))           allocated = allocatedPool[cpu_iid];
        LUTRAM#(TOKEN_ID, Maybe#(ISA_INSTRUCTION)) complete = completePool[cpu_iid];

        Reg#(TOKEN_ID)  numElems = numElemsPool[cpu_iid];
        Reg#(TOKEN_ID) oldestTok = oldestTokPool[cpu_iid];

        FIFOF#(BRANCH_ATTR)    attrQ = attrQPool[cpu_iid];
        FIFOF#(FETCH_BUNDLE) bundleQ = bundleQPool[cpu_iid];
    
        // Let's see if the oldest token has been completed.
        // If so, send it to the decodeQ.

        if (bundleToDecQ.canEnq(cpu_iid) && allocated[oldestTok] &&& complete.sub(oldestTok) matches tagged Valid .inst)
        begin

            // It's ready to go.
            let bundle = bundleQ.first();
            debugLog.record_next_cycle(cpu_iid, fshow("SEND READY: ") + fshow(bundle.token) + $format(" ADDR:0x%h", bundle.pc));
            
            // Marshall it up and send it to the decode queue.
            bundle.inst = inst;
            bundle.branchAttr = attrQ.first();
            bundleToDecQ.doEnq(cpu_iid, bundle);
            
            // Deallocate this token.
            attrQ.deq();
            bundleQ.deq();
            allocated[oldestTok] <= False;
            numElems <= numElems - 1;
            oldestTok <= oldestTok + 1;
           

        end
        else
        begin
            
            // We're not ready to send anything to the decodeQ.
            debugLog.record_next_cycle(cpu_iid, fshow("NO SEND"));
            bundleToDecQ.noEnq(cpu_iid);

        end
        
        // Now let's check for miss responses from the ICache.

        let m_complete <- rspFromICacheDelayed.receive(cpu_iid);
        
        if (m_complete matches tagged Valid .rsp)
        begin
        
            // We've got a new response.
            debugLog.record_next_cycle(cpu_iid, fshow("COMPLETE: ") + fshow(rsp.token));
            
            // Mark this token as complete.
            TOKEN_ID tok_id = tokTokenId(rsp.token);
            complete.upd(tok_id, tagged Valid rsp.instruction);

        end

        // Pass this context to the next stage.
        stage2Q.enq(cpu_iid);

    endrule
    

    // stage2_allocate
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to Fetch.
    
    // If there was no gap in the tokens we are done. Otherwise we
    // proceed to stage3.

    rule stage2_allocate (True);

        // Get our context from the previous stage.
        let cpu_iid = stage2Q.first();
        stage2Q.deq();
        
        // Get our local state based on the context.
        Reg#(Vector#(NUM_TOKENS, Bool)) allocated = allocatedPool[cpu_iid];
        LUTRAM#(TOKEN_ID, Maybe#(ISA_INSTRUCTION)) complete = completePool[cpu_iid];

        Reg#(TOKEN_ID)    numElems = numElemsPool[cpu_iid];
        Reg#(TOKEN_ID) oldestTok = oldestTokPool[cpu_iid];

        FIFOF#(BRANCH_ATTR)    attrQ = attrQPool[cpu_iid];
        FIFOF#(FETCH_BUNDLE) bundleQ = bundleQPool[cpu_iid];
    
        // Get any attributes from the branch predictor.
        let m_attr <- attrFromBP.receive(cpu_iid);
    
        if (m_attr matches tagged Valid .attr)
        begin
            attrQ.enq(attr);
        end

        // Check for any new allocations.
        let m_alloc <- allocateFromFetch.receive(cpu_iid);
        let new_num_elems = numElems;

        // If we just allocated the oldest guy, then this will tell us there's no need to search.
        Bool allocating_oldest = False;
        
        if (m_alloc matches tagged Valid {.tok, .addr, .comp})
        begin

            // A new allocation.
            debugLog.record(cpu_iid, fshow("ALLOC: ") + fshow(tok) + $format(" ADDR:0x%h", addr) + $format(" COMPLETE: %0b", pack(isValid(comp))));
            bundleQ.enq(FETCH_BUNDLE {token: tok, pc: addr, inst: ?, branchAttr: ?});
            complete.upd(tokTokenId(tok), comp);
            allocated[tokTokenId(tok)] <= True;
            new_num_elems = numElems + 1;
            allocating_oldest = tokTokenId(tok) == oldestTok;

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        if (new_num_elems < `IQ_NUM_SLOTS)
        begin
        
            // We still have room.
            debugLog.record(cpu_iid, fshow("SEND CREDIT"));
            creditToFetch.send(cpu_iid, tagged Valid (?));

        end
        else
        begin

            // We're full.
            debugLog.record(cpu_iid, fshow("NO CREDIT"));
            creditToFetch.send(cpu_iid, tagged Invalid);

        end
        
        // Update the element count.
        numElems <= new_num_elems;

        // See if we know who the oldest token is, or if we have to search for it.
        if (new_num_elems != 0 && !(allocated[oldestTok] || allocating_oldest))
        begin
            // We have to search for it.
            stage3Q.enq(cpu_iid);
        end
        else
        begin
            // We know it. 
            // End of Model Cycle. (path 1)
            localCtrl.endModelCycle(cpu_iid, 1);
        end
            
    endrule
    
    // stage3_findOldest
    
    // This should only occur uncommonly.
    // Search for the oldest allocated token, beginning at the current oldest.
    
    // This rule recurs until we find the oldest and dequeue the FIFO.

    rule stage3_findOldest (True);
    
        // Get our context from the FIFO.
        let cpu_iid = stage3Q.first();
        
        // Get our local state based on the current context.
        Reg#(TOKEN_ID) oldestTok = oldestTokPool[cpu_iid];
        Reg#(Vector#(NUM_TOKENS, Bool)) allocated = allocatedPool[cpu_iid];
    
        // Check for the oldest allocated token.
        if (allocated[oldestTok])
        begin

            // We've found it.
            stage3Q.deq();
            // End of model cycle. (path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        else
        begin

            // Keep looking.
            oldestTok <= oldestTok + 1;

        end
    
    endrule

endmodule
