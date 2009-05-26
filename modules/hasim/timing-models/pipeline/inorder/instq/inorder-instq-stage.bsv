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

    // Which tokens are complete? (Ready to issue)
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(INSTQ_SLOT_ID, Maybe#(ISA_INSTRUCTION))) completionsPool    <- mkMultiplexed(mkLUTRAM(tagged Valid (?)));

    // Queue to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE))   bundlesPool <- mkMultiplexed(mkLUTRAMU());

    // How many slots are taken?
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID))                 nextFreeSlotPool    <- mkMultiplexed(mkReg(0));
    
    // What's the oldest token we know about? (The next one which should issue when it's ready.)
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID))                 oldestSlotPool <- mkMultiplexed(mkReg(0));

    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_BRANCH_EPOCH)) curBranchEpochPool <- mkMultiplexed(mkReg(0));
    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_FAULT_EPOCH))  curFaultEpochPool  <- mkMultiplexed(mkReg(0));

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(INSTQ_SLOT_ID, FETCH_BUNDLE, Maybe#(ISA_INSTRUCTION)))   allocateFromFetch <- mkPortRecv_Multiplexed("Fet_to_InstQ_allocate", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED)                         rspFromICacheDelayed <- mkPortRecv_Multiplexed("ICache_to_CPU_delayed", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, FETCH_BUNDLE)    bundleToDec  <- mkPortSend_Multiplexed("InstQ_to_Dec_first");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, INSTQ_SLOT_ID) creditToFetch  <- mkPortSend_Multiplexed("InstQ_to_Fet_credit");
    
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)        deqFromDec <- mkPortRecvGuarded_Multiplexed("Dec_to_InstQ_deq", 0);
        
    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS))  inports  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();

    inports[0]  = allocateFromFetch.ctrl;
    inports[1]  = rspFromICacheDelayed.ctrl;
    outports[0] = creditToFetch.ctrl;
    outports[1] = bundleToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS)      localCtrl  <- mkLocalController(inports, outports);
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();


    // ****** Rules ******

    // stage1_sendAndComplete
    // Check if the oldest slot is ready to go.
    // If so we send it to decode.
    // We also check for miss responses from the ICache

    // Ports read:
    // * rspFromICacheDelayed
    
    // Ports written:
    // * bundleToDec

    (* conservative_implicit_conditions *)
    rule stage1_sendAndComplete (True);
                
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);


        // Get our local state based on the current context.
        LUTRAM#(INSTQ_SLOT_ID, Maybe#(ISA_INSTRUCTION)) completions = completionsPool[cpu_iid];
        LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE) bundles = bundlesPool[cpu_iid];

        Reg#(INSTQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID)   oldestSlot = oldestSlotPool[cpu_iid];
    
        // Let's see if the oldest token has been completed.
        // If so, send it to the decode.
        let empty = oldestSlot == nextFreeSlot;

        if (completions.sub(oldestSlot) matches tagged Valid .inst &&& !empty)
        begin

            // It's ready to go.
            let bundle = bundles.sub(oldestSlot);
            debugLog.record_next_cycle(cpu_iid, $format("SEND READY ADDR:0x%h", bundle.pc));
            
            // Marshall it up and send it to the decode queue.
            bundle.inst = inst;
            bundleToDec.send(cpu_iid, tagged Valid bundle);
            
        end
        else
        begin
            
            // We're not ready to send anything to the decodeQ.
            debugLog.record_next_cycle(cpu_iid, fshow("NO SEND"));
            bundleToDec.send(cpu_iid, tagged Invalid);

        end
        
        // Now let's check for miss responses from the ICache.

        let m_complete <- rspFromICacheDelayed.receive(cpu_iid);
        
        if (m_complete matches tagged Valid .rsp)
        begin
        
            // We've got a new response.
            debugLog.record_next_cycle(cpu_iid, fshow("COMPLETE: ") + fshow(rsp.instQSlot));
            
            // Mark this slot as complete.
            completions.upd(rsp.instQSlot, tagged Valid rsp.instruction);

        end
        
        // Pass this context to the next stage.
        stage2Ctrl.ready(cpu_iid);

    endrule
    

    // stage2_deallocate
    // Check if decode is dequeing.
    
    // Ports read:
    // * deqFromDec
    
    // Ports written:
    // * none

    (* conservative_implicit_conditions *)
    rule stage2_deallocate (True);

        // Get the info from the previous stage.
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get the local state for the current instance.
        Reg#(INSTQ_SLOT_ID)   oldestSlot = oldestSlotPool[cpu_iid];
        
        // Check for any dequeues/clears.
        let m_deq <- deqFromDec.receive(cpu_iid);
        
        if (isValid(m_deq))
        begin
            
            // A dequeue occurred. Just drop the oldest guy.
            debugLog.record(cpu_iid, fshow("DEQ"));
            oldestSlot <= oldestSlot + 1;

        end
        
        // Send it to the next stage.
        stage3Ctrl.ready(cpu_iid);

    endrule


    // stage3_allocate
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to Fetch.
    
    // Ports read:
    // * allocateFromFetch
    
    // Ports written:
    // * creditToFetch

    (* conservative_implicit_conditions *)
    rule stage3_allocate (True);

        // Get our context from the previous stage.
        let cpu_iid <- stage3Ctrl.nextReadyInstance();
        
        // Get our local state based on the context.
        LUTRAM#(INSTQ_SLOT_ID, Maybe#(ISA_INSTRUCTION)) completions = completionsPool[cpu_iid];
        LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE) bundles = bundlesPool[cpu_iid];

        Reg#(TOKEN_BRANCH_EPOCH) curBranchEpoch = curBranchEpochPool[cpu_iid];
        Reg#(TOKEN_FAULT_EPOCH) curFaultEpoch = curFaultEpochPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) oldestSlot   = oldestSlotPool[cpu_iid];

        // Check for any new allocations.
        let m_alloc <- allocateFromFetch.receive(cpu_iid);
        
        let new_next_free = nextFreeSlot;

        if (m_alloc matches tagged Valid {.slot, .bundle, .comp})
        begin

            // A new allocation.
            debugLog.record(cpu_iid, $format("ALLOC ADDR:0x%h", bundle.pc) + $format(" COMPLETE: %0b", pack(isValid(comp))));
            bundles.upd(slot, bundle);
            completions.upd(slot, comp);
            if (bundle.branchEpoch != curBranchEpoch || bundle.faultEpoch != curFaultEpoch)
            begin
                debugLog.record(cpu_iid, $format("EPOCH CHANGE. NEW HEAD: %0d.", nextFreeSlot));
                oldestSlot <= nextFreeSlot;
                curBranchEpoch <= bundle.branchEpoch;
                curFaultEpoch <= bundle.faultEpoch;
            end
            new_next_free = nextFreeSlot + 1;

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        let have_room = (new_next_free + 2 != oldestSlot); // This should really be +L+1, where L is the latency of the credit to Fetch.
        
        // Also check that the guy we're going to overwrite has already been completed.
        let safe_to_allocate = isValid(completions.sub(new_next_free));
        
        if (have_room && safe_to_allocate)
        begin
        
            // We still have room.
            debugLog.record(cpu_iid, fshow("SEND CREDIT"));
            creditToFetch.send(cpu_iid, tagged Valid new_next_free);

        end
        else
        begin

            // We're full.
            debugLog.record(cpu_iid, fshow("NO CREDIT"));
            creditToFetch.send(cpu_iid, tagged Invalid);

        end
        
        // Update the next free slot
        nextFreeSlot <= new_next_free;
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule
