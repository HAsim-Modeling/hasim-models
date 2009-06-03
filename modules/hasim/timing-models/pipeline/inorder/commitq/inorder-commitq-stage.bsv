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
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"


// mkCommitQueue

// A completion buffer which waits for load responses from cache misses. 
// This ensures that commits of faults occur at the proper time.

// Also reports writebacks of the loads to decode.

module [HASIM_MODULE] mkCommitQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commitq.out");

    // ****** Model State (per Context) ******

    // Which tokens are complete? (Ready to issue)
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(COMMITQ_SLOT_ID, Bool)) completionsPool    <- mkMultiplexed(mkLUTRAM(False));

    // Queue to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE))   bundlesPool <- mkMultiplexed(mkLUTRAMU());

    // How many slots are taken?
    MULTIPLEXED#(NUM_CPUS, Reg#(COMMITQ_SLOT_ID))                 nextFreeSlotPool    <- mkMultiplexed(mkReg(0));
    
    // What's the oldest slot we know about? (The next one which should issue when it's ready.)
    MULTIPLEXED#(NUM_CPUS, Reg#(COMMITQ_SLOT_ID))                 oldestSlotPool <- mkMultiplexed(mkReg(0));

    // ****** UnModel State ******
    
    FIFO#(CPU_INSTANCE_ID) stage2Q <- mkFIFO();
    FIFO#(CPU_INSTANCE_ID) stage3Q <- mkFIFO();

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(DMEM_BUNDLE, COMMITQ_SLOT_ID, Bool)) allocateFromDMem <- mkPortRecv_Multiplexed("commitQ_alloc", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)                       deqFromCom       <- mkPortRecvDependent_Multiplexed("commitQ_deq");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED)  rspFromDCacheDelayed <- mkPortRecv_Multiplexed("DCache_to_CPU_load_delayed", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE)     bundleToCom    <- mkPortSend_Multiplexed("commitQ_first");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, COMMITQ_SLOT_ID) slotToDMem     <- mkPortSend_Multiplexed("commitQ_credit");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE)     writebackToDec <- mkPortSend_Multiplexed("DMem_to_Dec_miss_writeback");

    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = allocateFromDMem.ctrl;
    inports[1]  = rspFromDCacheDelayed.ctrl;
    outports[0] = slotToDMem.ctrl;
    outports[1] = bundleToCom.ctrl;
    outports[2] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS)      localCtrl  <- mkLocalController(inports, outports);
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();


    // ****** Rules ******

    // stage1_sendAndComplete
    // Check if the oldest slot is ready to go.
    // If so we send it to commit.
    // We also check for miss responses from the DCache

    // Ports read:
    // * rspFromDCacheDelayed
    
    // Ports written:
    // * bundleToCom
    // * writebackToDec

    (* conservative_implicit_conditions *)
    rule stage1_sendAndComplete (True);
                
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get the local state for the current instance.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool[cpu_iid];
        LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE) bundles = bundlesPool[cpu_iid];

        Reg#(COMMITQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(COMMITQ_SLOT_ID)   oldestSlot = oldestSlotPool[cpu_iid];
    
        // Let's see if the oldest slot has been completed.
        // If so, send it to the decode.
        let empty = oldestSlot == nextFreeSlot;

        if (completions.sub(oldestSlot) && !empty)
        begin

            // It's ready to go.
            let bundle = bundles.sub(oldestSlot);
            debugLog.record_next_cycle(cpu_iid, fshow("1: SEND READY: ") + fshow(bundle.token));
            
            // Send it to commit.
            bundleToCom.send(cpu_iid, tagged Valid bundle);
            
        end
        else
        begin
            
            // We're not ready to send anything to commit.
            debugLog.record_next_cycle(cpu_iid, fshow("1: NO SEND"));
            bundleToCom.send(cpu_iid, tagged Invalid);
        end
        
        // Now let's check for miss responses from the DCache.

        let m_complete <- rspFromDCacheDelayed.receive(cpu_iid);
        
        if (m_complete matches tagged Valid .rsp)
        begin
        
            // We've got a new response.
            debugLog.record_next_cycle(cpu_iid, fshow("1: COMPLETE: ") + fshow(rsp.commitQSlot));
            
            // Mark this slot as complete.
            completions.upd(rsp.commitQSlot, True);

            // Tell Decode to writeback the destination.
            writebackToDec.send(cpu_iid, tagged Valid genBusMessage(rsp.token, rsp.dests));

        end
        else
        begin
        
            // No writebacks to report.
            writebackToDec.send(cpu_iid, tagged Invalid);

        end
        
        // Pass this context to the next stage.
        stage2Ctrl.ready(cpu_iid);

    endrule


    // stage2_deallocate
    // Check if commit is dequeing.
    
    // Ports read:
    // * deqFromCom
    
    // Ports written:
    // * none

    (* conservative_implicit_conditions *)
    rule stage2_deallocate (True);

        // Get the info from the previous stage.
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get the local state for the current instance.
        Reg#(COMMITQ_SLOT_ID)   oldestSlot = oldestSlotPool[cpu_iid];
        
        // Check for any dequeues/clears.
        let m_deq <- deqFromCom.receive(cpu_iid);
        
        if (isValid(m_deq))
        begin
            
            // A dequeue occurred. Just drop the oldest guy.
            debugLog.record(cpu_iid, fshow("2: DEQ"));
            oldestSlot <= oldestSlot + 1;

        end
        
        // Send it to the next stage.
        stage3Ctrl.ready(cpu_iid);

    endrule

    // stage3_allocate
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to DMem.
    
    // Ports read:
    // * allocateFromDMem
    
    // Ports written:
    // * slotToDMem

    (* conservative_implicit_conditions *)
    rule stage3_allocate (True);

        // Get our context from the previous stage.
        let cpu_iid <- stage3Ctrl.nextReadyInstance();
        
        // Get our local state based on the context.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool[cpu_iid];
        LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE) bundles = bundlesPool[cpu_iid];

        Reg#(COMMITQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(COMMITQ_SLOT_ID) oldestSlot   = oldestSlotPool[cpu_iid];

        // Check for any new allocations.
        let m_alloc <- allocateFromDMem.receive(cpu_iid);
        
        let new_next_free = nextFreeSlot;

        if (m_alloc matches tagged Valid {.bundle, .slot, .comp})
        begin

            // A new allocation.
            debugLog.record(cpu_iid, $format("3: ALLOC: ") + fshow(bundle.token) + $format(" COMPLETE: %0b", pack(comp)));
            bundles.upd(slot, bundle);
            completions.upd(slot, comp);
            new_next_free = nextFreeSlot + 1;

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        if (new_next_free + 2 != oldestSlot) // This should really be +L+1, where L is the latency of the credit to DMem.
        begin
        
            // We still have room.
            debugLog.record(cpu_iid, fshow("3: SEND CREDIT"));
            slotToDMem.send(cpu_iid, tagged Valid new_next_free);

        end
        else
        begin

            // We're full.
            debugLog.record(cpu_iid, fshow("3: NO CREDIT"));
            slotToDMem.send(cpu_iid, tagged Invalid);

        end
        
        // Update the next free slot
        nextFreeSlot <= new_next_free;
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule
