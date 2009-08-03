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
`include "asim/provides/l1_cache_base_types.bsh"


// mkCommitQueue

// A completion buffer which waits for load responses from cache misses. 
// This ensures that commits of faults occur at the proper time.

// Also reports writebacks of the loads to decode.

typedef 16 NUM_COMMITQ_SLOTS;

typedef Bit#(TLog#(NUM_COMMITQ_SLOTS)) COMMITQ_SLOT_ID;


module [HASIM_MODULE] mkCommitQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commitq.out");

    // ****** Model State (per Context) ******

    // Which tokens are complete? (Ready to issue)
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(COMMITQ_SLOT_ID, Bool)) completionsPool    <- mkMultiplexed(mkLUTRAM(False));

    // Queue to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE)) bundlesPool <- mkMultiplexed(mkLUTRAMU());

    // Which slot should we allocate into?
    MULTIPLEXED#(NUM_CPUS, Reg#(COMMITQ_SLOT_ID)) nextFreeSlotPool    <- mkMultiplexed(mkReg(0));
    
    // What's the oldest slot we know about? (The next one which should issue when it's ready.)
    MULTIPLEXED#(NUM_CPUS, Reg#(COMMITQ_SLOT_ID)) oldestSlotPool <- mkMultiplexed(mkReg(0));

    // Miss Address File (MAF).
    // Maps from DCache miss ID to commitq slot.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID)) mafPool <- mkMultiplexed(mkLUTRAMU());
    

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple2#(DMEM_BUNDLE, Maybe#(L1_DCACHE_MISS_ID))) allocateFromDMem <- mkPortRecv_Multiplexed("commitQ_alloc", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)                       deqFromCom       <- mkPortRecvDependent_Multiplexed("commitQ_deq");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED)  rspFromDCacheDelayed <- mkPortRecv_Multiplexed("DCache_to_CPU_load_delayed", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE)     bundleToCom    <- mkPortSend_Multiplexed("commitQ_first");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VOID) creditToDMem   <- mkPortSend_Multiplexed("commitQ_credit");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE)     writebackToDec <- mkPortSend_Multiplexed("DMem_to_Dec_miss_writeback");

    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = allocateFromDMem.ctrl;
    inports[1]  = rspFromDCacheDelayed.ctrl;
    outports[0] = creditToDMem.ctrl;
    outports[1] = bundleToCom.ctrl;
    outports[2] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS)      localCtrl  <- mkLocalController(inports, outports);
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage4Ctrl <- mkStageControllerVoid();


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
    rule stage1_sendFirst (True);
                
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
    
        stage2Ctrl.ready(cpu_iid);
    
    endrule
        
        
    rule stage2_complete (True);
    
        // Get the info from the previous stage.
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get the local state for the current instance.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool[cpu_iid];
        LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE) bundles = bundlesPool[cpu_iid];
        LUTRAM#(L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID) maf = mafPool[cpu_iid];        
    
        // Now let's check for miss responses from the DCache.
        let m_complete <- rspFromDCacheDelayed.receive(cpu_iid);
        
        if (m_complete matches tagged Valid .rsp)
        begin
        
            // A new completion came in. Get the slot from the MAF.
            COMMITQ_SLOT_ID slot = maf.sub(rsp.missID);
            DMEM_BUNDLE bundle = bundles.sub(slot);
            debugLog.record_next_cycle(cpu_iid, fshow("1: COMPLETE: ") + fshow(slot));
            
            // Mark this slot as complete.
            completions.upd(slot, True);

            // Tell Decode to writeback the destination.
            writebackToDec.send(cpu_iid, tagged Valid genBusMessage(bundle.token, bundle.dests));

        end
        else
        begin
        
            // No writebacks to report.
            writebackToDec.send(cpu_iid, tagged Invalid);

        end
        
        // Pass this context to the next stage.
        stage3Ctrl.ready(cpu_iid);

    endrule


    // stage3_deallocate
    // Check if commit is dequeing.
    
    // Ports read:
    // * deqFromCom
    
    // Ports written:
    // * none

    (* conservative_implicit_conditions *)
    rule stage3_deallocate (True);

        // Get the info from the previous stage.
        let cpu_iid <- stage3Ctrl.nextReadyInstance();

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
        stage4Ctrl.ready(cpu_iid);

    endrule

    // stage4_allocate
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to DMem.
    
    // Ports read:
    // * allocateFromDMem
    
    // Ports written:
    // * creditToDMem

    (* conservative_implicit_conditions *)
    rule stage4_allocate (True);

        // Get our context from the previous stage.
        let cpu_iid <- stage4Ctrl.nextReadyInstance();
        
        // Get our local state based on the context.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool[cpu_iid];
        LUTRAM#(COMMITQ_SLOT_ID, DMEM_BUNDLE) bundles = bundlesPool[cpu_iid];
        LUTRAM#(L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID) maf = mafPool[cpu_iid];

        Reg#(COMMITQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(COMMITQ_SLOT_ID) oldestSlot   = oldestSlotPool[cpu_iid];

        // Check for any new allocations.
        let m_alloc <- allocateFromDMem.receive(cpu_iid);

        if (m_alloc matches tagged Valid {.bundle, .m_miss_id})
        begin

            // A new allocation.
            debugLog.record(cpu_iid, $format("3: ALLOC: ") + fshow(bundle.token) + $format(" COMPLETE: %0b", pack(isValid(m_miss_id))));
            bundles.upd(nextFreeSlot, bundle);
            completions.upd(nextFreeSlot, !isValid(m_miss_id));
            nextFreeSlot <= nextFreeSlot + 1;

            if (m_miss_id matches tagged Valid .miss_id)
            begin
            
                debugLog.record(cpu_iid, $format("3: MISS ID: %0d mapped to slot %0d.", miss_id, nextFreeSlot));
                maf.upd(miss_id, nextFreeSlot);
                
            end

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        if (nextFreeSlot + 1 != oldestSlot) // This should really be +L+1, where L is the latency of the credit to DMem.
        begin
        
            // We still have room.
            debugLog.record(cpu_iid, fshow("3: SEND CREDIT"));
            creditToDMem.send(cpu_iid, tagged Valid (?));

        end
        else
        begin

            // We're full.
            debugLog.record(cpu_iid, fshow("3: NO CREDIT"));
            creditToDMem.send(cpu_iid, tagged Invalid);

        end
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule
