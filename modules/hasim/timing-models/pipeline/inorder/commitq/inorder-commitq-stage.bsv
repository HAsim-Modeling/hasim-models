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
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"


// mkCommitQueue

// A completion buffer which waits for load responses from cache misses. 
// This ensures that commits of faults occur at the proper time.

// Also reports writebacks of the loads to decode.

typedef 16 NUM_COMMITQ_SLOTS;

typedef Bit#(TLog#(NUM_COMMITQ_SLOTS)) COMMITQ_SLOT_ID;

//
// Bundles pool read ports
//
`define PORT_OLDEST   0
`define PORT_INCOMING 1


module [HASIM_MODULE] mkCommitQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commitq.out");

    // ****** Model State (per Context) ******

    // Which tokens are complete? (Ready to issue)
    MULTIPLEXED_LUTRAM_MULTI_WRITE#(MAX_NUM_CPUS, 2, COMMITQ_SLOT_ID, Bool) completionsPool <- mkMultiplexedLUTRAMPseudoMultiWrite(False);

    // Queue to rendezvous with instructions.
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(MAX_NUM_CPUS, 2, COMMITQ_SLOT_ID, DMEM_BUNDLE)
        bundlesPool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiRead(False));

    // Which slot should we allocate into?
    MULTIPLEXED_REG#(MAX_NUM_CPUS, COMMITQ_SLOT_ID) nextFreeSlotPool    <- mkMultiplexedReg(0);
    
    // What's the oldest slot we know about? (The next one which should issue when it's ready.)
    MULTIPLEXED_REG#(MAX_NUM_CPUS, COMMITQ_SLOT_ID) oldestSlotPool <- mkMultiplexedReg(0);

    // Miss Address File (MAF).
    // Maps from DCache miss ID to commitq slot.
    MULTIPLEXED_LUTRAM#(MAX_NUM_CPUS, L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID) mafPool <- mkMultiplexedLUTRAM(?);
    

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(DMEM_BUNDLE, Maybe#(L1_DCACHE_MISS_ID))) allocateFromDMem <- mkPortRecv_Multiplexed("commitQ_alloc", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                       deqFromCom       <- mkPortRecvDependent_Multiplexed("commitQ_deq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED)  rspFromDCacheDelayed <- mkPortRecv_Multiplexed("DCache_to_CPU_load_delayed", 1);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DMEM_BUNDLE)     bundleToCom    <- mkPortSend_Multiplexed("commitQ_first");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID) creditToDMem   <- mkPortSend_Multiplexed("commitQ_credit");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)     writebackToDec <- mkPortSend_Multiplexed("DMem_to_Dec_miss_writeback");

    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) depports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = allocateFromDMem.ctrl;
    inports[1]  = rspFromDCacheDelayed.ctrl;
    depports[0] = deqFromCom.ctrl;
    outports[0] = creditToDMem.ctrl;
    outports[1] = bundleToCom.ctrl;
    outports[2] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS)      localCtrl  <- mkNamedLocalControllerWithUncontrolled("Commit Queue", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(COMMITQ_SLOT_ID)) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER_VOID#(MAX_NUM_CPUS) stage4Ctrl <- mkBufferedStageControllerVoid();


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
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool.getRAMWithWritePort(cpu_iid, 0);

        Reg#(COMMITQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool.getReg(cpu_iid);
        Reg#(COMMITQ_SLOT_ID)   oldestSlot = oldestSlotPool.getReg(cpu_iid);
    
        // Let's see if the oldest slot has been completed.
        // If so, send it to the decode.
        Bool oldest_is_ready = completions.sub(oldestSlot) && (oldestSlot != nextFreeSlot);

        if (oldest_is_ready)
        begin
            // It's ready to go.  Read the bundle.
            bundlesPool.readPorts[`PORT_OLDEST].readReq(cpu_iid, oldestSlot);
        end
    
        stage2Ctrl.ready(cpu_iid, oldest_is_ready);
    
    endrule
        
        
    rule stage2_complete (True);
    
        // Get the info from the previous stage.
        match {.cpu_iid, .oldest_is_ready} <- stage2Ctrl.nextReadyInstance();

        // Get the local state for the current instance.
        LUTRAM#(L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID) maf = mafPool.getRAM(cpu_iid);
    
        // Complete forwarding of commit, having ready the bundle from BRAM
        if (oldest_is_ready)
        begin
            let bundle <- bundlesPool.readPorts[`PORT_OLDEST].readRsp(cpu_iid);
            debugLog.record(cpu_iid, fshow("2: SEND READY: ") + fshow(bundle.token));
            
            // Send it to commit.
            bundleToCom.send(cpu_iid, tagged Valid bundle);
        end
        else
        begin
            // We're not ready to send anything to commit.
            debugLog.record(cpu_iid, fshow("2: NO SEND"));
            bundleToCom.send(cpu_iid, tagged Invalid);
        end

        // Now let's check for miss responses from the DCache.
        let m_complete <- rspFromDCacheDelayed.receive(cpu_iid);
        
        if (m_complete matches tagged Valid .rsp)
        begin
            // A new completion came in. Get the slot from the MAF.
            COMMITQ_SLOT_ID slot = maf.sub(rsp.missID);
            debugLog.record(cpu_iid, fshow("2: COMPLETE: ") + fshow(slot));

            bundlesPool.readPorts[`PORT_INCOMING].readReq(cpu_iid, slot);

            // Pass this context to the next stage.
            stage3Ctrl.ready(cpu_iid, tagged Valid slot);
        end
        else
        begin
            stage3Ctrl.ready(cpu_iid, tagged Invalid);
        end
        
    endrule


    // stage3_deallocate
    // Check if commit is dequeing.
    
    // Ports read:
    // * deqFromCom
    
    // Ports written:
    // * none

    rule stage3_deallocate (True);

        // Get the info from the previous stage.
        match {.cpu_iid, .incoming_completion} <- stage3Ctrl.nextReadyInstance();

        // Get the local state for the current instance.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool.getRAMWithWritePort(cpu_iid, 0);

        if (incoming_completion matches tagged Valid .slot)
        begin
            let bundle <- bundlesPool.readPorts[`PORT_INCOMING].readRsp(cpu_iid);
            
            // Mark this slot as complete.
            completions.upd(slot, True);

            // Tell Decode to writeback the destination.  The branch epoch is
            // irrelevant at this late stage, so only send fault epoch.
            let epoch = initEpoch(?, bundle.faultEpoch);
            writebackToDec.send(cpu_iid,
                                tagged Valid genBusMessage(bundle.token,
                                                           epoch,
                                                           bundle.dests));
            debugLog.record(cpu_iid, fshow("3: FWD COMPLETE: ") + fshow(bundle.token));
        end
        else
        begin
            // No writebacks to report.
            writebackToDec.send(cpu_iid, tagged Invalid);
        end

        // Get the local state for the current instance.
        Reg#(COMMITQ_SLOT_ID)   oldestSlot = oldestSlotPool.getReg(cpu_iid);
        
        // Check for any dequeues/clears.
        let m_deq <- deqFromCom.receive(cpu_iid);
        
        if (isValid(m_deq))
        begin
            
            // A dequeue occurred. Just drop the oldest guy.
            debugLog.record(cpu_iid, fshow("3: DEQ"));
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

    rule stage4_allocate (True);

        // Get our context from the previous stage.
        let cpu_iid <- stage4Ctrl.nextReadyInstance();
        
        // Get our local state based on the context.
        LUTRAM#(COMMITQ_SLOT_ID, Bool) completions = completionsPool.getRAMWithWritePort(cpu_iid, 1);
        LUTRAM#(L1_DCACHE_MISS_ID, COMMITQ_SLOT_ID) maf = mafPool.getRAM(cpu_iid);

        Reg#(COMMITQ_SLOT_ID) nextFreeSlot = nextFreeSlotPool.getReg(cpu_iid);
        Reg#(COMMITQ_SLOT_ID) oldestSlot   = oldestSlotPool.getReg(cpu_iid);

        // Check for any new allocations.
        let m_alloc <- allocateFromDMem.receive(cpu_iid);
        let new_next_free = nextFreeSlot;

        if (m_alloc matches tagged Valid {.bundle, .m_miss_id})
        begin

            // A new allocation.
            debugLog.record(cpu_iid, $format("4: ALLOC: ") + fshow(bundle.token) + $format(" COMPLETE: %0b", pack(isValid(m_miss_id))));
            bundlesPool.write(cpu_iid, nextFreeSlot, bundle);
            completions.upd(nextFreeSlot, !isValid(m_miss_id));
            new_next_free = nextFreeSlot + 1;

            if (m_miss_id matches tagged Valid .miss_id)
            begin
            
                debugLog.record(cpu_iid, $format("4: MISS ID: %0d mapped to slot %0d.", miss_id, nextFreeSlot));
                maf.upd(miss_id, nextFreeSlot);
                
            end

        end
        
        // Now check if we have a credit to send to fetch based on our (simulated) length.
        if (new_next_free + 1 != oldestSlot) // This should really be +L+1, where L is the latency of the credit to DMem.
        begin
        
            // We still have room.
            debugLog.record(cpu_iid, fshow("4: SEND CREDIT"));
            creditToDMem.send(cpu_iid, tagged Valid (?));

        end
        else
        begin

            // We're full.
            debugLog.record(cpu_iid, fshow("4: NO CREDIT"));
            creditToDMem.send(cpu_iid, tagged Invalid);

        end
        
        nextFreeSlot <= new_next_free;
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule
