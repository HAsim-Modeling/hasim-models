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
import FIFO::*;
import FIFOF::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"


// ****** Timing Model imports *****
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"


// ****** Modules ******

// mkInstructionQueue

// A completion buffer which rendezvous ICache miss responses with
// the inorder instruction stream.

// External Interface:
//  "InstQ_to_fet_credit"
//  Whenever more space is freed up in the instruction queue, the instq sends
//  to fetch a count of how much space was freed (conceptually a bunch of
//  credits). The front end must not enqueue anything in the instq unless it
//  has one of these credits.
//
//  "Fet_to_InstQ_enq"
//  When the front end is ready, and if it has an instq credit, it can enqueue
//  a fetch bundle on the instq. The front end should indicate if this bundle
//  is associated with an icache miss, and if so provide the icache miss
//  number. The front end can also specified the bundle as Invalid, in which
//  case the instq doesn't enqueue it but reclaims the credit.
//
//  "ICache_to_CPU_delayed"
//  If there's a icache miss, it will take some time to get the instruction.
//  Once the icache gets the instruction, the icache passes the instruction to
//  the instq, completing the bundle with corresponding icache miss number that
//  has already been enqueued.
//
//  "Fet_to_InstQ_clear"
//  Clear all bundles from the instruction queue.
//
//  "InstQ_to_Dec_first", "Dec_to_InstQ_deq"
//  The instq will send the first bundle on the queue to decode if that bundle
//  is complete. That bundle remains first on the queue until decode explicitly
//  says to dequeue it.

typedef Bit#(TLog#(NUM_INSTQ_SLOTS)) INSTQ_SLOT_ID;


typedef Vector#(NUM_ICACHE_MISS_IDS, Bool) MISS_DROP_MAP;

module [HASIM_MODULE] mkInstructionQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_instq.out");


    // ****** Model State (per instance) ******

    // Queue to rendezvous with instructions.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_SLOT_ID, FETCH_BUNDLE) slotsPool <- mkMemory_Multiplexed(mkBRAM());
    MULTIPLEXED_LUTRAM_MULTI_WRITE#(MAX_NUM_CPUS, 2, INSTQ_SLOT_ID, Bool) completePool <- mkMultiplexedLUTRAMPseudoMultiWrite(False);

    // Record of which expected icache miss delayed updates we should ignore.
    MULTIPLEXED_REG#(MAX_NUM_CPUS, MISS_DROP_MAP) shouldDropPool <- mkMultiplexedReg(replicate(False));

    // Miss Address File (MAF).
    // Maps from icache miss ID to instq slot.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS, L1_ICACHE_MISS_ID, INSTQ_SLOT_ID) mafPool <- mkMemory_Multiplexed(mkBRAM());
    
    // Pointer to the head slot, which contains the next bundle for the decode
    // stage to look at.
    MULTIPLEXED_REG#(MAX_NUM_CPUS, INSTQ_SLOT_ID) headPtrPool <- mkMultiplexedReg(0);
    
    // Pointer to the tail slot, which is the next slot we'll use when
    // enqueuing a new bundle.
    // Note: Queue is EMPTY when headPtr == allocPtr 
    MULTIPLEXED_REG#(MAX_NUM_CPUS, INSTQ_SLOT_ID) tailPtrPool <- mkMultiplexedReg(0);

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_ENQUEUE)          enqFromFetch         <- mkPortRecv_Multiplexed("Fet_to_InstQ_enq", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                   clearFromFetch       <- mkPortRecv_Multiplexed("Fet_to_InstQ_clear", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_DELAYED)  rspFromICacheDelayed <- mkPortRecv_Multiplexed("ICache_to_CPU_load_delayed", 1);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, FETCH_BUNDLE)       bundleToDec   <- mkPortSend_Multiplexed("InstQ_to_Dec_first");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, INSTQ_CREDIT_COUNT) creditToFetch <- mkPortSend_Multiplexed("InstQ_to_Fet_credit");
    
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID) deqFromDec <- mkPortRecvDependent_Multiplexed("Dec_to_InstQ_deq");
        
    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls  = newVector();
    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) unctrl_ctrls = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = enqFromFetch.ctrl;
    inctrls[1]  = clearFromFetch.ctrl;
    inctrls[2]  = rspFromICacheDelayed.ctrl;
    unctrl_ctrls[0] = deqFromDec.ctrl;
    outctrls[0] = creditToFetch.ctrl;
    outctrls[1] = bundleToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("Instruction Queue", inctrls, unctrl_ctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage2Ctrl <- mkStageController();

    // Stages 2 -> 3 flows through a pair of 0 latency (and therefore unbuffered)
    // ports:  InstQ_to_Dec_first -> decode stage -> Dec_to_InstQ_deq.  The
    // stage controller must leave buffer space for the multi-FPGA-cycle path
    // to avoid simulator pipeline bubbles.
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage3Ctrl <- mkBufferedStageController();

    // ****** Rules ******

    // stage1_first
    // Send the head to decode.
    //
    // Ports Read:
    //  (none)
    // Ports Written
    // (none)

    (* conservative_implicit_conditions *)
    rule stage1_first (True);
                
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our local state based on the current instance.
        LUTRAM#(INSTQ_SLOT_ID, Bool) complete = completePool.getRAMWithWritePort(cpu_iid, 0);
        Reg#(INSTQ_SLOT_ID) head_ptr = headPtrPool.getReg(cpu_iid);
        Reg#(INSTQ_SLOT_ID) tail_ptr = tailPtrPool.getReg(cpu_iid);

        // Request slot associated with head in case it is needed.
        slotsPool.readReq(cpu_iid, head_ptr);

        // Send the head bundle to decode if ready.
        let empty = (head_ptr == tail_ptr);
        if (!empty && complete.sub(head_ptr))
        begin
            // It's ready to go.
            debugLog.record_next_cycle(cpu_iid, $format("1: SLOT READY: 0x%0h", head_ptr));
            stage2Ctrl.ready(cpu_iid, True);
        end
        else
        begin
            // We're not ready to send anything to the decode.
            debugLog.record_next_cycle(cpu_iid, $format("1: NO SEND: ")
                + $format(empty ? "EMPTY" : "HEAD NOT COMPLETE"));

            stage2Ctrl.ready(cpu_iid, False);
        end

    endrule

    // stage2_complete
    // Rendezvous an icache miss with it's bundle, if any.
    //
    // Ports Read:
    // * rspFromICacheDelayed
    // Ports Written
    // * bundleToDec
    rule stage2_complete (True);
        match {.cpu_iid, .slot_ready} <- stage2Ctrl.nextReadyInstance();
        let first <- slotsPool.readRsp(cpu_iid);

        // Get our local state based on the current instance.
        Reg#(MISS_DROP_MAP) shouldDrop = shouldDropPool.getReg(cpu_iid);
        MISS_DROP_MAP should_drop = shouldDrop;

        if (slot_ready)
        begin
            // It's ready to go.
            debugLog.record_next_cycle(cpu_iid, $format("2: SEND READY SLOT: ADDR:0x%h", first.pc));
            bundleToDec.send(cpu_iid, tagged Valid first);
        end
        else
        begin
            // We're not ready to send anything to the decode.
            bundleToDec.send(cpu_iid, tagged Invalid);
        end


        // Check for icache rendezvous.
        let m_rendezvous <- rspFromICacheDelayed.receive(cpu_iid);
        Bool is_complete = False;
        if (m_rendezvous matches tagged Valid .rsp)
        begin
            if (should_drop[rsp.missID])
            begin
                debugLog.record(cpu_iid, fshow("DROP DELAYED. MISS ID = ") + fshow(rsp.missID));
            end
            else
            begin
                is_complete = True;
                mafPool.readReq(cpu_iid, rsp.missID);

                debugLog.record(cpu_iid, fshow("COMPLETE. MISS ID = ") + fshow(rsp.missID));
            end
        end

        // Pass this instance to the next stage.
        stage3Ctrl.ready(cpu_iid, is_complete);
    endrule

    // stage3_deq_clear_enq_credit
    // Handle deq, clear, and enq
    // Send credits to fetch.
    //
    // Ports Read:
    // * deqFromDec
    // * clearFromFetch
    // * enqFromFetch
    // Ports Written
    // * creditToFetch
    rule stage3_deq_clear_enq_credit (True);
        // Get the info from the previous stage.
        match {.cpu_iid, .is_complete} <- stage3Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        LUTRAM#(INSTQ_SLOT_ID, Bool) complete0 = completePool.getRAMWithWritePort(cpu_iid, 0);
        LUTRAM#(INSTQ_SLOT_ID, Bool) complete1 = completePool.getRAMWithWritePort(cpu_iid, 1);
        Reg#(MISS_DROP_MAP) shouldDrop = shouldDropPool.getReg(cpu_iid);
        MISS_DROP_MAP should_drop = shouldDrop;
        Reg#(INSTQ_SLOT_ID) headPtr = headPtrPool.getReg(cpu_iid);
        Reg#(INSTQ_SLOT_ID) tailPtr = tailPtrPool.getReg(cpu_iid);
        INSTQ_SLOT_ID head_ptr = headPtr;
    
        INSTQ_CREDIT_COUNT credits = 0;

        // Check for dequeue from decode.
        let m_deq <- deqFromDec.receive(cpu_iid);

        // Finish completion from previous stage?
        if (is_complete)
        begin
            INSTQ_SLOT_ID slot <- mafPool.readRsp(cpu_iid);

            // A real instQ would update the instruction here.
            // However we already have the actual instruction, so we just mark
            // it as ready to go.
            complete0.upd(slot, True);
        end

        if (isValid(m_deq))
        begin
            // A dequeue occurred. Just drop the oldest guy.
            debugLog.record(cpu_iid, $format("DEQ. NEW HEAD: 0x%0h", head_ptr+1));
            head_ptr = head_ptr + 1;
            credits = credits + 1;
        end

        // Check for clear from fetch.
        let m_clear <- clearFromFetch.receive(cpu_iid);
        if (isValid(m_clear))
        begin
            debugLog.record(cpu_iid, $format("CLEAR"));

            INSTQ_CREDIT_COUNT add = tailPtr - head_ptr;
            credits = credits + add;
            head_ptr = tailPtr;
            should_drop = replicate(True);
        end


        // Check for enqueue
        let m_enq <- enqFromFetch.receive(cpu_iid);

        if (m_enq matches tagged Valid .enq)
        begin
            if (enq.bundle matches tagged Valid .bundle)
            begin
                // Enqueue a bundle

                slotsPool.write(cpu_iid, tailPtr, bundle);
                complete1.upd(tailPtr, !isValid(enq.missID));
                tailPtr <= tailPtr + 1;

                debugLog.record(cpu_iid, $format("ENQ pc = 0x%0h", bundle.pc));

                if (enq.missID matches tagged Valid .miss_id)
                begin
                    mafPool.write(cpu_iid, miss_id, tailPtr);
                    should_drop[miss_id] = False;
                end
            end
            else
            begin
                // No bundle to enqueue. Reclaim the credit.
                credits = credits+1;

                debugLog.record(cpu_iid, $format("NO ENQ"));

                if (enq.missID matches tagged Valid .miss_id)
                begin
                    should_drop[miss_id] = True;
                end
            end

        end

        // Send credits to fetch.
        debugLog.record(cpu_iid, $format("CREDITING TO FETCH %0d", credits));
        creditToFetch.send(cpu_iid, tagged Valid credits);

        // State updates.
        headPtr <= head_ptr;
        shouldDrop <= should_drop;
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule

