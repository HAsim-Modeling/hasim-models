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

typedef struct {
    FETCH_BUNDLE bundle;
    Bool complete;
} INSTQ_SLOT_DATA deriving(Bits, Eq);

typedef Vector#(NUM_ICACHE_MISS_IDS, Bool) MISS_DROP_MAP;

module [HASIM_MODULE] mkInstructionQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_instq.out");


    // ****** Model State (per instance) ******

    // Queue to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(INSTQ_SLOT_ID, INSTQ_SLOT_DATA)) slotsPool <- mkMultiplexed(mkLUTRAMU());

    // Record of which expected icache miss delayed updates we should ignore.
    MULTIPLEXED#(NUM_CPUS, Reg#(MISS_DROP_MAP)) shouldDropPool <- mkMultiplexed(mkRegU());

    // Miss Address File (MAF).
    // Maps from icache miss ID to instq slot.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(ICACHE_MISS_ID, INSTQ_SLOT_ID)) mafPool <- mkMultiplexed(mkLUTRAMU());
    
    // Pointer to the head slot, which contains the next bundle for the decode
    // stage to look at.
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID)) headPtrPool <- mkMultiplexed(mkReg(0));
    
    // Pointer to the tail slot, which is the next slot we'll use when
    // enqueuing a new bundle.
    // Note: Queue is EMPTY when headPtr == allocPtr 
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID)) tailPtrPool <- mkMultiplexed(mkReg(0));

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, INSTQ_ENQUEUE)          enqFromFetch         <- mkPortRecv_Multiplexed("Fet_to_InstQ_enq", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)                   clearFromFetch       <- mkPortRecv_Multiplexed("Fet_to_InstQ_clear", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED)  rspFromICacheDelayed <- mkPortRecv_Multiplexed("ICache_to_CPU_delayed", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, FETCH_BUNDLE)       bundleToDec   <- mkPortSend_Multiplexed("InstQ_to_Dec_first");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, INSTQ_CREDIT_COUNT) creditToFetch <- mkPortSend_Multiplexed("InstQ_to_Fet_credit");
    
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID) deqFromDec <- mkPortRecvDependent_Multiplexed("Dec_to_InstQ_deq");
        
    // ****** Local Controller ******

    Vector#(3, INSTANCE_CONTROL_IN#(NUM_CPUS)) inctrls  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = enqFromFetch.ctrl;
    inctrls[1]  = clearFromFetch.ctrl;
    inctrls[2]  = rspFromICacheDelayed.ctrl;
    outctrls[0] = creditToFetch.ctrl;
    outctrls[1] = bundleToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage4Ctrl <- mkStageControllerVoid();

    // ****** Rules ******

    // stage1_first
    // Send the head to decode.
    //
    // Ports Read:
    //  (none)
    // Ports Written
    // * bundleToDec

    (* conservative_implicit_conditions *)
    rule stage1_first (True);
                
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our local state based on the current instance.
        LUTRAM#(INSTQ_SLOT_ID, INSTQ_SLOT_DATA) slots = slotsPool[cpu_iid];
        INSTQ_SLOT_ID head_ptr = headPtrPool[cpu_iid];
        INSTQ_SLOT_ID tail_ptr = tailPtrPool[cpu_iid];

        // Send the head bundle to decode if ready.
        let empty = (head_ptr == tail_ptr);
        let first = slots.sub(head_ptr);
        if (!empty && first.complete)
        begin
            // It's ready to go.
            debugLog.record_next_cycle(cpu_iid, $format("SEND READY SLOT: 0x%0h, ADDR:0x%h", head_ptr, first.bundle.pc));
            bundleToDec.send(cpu_iid, tagged Valid first.bundle);
        end
        else
        begin
            // We're not ready to send anything to the decode.
            debugLog.record_next_cycle(cpu_iid, $format("NO SEND: ")
                + $format(empty ? "EMPTY" : "HEAD NOT COMPLETE"));

            bundleToDec.send(cpu_iid, tagged Invalid);
        end

        stage2Ctrl.ready(cpu_iid);

    endrule

    // stage2_complete
    // Rendezvous an icache miss with it's bundle, if any.
    //
    // Ports Read:
    // * rspFromICacheDelayed
    // Ports Written
    // (none)
    rule stage2_complete (True);
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        LUTRAM#(INSTQ_SLOT_ID, INSTQ_SLOT_DATA) slots = slotsPool[cpu_iid];
        Reg#(MISS_DROP_MAP) shouldDrop = shouldDropPool[cpu_iid];
        MISS_DROP_MAP should_drop = shouldDrop;
        LUTRAM#(ICACHE_MISS_ID, INSTQ_SLOT_ID) maf = mafPool[cpu_iid];

        // Check for icache rendezvous.
        let m_rendezvous <- rspFromICacheDelayed.receive(cpu_iid);
        if (m_rendezvous matches tagged Valid .rsp)
        begin
            if (should_drop[rsp.missID])
            begin
                debugLog.record(cpu_iid, fshow("DROP DELAYED. MISS ID = ") + fshow(rsp.missID));
            end
            else
            begin
                INSTQ_SLOT_ID slot = maf.sub(rsp.missID);

                let slot_data = slots.sub(slot);
                slot_data.bundle.inst = rsp.bundle.instruction;
                slot_data.complete = True;
                slots.upd(slot, slot_data);

                debugLog.record(cpu_iid, fshow("COMPLETE. MISS ID = ") + fshow(rsp.missID));
            end
        end

        // Pass this instance to the next stage.
        stage3Ctrl.ready(cpu_iid);
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
        let cpu_iid <- stage3Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        LUTRAM#(INSTQ_SLOT_ID, INSTQ_SLOT_DATA) slots = slotsPool[cpu_iid];
        Reg#(MISS_DROP_MAP) shouldDrop = shouldDropPool[cpu_iid];
        MISS_DROP_MAP should_drop = shouldDrop;
        LUTRAM#(ICACHE_MISS_ID, INSTQ_SLOT_ID) maf = mafPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) headPtr = headPtrPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) tailPtr = tailPtrPool[cpu_iid];
        INSTQ_SLOT_ID head_ptr = headPtr;
    
        INSTQ_CREDIT_COUNT credits = 0;

        // Check for dequeue from decode.
        let m_deq <- deqFromDec.receive(cpu_iid);

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
                let slot_data = INSTQ_SLOT_DATA {
                    bundle: bundle,
                    complete: !isValid(enq.missID)
                };
        
                slots.upd(tailPtr, slot_data);
                tailPtr <= tailPtr + 1;

                debugLog.record(cpu_iid, $format("ENQ pc = 0x%0h", bundle.pc));

                if (enq.missID matches tagged Valid .miss_id)
                begin
                    maf.upd(miss_id, tailPtr);
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

