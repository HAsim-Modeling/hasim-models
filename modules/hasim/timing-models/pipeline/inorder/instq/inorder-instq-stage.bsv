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
//  Instruction Queue assumes every model cycle the front end will want a new
//  slot in the instruction queue. If there's an available slot, instruction
//  queue "credits" the slot to the front end. The credited slot must be
//  "completed" before it can be reused.
//
//  "Fet_to_InstQ_allocate"
//  When the front end is ready, it should send the instq a fetch bundle and
//  info about how the slot will be used. This includes whether to use or
//  "poison" the slot, and whether or not the icache will later update the
//  slot. 
//
//  If poison is true, the data in the slot belongs to a wrong path
//  instruction. It will be dropped from the queue.  If the slot is
//  "poisoned", we assume that fetch will poison all credits not yet
//  allocated.
//
//  If complete is true, the slot is considered "complete". If
//  complete is false, the icache will send the correct instruction for that
//  slot some number of model cycles in the future via ICache_to_CPU_delayed.
// 
//  "ICache_to_CPU_delayed"
//  If there's a cache miss, it will take some time to get the instruction.
//  Once the cache gets it, it passes it to the instq, thus completing the
//  slot.
//
//  "Fet_to_InstQ_clear"
//  If there's a fault or rewind, the allocated slots should be cleared.
//
//  "InstQ_to_Dec_first", "Dec_to_InstQ_deq"
//  The instq will send the first bundle on the queue to decode if that bundle
//  is complete and not poisoned. That bundle remains first on the queue until
//  decode explicitly says to dequeue it. (Remember, poisoned bundles are
//  automatically dropped and are never sent to decode).

// Internal Stuff:
//  It's very complicated. Ask me some other time and maybe I'll tell you
//  about it if I'm in a good mood.

// The OUTSTANDING_BOARD marks which slots have outstanding potential icache
// rendezvous. If element [i][j] is True, then slot i, side j has an
// outstanding request.
typedef Vector#(NUM_INSTQ_SLOTS, Vector#(NUM_INSTQ_SIDES, Bool)) OUTSTANDING_BOARD;

// The SIDE_MAP says which side is being used for each slot.
typedef Vector#(NUM_INSTQ_SLOTS, INSTQ_SIDE) SIDE_MAP;

module [HASIM_MODULE] mkInstructionQueue
    // interface:
        ();
    
    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_instq.out");


    // ****** Model State (per instance) ******

    // Which slots have outstanding potential icache rendezvous .
    MULTIPLEXED#(NUM_CPUS, Reg#(OUTSTANDING_BOARD)) outstandingPool <- mkMultiplexed(mkReg(replicate(replicate(False))));

    // Queue to rendezvous with instructions.
    MULTIPLEXED#(NUM_CPUS, LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE)) bundlesPool <- mkMultiplexed(mkLUTRAMU());
    
    // What side are we using of each slot?
    MULTIPLEXED#(NUM_CPUS, Reg#(SIDE_MAP)) sideMapPool <- mkMultiplexed(mkRegU);
    
    // Pointer to the head slot, which contains the next bundle for the decode
    // stage to look at.
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID)) headPtrPool <- mkMultiplexed(mkReg(0));
    
    // Pointer to the next slot we expect fetch to "allocate". All slots
    // between here and the head slot contain bundles we plan to send to
    // decode eventually. On poison we reset the credit pointer here.
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID)) allocPtrPool <- mkMultiplexed(mkReg(0));

    // Pointer to the next slot we'll credit to the front end.
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_SLOT_ID)) creditPtrPool <- mkMultiplexed(mkReg(0));

    // Poison epoch. If incoming poisoned allocations don't match the epoch, we
    // reset the credit pointer back to the alloc pointer so we don't have any
    // holes in the queue.
    MULTIPLEXED#(NUM_CPUS, Reg#(INSTQ_POISON_EPOCH)) epochPool <- mkMultiplexed(mkReg(0));

    // Some interesting facts:
    //  Queue is EMPTY when headPtr == allocPtr 
    //  Queue is FULL when creditPtr+1 == headPtr
    //  Slots between headPtr and allocPtr contain bundles allocated by fetch
    //  but not yet dequeued by decode.
    //  Slots between allocPtr and creditPtr are slots fetch will soon be
    //  allocating. They may be poisoned.


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, INSTQ_ALLOCATION)      allocateFromFetch    <- mkPortRecv_Multiplexed("Fet_to_InstQ_allocate", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)                  clearFromFetch <- mkPortRecv_Multiplexed("Fet_to_InstQ_clear", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Bool)                  creditResetFromFetch <- mkPortRecv_Multiplexed("Fet_to_InstQ_creditReset", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED) rspFromICacheDelayed <- mkPortRecv_Multiplexed("ICache_to_CPU_delayed", 0);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, FETCH_BUNDLE)    bundleToDec  <- mkPortSend_Multiplexed("InstQ_to_Dec_first");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, INSTQ_CREDIT) creditToFetch  <- mkPortSend_Multiplexed("InstQ_to_Fet_credit");
    
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)        deqFromDec <- mkPortRecvDependent_Multiplexed("Dec_to_InstQ_deq");
        
    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(NUM_CPUS)) inctrls  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = allocateFromFetch.ctrl;
    inctrls[1]  = clearFromFetch.ctrl;
    inctrls[2]  = creditResetFromFetch.ctrl;
    inctrls[3]  = rspFromICacheDelayed.ctrl;
    outctrls[0] = creditToFetch.ctrl;
    outctrls[1] = bundleToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage3Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_CPUS) stage4Ctrl <- mkStageControllerVoid();

    // ****** Rules ******

    // Ports read:
    // * rspFromICacheDelayed 
    //
    // Ports written:
    // * bundleToDec;

    (* conservative_implicit_conditions *)
    rule stage1_first (True);
                
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our local state based on the current instance.
        Reg#(OUTSTANDING_BOARD) outstanding = outstandingPool[cpu_iid];
        Reg#(SIDE_MAP) sideMap = sideMapPool [cpu_iid];
        LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE) bundles = bundlesPool[cpu_iid];

        Reg#(INSTQ_SLOT_ID) headPtr = headPtrPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) allocPtr = allocPtrPool[cpu_iid];

        // Send the head bundle to decode if ready.
        let empty = (headPtr == allocPtr);
        let head_side = sideMap[headPtr];
        let head_rdy = !outstanding[headPtr][head_side];
        if (!empty && head_rdy)
        begin
            // It's ready to go.
            let bundle = bundles.sub(headPtr);
            debugLog.record_next_cycle(cpu_iid, $format("SEND READY SLOT: 0x%0h, ADDR:0x%h", headPtr, bundle.pc));
            bundleToDec.send(cpu_iid, tagged Valid bundle);
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

    rule stage2_complete;
        let cpu_iid <- stage2Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        Reg#(OUTSTANDING_BOARD) outstanding = outstandingPool[cpu_iid];
        Reg#(SIDE_MAP) sideMap = sideMapPool [cpu_iid];
        LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE) bundles = bundlesPool[cpu_iid];

        // Check for icache rendezvous.
        let m_rendezvous <- rspFromICacheDelayed.receive(cpu_iid);
        if (m_rendezvous matches tagged Valid .rsp)
        begin
            
            // TODO: use rsp.missID to determine which slot it goes into, rather than rsp.bundle.instQCredit.
        
            debugLog.record(cpu_iid, fshow("COMPLETE: ") + fshow(rsp.bundle.instQCredit));
            let credit = rsp.bundle.instQCredit;

            // Mark the virtual slot as no longer outstanding.
            outstanding[credit.slot][credit.side] <= False;

            // update the bundle if it's the right side
            if (credit.side == sideMap[credit.slot])
            begin
                let bundle = bundles.sub(credit.slot);
                bundle.inst = rsp.bundle.instruction;
                bundles.upd(credit.slot, bundle);
            end
        end

        // Pass this instance to the next stage.
        stage3Ctrl.ready(cpu_iid);

    endrule
    

    // stage3_deallocate
    // Check if decode is dequeing.
    //
    // Ports read:
    // * deqFromDec
    // Ports written:
    // (none)

    rule stage3_deallocate (True);

        // Get the info from the previous stage.
        let cpu_iid <- stage3Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        Reg#(INSTQ_SLOT_ID) headPtr = headPtrPool[cpu_iid];

        // Check for any dequeues from decode.
        let m_deq <- deqFromDec.receive(cpu_iid);

        if (isValid(m_deq))
        begin
            
            // A dequeue occurred. Just drop the oldest guy.
            debugLog.record(cpu_iid, $format("DEQ. NEW HEAD: 0x%0h", headPtr+1));
            headPtr <= headPtr + 1;

        end
        
        // Send it to the next stage.
        stage4Ctrl.ready(cpu_iid);

    endrule


    // stage4_clear_allocate_credit
    
    // Check for allocation requests. Based on allocate and deallocate
    // we can calculate a new credit to be sent to back to Fetch.
    
    // Ports read:
    // * clearFromFetch
    // * creditResetFromFetch
    // * allocateFromFetch
    // Ports written:
    // * creditToFetch

    rule stage4_clear_allocate_credit (True);

        // Get our instance from the previous stage.
        let cpu_iid <- stage4Ctrl.nextReadyInstance();

        // Get our local state based on the current instance.
        Reg#(OUTSTANDING_BOARD) outstandingReg = outstandingPool[cpu_iid];
        let outstanding = outstandingReg;
        Reg#(SIDE_MAP) sideMap = sideMapPool[cpu_iid];
        LUTRAM#(INSTQ_SLOT_ID, FETCH_BUNDLE) bundles = bundlesPool[cpu_iid];
        Reg#(INSTQ_POISON_EPOCH) epochReg = epochPool[cpu_iid];
        let epoch = epochReg;

        Reg#(INSTQ_SLOT_ID) headPtr = headPtrPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) allocPtr = allocPtrPool[cpu_iid];
        Reg#(INSTQ_SLOT_ID) creditPtr = creditPtrPool[cpu_iid];
        let head_ptr = headPtr;
        let alloc_ptr = allocPtr;
        let credit_ptr = creditPtr;

        // Check for a clear request.
        let m_clear <- clearFromFetch.receive(cpu_iid);
        let m_reset <- creditResetFromFetch.receive(cpu_iid);
        Bool keep_allocation = False;
        Bool credit_reset = False;
        if (isValid(m_clear))
        begin
            // Clear the queue.
            debugLog.record(cpu_iid, $format("CLEAR. New Head: 0x%0h", alloc_ptr));
            head_ptr = alloc_ptr;

            credit_reset = True;
            epoch = epoch + 1;
        end
        else if (m_reset matches tagged Valid .ka)
        begin
            // Reset the credit pointer
            debugLog.record(cpu_iid, $format("CREDIT RESET. New Credit Pointer: 0x%0h, keep alloc: ", alloc_ptr) + $format(ka ? "YES" : "NO"));

            keep_allocation = ka;
            credit_reset = True;
            epoch = epoch + 1;
        end

        
        // Check for any new allocations.
        let m_alloc <- allocateFromFetch.receive(cpu_iid);

        if (m_alloc matches tagged Valid .alloc)
        begin
            // A new allocation.
            debugLog.record(cpu_iid,
                $format("ALLOC ADDR:0x%h", alloc.bundle.pc)
              + $format(", CREDIT: ") + fshow(alloc.credit)
              + $format(", DELAYED: ", (isValid(alloc.missID) ? "YES" : "NO"))
            );

            if (alloc.credit.epoch == epoch || keep_allocation)
            begin
                bundles.upd(alloc.credit.slot, alloc.bundle);
                alloc_ptr = alloc.credit.slot+1;

                debugLog.record(cpu_iid, $format("ALLOC ACCEPTED. NEW ALLOC PTR: 0x%0h", alloc_ptr));
            end
            else
            begin
                debugLog.record(cpu_iid, $format("ALLOC POISONED. ALLOC PTR REMAINS: 0x%0h", alloc_ptr));
            end

            if (!isValid(alloc.missID))
            begin
                // This slot no longer is outstanding.
                debugLog.record(cpu_iid, $format("COMPLETE: ") + fshow(alloc.credit));
                outstanding[alloc.credit.slot][alloc.credit.side] = False;
            end

        end

        // Now do the credit reset if needed.
        if (credit_reset)
        begin
            credit_ptr = alloc_ptr;
        end

        
        // Now check if we have a credit to send to fetch.
        // NOTE: It's credit_ptr+2 so we don't advance the credit_ptr to the
        // head pointer.
        Bool full = (credit_ptr + 2 == head_ptr);
        Bool some_side_is_free = (!outstanding[credit_ptr][0] || !outstanding[credit_ptr][1]);
        if (!full && some_side_is_free)
        begin
            // We have room in this slot.
            INSTQ_SIDE free_side = outstanding[credit_ptr][0] ? 1 : 0;
            let credit = INSTQ_CREDIT {
                slot: credit_ptr,
                side: free_side,
                epoch: epoch
            };            
            creditToFetch.send(cpu_iid, tagged Valid credit);
            outstanding[credit_ptr][free_side] = True;
            sideMap[credit_ptr] <= free_side;
            credit_ptr = credit_ptr + 1;

            debugLog.record(cpu_iid, $format("SEND CREDIT: ") + fshow(credit));
        end
        else
        begin

            // No space here.
            debugLog.record(cpu_iid, fshow("NO CREDIT"));
            creditToFetch.send(cpu_iid, tagged Invalid);

        end

        outstandingReg <= outstanding;
        creditPtr <= credit_ptr;
        allocPtr <= alloc_ptr;
        headPtr <= head_ptr;
        epochReg <= epoch;

        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);
        
    endrule

endmodule

