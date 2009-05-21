//
// Copyright (C) 2009 Massachusetts Institute of Technology
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


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model imports *****

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/memory_base_types.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"


// ****** Local Datatypes ******

typedef union tagged
{
    void        STAGE2_bubble;
    ISA_ADDRESS STAGE2_fetch;
    void        STAGE2_faultRsp;
    ISA_ADDRESS STAGE2_rewindRsp;
}
PCC_STAGE2_STATE deriving (Bits, Eq);


// ****** Modules ******


// mkPCCalc

// Inorder PCCalc stage. Takes the responses from the branch predictor and 
// ITLB and ICache combines them with branch resteers from EXE and faults from
// Commit. Produces a new PC, and may enqueue into the InstructionQ.

// Normal flow:
// stage1: no fault, rewind, icache retry, or TLB page fault. Take branch prediction. Enqueue into InstructionQ

// Ways to end a model cycle:

// Path 1: Normal flow
// Path 2: A bubble because the InstQ is out of space.
// Path 3: A fault from commit.
// Path 4: A rewind due to branch misprediction, ICache retry, or ITLB fault.

// NOTE: ITLB faults may have a different call to the functional partition in the future.


module [HASIM_MODULE] mkPCCalc
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_pccalc.out");

    // ****** Model State (per Context) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_FAULT_EPOCH))  faultEpochPool  <- mkMultiplexed(mkReg(0));
    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_BRANCH_EPOCH)) branchEpochPool <- mkMultiplexed(mkReg(0));
    MULTIPLEXED#(NUM_CPUS, Reg#(IMEM_ITLB_EPOCH))    iTLBEpochPool   <- mkMultiplexed(mkReg(0));
    MULTIPLEXED#(NUM_CPUS, Reg#(IMEM_ICACHE_EPOCH))  iCacheEpochPool <- mkMultiplexed(mkReg(0));

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");
 
    Connection_Client#(FUNCP_REQ_HANDLE_FAULT,
                       FUNCP_RSP_HANDLE_FAULT)       handleFault <- mkConnection_Client("funcp_handleFault");
 

    // ****** Ports ******


    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple3#(INSTQ_SLOT_ID, FETCH_BUNDLE, Maybe#(ISA_INSTRUCTION))) bundleToInstQ <- mkPortSend_Multiplexed("Fet_to_InstQ_allocate");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH)) nextPCToFetch <- mkPortSend_Multiplexed("PCCalc_to_Fet_newpc");

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, TOKEN) faultFromCom  <- mkPortRecv_Multiplexed("Com_to_Fet_fault", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(TOKEN, TOKEN_FAULT_EPOCH, ISA_ADDRESS)) rewindFromExe <- mkPortRecv_Multiplexed("Exe_to_Fet_rewind", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple2#(IMEM_EPOCH, ISA_ADDRESS))  faultFromIMem <- mkPortRecv_Multiplexed("IMem_to_Fet_fault", 0);

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE)  immRspFromICache <- mkPortRecv_Multiplexed("ICache_to_CPU_immediate", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS)              predFromBP       <- mkPortRecv_Multiplexed("BP_to_Fet_pred", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BRANCH_ATTR)              attrFromBP       <- mkPortRecv_Multiplexed("BP_to_Fet_attr", 0);



    // ****** Local Controller ******

    Vector#(6, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();

    inports[0] = faultFromCom.ctrl;
    inports[1] = rewindFromExe.ctrl;
    inports[2] = faultFromIMem.ctrl;
    inports[3] = immRspFromICache.ctrl;
    inports[4] = predFromBP.ctrl;
    inports[5] = attrFromBP.ctrl;
    outports[0]  = bundleToInstQ.ctrl;
    outports[1]  = nextPCToFetch.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, PCC_STAGE2_STATE) stage2Ctrl <- mkStageController();


    // ****** Rules ******
    
    // stage1_nextPC
    
    // Calculate the next PC based on our input ports.
    // Also calculate the ICache and ITLB epochs.
    // These epochs are not passed to the rest of the pipeline.
    
    // Also enqueues the ICache response into the instructionQ.
    
    // Ports read:
    // * rewindFromExe
    // * faultFromCom
    // * immRspFromICache
    // * faultFromIMem
    // * predFromBP
    
    // Ports written:
    // * bundleToInstQ
    
    (* conservative_implicit_conditions *)
    rule stage1_nextPC (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our state from the contexts.
        Reg#(TOKEN_FAULT_EPOCH)   faultEpoch = faultEpochPool[cpu_iid];
        Reg#(TOKEN_BRANCH_EPOCH) branchEpoch = branchEpochPool[cpu_iid];
        Reg#(IMEM_ITLB_EPOCH)      iTLBEpoch = iTLBEpochPool[cpu_iid];
        Reg#(IMEM_ICACHE_EPOCH)  iCacheEpoch = iCacheEpochPool[cpu_iid];

        // Receive potential new PCs from our incoming ports.
        let m_rewind     <- rewindFromExe.receive(cpu_iid);
        let m_fault      <- faultFromCom.receive(cpu_iid);
        let m_cache_rsp  <- immRspFromICache.receive(cpu_iid);
        let m_itlb_fault <- faultFromIMem.receive(cpu_iid);
        let m_pred_pc    <- predFromBP.receive(cpu_iid);
        
        // Later pipeline stages are given higher priority.
        // fault > rewind > icache retry > itlb fault > prediction
        
        // Branch epochs are stored in the token and the functional partition.
        // We track the fault epoch ourselves here.
        // Assumed invariant:
        // We will get no more redirects from that stage until an instruction
        // of the new epoch reaches it. 
        // (IE the stage has a local copy of the epoch and silently drops 
        //  until it sees the new epoch.)

        if (m_fault matches tagged Valid { .tok })
        begin
            
            // A fault occurred. Get the handler PC from the functional partition.
            debugLog.record_next_cycle(cpu_iid, fshow("FAULT: ") + fshow(tok));
            handleFault.makeReq(initFuncpReqHandleFault(tok));
            faultEpoch <= faultEpoch + 1;

            // Tell the next stage to get the response from the functional partition.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_faultRsp);

        end
        else if (m_rewind matches tagged Valid { .tok, .fault_epoch, .addr} &&&
                 fault_epoch == faultEpoch)
        begin

            // A branch misprediction occured.
            // Epoch check ensures we haven't already redirected from a fault.
            debugLog.record_next_cycle(cpu_iid, fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            branchEpoch <= branchEpoch + 1;

            // Tell the next stage to wait until the functional partition indicates the rewind is complete.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_rewindRsp addr);

        end
        else if (m_cache_rsp matches tagged Valid .rsp &&&
                 rsp.rspType matches tagged ICACHE_retry &&&
                  rsp.bundle.epoch.fault == faultEpoch &&&
                 rsp.bundle.epoch.branch == branchEpoch &&&
                 rsp.bundle.epoch.iCache == iCacheEpoch)
        begin

            // A cache retry occured.
            // Epoch check ensures we haven't already redirected from a fault or a branch misprediction.
            debugLog.record_next_cycle(cpu_iid, $format("ICACHE RETRY ADDR:0x%h", rsp.bundle.virtualAddress));
            
            iCacheEpoch <= iCacheEpoch + 1;
            
            // The next stage will redirect to the new PC.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_fetch rsp.bundle.virtualAddress);

        
        end
        else if (m_itlb_fault matches tagged Valid {.epoch, .handler_addr} &&&
                          epoch.fault == faultEpoch  &&&
                         epoch.branch == branchEpoch &&&
                         epoch.iCache == iCacheEpoch)
        begin
        
            // An ITLB fault occured. Jump to the provided handler address.
            // (Note: in the future we could be the one to retrieve the handler, as with faults.)
            debugLog.record_next_cycle(cpu_iid, $format("ITLB PAGE FAULT ADDR:0x%h", handler_addr));
            
            iTLBEpoch <= iTLBEpoch + 1;

            // The next stage will redirect to the new PC.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_fetch handler_addr);
        
        end
        else if (m_pred_pc matches tagged Valid .pred_pc)
        begin

            // We got a response from the branch predictor. (This should be the normal flow.)
            debugLog.record_next_cycle(cpu_iid, $format("BRANCH PRED:0x%h", pred_pc));

            // The next stage will fetch the predicted PC.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_fetch pred_pc);

        end
        else
        begin
            
            // There's a bubble, so keep the PC the same.
            debugLog.record_next_cycle(cpu_iid, fshow("BUBBLE"));

            // The next stage will propogate the bubble.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

        end

        // Now we handle enqueuing into the instructionQ. First, get the branch attributes.
        let m_attr <- attrFromBP.receive(cpu_iid);

        case (m_cache_rsp) matches

            tagged Invalid:
            begin

                // No ICache response, so propogate the bubble.
                debugLog.record_next_cycle(cpu_iid, fshow("CACHE BUBBLE"));
                bundleToInstQ.send(cpu_iid, tagged Invalid); 
                
            end

            tagged Valid .rsp &&& (rsp.rspType == ICACHE_hit):
            begin

                // We got a hit, but maybe it follows a retry.
                if (rsp.bundle.epoch.iCache == iCacheEpoch)
                begin

                    // It's on the right path.
                    debugLog.record_next_cycle(cpu_iid, $format("INSTQ ALLOC COMPLETED SLOT: %0d ADDR:0x%h", rsp.bundle.instQSlot, rsp.bundle.virtualAddress));

                    // assert isValid m_attr
                    let bundle = FETCH_BUNDLE {branchEpoch: rsp.bundle.epoch.branch, faultEpoch: rsp.bundle.epoch.fault, pc: rsp.bundle.virtualAddress, inst: ?, branchAttr: validValue(m_attr)};

                    // Enqueue it completed with an instruction.
                    bundleToInstQ.send(cpu_iid, tagged Valid tuple3(rsp.bundle.instQSlot, bundle, tagged Valid rsp.bundle.instruction));

                end
                else
                begin

                    // It's on the wrong path.
                    debugLog.record_next_cycle(cpu_iid, $format("CACHE HIT EPOCH DROP SLOT: %0d ADDR:0x%h", rsp.bundle.instQSlot, rsp.bundle.virtualAddress));
                    bundleToInstQ.send(cpu_iid, tagged Invalid); 

                end

            end

            tagged Valid .rsp &&& (rsp.rspType == ICACHE_miss):
            begin
                
                // We got a miss, but maybe it follows a retry.
                if (rsp.bundle.epoch.iCache == iCacheEpoch)
                begin

                    // It's on the right path.
                    debugLog.record_next_cycle(cpu_iid, $format("INSTQ ALLOC INCOMPLETE SLOT: %0d ADDR:0x%h", rsp.bundle.instQSlot, rsp.bundle.virtualAddress));

                    // Update the bundle epochs
                    let epoch = initEpoch(rsp.bundle.epoch.branch, rsp.bundle.epoch.fault);

                    // assert isValid m_attr
                    let bundle = FETCH_BUNDLE {branchEpoch: rsp.bundle.epoch.branch, faultEpoch: rsp.bundle.epoch.fault, pc: rsp.bundle.virtualAddress, inst: ?, branchAttr: validValue(m_attr)};

                    // Enqueue it incomplete with no instruction.
                    bundleToInstQ.send(cpu_iid, tagged Valid tuple3(rsp.bundle.instQSlot, bundle, tagged Invalid));

                end
                else
                begin

                    // It's on the wrong path.
                    debugLog.record_next_cycle(cpu_iid, $format("CACHE MISS EPOCH SLOT: %0d DROP ADDR:0x%h", rsp.bundle.instQSlot, rsp.bundle.virtualAddress));
                    bundleToInstQ.send(cpu_iid, tagged Invalid); 

                end
            end
            
            tagged Valid .rsp &&& (rsp.rspType == ICACHE_retry):
            begin
            
                // It's a retry. Don't enqueue anything.
                debugLog.record_next_cycle(cpu_iid, $format("CACHE RETRY NO ENQ SLOT: %0d ADDR:0x%h", rsp.bundle.instQSlot, rsp.bundle.virtualAddress));
                bundleToInstQ.send(cpu_iid, tagged Invalid); 
            
            end
        endcase

    endrule


    // stage2_rewindRsp
    
    // Get a rewind/fault response (if any), transmit the next PC.
    
    // Ports read:
    // * None
    
    // Ports written:
    // * nextPCToFetch

    rule stage2_rewindRsp (True);
        
        
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get the state for the the current active instance.
        Reg#(IMEM_ITLB_EPOCH)      iTLBEpoch = iTLBEpochPool[cpu_iid];
        Reg#(IMEM_ICACHE_EPOCH)  iCacheEpoch = iCacheEpochPool[cpu_iid];
        Reg#(TOKEN_FAULT_EPOCH)   faultEpoch = faultEpochPool[cpu_iid];
        Reg#(TOKEN_BRANCH_EPOCH) branchEpoch = branchEpochPool[cpu_iid];
        
        if (state matches tagged STAGE2_bubble)
        begin
        
            // Send the new PC to the Fetch unit.        
            nextPCToFetch.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE2_fetch .new_pc)
        begin
        
            // Send the new PC to the Fetch unit.        
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, initIMemEpoch(iTLBEpoch, iCacheEpoch, branchEpoch, faultEpoch)));

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        if (state matches tagged STAGE2_rewindRsp .new_pc)
        begin

            // Get the rewind response.
            let rsp = rewindToToken.getResp();
            rewindToToken.deq();

            // Send the new PC to the Fetch unit.        
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, initIMemEpoch(iTLBEpoch, iCacheEpoch, branchEpoch, faultEpoch)));

            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(cpu_iid, 3);

        end
        else if (state matches tagged STAGE2_faultRsp)
        begin

            // Get the fault response.
            let rsp = handleFault.getResp();
            handleFault.deq();

            // Resteer to the handler.
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(rsp.nextInstructionAddress, initIMemEpoch(iTLBEpoch, iCacheEpoch, branchEpoch, faultEpoch)));

            // End of model cycle. (Path 4)
            localCtrl.endModelCycle(cpu_iid, 4);
        
        end

    endrule

endmodule

