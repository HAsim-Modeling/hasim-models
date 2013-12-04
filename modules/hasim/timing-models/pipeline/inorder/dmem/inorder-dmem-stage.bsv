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


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

import FIFOF::*;
`include "asim/provides/fpga_components.bsh"
// ****** Generated files ******

`include "asim/dict/EVENTS_DMEM.bsh"


// mkDMem


// Multiplexed DMem module which interacts with a store buffer and a data cache. 
// Note that this version assumes that the cache is blocking.

// The module may block on either the cache or the store buffer response.

// This module is pipelined across instances. Stages:

// Stage 1* -> Stage 2** -> Stage 3 -> Stage 4 -> Stage 5
// * Stage 1 stalls on a bubble from the MemQ. It dequeues the cache response.
// ** Stage 2 stalls when the Store Buffer is full. It dequeues the cache response.

// Possible ways the model cycle can end:
//   Path 1: An instruction is passed to the CommitQ.
//   Path 2: The MemQ is non-empty, but the DCache hasn't gotten back to us with a response.
//   Path 3: The MemQ is empty, or the CommitQ is full, so there's a bubble.
//   Path 4: The Store Buffer is full, so there's a bubble and we retry next cycle.

typedef union tagged
{
    DMEM_BUNDLE STAGE2_completed;
    void        STAGE2_loadRsp;
    void        STAGE2_bubble;
}
DMEM_STAGE2_STATE deriving (Bits, Eq);

typedef union tagged
{
    DMEM_BUNDLE                                     STAGE3_completed;
    Tuple2#(DMEM_BUNDLE, Maybe#(L1_DCACHE_MISS_ID)) STAGE3_loadRsp;
    void                                            STAGE3_bubble;
}
DMEM_STAGE3_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDMem ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_mem.out");

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS) doLoads  <- mkConnection_Client("funcp_doLoads");

    // ****** Ports *****
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DMEM_BUNDLE) bundleFromDMemQ   <- mkPortStallRecv_Multiplexed("DMemQ");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID)              creditFromCommitQ <- mkPortRecv_Multiplexed("commitQ_credit", 1);


    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_INPUT)          loadToDCache   <- mkPortSend_Multiplexed("CPU_to_DCache_load");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, SB_INPUT)                   reqToSB        <- mkPortSend_Multiplexed("DMem_to_SB_req");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, WB_SEARCH_INPUT)            searchToWB     <- mkPortSend_Multiplexed("DMem_to_WB_search");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(DMEM_BUNDLE, Maybe#(L1_DCACHE_MISS_ID))) allocToCommitQ <- mkPortSend_Multiplexed("commitQ_alloc");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)                writebackToDec <- mkPortSend_Multiplexed("DMem_to_Dec_hit_writeback");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspFromDCache <- mkPortRecvDependent_Multiplexed("DCache_to_CPU_load_immediate");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, WB_SEARCH_OUTPUT)              rspFromWB         <- mkPortRecvDependent_Multiplexed("WB_to_DMem_rsp");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, SB_OUTPUT)                     rspFromSB         <- mkPortRecvDependent_Multiplexed("SB_to_DMem_rsp");


    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports  = newVector();
    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) depports = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromDMemQ.ctrl.in;
    inports[1]  = creditFromCommitQ.ctrl;
    depports[0] = loadRspFromDCache.ctrl;
    depports[1] = rspFromWB.ctrl;
    depports[2] = rspFromSB.ctrl;
    outports[0] = loadToDCache.ctrl;
    outports[1] = reqToSB.ctrl;
    outports[2] = allocToCommitQ.ctrl;
    outports[3] = searchToWB.ctrl;
    outports[4] = bundleFromDMemQ.ctrl.out;
    outports[5] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("DMem", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DMEM_STAGE2_STATE) stage2Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DMEM_STAGE3_STATE) stage3Ctrl <- mkBufferedStageController();


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMem <- mkEventRecorder_Multiplexed(`EVENTS_DMEM_INSTRUCTION_MEM);

    
    // ****** Rules ******


    // stage1_begin
    
    // Begin a new model cycle for the next context.
    // If the MemQ is non-empty and the CommitQ is non-full then
    // do the memory ops for this instruction.
    // * Non-memory instructions are passed through.
    // * Stores are sent to the store buffer.
    // * Loads are sent to the store buffer and dcache simultaneously.

    // Ports read:
    // * creditFromCommitQ
    // * bundleFromDMemQ

    // Ports written:
    // * reqToSB
    // * searchToWB
    // * loadToDCache

    (* conservative_implicit_conditions *)
    rule stage1_begin (True);
    
        // Begin a model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
    
        // Let's see if we have room in our output buffers.
        let m_credit <- creditFromCommitQ.receive(cpu_iid);
        let m_bundle <- bundleFromDMemQ.receive(cpu_iid);
    
        if (m_bundle matches tagged Valid .bundle &&& isValid(m_credit))
        begin

            // The DMemQ has an instruction in it... and the CommitQ has room.
            let tok = bundle.token;

            // Let's see if we should contact the store buffer and dcache.
            if (bundle.isLoad && !tokIsPoisoned(tok) && !tokIsDummy(tok))
            begin

                // It's a load which did not page fault.
                debugLog.record_next_cycle(cpu_iid, fshow("1: LOAD REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Check if the the load result is either in the cache or the store buffer or the write buffer.
                reqToSB.send(cpu_iid, tagged Valid initSBSearch(bundle));
                searchToWB.send(cpu_iid, tagged Valid initWBSearch(bundle));
                loadToDCache.send(cpu_iid, tagged Valid initDCacheLoad(bundle));
                
                // Tell the next stage to look for the load responses from the various ports.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_loadRsp);

            end
            else if (bundle.isStore && !tokIsPoisoned(tok) && !tokIsDummy(tok))
            begin
                
                // A store which did not page fault.
                debugLog.record_next_cycle(cpu_iid, fshow("1: STORE REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Tell the store buffer about this new store.
                reqToSB.send(cpu_iid, tagged Valid initSBComplete(bundle));

                // No load to the DCache. (The write buffer will do the store after it leaves the store buffer.)
                loadToDCache.send(cpu_iid, tagged Invalid);
                searchToWB.send(cpu_iid, tagged Invalid);

                // Tell the next stage to send the bundle on completed.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_completed bundle);

            end
            else
            begin

                // Not a memory operation. (Or a faulted memory operation.)
                debugLog.record_next_cycle(cpu_iid, fshow("1: NO-MEMORY ") + fshow(tok));

                // Don't tell the cache/SB/WB about it.
                reqToSB.send(cpu_iid, tagged Invalid);
                searchToWB.send(cpu_iid, tagged Invalid);
                loadToDCache.send(cpu_iid, tagged Invalid);

                stage2Ctrl.ready(cpu_iid, tagged STAGE2_completed bundle);

            end

        end
        else
        begin

            // A bubble.
            debugLog.record_next_cycle(cpu_iid, fshow("1: BUBBLE"));

            // No requests for the store buffer or dcache.
            reqToSB.send(cpu_iid, tagged Invalid);
            loadToDCache.send(cpu_iid, tagged Invalid);
            searchToWB.send(cpu_iid, tagged Invalid);
            
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

        end

    endrule


    // stage2_loadRsp
    
    // Get the dcache, write buffer, and store buffer response (if any).
    // If we got a hit, report the writeback to decode.

    // Ports read:
    // * rspFromSB
    // * rspFromWB
    // * loadRspFromDCache
    
    // Ports written:
    // * writebackToDec

    (* conservative_implicit_conditions *)
    rule stage2_loadRsp (True);

        // Get the current instance id from the previous stage.
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get the responses from the store buffer, write buffer, and dcache.
        let m_sb_rsp <- rspFromSB.receive(cpu_iid);
        let m_wb_rsp <- rspFromWB.receive(cpu_iid);
        let m_dc_rsp <- loadRspFromDCache.receive(cpu_iid);
        
        if (state matches STAGE2_bubble)
        begin

            // It was a bubble, so we have no writebacks to report.
            debugLog.record(cpu_iid, fshow("2: BUBBLE"));
        
            // Don't report any writebacks.
            writebackToDec.send(cpu_iid, tagged Invalid);

            // Finish the bubble in the next stage.
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_bubble);

        end
        else if (state matches tagged STAGE2_completed .info)
        begin
        
            // We're just here because of something that was completed.
            debugLog.record(cpu_iid, fshow("2: NON-LOAD"));
        
            // Don't report any writebacks.
            writebackToDec.send(cpu_iid, tagged Invalid);

            // Do the enqueue in the next stage.
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_completed info);
        
        end
        else if (state matches tagged STAGE2_loadRsp)
        begin
        
            // Let's check the responses from the DCache/ Write buffer/ Store buffer
        
            if (m_sb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged SB_hit)
            begin

                // We found the data in the Store buffer, 
                // so we don't have to look at the DCache response or Write Buffer response.
                // (Stores in the SB are always younger than those.)
                debugLog.record(cpu_iid, fshow("2: SB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));

                // Send the writeback to decode.  The branch epoch is irrelevant
                // at this late stage.
                let epoch = initEpoch(?, rsp.bundle.faultEpoch);
                writebackToDec.send(cpu_iid, tagged Valid genBusMessage(rsp.bundle.token,
                                                                        epoch,
                                                                        rsp.bundle.dests));

                // Pass it to the next stage through the functional partition, 
                // which actually retrieves the data.
                doLoads.makeReq(initFuncpReqDoLoads(rsp.bundle.token));

                // Tell the next stage it was a hit:
                stage3Ctrl.ready(cpu_iid, tagged STAGE3_loadRsp tuple2(rsp.bundle, tagged Invalid));

            end
            else if (m_wb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged WB_hit)
            begin

                // We found the data in the Write buffer, 
                // so we don't have to look at the DCache response.
                debugLog.record(cpu_iid, fshow("2: SB MISS/WB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));

                // Send the writeback to decode.
                let epoch = initEpoch(?, rsp.bundle.faultEpoch);
                writebackToDec.send(cpu_iid, tagged Valid genBusMessage(rsp.bundle.token,
                                                                        epoch,
                                                                        rsp.bundle.dests));

                // Pass it to the next stage through the functional partition, 
                // which actually retrieves the data.
                doLoads.makeReq(initFuncpReqDoLoads(rsp.bundle.token));

                // Tell the next stage it was a hit:
                stage3Ctrl.ready(cpu_iid, tagged STAGE3_loadRsp tuple2(rsp.bundle, tagged Invalid));

            end
            else if (m_dc_rsp matches tagged Valid .rsp)
            begin

                // Let's see if the DCache found it.
                let bundle = rsp.bundle;
                let tok = rsp.bundle.token;

                case (rsp.rspType) matches

                    tagged DCACHE_hit:
                    begin

                        // Well, the cache found it.
                        debugLog.record(cpu_iid, fshow("2: SB/WB MISS, DCACHE HIT ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                        // Send the writeback to decode.
                        let epoch = initEpoch(?, bundle.faultEpoch);
                        writebackToDec.send(cpu_iid, tagged Valid genBusMessage(tok,
                                                                                epoch,
                                                                                bundle.dests));

                        // Pass it to the next stage through the functional partition, 
                        // which actually retrieves the data.
                        doLoads.makeReq(initFuncpReqDoLoads(tok));

                        // Tell the next stage it was a hit:
                        stage3Ctrl.ready(cpu_iid, tagged STAGE3_loadRsp tuple2(bundle, tagged Invalid));
                    end

                    tagged DCACHE_miss .miss_id:
                    begin

                        // The cache missed, but is handling it. 
                        debugLog.record(cpu_iid, fshow("2: SB/WB MISS, DCACHE MISS ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                        // No writebacks to report.
                        writebackToDec.send(cpu_iid, tagged Invalid);

                        // Pass it to the next stage through the functional partition, 
                        // which actually retrieves the data.
                        doLoads.makeReq(initFuncpReqDoLoads(tok));

                        // Tell the next stage it was a miss:
                        stage3Ctrl.ready(cpu_iid, tagged STAGE3_loadRsp tuple2(bundle, tagged Valid miss_id));

                    end

                    tagged DCACHE_retry:
                    begin

                        // The SB/WB Missed, and the cache needs us to retry.
                        debugLog.record(cpu_iid, fshow("2: SB MISS, DCACHE RETRY ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                        // No writebacks to report.
                        writebackToDec.send(cpu_iid, tagged Invalid);

                        stage3Ctrl.ready(cpu_iid, tagged STAGE3_bubble);

                    end

                endcase

            end

        end
        
    endrule
    
    // stage3_fpRsp
    
    // Get the functional partition response (if any).
    // Enqueue the instruction into the commitQ, completed if its response is not coming from the DCache.

    // Ports read:
    // * None
    
    // Ports written:
    // * bundleFromDMemQ
    // * allocToCommitQ

    rule stage3_loadRsp (True);
    
        match {.cpu_iid, .state} <- stage3Ctrl.nextReadyInstance();
        
        if (state matches tagged STAGE3_bubble)
        begin
        
            // Don't pass anything to the commitQ.
            bundleFromDMemQ.noDeq(cpu_iid);
            allocToCommitQ.send(cpu_iid, tagged Invalid);
            eventMem.recordEvent(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE3_completed .bundle)
        begin
        
            // Add it to the CommitQ with no miss id.
            bundleFromDMemQ.doDeq(cpu_iid);
            allocToCommitQ.send(cpu_iid, tagged Valid tuple2(bundle, tagged Invalid));
            eventMem.recordEvent(cpu_iid, tagged Valid zeroExtend(tokTokenId(bundle.token)));

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        else if (state matches tagged STAGE3_loadRsp {.bundle, .m_miss_id})
        begin
        
            // Get the response from the functional partition.
            let rsp = doLoads.getResp();
            doLoads.deq();

            // Update the bundle with the new token.
            let new_bundle = bundle;
            new_bundle.token = rsp.token;

            // Send it to the commitQ, completed if we got a hit.
            bundleFromDMemQ.doDeq(cpu_iid);
            allocToCommitQ.send(cpu_iid, tagged Valid tuple2(new_bundle, m_miss_id));
            eventMem.recordEvent(cpu_iid, tagged Valid zeroExtend(tokTokenId(new_bundle.token)));

            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(cpu_iid, 3);

        end
    
    endrule

endmodule

