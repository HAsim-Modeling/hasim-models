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
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/memory_base_types.bsh"

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

module [HASIM_MODULE] mkDMem ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_mem.out");

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS) doLoads  <- mkConnection_Client("funcp_doLoads");

    // ****** Ports *****
    
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE)       bundleFromDMemQ   <- mkPortStallRecv_Multiplexed("DMemQ");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)                    creditFromCommitQ <- mkPortRecv_Multiplexed("commitQ_credit", 1);


    PORT_SEND_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_INPUT)          loadToDCache   <- mkPortSend_Multiplexed("CPU_to_DCache_load");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, SB_INPUT)                   reqToSB        <- mkPortSend_Multiplexed("DMem_to_SB_req");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, WB_SEARCH_INPUT)            searchToWB     <- mkPortSend_Multiplexed("DMem_to_WB_search");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple2#(DMEM_BUNDLE, Bool)) allocToCommitQ <- mkPortSend_Multiplexed("commitQ_alloc");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE)                writebackToDec <- mkPortSend_Multiplexed("DMem_to_Dec_hit_writeback");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE)  loadRspFromDCache <- mkPortRecvGuarded_Multiplexed("DCache_to_CPU_load_immediate", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, WB_SEARCH_OUTPUT)              rspFromWB         <- mkPortRecvGuarded_Multiplexed("WB_to_DMem_rsp", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, SB_OUTPUT)                     rspFromSB         <- mkPortRecvGuarded_Multiplexed("SB_to_DMem_rsp", 0);

    // ****** UnModel State ******
    
    FIFO#(CPU_INSTANCE_ID) stage2Q <- mkFIFO();
    FIFO#(Bool)       stage3Q <- mkFIFO();


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) inports  = newVector();
    Vector#(5, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromDMemQ.ctrl;
    inports[1]  = creditFromCommitQ.ctrl;
    outports[0] = loadToDCache.ctrl;
    outports[1] = reqToSB.ctrl;
    outports[2] = allocToCommitQ.ctrl;
    outports[3] = searchToWB.ctrl;
    outports[4] = bundleFromDMemQ.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventMem <- mkEventRecorder_Multiplexed(`EVENTS_DMEM_INSTRUCTION_MEM);

    
    // ****** Rules ******


    // stage1_begin
    
    // Begin a new model cycle for the next context.
    // If the MemQ is non-empty and the CommitQ is non-full then
    // do the memory ops for this instruction.
    // * Non-memory instructions are passed through.
    // * Stores are sent to the store buffer.
    // * Loads are sent to the store buffer and dcache simultaneously.

    rule stage1_begin (True);
    
        // Begin a model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
    
        // Let's see if we have room in our output buffers.
        let credit_from_commitQ <- creditFromCommitQ.receive(cpu_iid);
    
        if (isValid(credit_from_commitQ) && bundleFromDMemQ.canDeq(cpu_iid))
        begin

            // The DMemQ has an instruction in it... and the CommitQ has room.
            let bundle = bundleFromDMemQ.peek(cpu_iid);
            let tok = bundle.token;

            // Let's see if we should contact the store buffer and dcache.
            if (bundle.isLoad && !tokIsPoisoned(tok) && !bundle.isJunk)
            begin

                // It's a load which did not page fault.
                debugLog.record_next_cycle(cpu_iid, fshow("LOAD REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Check if the the load result is either in the cache or the store buffer or the write buffer.
                reqToSB.send(cpu_iid, tagged Valid initSBSearch(bundle));
                searchToWB.send(cpu_iid, tagged Valid initWBSearch(bundle));
                loadToDCache.send(cpu_iid, tagged Valid initDCacheLoad(bundle));

            end
            else if (bundle.isStore && !tokIsPoisoned(tok) && !bundle.isJunk)
            begin
                
                // A store which did not page fault.
                debugLog.record_next_cycle(cpu_iid, fshow("STORE REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Tell the store buffer about this new store.
                reqToSB.send(cpu_iid, tagged Valid initSBComplete(bundle));

                // No load to the DCache. (The write buffer will do the store after it leaves the store buffer.)
                loadToDCache.send(cpu_iid, tagged Invalid);
                searchToWB.send(cpu_iid, tagged Invalid);

                // Add it to the CommitQ already completed.
                bundleFromDMemQ.doDeq(cpu_iid);
                eventMem.recordEvent(cpu_iid, tagged Valid zeroExtend(tokTokenId(tok)));
                allocToCommitQ.send(cpu_iid, tagged Valid tuple2(bundle, True));


            end
            else
            begin

                // Not a memory operation. (Or a faulted memory operation.)
                debugLog.record_next_cycle(cpu_iid, fshow("NO-MEMORY ") + fshow(tok));

                // Don't tell the cache/SB/WB about it.
                reqToSB.send(cpu_iid, tagged Invalid);
                searchToWB.send(cpu_iid, tagged Invalid);
                loadToDCache.send(cpu_iid, tagged Invalid);

                // Add it to the CommitQ already completed.
                bundleFromDMemQ.doDeq(cpu_iid);
                eventMem.recordEvent(cpu_iid, tagged Valid zeroExtend(tokTokenId(tok)));
                allocToCommitQ.send(cpu_iid, tagged Valid tuple2(bundle, True));

            end

        end
        else
        begin

            // A bubble.
            debugLog.record_next_cycle(cpu_iid, fshow("BUBBLE"));

            // No requests for the store buffer or dcache.
            reqToSB.send(cpu_iid, tagged Invalid);
            loadToDCache.send(cpu_iid, tagged Invalid);
            searchToWB.send(cpu_iid, tagged Invalid);
            
            // Propogate the bubble.
            bundleFromDMemQ.noDeq(cpu_iid);
            allocToCommitQ.send(cpu_iid, tagged Invalid);
            eventMem.recordEvent(cpu_iid, tagged Invalid);

        end

        // Get the sb + wb + dcache response in the next stage.
        stage2Q.enq(cpu_iid);

    endrule



    // stage2_loadRsp
    
    // Get the dcache and store buffer response and enqueue it into the CommitQ.
    // If we are in this stage we know that the MemQ is not empty, and the CommitQ is not full.

    rule stage2_loadRsp (True);

        // Get our local context from the previous stage.
        let cpu_iid = stage2Q.first();
        stage2Q.deq();
        
        // Get the responses from the store buffer and dcache.
        let m_sb_rsp <- rspFromSB.receive(cpu_iid);
        let m_wb_rsp <- rspFromWB.receive(cpu_iid);
        let m_dc_rsp <- loadRspFromDCache.receive(cpu_iid);

        if (m_sb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged SB_hit)
        begin

            // We found the data in the Store buffer, 
            // so we don't have to look at the DCache response or Write Buffer response.
            // (Stores in the SB are always younger than those.)
            debugLog.record(cpu_iid, fshow("SB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));
            
            // Send the writeback to decode.
            writebackToDec.send(cpu_iid, tagged Valid genBusMessage(rsp.bundle.token, rsp.bundle.dests));
            
            // Pass it to the next stage through the functional partition, 
            // which actually retrieves the data.
            doLoads.makeReq(initFuncpReqDoLoads(rsp.bundle.token));

            // Tell the next stage it was a hit:
            stage3Q.enq(True);

        end
        else if (m_wb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged WB_hit)
        begin
        
            // We found the data in the Store buffer, 
            // so we don't have to look at the DCache response.
            debugLog.record(cpu_iid, fshow("WB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));

            // Send the writeback to decode.
            writebackToDec.send(cpu_iid, tagged Valid genBusMessage(rsp.bundle.token, rsp.bundle.dests));
            
            // Pass it to the next stage through the functional partition, 
            // which actually retrieves the data.
            doLoads.makeReq(initFuncpReqDoLoads(rsp.bundle.token));

            // Tell the next stage it was a hit:
            stage3Q.enq(True);

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
                    debugLog.record(cpu_iid, fshow("SB MISS, DCACHE HIT ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                    // Send the writeback to decode.
                    writebackToDec.send(cpu_iid, tagged Valid genBusMessage(tok, bundle.dests));

                    // Pass it to the next stage through the functional partition, 
                    // which actually retrieves the data.
                    doLoads.makeReq(initFuncpReqDoLoads(tok));

                    // Tell the next stage it was a hit:
                    stage3Q.enq(True);
                end

                tagged DCACHE_miss:
                begin
                
                    // The cache missed, but is handling it. 
                    debugLog.record(cpu_iid, fshow("SB MISS, DCACHE MISS ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                    // No writebacks to report.
                    writebackToDec.send(cpu_iid, tagged Invalid);

                    // Pass it to the next stage through the functional partition, 
                    // which actually retrieves the data.
                    doLoads.makeReq(initFuncpReqDoLoads(tok));
                    
                    // Tell the next stage it was a miss:
                    stage3Q.enq(False);


                end

                tagged DCACHE_retry:
                begin
                
                    // The SB/WB Missed, and the cache needs us to retry.
                    debugLog.record(cpu_iid, fshow("SB MISS, DCACHE RETRY ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));
                
                    // Don't pass it on to the commitQ. Don't do the load to the functional partition.
                    bundleFromDMemQ.noDeq(cpu_iid);
                    allocToCommitQ.send(cpu_iid, tagged Invalid);
                    eventMem.recordEvent(cpu_iid, tagged Invalid);

                    // No writebacks to report.
                    writebackToDec.send(cpu_iid, tagged Invalid);

                    // End of model cycle. (Path 1)
                    localCtrl.endModelCycle(cpu_iid, 1);

                end

            endcase
        
        end
        else
        begin

            // Otherwise we're just here because of something that wasn't a load.
            debugLog.record(cpu_iid, fshow("NON-LOAD FINISH "));
        
            // Don't report any writebacks.
            writebackToDec.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);
        end
        
    endrule
    
    rule stage3_loadRsp (True);
    
        // Get hit/miss from previous stage.
        let hit = stage3Q.first();
        stage3Q.deq();
        
        // Get the response from the functional partition.
        let rsp = doLoads.getResp();
        doLoads.deq();
        
        // Get our context from the token.
        let ctx = tokContextId(rsp.token);
        
        // assert DMemQ.canDeq
        let bundle = bundleFromDMemQ.peek(ctx);

        // Update the bundle with the new token.
        bundle.token = rsp.token;
        
        // Send it to the commitQ, completed if we got a hit.
        allocToCommitQ.send(ctx, tagged Valid tuple2(bundle, hit));
        bundleFromDMemQ.doDeq(ctx);
        eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(bundle.token)));
        
        // End of model cycle. (Path 3)
        localCtrl.endModelCycle(ctx, 3);
    
    endrule

endmodule

