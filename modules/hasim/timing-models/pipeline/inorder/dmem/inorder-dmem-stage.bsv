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
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/memory_base_types.bsh"

import FIFOF::*;
`include "asim/provides/fpga_components.bsh"
// ****** Generated files ******

`include "asim/dict/EVENTS_DMEM.bsh"


// mkDMem


// Multi-context DMem module which interacts with a store buffer and a data cache. 
// Note that this version assumes that the cache is blocking.

// The module may block on either the cache or the store buffer response.

// This module is pipelined across contexts. Stages:

// Stage 1* -> Stage 2** -> Stage 3 -> Stage 4 -> Stage 5
// * Stage 1 stalls on a bubble from the MemQ. It dequeues the cache response.
// ** Stage 2 stalls when the Store Buffer is full. It dequeues the cache response.

// Possible ways the model cycle can end:
//   Path 1: An instruction is passed to the CommitQ.
//   Path 2: The MemQ is non-empty, but the DCache hasn't gotten back to us with a response.
//   Path 3: The MemQ is empty, or the CommitQ is full, so there's a bubble.
//   Path 4: The Store Buffer is full, so there's a bubble and we retry next cycle.


module [HASIM_MODULE] mkDMem ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_mem.out");


    // ****** Ports *****
    
    PORT_STALL_RECV_MULTICTX#(DMEM_BUNDLE)         bundleFromDMemQ <- mkPortStallRecv_MultiCtx("DMemQ");
    PORT_RECV_MULTICTX#(VOID)                    creditFromCommitQ <- mkPortRecv_MultiCtx("commitQ_credit", 1);


    PORT_SEND_MULTICTX#(DCACHE_LOAD_INPUT)          loadToDCache   <- mkPortSend_MultiCtx("CPU_to_DCache_load");
    PORT_SEND_MULTICTX#(SB_INPUT)                   reqToSB        <- mkPortSend_MultiCtx("DMem_to_SB_req");
    PORT_SEND_MULTICTX#(WB_SEARCH_INPUT)            searchToWB     <- mkPortSend_MultiCtx("DMem_to_WB_search");
    PORT_SEND_MULTICTX#(Tuple2#(DMEM_BUNDLE, Bool)) allocToCommitQ <- mkPortSend_MultiCtx("commitQ_alloc");

    PORT_SEND_MULTICTX#(BUS_MESSAGE)                writebackToDec <- mkPortSend_MultiCtx("DMem_to_Dec_hit_writeback");

    // Zero-latency response ports for stage 2.
    PORT_RECV_MULTICTX#(DCACHE_LOAD_OUTPUT_IMMEDIATE)  loadRspFromDCache <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_load_immediate", 0);
    PORT_RECV_MULTICTX#(WB_SEARCH_OUTPUT)              rspFromWB         <- mkPortRecvGuarded_MultiCtx("WB_to_DMem_rsp", 0);
    PORT_RECV_MULTICTX#(SB_OUTPUT)                     rspFromSB         <- mkPortRecvGuarded_MultiCtx("SB_to_DMem_rsp", 0);

    // ****** UnModel State ******
    
    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(5, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromDMemQ.ctrl;
    inports[1]  = creditFromCommitQ.ctrl;
    outports[0] = loadToDCache.ctrl;
    outports[1] = reqToSB.ctrl;
    outports[2] = allocToCommitQ.ctrl;
    outports[3] = searchToWB.ctrl;
    outports[4] = bundleFromDMemQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Events and Stats ******

    EVENT_RECORDER_MULTICTX eventMem <- mkEventRecorder_MultiCtx(`EVENTS_DMEM_INSTRUCTION_MEM);

    
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
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
    
        // Let's see if we have room in our output buffers.
        let credit_from_commitQ <- creditFromCommitQ.receive(ctx);
    
        if (isValid(credit_from_commitQ) && bundleFromDMemQ.canDeq(ctx))
        begin

            // The DMemQ has an instruction in it... and the CommitQ has room.
            let bundle = bundleFromDMemQ.peek(ctx);
            let tok = bundle.token;

            // Let's see if we should contact the store buffer and dcache.
            if (bundle.isLoad && !tokIsPoisoned(tok) && !bundle.isJunk)
            begin

                // It's a load which did not page fault.
                debugLog.record_next_cycle(ctx, fshow("LOAD REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Check if the the load result is either in the cache or the store buffer or the write buffer.
                reqToSB.send(ctx, tagged Valid initSBSearch(bundle));
                searchToWB.send(ctx, tagged Valid initWBSearch(bundle));
                loadToDCache.send(ctx, tagged Valid initDCacheLoad(bundle));

            end
            else if (bundle.isStore && !tokIsPoisoned(tok) && !bundle.isJunk)
            begin
                
                // A store which did not page fault.
                debugLog.record_next_cycle(ctx, fshow("STORE REQ ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                // Tell the store buffer about this new store.
                reqToSB.send(ctx, tagged Valid initSBComplete(bundle));

                // No load to the DCache. (The write buffer will do the store after it leaves the store buffer.)
                loadToDCache.send(ctx, tagged Invalid);
                searchToWB.send(ctx, tagged Invalid);

                // Add it to the CommitQ already completed.
                bundleFromDMemQ.doDeq(ctx);
                eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(tok)));
                allocToCommitQ.send(ctx, tagged Valid tuple2(bundle, True));


            end
            else
            begin

                // Not a memory operation. (Or a faulted memory operation.)
                debugLog.record_next_cycle(ctx, fshow("NO-MEMORY ") + fshow(tok));

                // Don't tell the cache/SB/WB about it.
                reqToSB.send(ctx, tagged Invalid);
                searchToWB.send(ctx, tagged Invalid);
                loadToDCache.send(ctx, tagged Invalid);

                // Add it to the CommitQ already completed.
                bundleFromDMemQ.doDeq(ctx);
                eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(tok)));
                allocToCommitQ.send(ctx, tagged Valid tuple2(bundle, True));

            end

        end
        else
        begin

            // A bubble.
            debugLog.record_next_cycle(ctx, fshow("BUBBLE"));

            // No requests for the store buffer or dcache.
            reqToSB.send(ctx, tagged Invalid);
            loadToDCache.send(ctx, tagged Invalid);
            searchToWB.send(ctx, tagged Invalid);
            
            // Propogate the bubble.
            bundleFromDMemQ.noDeq(ctx);
            allocToCommitQ.send(ctx, tagged Invalid);
            eventMem.recordEvent(ctx, tagged Invalid);

        end

        // Get the sb + wb + dcache response in the next stage.
        stage2Q.enq(ctx);

    endrule



    // stage2_loadRsp
    
    // Get the dcache and store buffer response and enqueue it into the CommitQ.
    // If we are in this stage we know that the MemQ is not empty, and the CommitQ is not full.

    rule stage2_loadRsp (True);

        // Get our local context from the previous stage.
        let ctx = stage2Q.first();
        stage2Q.deq();
        
        // Get the responses from the store buffer and dcache.
        let m_sb_rsp <- rspFromSB.receive(ctx);
        let m_wb_rsp <- rspFromWB.receive(ctx);
        let m_dc_rsp <- loadRspFromDCache.receive(ctx);

        if (m_sb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged SB_hit)
        begin

            // We found the data in the Store buffer, 
            // so we don't have to look at the DCache response or Write Buffer response.
            // (Stores in the SB are always younger than those.)
            debugLog.record(ctx, fshow("SB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));
            
            // Send it to the commitQ already completed.
            bundleFromDMemQ.doDeq(ctx);
            allocToCommitQ.send(ctx, tagged Valid tuple2(rsp.bundle, True));
            eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(rsp.bundle.token)));
            
            // Send the writeback to decode.
            writebackToDec.send(ctx, tagged Valid genBusMessage(rsp.bundle.token, rsp.bundle.dests));
            
            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(ctx, 1);

        end
        else if (m_wb_rsp matches tagged Valid .rsp &&& rsp.rspType matches tagged WB_hit)
        begin
        
            // We found the data in the Store buffer, 
            // so we don't have to look at the DCache response.
            debugLog.record(ctx, fshow("WB HIT ") + fshow(rsp.bundle.token) + fshow(" ADDR:") + fshow(rsp.bundle.physicalAddress));

            // Send it to the commitQ already completed.
            bundleFromDMemQ.doDeq(ctx);
            allocToCommitQ.send(ctx, tagged Valid tuple2(rsp.bundle, True));
            eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(rsp.bundle.token)));
            
            // Send the writeback to decode.
            writebackToDec.send(ctx, tagged Valid genBusMessage(rsp.bundle.token, rsp.bundle.dests));
            
            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(ctx, 2);

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
                    debugLog.record(ctx, fshow("SB MISS, DCACHE HIT ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                    // Send it to the commitQ already completed.
                    bundleFromDMemQ.doDeq(ctx);
                    allocToCommitQ.send(ctx, tagged Valid tuple2(bundle, True));
                    eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(bundle.token)));

                    // Send the writeback to decode.
                    writebackToDec.send(ctx, tagged Valid genBusMessage(tok, bundle.dests));

                    // End of model cycle. (Path 2)
                    localCtrl.endModelCycle(ctx, 2);

                end

                tagged DCACHE_miss:
                begin
                
                    // The cache missed, but is handling it. 
                    debugLog.record(ctx, fshow("SB MISS, DCACHE MISS ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));

                    // Send it to the commitQ but incomplete.
                    bundleFromDMemQ.doDeq(ctx);
                    allocToCommitQ.send(ctx, tagged Valid tuple2(bundle, False));
                    eventMem.recordEvent(ctx, tagged Valid zeroExtend(tokTokenId(rsp.bundle.token)));

                    // No writebacks to report.
                    writebackToDec.send(ctx, tagged Invalid);

                    // End of model cycle. (Path 3)
                    localCtrl.endModelCycle(ctx, 3);

                end

                tagged DCACHE_retry:
                begin
                
                    // The SB/WB Missed, and the cache needs us to retry.
                    debugLog.record(ctx, fshow("SB MISS, DCACHE RETRY ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.physicalAddress));
                
                    // Don't pass it on to the next stage.
                    bundleFromDMemQ.noDeq(ctx);
                    allocToCommitQ.send(ctx, tagged Invalid);
                    eventMem.recordEvent(ctx, tagged Invalid);

                    // No writebacks to report.
                    writebackToDec.send(ctx, tagged Invalid);

                    // End of model cycle. (Path 4)
                    localCtrl.endModelCycle(ctx, 4);

                end

            endcase
        
        end
        else
        begin

            // Otherwise we're just here because of something that wasn't a load.
            debugLog.record(ctx, fshow("NON-LOAD FINISH "));
        
            // Don't report any writebacks.
            writebackToDec.send(ctx, tagged Invalid);

            // End of model cycle. (Path 5)
            localCtrl.endModelCycle(ctx, 5);
        end
        
    endrule

endmodule

