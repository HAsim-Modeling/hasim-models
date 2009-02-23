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
`include "asim/provides/hasim_icache.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_DMEM.bsh"


// ****** Local types ******

// MEM1_STATE

// The first stage may stall for a bubble.

typedef union tagged
{
    void MEM1_ready;
    CONTEXT_ID MEM1_stall;
}
    MEM1_STATE
        deriving (Bits, Eq);


// MEM2_STATE

// The second stage may stall if the store buffer is full.

typedef union tagged
{
    void MEM2_ready;
    CONTEXT_ID MEM2_stall;
}
    MEM2_STATE
        deriving (Bits, Eq);


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


    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(Bool)) ctx_stalled <- mkMultiCtx(mkReg(False));
    

    // ****** Ports *****
    
    PORT_STALL_RECV_MULTICTX#(BUNDLE) bundleFromMemQ  <- mkPortStallRecv_MultiCtx("MemQ");
    PORT_STALL_SEND_MULTICTX#(BUNDLE) bundleToCommitQ <- mkPortStallSend_MultiCtx("CommitQ");

    PORT_SEND_MULTICTX#(BUS_MESSAGE) writebackToDec <- mkPortSend_MultiCtx("Mem_to_Dec_writeback");

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheInput))                      reqToDCache <- mkPortSend_MultiCtx("CPU_to_DCache_speculative");
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed))     rspFromDCacheDelayed <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_speculative_delayed", 0);
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate)) rspFromDCacheImmediate <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_speculative_immediate", 0);

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheInput))  reqToSB   <- mkPortSend_MultiCtx("DMem_to_SB_req");
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, SB_RESPONSE)) rspFromSB <- mkPortRecvGuarded_MultiCtx("SB_to_DMem_rsp", 0);


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE, FUNCP_RSP_DO_DTRANSLATE) doDTranslate  <- mkConnection_Client("funcp_doDTranslate");
    Connection_Client#(FUNCP_REQ_DO_LOADS, FUNCP_RSP_DO_LOADS) doLoads  <- mkConnection_Client("funcp_doLoads");
    Connection_Client#(FUNCP_REQ_DO_STORES,FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(5, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromMemQ.ctrl;
    // inports[1]  = rspFromDCacheDelayed.ctrl;
    // inports[2]  = rspFromDCacheImmediate.ctrl;
    // inports[3]  = rspFromSB.ctrl;
    inports[1]  = bundleToCommitQ.ctrl;
    outports[0] = bundleToCommitQ.ctrl;
    outports[1] = writebackToDec.ctrl;
    outports[2] = reqToDCache.ctrl;
    outports[3] = reqToSB.ctrl;
    outports[4] = bundleFromMemQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** UnModel Pipeline State ******

    Reg#(MEM1_STATE) stage1State <- mkReg(MEM1_ready);
    Reg#(MEM2_STATE) stage2State <- mkReg(MEM2_ready);
    
    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();

    // ****** Events and Stats ******

    EVENT_RECORDER_MULTICTX eventMem <- mkEventRecorder_MultiCtx(`EVENTS_DMEM_INSTRUCTION_MEM);


    // ****** Helper Functions ******

    // finishCycle
    
    // Finish the cycle for any instruction.

    function Action finishCycle(TOKEN tok, BUNDLE bundle);
    action
        
        // Retrieve the context.
        let ctx = tokContextId(tok);
    
        // Dequeue the MemQ
        bundleFromMemQ.doDeq(ctx);
        
        // Record any token updates from the FP.
        bundle.token = tok;
        
        // Enqueue the instruction in the CommitQ
        bundleToCommitQ.doEnq(ctx, bundle);

        if (bundle.isLoad)
        begin
            // Mark a load's destinations as ready.
            writebackToDec.send(ctx, tagged Valid genBusMessage(tok, bundle.dests, False));
            debugLog.record(ctx, fshow(tok) + fshow(": marking load dest reg(s) valid"));
        end
        else
        begin
            // No writebacks to report.
            writebackToDec.send(ctx, tagged Invalid);
        end
        
        // End of model cycle. (Path 1)
        eventMem.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
        localCtrl.endModelCycle(ctx, 1);

    endaction
    endfunction
    
    // finishCycleStall
    
    // Finish a cycle where there's been a stall on the DCache.
    // This is only called if the MemQ is non-empty.

    function Action finishCycleStall(CONTEXT_ID ctx);
    action
    
        // assert bundleFromMemQ.canDeq, otherwise wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);
        let tok = bundle.token;

        debugLog.record(ctx, fshow("DCACHE STALL ") + fshow(tok));
        
        // Don't dequeue the MemQ.
        bundleFromMemQ.noDeq(ctx);
        
        // Don't enqueue anything to the CommitQ.
        bundleToCommitQ.noEnq(ctx);

        // No writebacks to report.
        writebackToDec.send(ctx, tagged Invalid);
        
        // End of model cycle. (Path 2)
        eventMem.recordEvent(ctx, tagged Invalid);
        localCtrl.endModelCycle(ctx, 2);

    endaction
    endfunction


    // makeDTransReq
    
    // For memory ops, request the physical address from the functional partition.
    // For others, just end the model cycle.

    // Note: in a more realistic memory hierarchy this call is better made by
    // the simulated TLB.

    function Action makeDTransReq (CONTEXT_ID ctx);
    action
    
        // assert portFromMemQ.canDeq, otherwise wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);
        let tok = bundle.token;

        // For memory ops, get the translation.
        if (bundle.isLoad)
        begin
            debugLog.record(ctx, fshow("FUNCP-REQ LOAD DTRANS ") + fshow(tok));
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
        end
        else if (bundle.isStore)
        begin
            debugLog.record(ctx, fshow("FUNCP-REQ STORE DTRANS") + fshow(tok));
            doDTranslate.makeReq(initFuncpReqDoDTranslate(tok));
        end
        else
        begin
            finishCycle(tok, bundle);
        end

    endaction
    endfunction
    
    
    // ****** Rules ******


    // stage1_begin
    
    // Begin a new model cycle for the next context.
    // If we're stalling on the cache, wait for a response. 
    // (If so, then we previously determined the MemQ is non-empty, and the CommitQ non-full.)
    // Otherwise, if the MemQ is non-empty and the CommitQ is non-full then
    // do the memory ops for this instruction. Non-memory instructions are passed through.
    // Memory instructions are sent to the store buffer and passed to the next stage.

    rule stage1_begin (stage1State matches tagged MEM1_ready);
    
        // Begin a model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
    
        Reg#(Bool) stalled = ctx_stalled[ctx];
    
        if (stalled)
        begin
        
            // Assert MemQ.canDeq() from a previous cycle.
            let bundle = bundleFromMemQ.peek(ctx);
            let tok = bundle.token;
        
            // This context is stalling on a previous cache miss.
            debugLog.record_next_cycle(ctx, fshow("SB NO REQ, BECAUSE STALLED ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
            
            // Don't request anything from the store buffer.
            reqToSB.send(ctx, tagged Invalid);

            // Send it to the next stage, which will try to un-stall.
            stage2Q.enq(ctx);
        
        end
        else 
        begin
    
            if (!bundleToCommitQ.canEnq(ctx) || !bundleFromMemQ.canDeq(ctx))
            begin

                // A bubble.
                debugLog.record_next_cycle(ctx, fshow("BUBBLE"));
                
                // Don't dequeue the MemQ.
                bundleFromMemQ.noDeq(ctx);

                // Don't enqueue anything into the commitQ.
                bundleToCommitQ.noEnq(ctx);

                // Don't report any writebacks.
                writebackToDec.send(ctx, tagged Invalid);

                // No requests for the store buffer or cache.
                reqToSB.send(ctx, tagged Invalid);
                reqToDCache.send(ctx, tagged Invalid);
                
                // Stall the pipeline for the sb + dcache response.
                stage1State <= tagged MEM1_stall ctx;

            end
            else
            begin

                // The MemQ has an instruction in it... and the CommitQ has room.
                let bundle = bundleFromMemQ.peek(ctx);
                let tok = bundle.token;


                // Let's see if we should contact the store buffer.
                if (bundle.isLoad)
                begin

                    // Check if the store buffer has the data for this load.
                    debugLog.record_next_cycle(ctx, fshow("SB LOAD ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                    reqToSB.send(ctx, tagged Valid tuple2(tok, Data_read_mem_ref(tuple2(?/*passthru*/, bundle.effAddr))));

                end
                else if (bundle.isStore)
                begin
                    
                    // Tell the store buffer about this new store.
                    debugLog.record_next_cycle(ctx, fshow("SB STORE ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                    reqToSB.send(ctx, tagged Valid tuple2(tok, Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr))));

                end
                else
                begin

                    // Nothing to tell the store buffer.
                    debugLog.record_next_cycle(ctx, fshow("NO-MEMORY ") + fshow(tok));
                    reqToSB.send(ctx, tagged Invalid);

                end

                // Send it to the next stage.
                stage2Q.enq(ctx);

            end

        end

    endrule


    // stage1_finishStall
    
    // Get the cache and SB responses and drop them.

    // Note: if we know that the local controller won't tell us to start
    // the same context again until this context is done, then we wouldn't
    // have to stall the pipeline for this case.

    rule stage1_finishStall (stage1State matches tagged MEM1_stall .ctx);

        // assert all of these are Invalid.
        let imm <- rspFromDCacheImmediate.receive(ctx);
        let del <- rspFromDCacheDelayed.receive(ctx);
        let sbr <- rspFromSB.receive(ctx);

        // Unstall the pipeline.
        stage1State <= tagged MEM1_ready;

        // End of model cycle. (Path 3)
        eventMem.recordEvent(ctx, tagged Invalid);
        localCtrl.endModelCycle(ctx, 3);

    endrule


    // stage2_storeBufferRsp
    
    // Get the store buffer response and send it to the cache.
    // If we are in this stage we know that the MemQ is not empty, and the CommitQ is not full.

    rule stage2_storeBufferRsp (stage2State matches tagged MEM2_ready);

        // Get our local context from the previous stage.
        let ctx = stage2Q.first();
        stage2Q.deq();

        // assert bundleFromMemQ.canDeq(). Otherwise we wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);
        
        // Get the response from the store buffer.
        let m_rsp <- rspFromSB.receive(ctx);

        case (m_rsp) matches
            tagged Invalid:
            begin

                // It was a non-memory operation, or we are stalled.
                // Either way, there's nothing to tell the cache.
                reqToDCache.send(ctx, tagged Invalid);

            end
            tagged Valid { .tok, .rsp }:
            begin
            
                case (rsp) matches
                    SB_HIT:
                    begin

                        // We found the data in the Store buffer, 
                        // so we don't have to ask the DCache.
                        debugLog.record(ctx, fshow("SB HIT ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                        reqToDCache.send(ctx, tagged Invalid);

                    end
                    SB_MISS:
                    begin

                        // We missed in the store buffer, so ask the DCache.
                        debugLog.record(ctx, fshow("SB MISS, DCACHE LOAD ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                        reqToDCache.send(ctx, tagged Valid tuple2(tok, Data_read_mem_ref(tuple2(?/*passthru*/, bundle.effAddr))));

                    end
                    SB_STALL:
                    begin
                        
                        // The store buffe is full, so we'll retry this request next cycle.
                        debugLog.record(ctx, fshow("SB STALL ") + fshow(tok) + fshow(" ADDR:") + fshow(bundle.effAddr));
                        reqToDCache.send(ctx, tagged Invalid);
                        
                        // Stall this stage for the dcache response.
                        stage2State <= tagged MEM2_stall ctx;

                    end
                endcase
            end
        endcase
        
        // Either way, pass it on to the next stage.
        stage3Q.enq(ctx);

    endrule


    rule stage2_finishStall (stage2State matches tagged MEM2_stall .ctx);

        // assert these are Invalid.
        let imm <- rspFromDCacheImmediate.receive(ctx);
        let del <- rspFromDCacheDelayed.receive(ctx);

        // Unstall the pipeline.
        stage2State <= tagged MEM2_ready;

        // End of model cycle. (Path 4)
        eventMem.recordEvent(ctx, tagged Invalid);
        localCtrl.endModelCycle(ctx, 4);

    endrule


    // stage3_dcacheRsp
    
    // Get the DCache responses (if any). When the DCache comes 
    // back we can ask the functional partition to do the actual operation.

    // Note

    rule stage3_dcacheRsp (True);

        // Get our local state from the context.
        let ctx = stage3Q.first();
        stage3Q.deq();
        Reg#(Bool) stalled = ctx_stalled[ctx];

        // Check the DCache responses.
        let m_imm <- rspFromDCacheImmediate.receive(ctx);
        let m_del <- rspFromDCacheDelayed.receive(ctx);

        // assert !isValid(m_imm) && !isValid(m_del)

        case (m_del) matches

            tagged Invalid:
            begin

                // No delayed response, so check the immediate response.

                case (m_imm) matches

                    tagged Invalid:
                    begin

                        // No cache response.
                        if (stalled)
                        begin

                            // It's because we're still stalled on a miss.
                            finishCycleStall(ctx);

                        end
                        else
                        begin

                            // It's because it was a non-memory op. 
                            // Just go to the next stage.
                            makeDTransReq(ctx);

                        end

                    end
                    
                    tagged Valid { .tok, .msg }:
                    begin

                        case (msg) matches
                        
                            tagged Hit .*:
                            begin
                            
                                // A hit! Go on to the next stage via 
                                makeDTransReq(ctx);
                                
                            end

                            tagged Miss_servicing .*:
                            begin

                                // This module will no longer check the MemQ 
                                // until a response comes back.
                                stalled <= True;
                                finishCycleStall(ctx);

                            end

                            tagged Hit_servicing .*:
                            begin

                                noAction;
                                
                            end

                            tagged Miss_retry .*:
                            begin

                                // We must retry our request.
                                // Don't dequeue the MemQ and try again next cycle.
                                finishCycleStall(ctx);
                            end

                        endcase

                    end

                endcase

            end

            tagged Valid { .tok, .msg }:
            begin

                // assert stalled == True
                // assert m_imm == Invalid

                case (msg) matches
                
                    tagged Miss_response .*:
                    begin

                        // It came back, so we can resume normal operations.
                        stalled <= False;
                        makeDTransReq(ctx);

                    end

                    tagged Hit_response .*:
                    begin

                        // FIXME: This currently is unimplemented.
                        noAction;

                    end

                endcase

            end

        endcase

    endrule


    // stage4_dtransRsp
    
    // Get the physical address from the functional partition. 
    // Ask it to begin the actual load/store operation.

    rule stage4_dtransRsp (True);

        // Get the response from the functional partition.
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();

        // Get our local state from the context.
        let tok = rsp.token;
        let ctx = tokContextId(tok);

        // assert bundleFromMemQ.canDeq(). Otherwise we wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);

        // Wait for all translation responses.  Stay in this state if more are
        // coming.  Otherwise, do the operation.
        if (! rsp.hasMore)
        begin

            if (bundle.isLoad)
            begin

                debugLog.record(ctx, fshow("FUNCP-REQ LOAD ") + fshow(tok));
                doLoads.makeReq(initFuncpReqDoLoads(rsp.token));

            end
            else if (bundle.isStore)
            begin

                debugLog.record(ctx, fshow("FUNCP-REQ STORE ") + fshow(tok));
                doStores.makeReq(initFuncpReqDoStores(rsp.token));

            end

        end

    endrule


    // stage5_loadRsp
    
    // Get the load response and pass it to the commit queue.
    // Also signals Decode that the register has been written back.
    
    rule stage5_loadRsp (True);
    
        // Get the response from the functional partition
        let rsp = doLoads.getResp();
        doLoads.deq();
        let ctx = tokContextId(rsp.token);

        // assert fromMemQ.peek() is valid. Otherwise we wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);

        finishCycle(rsp.token, bundle);

    endrule


    // stage5_storeRsp
    
    // Get the store response and pass it to the commit queue.
    // Let's favor loads over stores, arbitrarily. 
    // (This has no impact on simulation results.)

    (* descending_urgency="stage5_loadRsp, stage5_storeRsp"  *)
    rule stage5_storeRsp (True);

        // Get the response from the functional partition.
        let rsp = doStores.getResp();
        doStores.deq();
        let ctx = tokContextId(rsp.token);
        
        // assert fromMemQ.peek() is valid. Otherwise we wouldn't be here.
        let bundle = bundleFromMemQ.peek(ctx);
        
        finishCycle(rsp.token, bundle);

    endrule

endmodule
