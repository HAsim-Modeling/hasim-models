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

// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_controller.bsh"

// ****** Generated files ******

`include "asim/dict/EVENTS_WRITEBACK.bsh"
`include "asim/dict/STATS_WRITEBACK.bsh"


// ****** Local types ******

// COM1_STATE

// The first stage may stall for a bubble.

typedef union tagged
{
    void COM1_ready;
    CONTEXT_ID COM1_bubble;
}
    COM1_STATE
        deriving (Bits, Eq);


// COM2_STATE

// The second stage stalls on stores until it gets a response.

typedef union tagged
{
    void COM2_ready;
    CONTEXT_ID COM2_storeRsp;
}
    COM2_STATE
        deriving (Bits, Eq);


// mkCommit

// The commit module commits instructions in order, reporting their
// destinations as ready.

// If an instruction is committed which had a fault, the Fetch unit
// will be redirected and we will use an epoch to drop all younger
// instructions until the new epoch arrives.

// Stores are commited from the store buffer in this stage, which
// ensures that if a previous instruction has a fault then the
// stores will not be sent to the memory system erroneously.

// This module is pipelined across contexts. Stages:
// Stage 1* -> Stage 2**
// * Stage 1 stalls on bubbles to handle the DCache response.
// ** Stage 2 stalls on stores for the functional partition's response to commitStores.

// Possible ways the model cycle can end:
//   Path 1: An instruction has been succefully committed.
//   Path 2: A bubble, or a dropped instruction from the wrong epoch.
//   Path 3: We tried to commit a store, and the DCache told us to retry.

module [HASIM_MODULE] mkCommit ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_writeback.out");


    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(TOKEN_FAULT_EPOCH)) ctx_faultEpoch <- mkMultiCtx(mkReg(0));


    // ****** Ports ******

    PORT_STALL_RECV_MULTICTX#(BUNDLE) bundleFromCommitQ  <- mkPortStallRecv_MultiCtx("CommitQ");

    PORT_SEND_MULTICTX#(TOKEN) writebackToDec <- mkPortSend_MultiCtx("Com_to_Dec_writeback");

    PORT_SEND_MULTICTX#(TOKEN) faultToFet <- mkPortSend_MultiCtx("Com_to_Fet_fault");

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheInput)) reqToDCache <- mkPortSend_MultiCtx("CPU_to_DCache_committed");
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed)) rspFromDCacheDelayed <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_committed_delayed", 0);
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate)) rspFromDCacheImmediate <- mkPortRecvGuarded_MultiCtx("DCache_to_CPU_committed_immediate", 0);

    PORT_SEND_MULTICTX#(TOKEN) deallocToSB <- mkPortSend_MultiCtx("Com_to_SB_dealloc");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS) inports  = newVector();
    Vector#(5, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromCommitQ.ctrl;
    // inports[1]  = rspFromDCacheDelayed.ctrl;
    // inports[2]  = rspFromDCacheImmediate.ctrl;
    outports[0] = writebackToDec.ctrl;
    outports[1] = faultToFet.ctrl;
    outports[2] = deallocToSB.ctrl;
    outports[3] = reqToDCache.ctrl;
    outports[4] = bundleFromCommitQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** UnModel Pipeline State ******

    Reg#(COM1_STATE) stage1State <- mkReg(COM1_ready);
    Reg#(COM2_STATE) stage2State <- mkReg(COM2_ready);


    // ****** Events and Stats *****

    EVENT_RECORDER_MULTICTX eventCom <- mkEventRecorder_MultiCtx(`EVENTS_WRITEBACK_INSTRUCTION_WRITEBACK);

    STAT_RECORDER_MULTICTX statCom <- mkStatCounter_MultiCtx(`STATS_WRITEBACK_INSTS_COMMITTED);


    // ****** Helper Functions ******
    
    
    // finishCycle

    // Finish committing an instruction.
    // If it was marked as a termination, then we start the process
    // of ending simulation.

    function Action finishCycle(CONTEXT_ID ctx, BUNDLE bundle);
    action

        let tok = bundle.token;

        debugLog.record(ctx, fshow("DONE: ") + fshow(tok) + fshow(" ") + fshow(bundle));

        // Check for a termination instruction, which ends simulation for this context.
        if (bundle.isTerminate matches tagged Valid .pf)
        begin
            localCtrl.contextDone(ctx, pf);
        end
        
        // Dequeue the CommitQ
        bundleFromCommitQ.doDeq(ctx);

        // Signal the writeback of the destinations.
        writebackToDec.send(ctx, tagged Valid tok);

        // Keep the controller informed about the number of instructions committed.
        linkModelCommit.send(tuple2(ctx, 1));

        // End of model cycle. (Path 1)
        eventCom.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
        statCom.incr(ctx);
        localCtrl.endModelCycle(ctx, 1);

    endaction
    endfunction


    // ****** Rules ******
    

    // stage1_begin
    
    // Begin a new model cycle for a given context.
    // First see if the CommitQ is non-empty. If so,
    // check if the instruction is from the correct
    // epoch. If so, and it has no fault, then we
    // start to commit it and pass it to the next stage.

    // Otherwise we drop it and stall on the dcache response.

    rule stage1_begin (stage1State matches tagged COM1_ready);
    
        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
        
        // Get our local state from the context.
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = ctx_faultEpoch[ctx];
        
        // Is there anything for us to commit?
        if (!bundleFromCommitQ.canDeq(ctx))
        begin
        
            // The queue is empty. A bubble.
            debugLog.record_next_cycle(ctx, $format("BUBBLE"));

            // Acknowledge the empty queue.
            bundleFromCommitQ.noDeq(ctx);

            // Propogate the bubble.
            writebackToDec.send(ctx, tagged Invalid);
            faultToFet.send(ctx, tagged Invalid);
            deallocToSB.send(ctx, tagged Invalid);
            reqToDCache.send(ctx, tagged Invalid);

            // Stall for the cache response.
            stage1State <= tagged COM1_bubble ctx;

        end
        else
        begin

            // Let's try to commit this instruction.
            let bundle = bundleFromCommitQ.peek(ctx);
            let tok = bundle.token;

            if (! tokIsPoisoned(tok) && (tokFaultEpoch(tok) == faultEpoch))
            begin

                // Normal commit flow for a good instruction.
                debugLog.record_next_cycle(ctx, fshow("COMMIT: ") + fshow(tok) + fshow(" ") + fshow(bundle));

                // No fault occurred.
                faultToFet.send(ctx, tagged Invalid);

                // Start to handle stores.
                if (bundle.isStore) 
                begin

                    // Tell the cache to do the store.
                    debugLog.record_next_cycle(ctx, fshow("DCACHE STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.effAddr));
                    reqToDCache.send(ctx, tagged Valid tuple2(tok,Data_write_mem_ref(tuple2(?/*passthru*/, bundle.effAddr))));

                end
                else
                begin

                    // No memory op to be done.
                    reqToDCache.send(ctx, tagged Invalid);

                end

                // Have the functional partition commit its local results.
                // The response will be handled by the next stage.
                commitResults.makeReq(initFuncpReqCommitResults(tok));

            end
            else
            begin

                // Exception flow. Deq the CommitQ.
                bundleFromCommitQ.doDeq(ctx);

                // Even if this was a store, we're not committing it.
                reqToDCache.send(ctx, Invalid);

                // Instruction no longer in flight.
                // Instructions dependent on this guy should be allowed to proceed.
                writebackToDec.send(ctx, tagged Valid tok);

                if (bundle.isStore())
                begin
                
                    // Drop token from store buffer.
                    deallocToSB.send(ctx, tagged Valid tok);

                end
                else
                begin

                    deallocToSB.send(ctx, tagged Invalid);

                end

                // So was it a fault, or just from the wrong epoch?
                if (tokFaultEpoch(tok) != faultEpoch)
                begin

                    // Just draining following an earlier fault.
                    debugLog.record_next_cycle(ctx, fshow("DRAIN: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                    faultToFet.send(ctx, tagged Invalid);

                end
                else
                begin

                    // Fault.  Redirect the front end to handle the fault.
                    debugLog.record_next_cycle(ctx, fshow("FAULT: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                    faultEpoch <= faultEpoch + 1;
                    faultToFet.send(ctx, tagged Valid tok);

                end
                
                // Stall this stage for the cache response.
                stage1State <= tagged COM1_bubble ctx;
            end

        end
    
    endrule
    

    // stage1_finishBubble
    
    // Just drop the cache responses.
    
    // Note: if we could guarantee that the Local Controller wouldn't start
    // us on the same context again before this happened, then we would not
    // have to stall the pipeline in this case.

    rule stage1_finishBubble (stage1State matches tagged COM1_bubble .ctx);
    
        // Drop the cache responses.
        let imm <- rspFromDCacheImmediate.receive(ctx);
        let del <- rspFromDCacheDelayed.receive(ctx);

        // Unstall the pipeline.
        stage1State <= tagged COM1_ready;

        // End of model cycle. (Path 2)
        eventCom.recordEvent(ctx, tagged Invalid);
        localCtrl.endModelCycle(ctx, 2);
        
    endrule

    // stage2_dcacheRsp
    
    // Get the response from the DCache (if any).
    // If the instruction is a store, tell the functional partition to
    // make its effects globally visible.

    rule stage2_dcacheRsp (stage2State matches tagged COM2_ready);
    
        // Get the response from the functional partition.
        let rsp = commitResults.getResp();
        commitResults.deq();
        
        // Get our context from the token.
        let tok = rsp.token;
        let ctx = tokContextId(tok);
        
        // assert bundleFromCommitQ.canDeq, otherwise we wouldn't be here.
        let bundle = bundleFromCommitQ.peek(ctx);
    
        // Get the cache responses.
        let imm <- rspFromDCacheImmediate.receive(ctx);
        let del <- rspFromDCacheDelayed.receive(ctx);
        
        // assert !isValid(imm) && !isValid(del)

        if (del == Invalid &&& imm matches tagged Valid { .tok2, tagged Miss_retry .* })
        begin

            // Cache told us to retry, so end this cycle and try again from the top.
            debugLog.record(ctx, fshow("DCACHE RETRY ") + fshow(tok));

            // Don't dequeue the CommitQ.
            bundleFromCommitQ.noDeq(ctx);
            
            // Don't deallocate the store buffer yet.
            deallocToSB.send(ctx, tagged Invalid);

            // No writebacks to report.
            writebackToDec.send(ctx, tagged Invalid);
            
            // End of model cycle. (Path 3)
            eventCom.recordEvent(ctx, tagged Invalid);
            localCtrl.endModelCycle(ctx, 3);

        end
        else
        begin
        
            // The response was either invalid, a delayed Miss_resp, or an immediate Hit
            // It's okay to ignore Miss_servicing since anyone who references the
            // address will block anyway.

            if (bundle.isStore)
            begin
                
                // Deallocate the store from the store buffer.
                deallocToSB.send(ctx, tagged Valid tok);

                // Tell the functional partition to commit the store.
                commitStores.makeReq(initFuncpReqCommitStores(tok));

                // Stall for the functional partition response.
                stage2State <= tagged COM2_storeRsp ctx;

            end
            else
            begin

                // Don't deallocate the store buffer. End the model cycle.
                deallocToSB.send(ctx, tagged Invalid);
                finishCycle(ctx, bundle);
            end
        end
    endrule

    // stage2_storeRsp
    
    // Just drop the functional partition response and finish the cycle.
    
    // Note: if we could guarantee that the Local Controller wouldn't start
    // us on the same context again before this happened, then we would not
    // have to stall the pipeline in this case.

    rule stage2_storeRsp (stage2State matches tagged COM2_storeRsp .ctx);
        commitStores.deq();

        // assert bundleFromCommitQ.canDeq, otherwise we wouldn't be here.
        let bundle = bundleFromCommitQ.peek(ctx);

        // Unstall the pipeline.
        stage2State <= tagged COM2_ready;

        finishCycle(ctx, bundle);
    endrule

endmodule
