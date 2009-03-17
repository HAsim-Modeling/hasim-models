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
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_controller.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/memory_base_types.bsh"

// ****** Generated files ******

`include "asim/dict/EVENTS_COMMIT.bsh"
`include "asim/dict/STATS_COMMIT.bsh"


// mkCommit

// The commit module commits instructions in order, reporting their
// destinations as ready.

// If an instruction is committed which had a fault, the Fetch unit
// will be redirected and we will use an epoch to drop all younger
// instructions until the new epoch arrives.

// Stores are deallocated store buffer in this stage, which
// ensures that if a previous instruction has a fault then the
// stores will not be sent to the memory system erroneously.
// The write buffer will actually commit the stores.

// This module is pipelined across contexts. Normal flow:
// Stage 1: A non-fault instruction from the correct epoch. -> Stage 2: A non-store response.

// Possible ways the model cycle can end:
//   Path 1: A bubble, nothing in the retireQ.
//   Path 2: A fault, or an instruction from the wrong epoch.
//   Path 3: An instruction succesfully committed.

module [HASIM_MODULE] mkCommit ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_commit.out");


    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(TOKEN_FAULT_EPOCH)) ctx_faultEpoch <- mkMultiCtx(mkReg(0));


    // ****** Ports ******

    PORT_STALL_RECV_MULTICTX#(DMEM_BUNDLE) bundleFromRetireQ  <- mkPortStallRecv_MultiCtx("RetireQ");

    PORT_SEND_MULTICTX#(TOKEN) writebackToDec <- mkPortSend_MultiCtx("Com_to_Dec_writeback");

    PORT_SEND_MULTICTX#(TOKEN) faultToFet <- mkPortSend_MultiCtx("Com_to_Fet_fault");

    PORT_SEND_MULTICTX#(SB_DEALLOC_INPUT) deallocToSB <- mkPortSend_MultiCtx("Com_to_SB_dealloc");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    // Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES) commitStores  <- mkConnection_Client("funcp_commitStores");

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS) inports  = newVector();
    Vector#(4, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromRetireQ.ctrl;
    outports[0] = writebackToDec.ctrl;
    outports[1] = faultToFet.ctrl;
    outports[2] = deallocToSB.ctrl;
    outports[3] = bundleFromRetireQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats *****

    EVENT_RECORDER_MULTICTX eventCom <- mkEventRecorder_MultiCtx(`EVENTS_COMMIT_INSTRUCTION_WRITEBACK);

    STAT_RECORDER_MULTICTX statCom <- mkStatCounter_MultiCtx(`STATS_COMMIT_INSTS_COMMITTED);

    // ****** Rules ******
    

    // stage1_begin
    
    // Begin a new model cycle for a given context.
    // First see if the Retire is non-empty. If so,
    // check if the instruction is from the correct
    // epoch. If so, and it has no fault, then we
    // start to commit it and pass it to the next stage.

    // Otherwise we drop it and stall on the dcache response.

    rule stage1_begin (True);
    
        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
        
        // Get our local state from the context.
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = ctx_faultEpoch[ctx];
        
        // Is there anything for us to commit?
        if (!bundleFromRetireQ.canDeq(ctx))
        begin
        
            // The queue is empty. A bubble.
            debugLog.record_next_cycle(ctx, $format("BUBBLE"));

            // Acknowledge the empty queue.
            bundleFromRetireQ.noDeq(ctx);

            // Propogate the bubble.
            writebackToDec.send(ctx, tagged Invalid);
            faultToFet.send(ctx, tagged Invalid);
            deallocToSB.send(ctx, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(ctx, 1);

        end
        else
        begin

            // Let's try to commit this instruction.
            let bundle = bundleFromRetireQ.peek(ctx);
            let tok = bundle.token;

            if (! tokIsPoisoned(tok) && !bundle.isJunk && (tokFaultEpoch(tok) == faultEpoch))
            begin

                // Normal commit flow for a good instruction.
                debugLog.record_next_cycle(ctx, fshow("COMMIT: ") + fshow(tok) + fshow(" ") + fshow(bundle));

                // No fault occurred.
                faultToFet.send(ctx, tagged Invalid);

                // Have the functional partition commit its local results.
                // The response will be handled by the next stage.
                commitResults.makeReq(initFuncpReqCommitResults(tok));

            end
            else
            begin

                // Exception flow. Deq the RetireQ.
                bundleFromRetireQ.doDeq(ctx);

                if (bundle.isStore())
                begin
                
                    // Drop token from store buffer.
                    deallocToSB.send(ctx, tagged Valid initSBDrop(tok));

                end
                else
                begin

                    deallocToSB.send(ctx, tagged Invalid);

                end

                // So was it a fault, or just from the wrong epoch?
                if ((tokFaultEpoch(tok) != faultEpoch) || bundle.isJunk)
                begin

                    // Just draining following an earlier fault or branch mispredict.
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
                
                // Instruction no longer in flight.
                // Instructions dependent on this guy should be allowed to proceed.
                writebackToDec.send(ctx, tagged Valid tok);

                // End of model cycle. (Path 2)
                localCtrl.endModelCycle(ctx, 2);
            end

        end
    
    endrule
    
    // stage2_commitRsp
    
    // Get the response from the functional partition.
    // 
    // If the instruction is a store, tell the store buffer to send it to the write buffer.

    rule stage2_commitRsp (True);
    
        // Get the response from the functional partition.
        let rsp = commitResults.getResp();
        commitResults.deq();
        
        // Get our context from the token.
        let tok = rsp.token;
        let ctx = tokContextId(tok);
        
        // assert bundleFromRetireQ.canDeq, otherwise we wouldn't be here.
        let bundle = bundleFromRetireQ.peek(ctx);
    
        // Handle stores.
        if (bundle.isStore) 
        begin

            // Tell the store buffer to deallocate and do the store.
            debugLog.record_next_cycle(ctx, fshow("SB STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.virtualAddress));
            deallocToSB.send(ctx, tagged Valid initSBWriteback(tok));

        end
        else
        begin

            // No store to be done.
            deallocToSB.send(ctx, tagged Invalid);

        end
        
        // Check for a termination instruction, which ends simulation for this context.
        if (bundle.isTerminate matches tagged Valid .pf)
        begin
            localCtrl.contextDone(ctx, pf);
        end
    
        // Send the final deallocation to decode.
        writebackToDec.send(ctx, tagged Valid tok);
    
        // Dequeue the retireQ. The instruction is done (except for stores).
        bundleFromRetireQ.doDeq(ctx);
    
        // End of model cycle. (Path 3)
        statCom.incr(ctx);
        eventCom.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
        linkModelCommit.send(tuple2(ctx, 1));
        localCtrl.endModelCycle(ctx, 3);

    endrule

endmodule
