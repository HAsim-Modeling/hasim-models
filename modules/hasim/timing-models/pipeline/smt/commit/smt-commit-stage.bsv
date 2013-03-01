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
`include "asim/provides/common_services.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

// ****** Generated files ******

`include "asim/dict/EVENTS_COMMIT.bsh"


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

// This module is pipelined across instances. Normal flow:
// Stage 1: A non-fault instruction from the correct epoch. -> Stage 2: A non-store response.

// Possible ways the model cycle can end:
//   Path 1: A bubble, nothing in the retireQ.
//   Path 2: A fault, or an instruction from the wrong epoch.
//   Path 3: An instruction succesfully committed.


typedef union tagged
{
    Tuple2#(TOKEN, Bool) STAGE2_dummyRsp;
    DMEM_BUNDLE STAGE2_commitRsp;
    void STAGE2_bubble;
}
COM_STAGE2_STATE deriving (Bits, Eq);


module [HASIM_MODULE] mkCommit ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commit.out");


    // ****** Model State (per Context) ******

    MULTIPLEXED_REG#(MAX_NUM_CPUS, MULTITHREADED#(TOKEN_FAULT_EPOCH)) faultEpochsPool <- mkMultiplexedReg(multithreaded(0));

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DMEM_BUNDLE) bundleFromCommitQ  <- mkPortRecv_Multiplexed("commitQ_first", 0);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN) writebackToDec <- mkPortSend_Multiplexed("Com_to_Dec_writeback");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, THREAD_ID)  rewindToDec <- mkPortSend_Multiplexed("Com_to_Dec_fault");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN, ISA_ADDRESS)) faultToFet <- mkPortSend_Multiplexed("Com_to_Fet_fault");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, SB_DEALLOC_INPUT) deallocToSB <- mkPortSend_Multiplexed("Com_to_SB_dealloc");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID) deqToCommitQ <- mkPortSend_Multiplexed("commitQ_deq");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");


    // ****** Local Controller ******

    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports  = newVector();
    Vector#(5, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromCommitQ.ctrl;
    outports[0] = writebackToDec.ctrl;
    outports[1] = faultToFet.ctrl;
    outports[2] = deallocToSB.ctrl;
    outports[3] = deqToCommitQ.ctrl;
    outports[4] = rewindToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, COM_STAGE2_STATE) stage2Ctrl <- mkStageController();


    // ****** Events and Stats *****

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_COMMIT_INSTRUCTION_WRITEBACK);

    STAT_VECTOR#(MAX_NUM_CPUS) statCom <-
        mkStatCounter_Multiplexed(statName("COMMIT_INSTS_COMMITTED",
                                           "Instructions Committed"));

    // ****** Rules ******
    

    // stage1_begin
    
    // Begin a new model cycle for a given instance.
    // First see if the CommitQ is non-empty. If so,
    // check if the instruction is from the correct
    // epoch. If so, and it has no fault, then we
    // start to commit it and pass it to the next stage.

    // Otherwise we drop it and stall on the dcache response.

    // Ports read:
    // * bundleFromCommitQ
    
    // Ports written:
    // * deqToCommitQ

    (* conservative_implicit_conditions *)
    rule stage1_begin (True);
    
        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        
        // Get our local state from the context.
        Reg#(MULTITHREADED#(TOKEN_FAULT_EPOCH)) faultEpochs = faultEpochsPool.getReg(cpu_iid);
        
        let m_bundle <- bundleFromCommitQ.receive(cpu_iid);
        
        // Is there anything for us to commit?
        if (m_bundle matches tagged Valid .bundle)
        begin

            // Let's try to commit this instruction.
            let tok = bundle.token;
            
            // Dequeue the queue, since this stage never stalls.
            deqToCommitQ.send(cpu_iid, tagged Valid (?));

            if (!tokIsDummy(tok) && bundle.faultEpoch == faultEpochs[tokThreadId(tok)])
            begin

                // Normal commit flow for a good instruction.
                debugLog.record_next_cycle(cpu_iid, fshow("1: COMMIT: ") + fshow(tok) + fshow(" ") + fshow(bundle));

                // Have the functional partition commit its local results,
                // and handle any faults that might have occurred.
                commitResults.makeReq(initFuncpReqCommitResults(tok));

                // The response will be handled by the next stage.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_commitRsp bundle);

            end
            else
            begin

                // This token followed a branch mispredict or fault. We just need to undo its effects
                // and reclaim its registers. This looks like a normal commit req, but is handled
                // differently by the functional partition.
                debugLog.record_next_cycle(cpu_iid, fshow("1: RECLAIM DUMMY: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                //tok.dummy = True;
                //commitResults.makeReq(initFuncpReqCommitResults(tok));

                // The response will be handled by the next stage.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_dummyRsp tuple2(tok, bundle.isStore));

            end

        end
        else
        begin
        
            // The queue is empty. A bubble.
            debugLog.record_next_cycle(cpu_iid, $format("1: BUBBLE"));

            // No dequeue.
            deqToCommitQ.send(cpu_iid, tagged Invalid);

            stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

        end
    
    endrule
    
    // stage2_commitRsp
    
    // Get the response from the functional partition (if any)
    // Report the writebacks to decode.
    // If the instruction is a store, tell the store buffer to send it to the write buffer.

    // Ports read:
    // * None
    
    // Ports written:
    // * deallocToSB
    // * rewindToDec
    // * faultToFet
    // * writebackToDec

    rule stage2_commitRsp (True);

        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get our local state from the context.
        Reg#(MULTITHREADED#(TOKEN_FAULT_EPOCH)) faultEpochs = faultEpochsPool[cpu_iid];
        
        if (state matches tagged STAGE2_bubble)
        begin

            // Propogate the bubble.
            writebackToDec.send(cpu_iid, tagged Invalid);
            deallocToSB.send(cpu_iid, tagged Invalid);
            faultToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE2_dummyRsp {.tok, .is_store})
        begin
    
            // Get the commit response from the functional partition.
            //let rsp = commitResults.getResp();
            //commitResults.deq();
        
            // Get our context from the token.
            //let tok = rsp.token;

            // Instruction is no longer in flight.

            if (is_store)
            begin
            
                // Stores should be dropped from the store buffer.
                deallocToSB.send(cpu_iid, tagged Valid initSBDrop(tok));

            end
            else
            begin

                deallocToSB.send(cpu_iid, tagged Invalid);

            end
            
            // No fault.
            faultToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);
            
            // Instructions dependent on this guy should be allowed to proceed.
            writebackToDec.send(cpu_iid, tagged Valid tok);

            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(cpu_iid, 3);

        end
        else if (state matches tagged STAGE2_commitRsp .bundle)
        begin
    
            // Get the commit response from the functional partition.
            let rsp = commitResults.getResp();
            commitResults.deq();

            // Get our context from the token.
            let tok = rsp.token;

            // Send the final deallocation to decode.
            writebackToDec.send(cpu_iid, tagged Valid tok);

            // Handle faults.
            if (rsp.faultRedirect matches tagged Valid .redirect_addr)
            begin

                // A fault occcurred.
                debugLog.record(cpu_iid, fshow("2: FAULT REDIRECT ") + fshow(tok) + $format(" ADDR: 0x%h", redirect_addr));
                faultEpochs[tokThreadId(tok)] <= faultEpochs[tokThreadId(tok)] + 1;

                // Pass on the redirection. 
                faultToFet.send(cpu_iid, tagged Valid tuple2(tok, redirect_addr));
                rewindToDec.send(cpu_iid, tagged Valid tokThreadId(tok));

                if (bundle.isStore) 
                begin

                    // Faulting stores should be dropped from the store buffer.
                    deallocToSB.send(cpu_iid, tagged Valid initSBDrop(tok));

                end
                else
                begin
                
                    // No store to be done.
                    deallocToSB.send(cpu_iid, tagged Invalid);

                end

            end
            else 
            begin
    
                // No fault.
                faultToFet.send(cpu_iid, tagged Invalid);
                rewindToDec.send(cpu_iid, tagged Invalid);

                // Stores should actually update memory.
                if (rsp.storeToken matches tagged Valid .st_tok)
                begin

                    // Tell the store buffer to deallocate and do the store.
                    debugLog.record(cpu_iid, fshow("2: SB STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.virtualAddress) + fshow("STORE TOKEN: ") + fshow(st_tok));
                    deallocToSB.send(cpu_iid, tagged Valid initSBWriteback(tok, st_tok));

                end
                else if (bundle.isStore)
                begin

                    // Squashed store
                    debugLog.record(cpu_iid, fshow("2: SB SQUASHED STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.virtualAddress));
                    deallocToSB.send(cpu_iid, tagged Valid initSBDrop(tok));

                end
                else
                begin

                    // No store to be done.
                    deallocToSB.send(cpu_iid, tagged Invalid);

                end

             end

            // Check for a termination instruction, which ends simulation for this context.
            if (bundle.isTerminate matches tagged Valid .pf)
            begin

                localCtrl.instanceDone(cpu_iid, pf);

            end


            // End of model cycle. (Path 4)
            statCom.incr(cpu_iid);
            eventCom.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));
            linkModelCommit.send(tuple2(tokContextId(tok), 1));
            localCtrl.endModelCycle(cpu_iid, 4);
        
        end

    endrule

endmodule
