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

// This module is pipelined across instances. Normal flow:
// Stage 1: A non-fault instruction from the correct epoch. -> Stage 2: A non-store response.

// Possible ways the model cycle can end:
//   Path 1: A bubble, nothing in the retireQ.
//   Path 2: A fault, or an instruction from the wrong epoch.
//   Path 3: An instruction succesfully committed.


typedef union tagged
{
    DMEM_BUNDLE STAGE2_commitRsp;
    Tuple2#(TOKEN, Bool) STAGE2_dummyRsp;
    void STAGE2_bubble;
}
COM_STAGE2_STATE deriving (Bits, Eq);


module [HASIM_MODULE] mkCommit ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commit.out");


    // ****** Model State (per Context) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_FAULT_EPOCH)) faultEpochPool <- mkMultiplexed(mkReg(0));

    // ****** Ports ******

    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE) bundleFromRetireQ <- mkPortStallRecv_Multiplexed("RetireQ");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, TOKEN)  rewindToDec <- mkPortSend_Multiplexed("Com_to_Dec_fault");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple2#(TOKEN, ISA_ADDRESS)) faultToFet <- mkPortSend_Multiplexed("Com_to_Fet_fault");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, SB_DEALLOC_INPUT) deallocToSB <- mkPortSend_Multiplexed("Com_to_SB_dealloc");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");


    // ****** Local Controller ******

    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromRetireQ.ctrl.in;
    outports[0] = faultToFet.ctrl;
    outports[1] = deallocToSB.ctrl;
    outports[2] = rewindToDec.ctrl;
    outports[3]  = bundleFromRetireQ.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, COM_STAGE2_STATE) stage2Ctrl <- mkStageController();


    // ****** Events and Stats *****

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_COMMIT_INSTRUCTION_WRITEBACK);

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCom <- mkStatCounter_Multiplexed(`STATS_COMMIT_INSTS_COMMITTED);

    // ****** Rules ******
    

    // stage1_begin
    
    // Begin a new model cycle for a given instance.
    // First see if the CommitQ is non-empty. If so,
    // check if the instruction is from the correct
    // epoch. If so, and it has no fault, then we
    // start to commit it and pass it to the next stage.

    // Otherwise we drop it and stall on the dcache response.

    // Ports read:
    // * bundleFromRetireQ
    
    // Ports written:
    // * deqToRetireQ

    (* conservative_implicit_conditions *)
    rule stage1_begin (True);
    
        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        
        // Get our local state from the context.
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = faultEpochPool[cpu_iid];
        
        let m_bundle <- bundleFromRetireQ.receive(cpu_iid);
        
        // Is there anything for us to commit?
        if (m_bundle matches tagged Valid .bundle)
        begin

            // Let's try to commit this instruction.
            let tok = bundle.token;
            
            // Dequeue the queue, since this stage never stalls.
            bundleFromRetireQ.doDeq(cpu_iid);

            if (bundle.faultEpoch == faultEpoch && !tokIsDummy(tok))
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

                // This token followed a branch mispredict or fault. It was actually already rewound.
                // Therefore we just silently drop it.
                debugLog.record_next_cycle(cpu_iid, fshow("1: RECLAIM DUMMY: ") + fshow(tok) + fshow(" ") + fshow(bundle));

                // The response will be handled by the next stage.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_dummyRsp tuple2(tok, bundle.isStore));

            end

        end
        else
        begin
        
            // The queue is empty. A bubble.
            debugLog.record_next_cycle(cpu_iid, $format("1: BUBBLE"));

            // No dequeue.
            bundleFromRetireQ.noDeq(cpu_iid);

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

    rule stage2_commitRsp (True);

        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        // Get our local state from the context.
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = faultEpochPool[cpu_iid];
        
        if (state matches tagged STAGE2_bubble)
        begin

            // Propogate the bubble.
            deallocToSB.send(cpu_iid, tagged Invalid);
            faultToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else if (state matches tagged STAGE2_dummyRsp {.tok, .is_store})
        begin

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

            // Handle faults.
            if (rsp.faultRedirect matches tagged Valid .redirect_addr)
            begin

                // A fault occcurred.
                debugLog.record(cpu_iid, fshow("2: FAULT REDIRECT ") + fshow(tok) + $format(" ADDR: 0x%h", redirect_addr));
                faultEpoch <= faultEpoch + 1;

                // Pass on the redirection. 
                faultToFet.send(cpu_iid, tagged Valid tuple2(tok, redirect_addr));
                rewindToDec.send(cpu_iid, tagged Valid tok);

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
                if (bundle.isStore) 
                begin

                    // assert .storeToken isValid
                    let st_tok = validValue(rsp.storeToken);

                    // Tell the store buffer to deallocate and do the store.
                    debugLog.record(cpu_iid, fshow("2: SB STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.virtualAddress) + fshow("STORE TOKEN: ") + fshow(st_tok));
                    deallocToSB.send(cpu_iid, tagged Valid initSBWriteback(tok, st_tok));

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
            linkModelCommit.send(tuple2(cpu_iid, 1));
            localCtrl.endModelCycle(cpu_iid, 4);
        
        end

    endrule

endmodule
