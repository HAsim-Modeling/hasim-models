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
`include "asim/provides/chip_base_types.bsh"
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

// This module is pipelined across instances. Normal flow:
// Stage 1: A non-fault instruction from the correct epoch. -> Stage 2: A non-store response.

// Possible ways the model cycle can end:
//   Path 1: A bubble, nothing in the retireQ.
//   Path 2: A fault, or an instruction from the wrong epoch.
//   Path 3: An instruction succesfully committed.

module [HASIM_MODULE] mkCommit ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_commit.out");


    // ****** Model State (per Context) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(TOKEN_FAULT_EPOCH)) faultEpochPool <- mkMultiplexed(mkReg(0));

    // ****** Ports ******

    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE) bundleFromRetireQ  <- mkPortStallRecv_Multiplexed("RetireQ");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, TOKEN) writebackToDec <- mkPortSend_Multiplexed("Com_to_Dec_writeback");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VOID)  rewindToDec <- mkPortSend_Multiplexed("Com_to_Dec_fault");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, TOKEN) faultToFet <- mkPortSend_Multiplexed("Com_to_Fet_fault");

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, SB_DEALLOC_INPUT) deallocToSB <- mkPortSend_Multiplexed("Com_to_SB_dealloc");


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");

    // Number of commits (to go along with heartbeat)
    Connection_Send#(CONTROL_MODEL_COMMIT_MSG) linkModelCommit <- mkConnection_Send("model_commits");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS#(NUM_CPUS)) inports  = newVector();
    Vector#(5, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromRetireQ.ctrl;
    outports[0] = writebackToDec.ctrl;
    outports[1] = faultToFet.ctrl;
    outports[2] = deallocToSB.ctrl;
    outports[3] = bundleFromRetireQ.ctrl;
    outports[4] = rewindToDec.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats *****

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_COMMIT_INSTRUCTION_WRITEBACK);

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCom <- mkStatCounter_Multiplexed(`STATS_COMMIT_INSTS_COMMITTED);

    // ****** Rules ******
    

    // stage1_begin
    
    // Begin a new model cycle for a given instance.
    // First see if the Retire is non-empty. If so,
    // check if the instruction is from the correct
    // epoch. If so, and it has no fault, then we
    // start to commit it and pass it to the next stage.

    // Otherwise we drop it and stall on the dcache response.

    rule stage1_begin (True);
    
        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        
        // Get our local state from the context.
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = faultEpochPool[cpu_iid];
        
        // Is there anything for us to commit?
        if (!bundleFromRetireQ.canDeq(cpu_iid))
        begin
        
            // The queue is empty. A bubble.
            debugLog.record_next_cycle(cpu_iid, $format("BUBBLE"));

            // Acknowledge the empty queue.
            bundleFromRetireQ.noDeq(cpu_iid);

            // Propogate the bubble.
            writebackToDec.send(cpu_iid, tagged Invalid);
            faultToFet.send(cpu_iid, tagged Invalid);
            rewindToDec.send(cpu_iid, tagged Invalid);
            deallocToSB.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else
        begin

            // Let's try to commit this instruction.
            let bundle = bundleFromRetireQ.peek(cpu_iid);
            let tok = bundle.token;

            if (! tokIsPoisoned(tok) && !tokIsDummy(tok) && (bundle.faultEpoch == faultEpoch))
            begin

                // Normal commit flow for a good instruction.
                debugLog.record_next_cycle(cpu_iid, fshow("COMMIT: ") + fshow(tok) + fshow(" ") + fshow(bundle));

                // No fault occurred.
                faultToFet.send(cpu_iid, tagged Invalid);
                rewindToDec.send(cpu_iid, tagged Invalid);

                // Have the functional partition commit its local results.
                // The response will be handled by the next stage.
                commitResults.makeReq(initFuncpReqCommitResults(tok));

            end
            else
            begin

                // Exception flow. Deq the RetireQ.
                bundleFromRetireQ.doDeq(cpu_iid);

                if (bundle.isStore())
                begin
                
                    // Drop token from store buffer.
                    deallocToSB.send(cpu_iid, tagged Valid initSBDrop(tok));

                end
                else
                begin

                    deallocToSB.send(cpu_iid, tagged Invalid);

                end

                // So was it a fault, or just from the wrong epoch?
                if ((bundle.faultEpoch != faultEpoch) || tokIsDummy(tok))
                begin

                    // Just draining following an earlier fault or branch mispredict.
                    debugLog.record_next_cycle(cpu_iid, fshow("DRAIN: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                    faultToFet.send(cpu_iid, tagged Invalid);
                    rewindToDec.send(cpu_iid, tagged Invalid);

                end
                else
                begin

                    // Fault.  Redirect the front end to handle the fault.
                    debugLog.record_next_cycle(cpu_iid, fshow("FAULT: ") + fshow(tok) + fshow(" ") + fshow(bundle));
                    faultEpoch <= faultEpoch + 1;
                    faultToFet.send(cpu_iid, tagged Valid tok);
                    rewindToDec.send(cpu_iid, tagged Valid (?));



                end
                
                // Instruction no longer in flight.
                // Instructions dependent on this guy should be allowed to proceed.
                writebackToDec.send(cpu_iid, tagged Valid tok);
                
                localCtrl.endModelCycle(cpu_iid, 2);

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
        let cpu_iid = tokCpuInstanceId(tok);
        
        // assert bundleFromRetireQ.canDeq, otherwise we wouldn't be here.
        let bundle = bundleFromRetireQ.peek(cpu_iid);
    
        // Handle stores.
        if (bundle.isStore) 
        begin

            // Tell the store buffer to deallocate and do the store.
            debugLog.record_next_cycle(cpu_iid, fshow("SB STORE ") + fshow(tok) + fshow(" ADDR: ") + fshow(bundle.virtualAddress));
            deallocToSB.send(cpu_iid, tagged Valid initSBWriteback(tok));

        end
        else
        begin

            // No store to be done.
            deallocToSB.send(cpu_iid, tagged Invalid);

        end
        
        // Check for a termination instruction, which ends simulation for this context.
        if (bundle.isTerminate matches tagged Valid .pf)
        begin
            localCtrl.instanceDone(cpu_iid, pf);
        end
    
        // Send the final deallocation to decode.
        writebackToDec.send(cpu_iid, tagged Valid tok);
    
        // Dequeue the retireQ. The instruction is done (except for stores).
        bundleFromRetireQ.doDeq(cpu_iid);
    
        // End of model cycle. (Path 2)
        statCom.incr(cpu_iid);
        eventCom.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));
        linkModelCommit.send(tuple2(cpu_iid, 1));
        localCtrl.endModelCycle(cpu_iid, 3);

    endrule

endmodule
