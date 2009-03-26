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

import Vector::*;
import FIFO::*;
import FShow::*;

//HASim library imports
`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_controller.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/fpga_components.bsh"

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"

`include "asim/dict/EVENTS_CPU.bsh"
`include "asim/dict/STATS_CPU.bsh"

`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"

//************************* Simple Timing Partition ***********************//
//                                                                         //
// This is about the simplest timing partition you can conceive of. It     //
// simply fetches one instruction at a time, executes it, then moves to    //
// the next instruction. This can serve as a good mechanism to verify      //
// the functional partition and can serve as a "golden model" for more     //
// complex timing partitions.                                              //
//                                                                         //
//*************************************************************************//

module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_cpu.out");

    //********* State Elements *********//

    // Program counters
    LUTRAM#(CPU_INSTANCE_ID, ISA_ADDRESS) pcPool <- mkLUTRAM(`PROGRAM_START_ADDR);

    // Pipe from any stage that flows to local commit
    FIFO#(TOKEN) commitQ <- mkFIFO();

    //********* Connections *********//

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         linkModelCycle <- mkConnection_Send("model_cycle");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)        linkModelCommit <- mkConnection_Send("model_commits");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT, 
                       FUNCP_RSP_NEW_IN_FLIGHT)       linkToTOK <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE, 
                       FUNCP_RSP_DO_ITRANSLATE)       linkToITR <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)     linkToFET <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES)    linkToDEC <- mkConnection_Client("funcp_getDependencies");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS)         linkToEXE <- mkConnection_Client("funcp_getResults");

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE,
                       FUNCP_RSP_DO_DTRANSLATE)       linkToDTR <- mkConnection_Client("funcp_doDTranslate");

    Connection_Client#(FUNCP_REQ_DO_LOADS,
                       FUNCP_RSP_DO_LOADS)            linkToLOA <- mkConnection_Client("funcp_doLoads");

    Connection_Client#(FUNCP_REQ_DO_STORES,
                       FUNCP_RSP_DO_STORES)           linkToSTO <- mkConnection_Client("funcp_doSpeculativeStores");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS,
                       FUNCP_RSP_COMMIT_RESULTS)      linkToLCO <- mkConnection_Client("funcp_commitResults");

    Connection_Client#(FUNCP_REQ_COMMIT_STORES,
                       FUNCP_RSP_COMMIT_STORES)       linkToGCO <- mkConnection_Client("funcp_commitStores");

    Connection_Client#(FUNCP_REQ_HANDLE_FAULT, 
                       FUNCP_RSP_HANDLE_FAULT)   linkToHandleFault <- mkConnection_Client("funcp_handleFault");

    // For killing. UNUSED

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, 
                       FUNCP_RSP_REWIND_TO_TOKEN)     linkToRewindToToken <- mkConnection_Client("funcp_rewindToToken");


    // Events
    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_CPU_INSTRUCTION_COMMIT);

    // Stats
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCom <- mkStatCounter_Multiplexed(`STATS_CPU_INSTRUCTION_COMMIT);

    Vector#(0, PORT_CONTROLS#(NUM_CPUS)) inports = newVector();
    Vector#(0, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    //********* Rules *********//

    // Mapping from cpu id to context ids and back.
    function CPU_INSTANCE_ID tokCpuInstanceId(TOKEN tok) = tokContextId(tok);
    function CONTEXT_ID getContextId(CPU_INSTANCE_ID cpu_iid) = cpu_iid;


    //
    // endModelCycle --
    //     Invoked by any rule that is completely done with a token.
    //
    function Action endModelCycle(TOKEN tok);
    action
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Model cycle complete"));

        // Sample event & statistic (commit)
        eventCom.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));
        statCom.incr(cpu_iid);

        // Commit counter for heartbeat
        linkModelCommit.send(tuple2(cpu_iid, 1));
        
        // End the model cycle.
        localCtrl.endModelCycle(cpu_iid, 1);
    endaction
    endfunction


    //
    // Whether an instruction is a load or store is stored in token scratchpad
    // memory.  These are accessor functions...
    //
    function Bool tokIsLoad(TOKEN tok) = unpack(tok.timep_info.scratchpad[0]);
    function Bool tokIsStore(TOKEN tok) = unpack(tok.timep_info.scratchpad[1]);

    rule tok_req (True);

        let cpu_iid <- localCtrl.startModelCycle();
        let ctx_id = getContextId(cpu_iid);

        // Request a Token
        debugLog.nextModelCycle(cpu_iid);
        linkModelCycle.send(cpu_iid);

        debugLog.record_next_cycle(cpu_iid, $format("Requesting a new token"));
        linkToTOK.makeReq(initFuncpReqNewInFlight(ctx_id));

    endrule

    rule tok_rsp_itr_req (True);
        // Get the response from tok_req
        let rsp = linkToTOK.getResp();
        linkToTOK.deq();

        let tok = rsp.newToken;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": TOK Responded"));

        // Translate next pc.
        let pc = pcPool.sub(cpu_iid);
        linkToITR.makeReq(initFuncpReqDoITranslate(tok, pc));
        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Translating at address 0x%h", pc));
    endrule

    rule itr_rsp_fet_req (True);
        // Get the ITrans response started by tok_rsp_itr_req
        let rsp = linkToITR.getResp();
        linkToITR.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);
        let pc = pcPool.sub(cpu_iid);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": ITR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin
            // Fetch the next instruction
            linkToFET.makeReq(initFuncpReqGetInstruction(tok));
            debugLog.record(cpu_iid, fshow(tok.index) + $format(": Fetching at address 0x%h", pc));
        end
    endrule

    rule fet_rsp_dec_req (True);
        // Get the instruction response
        let rsp = linkToFET.getResp();
        linkToFET.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": FET Responded"));

        // Record load and store properties for the instruction in the token
        tok.timep_info.scratchpad[0] = pack(isaIsLoad(rsp.instruction));
        tok.timep_info.scratchpad[1] = pack(isaIsStore(rsp.instruction));

        // Decode the current inst
        linkToDEC.makeReq(initFuncpReqGetDependencies(tok));
        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Decoding"));
    endrule

    rule dec_rsp_exe_req (True);
        // Get the decode response
        let rsp = linkToDEC.getResp();
        linkToDEC.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": DEC Responded"));

        // In a more complex processor we would use the dependencies 
        // to determine if we can issue the instruction.

        // Execute the instruction
        linkToEXE.makeReq(initFuncpReqGetResults(tok));
        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Executing"));
    endrule

    rule exe_rsp (True);
        // Get the execution result
        let exe_resp = linkToEXE.getResp();
        linkToEXE.deq();

        let tok = exe_resp.token;
        let res = exe_resp.result;

        let cpu_iid = tokCpuInstanceId(tok);
        let pc = pcPool.sub(cpu_iid);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": EXE Responded"));

        // If it was a branch we must update the PC.
        case (res) matches
            tagged RBranchTaken .addr:
            begin
                debugLog.record(cpu_iid, $format("Branch taken to address %h", addr));
                pcPool.upd(cpu_iid, addr);
            end

            tagged RBranchNotTaken .addr:
            begin
                debugLog.record(cpu_iid, $format("Branch not taken"));
                pcPool.upd(cpu_iid, pc + 4);
            end

            tagged RTerminate .pf:
            begin
                debugLog.record(cpu_iid, $format("Terminating Execution"));
                localCtrl.instanceDone(cpu_iid, pf);
            end

            default:
            begin
                pcPool.upd(cpu_iid, pc + 4);
            end
        endcase

        if (tokIsLoad(tok) || tokIsStore(tok))
        begin
            // Memory ops require more work.
            debugLog.record(cpu_iid, fshow(tok.index) + $format(": DTranslate"));

            // Get the physical address(es) of the memory access.
            linkToDTR.makeReq(initFuncpReqDoDTranslate(tok));
        end
        else
        begin
            // Everything else should just be committed.
            commitQ.enq(tok);
        end
    endrule

    rule dtr_rsp_mem_req (True);
        // Get the response from dTranslate
        let rsp = linkToDTR.getResp();
        linkToDTR.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": DTR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin
            if (tokIsLoad(tok))
            begin
                // Request the load(s).
                linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));
            end
            else
            begin
                // Request the store(s)
                linkToSTO.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));
            end
        end
    endrule

    rule loa_rsp (True);
        // Get the load response
        let rsp = linkToLOA.getResp();
        linkToLOA.deq();

        let tok = rsp.token;
        let cpu_iid= tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Load ops responded"));

        commitQ.enq(tok);
    endrule

    rule sto_rsp (True);
        // Get the store response
        let rsp = linkToSTO.getResp();
        linkToSTO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Store ops responded"));

        commitQ.enq(tok);
    endrule

    rule lco_req (True);
        // Get the current token from previous rule
        let tok = commitQ.first();
        let cpu_iid = tokCpuInstanceId(tok);
        commitQ.deq();

        if (! tokIsPoisoned(tok))
        begin
            // Locally commit the token.
            linkToLCO.makeReq(initFuncpReqCommitResults(tok));
            debugLog.record(cpu_iid, fshow(tok.index) + $format(": Locally committing"));
        end
        else
        begin
            // Token is poisoned.  Invoke fault handler.
            linkToHandleFault.makeReq(initFuncpReqHandleFault(tok));
            debugLog.record(cpu_iid, fshow(tok.index) + $format(": Handling fault"));
        end
    endrule

    rule lco_rsp_gco_req (True);
        let rsp = linkToLCO.getResp();
        linkToLCO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": LCO Responded"));

        if (tokIsStore(tok))
        begin
            // Request global commit of stores.
            linkToGCO.makeReq(initFuncpReqCommitStores(tok));
            debugLog.record(cpu_iid, fshow(tok.index) + $format(": Globally committing"));
        end
        else
        begin
            endModelCycle(tok);
        end
    endrule

    rule gco_rsp (True);
        // Get the global commit response
        let rsp = linkToGCO.getResp();
        linkToGCO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": GCO Responded"));

        endModelCycle(tok);
    endrule

    (* descending_urgency = "fault_rsp, gco_rsp, lco_rsp_gco_req, sto_rsp, loa_rsp, exe_rsp, tok_req" *)
    rule fault_rsp (True);
        // Fault response (started by lco_req)
        let rsp = linkToHandleFault.getResp();
        linkToHandleFault.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, fshow(tok.index) + $format(": Handle fault responded PC 0x%0x", rsp.nextInstructionAddress));

        // Next PC following fault
        pcPool.upd(cpu_iid, rsp.nextInstructionAddress);

        endModelCycle(tok);
    endrule
endmodule

