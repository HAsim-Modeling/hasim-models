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
    MULTIPLEXED#(NUM_CPUS, Reg#(ISA_ADDRESS)) pcPool <- mkMultiplexed(mkReg(`PROGRAM_START_ADDR));

    // Intra-stage queues.
    FIFO#(Tuple2#(Bool, Bool)) decQ <- mkFIFO();
    FIFO#(TOKEN) dtrQ <- mkFIFO();
    FIFO#(TOKEN) dmemQ <- mkFIFO();
    FIFO#(TOKEN) commitQ <- mkFIFO();

    //********* Connections *********//

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         linkModelCycle <- mkConnection_Send("model_cycle");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)        linkModelCommit <- mkConnection_Send("model_commits");

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

    Vector#(0, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(0, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    //********* Rules *********//

    // Mapping from cpu id to context ids and back.
    function CPU_INSTANCE_ID getCpuInstanceId(CONTEXT_ID ctx_id) = ctx_id;
    function CPU_INSTANCE_ID tokCpuInstanceId(TOKEN tok) = tokContextId(tok);
    function CPU_INSTANCE_ID storeTokCpuInstanceId(STORE_TOKEN st_tok) = storeTokContextId(st_tok);
    function CONTEXT_ID getContextId(CPU_INSTANCE_ID cpu_iid) = cpu_iid;


    //
    // endModelCycle --
    //     Invoked by any rule that is completely done with a token.
    //
    function Action endModelCycle(CPU_INSTANCE_ID cpu_iid);
    action

        debugLog.record(cpu_iid, $format("Model cycle complete."));

        // Sample event & statistic (commit)
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];
        eventCom.recordEvent(cpu_iid, tagged Valid truncate(pc));
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
    function Bool tokIsLoadOrStore(TOKEN tok) = tokIsLoad(tok) || tokIsStore(tok);

    rule stage1_itrReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        linkModelCycle.send(cpu_iid);


        // Translate next pc.
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];
        let ctx_id = getContextId(cpu_iid);
        linkToITR.makeReq(initFuncpReqDoITranslate(ctx_id, pc));
        debugLog.record(cpu_iid, $format("Translating virtual address: 0x%h", pc));

    endrule

    rule stage2_itrRsp_fetReq (True);

        // Get the ITrans response started by stage1_itrReq
        let rsp = linkToITR.getResp();
        linkToITR.deq();

        let cpu_iid = getCpuInstanceId(rsp.contextId);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("ITR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin

            // Fetch the next instruction
            linkToFET.makeReq(initFuncpReqGetInstruction(rsp.contextId, rsp.physicalAddress, rsp.offset));
            debugLog.record(cpu_iid, $format("Fetching physical address: 0x%h, offset: 0x%h", rsp.physicalAddress, rsp.offset));

        end

    endrule

    rule stage3_fetRsp_decReq (True);

        // Get the instruction response
        let rsp = linkToFET.getResp();
        linkToFET.deq();

        let cpu_iid = getCpuInstanceId(rsp.contextId);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("FET Responded"));

        // Tell the functional partition to decode the current instruction and place it in flight.
        linkToDEC.makeReq(initFuncpReqGetDependencies(rsp.contextId, rsp.instruction, pc));
        debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", rsp.instruction));

        decQ.enq(tuple2(isaIsLoad(rsp.instruction), isaIsStore(rsp.instruction)));

    endrule

    rule stage4_decRsp_exeReq (True);

        match {.is_load, .is_store} = decQ.first();
        decQ.deq();

        // Get the decode response
        let rsp = linkToDEC.getResp();
        linkToDEC.deq();

        // Get the token the functional partition assigned to this instruction.
        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("DEC Responded with token: %0d", tokTokenId(tok)));

        // Record load and store properties for the instruction in the token
        tok.timep_info.scratchpad[0] = pack(is_load);
        tok.timep_info.scratchpad[1] = pack(is_store);

        // In a more complex processor we would use the dependencies 
        // to determine if we can issue the instruction.

        // Execute the instruction
        linkToEXE.makeReq(initFuncpReqGetResults(tok));
        debugLog.record(cpu_iid, $format("Executing token: %0d", tokTokenId(tok)));

    endrule

    rule stage5_exeRsp (True);

        // Get the execution result
        let exe_resp = linkToEXE.getResp();
        linkToEXE.deq();

        let tok = exe_resp.token;
        let res = exe_resp.result;

        let cpu_iid = tokCpuInstanceId(tok);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("EXE Responded with token: %0d", tokTokenId(tok)));

        // If it was a branch we must update the PC.
        case (res) matches

            tagged RBranchTaken .addr:
            begin

                debugLog.record(cpu_iid, $format("Branch taken to address: 0x%h", addr));
                pc <= addr;

            end

            tagged RBranchNotTaken .addr:
            begin

                debugLog.record(cpu_iid, $format("Branch not taken"));
                pc <= pc + 4;

            end

            tagged RTerminate .pf:
            begin

                debugLog.record(cpu_iid, $format("Terminating Execution. PassFail: %0b", pf));
                localCtrl.instanceDone(cpu_iid, pf);

            end

            default:
            begin

                pc <= pc + 4;

            end

        endcase

        if (tokIsLoad(tok) || tokIsStore(tok))
        begin

            // Memory ops require more work.
            debugLog.record(cpu_iid, $format("DTranslate request for token: %0d", tokTokenId(tok)));

            // Get the physical address(es) of the memory access.
            linkToDTR.makeReq(initFuncpReqDoDTranslate(tok));

        end

        // Everything else is a no-op in the next stage.
        dtrQ.enq(tok);

    endrule

    rule stage6_dtrRsp_memReq (tokIsLoadOrStore(dtrQ.first()));
    
        // Get the response from dTranslate
        let rsp = linkToDTR.getResp();
        linkToDTR.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("DTR Responded for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

        if (! rsp.hasMore)
        begin

            dtrQ.deq();
            dmemQ.enq(tok);

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

    rule stage6_nonMemoryOp (!tokIsLoadOrStore(dtrQ.first()));

        // Just pass to the next stage.
        dmemQ.enq(dtrQ.first());
        dtrQ.deq();

    endrule

    rule stage7_loaRsp (tokIsLoad(dmemQ.first()));

        dmemQ.deq();

        // Get the load response
        let rsp = linkToLOA.getResp();
        linkToLOA.deq();

        let tok = rsp.token;
        let cpu_iid= tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("Load ops responded for token: %0d", tokTokenId(tok)));

        commitQ.enq(tok);

    endrule

    (* descending_urgency = "stage7_stoRsp, stage7_loaRsp" *)
    rule stage7_stoRsp (tokIsStore(dmemQ.first()));

        dmemQ.deq();

        // Get the store response
        let rsp = linkToSTO.getResp();
        linkToSTO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format(": Store ops responded for token: %0d", tokTokenId(tok)));

        commitQ.enq(tok);

    endrule

    rule stage7_nonMemoryOp (!tokIsLoad(dmemQ.first()) && !tokIsStore(dmemQ.first()));
    
        // Just pass to the next stage.
        commitQ.enq(dmemQ.first());
        dmemQ.deq();
    
    endrule

    rule stage8_lcoReq (True);

        // Get the current token from previous rule
        let tok = commitQ.first();
        let cpu_iid = tokCpuInstanceId(tok);
        commitQ.deq();

        if (! tokIsPoisoned(tok))
        begin

            // Locally commit the token.
            linkToLCO.makeReq(initFuncpReqCommitResults(tok));
            debugLog.record(cpu_iid, $format("Locally committing token: %0d", tokTokenId(tok)));

        end
        else
        begin

            // Token is poisoned.  Invoke fault handler.
            linkToHandleFault.makeReq(initFuncpReqHandleFault(tok));
            debugLog.record(cpu_iid, $format("Handling fault for token: %0d", tokTokenId(tok)));

        end

    endrule

    rule stage9_lcoRsp_gcoReq (True);

        let rsp = linkToLCO.getResp();
        linkToLCO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("LCO responded for token: %0d", tokTokenId(tok)));

        if (tokIsStore(tok))
        begin

            // Request global commit of stores.
            let st_tok = validValue(rsp.storeToken);
            linkToGCO.makeReq(initFuncpReqCommitStores(st_tok));
            debugLog.record(cpu_iid, $format("Globally committing stores for token: %0d (store token: %0d)", tokTokenId(tok), storeTokTokenId(st_tok)));

        end
        else
        begin

            endModelCycle(cpu_iid);

        end

    endrule

    rule stage10_gcoRsp (True);

        // Get the global commit response
        let rsp = linkToGCO.getResp();
        linkToGCO.deq();

        let st_tok = rsp.storeToken;
        let cpu_iid = storeTokCpuInstanceId(st_tok);

        debugLog.record(cpu_iid, fshow(st_tok.index) + $format("GCO Responded for store token: %0d", storeTokTokenId(st_tok)));

        endModelCycle(cpu_iid);

    endrule

    (* descending_urgency = "stage10_faultRsp, stage10_gcoRsp, stage9_lcoRsp_gcoReq" *)
    (* descending_urgency = "stage10_faultRsp, stage5_exeRsp" *)
    rule stage10_faultRsp (True);

        // Fault response (started by lco_req)
        let rsp = linkToHandleFault.getResp();
        linkToHandleFault.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);
        
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("Handle fault responded for token: %0d, new PC: 0x%h", tokTokenId(tok), rsp.nextInstructionAddress));

        // Next PC following fault
        pc <= rsp.nextInstructionAddress;

        endModelCycle(cpu_iid);

    endrule

endmodule

