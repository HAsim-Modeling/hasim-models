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
`include "asim/provides/common_services.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/fpga_components.bsh"

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"

`include "asim/dict/EVENTS_CPU.bsh"
`include "asim/dict/STATS_CPU.bsh"

`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"


/***************** Unpipelined Core With Caches  *****************/
/*                                                               */
/* Like the original unpipelined model, but connects to a        */
/* cache hierarchy. The model blocks on a cache miss and resumes */
/* when the miss returns, so therefore there will never be more  */
/* than one outstanding request in the memory subsystem.         */
/*                                                               */
/*****************************************************************/

// CORE_STATE

// The state of the core - either running, waiting for a cache miss,
// performing the second half of an unaligned data access,
// or retrying a dcache request.

typedef union tagged
{

    void CORE_NORMAL;
    ISA_INSTRUCTION CORE_WAIT_FOR_ICACHE;
    TOKEN SECOND_UNALIGNED_DADDR;
    TOKEN CORE_WAIT_FOR_DCACHE;
    TOKEN CORE_RETRY_DCACHE;

}
CORE_STATE deriving (Eq, Bits);



module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_cpu.out");

    //********* State Elements *********//

    // Program counters
    MULTIPLEXED#(NUM_CPUS, Reg#(ISA_ADDRESS)) pcPool <- mkMultiplexed(mkReg(`PROGRAM_START_ADDR));

    // Processor state
    MULTIPLEXED#(NUM_CPUS, Reg#(PROC_STATE)) statePool <- mkMultiplexed(mkReg(tagged CORE_NORMAL));
    
    //********* UnModel State *******//
    
    Reg#(Maybe#(INSTANCE_ID#(NUM_CPUS))) dTransStall <- mkReg(tagged Invalid);

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

    // For killing. UNUSED

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, 
                       FUNCP_RSP_REWIND_TO_TOKEN)     linkToRewindToToken <- mkConnection_Client("funcp_rewindToToken");

    // Ports
    
    // To/From ICache
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immRspFromICache <- mkPortRecvDependent_Multiplexed("ICache_to_CPU_load_immediate");
    
    // To/From DCache

    // Events
    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_CPU_INSTRUCTION_COMMIT);

    // Stats
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCom <- mkStatCounter_Multiplexed(`STATS_CPU_INSTRUCTION_COMMIT);

    Vector#(0, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(0, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);
    
    STAGE_CONTROLLER#(NUM_CPUS, Tuple2#(Bool, Bool)) stage4Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, TOKEN) stage6Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, TOKEN) stage7Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, TOKEN) stage8Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_CPUS, Bool) stage10Ctrl <- mkStageController();

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

    (* conservative_implicit_conditions *)
    rule stage1_itrReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        linkModelCycle.send(cpu_iid);
        debugLog.nextModelCycle(cpu_iid);
        
        // Check for delayed responses from the caches.
        // We assume that these are mutually exclusive.
        
        let m_irsp <- delRspFromICache.receive(cpu_iid);
        let m_drsp <- delRspFromDCache.receive(cpu_iid);
        
        if (m_irsp matches tagged Valid .rsp)
        begin
        
            // assert state == WAIT_FOR_ICACHE
        
            // Unstall.
            state <= tagged CORE_ICACHE_RESPONSE;
            debugLog.record_next_cycle(cpu_iid, $format("ICache responded with address 0x%0h", rsp.address);
            
            // No ITranslate request.
            stage2Ctrl.ready(CPU_IID, tagged STAGE2_BUBBLE);
        
        end
        else if (m_drsp matches tagged Valid .rsp)
        begin
        
            // assert state == WAIT_FOR_DCACHE

            // Unstall.
            state <= tagged CORE_DCACHE_RESPONSE;
            debugLog.record_next_cycle(cpu_iid, $format("DCache responded with address 0x%0h", rsp.address);
            
            // No ITranslate request.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_BUBBLE);
        
        end
        else
        begin
        
            // No cache response. Are we stalled?
            
            if (state == CORE_NORMAL)
            begin
            
                // Translate next pc.
                Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];
                let ctx_id = getContextId(cpu_iid);
                linkToITR.makeReq(initFuncpReqDoITranslate(ctx_id, pc));
                debugLog.record_next_cycle(cpu_iid, $format("Translating virtual address: 0x%h", pc));
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_TRANSLATE);

            end
            else
            begin
                
                // Stalled, so just bubble.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_BUBBLE);
                
            end
        
        end


    endrule

    rule stage2_itrRsp_fetReq (True);

        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        if (state matches tagged STAGE2_TRANSLATE)
        begin
        
            // Get the ITrans response started by stage1_itrReq
            let rsp = linkToITR.getResp();
            linkToITR.deq();

            let cpu_iid = getCpuInstanceId(rsp.contextId);
            Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

            debugLog.record(cpu_iid, $format("ITR Responded, hasMore: %0d", rsp.hasMore));

            // This model assumes the instruction is aligned.
            
            // Read the iCache to see if it hits
            portToICache.send(XXX);

            // Fetch the next instruction
            linkToFET.makeReq(initFuncpReqGetInstruction(rsp.contextId, rsp.physicalAddress, rsp.offset));
            debugLog.record(cpu_iid, $format("Fetching physical address: 0x%h, offset: 0x%h", rsp.physicalAddress, rsp.offset));
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_FETCH);

        end
        else
        begin
        
            // Propogate the bubble.
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_BUBBLE);
        
        end

    endrule

    rule stage3_fetRsp_decReq (True);
    
        match {.cpu_iid, .state} <- stage3Ctrl.nextReadyInstance();
        
        if (state matches tagged STAGE3_FETCH)
        begin

            // Get the instruction response
            let rsp = linkToFET.getResp();
            linkToFET.deq();
            debugLog.record(cpu_iid, $format("FET Responded"));

            // See if the icache hit.
            let m_rsp <- immRspFromICache.receive(cpu_iid);

            if (m_rsp matches tagged Valid .ic_rsp)
            begin

                case (ic_rsp.rspType) matches 
                    tagged .ICACHE_hit:
                    begin

                        // Get the PC for decoding the instruction.
                        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

                        // Tell the functional partition to decode the current instruction and place it in flight.
                        linkToDEC.makeReq(initFuncpReqGetDependencies(rsp.contextId, rsp.instruction, pc));
                        debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", rsp.instruction));

                        // Continue processing the next stage.
                        stage4Ctrl.ready(cpu_iid, tagged STAGE4_DECODE tuple2(isaIsLoad(rsp.instruction), isaIsStore(rsp.instruction)));

                    end
                    tagged .ICACHE_miss:
                    begin

                        // Bubble out, and start waiting for the response.
                        state <= tagged CORE_WAIT_FOR_ICACHE;
                        stalledInstruction <= rsp.instruction;
                        debugLog.record(cpu_iid, $format("Stalling on ICache miss: 0x%h"));
                        stage4Ctrl.ready(cpu_iid, tagged STAGE4_BUBBLE);

                    end
                    default:
                    begin

                        // A retry.
                        // Send a bubble on down, we'll try again next cycle.
                        debugLog.record(cpu_iid, $format("ICache retry"));
                        stage4Ctrl.ready(cpu_iid, tagged STAGE4_BUBBLE);

                    end
                endcase

            end
            else
            begin

                // Propogate the bubble.
                stage4Ctrl.ready(cpu_iid, tagged STAGE4_BUBBLE);

            end
        end
        begin

            // See if we're recovering from an earlier bubble.
            if (state == CORE_ICACHE_RESPONSE)
            begin
            
                 // Get the PC for decoding the instruction.
                 Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

                 // Tell the functional partition to decode the current instruction and place it in flight.
                 linkToDEC.makeReq(initFuncpReqGetDependencies(getContextId(cpu_iid), stalledInstruction, pc));
                 debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", rsp.instruction));

                 // Continue processing the next stage.
                 stage4Ctrl.ready(cpu_iid, tagged STAGE4_DECODE tuple2(isaIsLoad(rsp.instruction), isaIsStore(rsp.instruction)));

            end
            else
            begin

                // Propogate the bubble.
                stage4Ctrl.ready(cpu_iid, tagged STAGE4_BUBBLE);

            end

        end

    endrule

    rule stage4_decRsp_exeReq (True);

        match {.cpu_iid, .state} <- stage4Ctrl.nextReadyInstance();

        if (state matches tagged STAGE4_DECODE {.is_load, .is_store})
        begin

            // Get the decode response
            let rsp = linkToDEC.getResp();
            linkToDEC.deq();

            // Get the token the functional partition assigned to this instruction.
            let tok = rsp.token;
            debugLog.record(cpu_iid, $format("DEC Responded with token: %0d", tokTokenId(tok)));

            // Record load and store properties for the instruction in the token
            tok.timep_info.scratchpad[0] = pack(is_load);
            tok.timep_info.scratchpad[1] = pack(is_store);

            // In a more complex processor we would use the dependencies 
            // to determine if we can issue the instruction.

            // Execute the instruction
            linkToEXE.makeReq(initFuncpReqGetResults(tok));
            debugLog.record(cpu_iid, $format("Executing token: %0d", tokTokenId(tok)));

            stage5Ctrl.ready(cpu_iid, tagged STAGE5_EXECUTE);

        end
        else
        begin
        
            // Propogate the bubble
            stage5Ctrl.ready(cpu_iid, tagged STAGE5_BUBBLE);
        
        end

    endrule

    rule stage5_exeRsp (True);

        match {.cpu_iid, .state} <- stage5Ctrl.nextReadyInstance();

        if (state matches tagged STAGE6_EXECUTE)
        begin

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

            let is_load = tokIsLoad(tok);
            let is_store = tokIsStore(tok);

            if (is_load || is_store)
            begin

                // Memory ops require more work.
                debugLog.record(cpu_iid, $format("DTranslate request for token: %0d", tokTokenId(tok)));

                // Get the physical address(es) of the memory access.
                linkToDTR.makeReq(initFuncpReqDoDTranslate(tok));
                stage6Ctrl.ready(cpu_iid, tagged STAGE6_TRANSLATE tok);

            end
            else
            begin
            
                // Everything else is a no-op in the next stage.
                stage6Ctrl.ready(cpu_iid, tagged STAGE6_NOMEM tok);
            
            end

        end
        else
        begin
        
            // Propogate the bubble.
            stage6Ctrl.ready(cpu_iid, tagged STAGE6_BUBBLE);
        
        end

    endrule

    rule stage6_dtrRsp_memReq (!isValid(dTransStall));
    
        match {.cpu_iid, .state} <- stage6Ctrl.nextReadyInstance();

        if (state matches tagged STAGE6_TRANSLATE .tok)
        begin

            // Get the response from dTranslate
            let rsp = linkToDTR.getResp();
            linkToDTR.deq();

            tok = rsp.token;

            debugLog.record(cpu_iid, $format("DTR Responded for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

            if (! rsp.hasMore)
            begin

                if (tokIsLoad(tok))
                begin

                    // Request the load(s).
                    linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                    debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));

                end
                else if (tokIsStore(tok))
                begin

                    // Request the store(s)
                    linkToSTO.makeReq(initFuncpReqDoStores(tok));
                    debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));

                end

                stage7Ctrl.ready(cpu_iid, tagged STAGE7_TRANSLATE tok);

            end
            else
            begin

                // Get the rest of the unaligned address.
                dTransStall <= tagged Valid cpu_iid;
            end

        end
        else
        begin
            stage7Ctrl.ready(cpu_iid, tagged STAGE7_NOMEM tok);
        end

    endrule
    
    rule stage6_dtrRsp_unaligned (dTransStall matches tagged Valid .cpu_iid);
    
        match {.cpu_iid, .state} <- stage6Ctrl.nextReadyInstance();
       
        
        // Get the response from dTranslate
        let rsp = linkToDTR.getResp();
        linkToDTR.deq();

        let tok = rsp.token;

        debugLog.record(cpu_iid, $format("DTR Responded (unaligned) for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

        if (! rsp.hasMore)
        begin

            if (tokIsLoad(tok))
            begin

                // Request the load(s).
                linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));

            end
            else if (tokIsStore(tok))
            begin

                // Request the store(s)
                linkToSTO.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));

            end

            stage7Ctrl.ready(cpu_iid, tok);
            dTransStall <= tagged Invalid;

        end
        
    endrule

    rule stage7_memRsp (True);

        match {.cpu_iid, .state} <- stage7Ctrl.nextReadyInstance();
        
        // Get the DCache response.
        let m_rsp <- immRspFromDCache.receive(cpu_iid);
                
        case (state) matches
            tagged STAGE7_BUBBLE:
            begin
            
            end
            tagged STAGE7_NOMEM .tok:
            begin
            
            end
            tagged STAGE7_TRANSLATE .is_load:
            begin
            
                if (is_load)
                begin

                    // Get the load response
                    let rsp = linkToLOA.getResp();
                    linkToLOA.deq();

                    tok = rsp.token;
                    debugLog.record(cpu_iid, $format("Load ops responded for token: %0d", tokTokenId(tok)));

                end
                else
                begin
                
                    // Get the store response
                    let rsp = linkToSTO.getResp();
                    linkToSTO.deq();

                    tok = rsp.token;
                    debugLog.record(cpu_iid, $format(": Store ops responded for token: %0d", tokTokenId(tok)));

                end

                // See if the DCache hit.
                if (m_rsp matches tagged .rsp)
                begin
                
                    case (rsp) matches
                    tagged DCACHE_hit:
                    begin
                    
                    end
                    tagged DCACHE_miss:
                    begin
                    
                    end
                    tagged DCACHE_retry:
                    begin
                        
                        state <= tagged CORE_RETRY_DCACHE tok;
                        
                    end
                
                end


                stage8Ctrl.ready(cpu_iid, tok);


            end
        endcase
        
        
    endrule

    rule stage8_lcoReq (True);

        match {.*, .tok} <- stage8Ctrl.nextReadyInstance();

        // Get the current token from previous rule
        let cpu_iid = tokCpuInstanceId(tok);

        // Locally commit the token.
        linkToLCO.makeReq(initFuncpReqCommitResults(tok));
        debugLog.record(cpu_iid, $format("Locally committing token: %0d", tokTokenId(tok)));

    endrule

    rule stage9_lcoRsp_gcoReq (True);

        let rsp = linkToLCO.getResp();
        linkToLCO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("LCO responded for token: %0d", tokTokenId(tok)));

        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        if (rsp.faultRedirect matches tagged Valid .new_addr)
        begin

            debugLog.record(cpu_iid, $format("LCO responded with fault for token: %0d, new PC: 0x%h", tokTokenId(tok), new_addr));

            // Next PC following fault
            pc <= new_addr;

            stage10Ctrl.ready(cpu_iid, False);

        end
        else if (rsp.storeToken matches tagged Valid .st_tok)
        begin

            // Request global commit of stores.
            linkToGCO.makeReq(initFuncpReqCommitStores(st_tok));
            debugLog.record(cpu_iid, $format("Globally committing stores for token: %0d (store token: %0d)", tokTokenId(tok), storeTokTokenId(st_tok)));
            stage10Ctrl.ready(cpu_iid, True);

        end
        else
        begin

            stage10Ctrl.ready(cpu_iid, False);

        end

    endrule
  
    (* descending_urgency = "stage10_gcoRsp, stage9_lcoRsp_gcoReq" *)
    rule stage10_gcoRsp (True);

        match {.cpu_iid, .need_store_rsp} <- stage10Ctrl.nextReadyInstance();

        if (need_store_rsp)
        begin

            // Get the global commit response
            let rsp = linkToGCO.getResp();
            linkToGCO.deq();

            let st_tok = rsp.storeToken;

            debugLog.record(cpu_iid, fshow(st_tok.index) + $format("GCO Responded for store token: %0d", storeTokTokenId(st_tok)));

        end
        
        endModelCycle(cpu_iid);

    endrule


endmodule

