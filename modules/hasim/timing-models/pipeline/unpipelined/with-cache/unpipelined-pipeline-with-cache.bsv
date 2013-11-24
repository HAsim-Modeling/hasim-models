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
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

`include "asim/dict/EVENTS_CPU.bsh"

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

typedef union tagged
{
    void UNSTALLED;
    t_DATA STALLED;
    t_DATA UNSTALLING;
}
STALL_STATE#(parameter type t_DATA) deriving (Eq, Bits);

typedef STALL_STATE#(ISA_INSTRUCTION) STALL_FETCH;
typedef STALL_STATE#(TOKEN) STALL_LOAD;
typedef STALL_STATE#(Tuple2#(TOKEN, MEM_ADDRESS)) STALL_STORE;


module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_cpu.out");

    //********* State Elements *********//

    // Program counters
    MULTIPLEXED_REG_MULTI_WRITE#(MAX_NUM_CPUS, 3, ISA_ADDRESS) pcPool <- mkMultiplexedRegMultiWrite(`PROGRAM_START_ADDR);

    // Processor state
    MULTIPLEXED_REG_MULTI_WRITE#(MAX_NUM_CPUS, 2, STALL_FETCH) stalledFetchPool <- mkMultiplexedRegMultiWrite(tagged UNSTALLED);
    MULTIPLEXED_REG_MULTI_WRITE#(MAX_NUM_CPUS, 2, STALL_LOAD)  stalledLoadPool  <- mkMultiplexedRegMultiWrite(tagged UNSTALLED);
    MULTIPLEXED_REG_MULTI_WRITE#(MAX_NUM_CPUS, 3, STALL_STORE) stalledStorePool <- mkMultiplexedRegMultiWrite(tagged UNSTALLED);
    
    //********* UnModel State *******//
    
    Reg#(Maybe#(INSTANCE_ID#(MAX_NUM_CPUS))) dTransStall <- mkReg(tagged Invalid);

    //********* Connections *********//

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         linkModelCycle <- mkConnection_Send("model_cycle");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)        linkModelCommit <- mkConnection_Send("model_commits");

    Connection_Client#(Maybe#(FUNCP_REQ_DO_ITRANSLATE),
                       FUNCP_RSP_DO_ITRANSLATE)       linkToITR <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)     linkToFET <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES)    linkToDEC <- mkConnection_Client("funcp_getDependencies");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS)         linkToEXE <- mkConnection_Client("funcp_getResults");

    Connection_Client#(Maybe#(FUNCP_REQ_DO_DTRANSLATE),
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
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_INPUT)                loadToICache <- mkPortSend_Multiplexed("CPU_to_ICache_load");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immRspFromICache <- mkPortRecvDependent_Multiplexed("ICache_to_CPU_load_immediate");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_DELAYED)   delRspFromICache <- mkPortRecv_Multiplexed("ICache_to_CPU_load_delayed", 1);

    // To/From DCache
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_INPUT)                    loadToDCache <- mkPortSend_Multiplexed("CPU_to_DCache_load");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) immLoadRspFromDCache <- mkPortRecvDependent_Multiplexed("DCache_to_CPU_load_immediate");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED)   delLoadRspFromDCache <- mkPortRecv_Multiplexed("DCache_to_CPU_load_delayed", 1);


    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_INPUT)                    storeToDCache <- mkPortSend_Multiplexed("CPU_to_DCache_store");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) immStoreRspFromDCache <- mkPortRecvDependent_Multiplexed("DCache_to_CPU_store_immediate");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED)   delStoreRspFromDCache <- mkPortRecv_Multiplexed("DCache_to_CPU_store_delayed", 1);

    // Events
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_CPU_INSTRUCTION_COMMIT);

    // Stats
    STAT_VECTOR#(MAX_NUM_CPUS) statCom <-
        mkStatCounter_Multiplexed(statName("CPU_INSTRUCTION_COMMIT",
                                           "Committed Instructions"));

    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) depports = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0] = delRspFromICache.ctrl;
    inports[1] = delLoadRspFromDCache.ctrl;
    inports[2] = delStoreRspFromDCache.ctrl;
    depports[0] = immLoadRspFromDCache.ctrl;
    depports[1] = immStoreRspFromDCache.ctrl;
    depports[2] = immRspFromICache.ctrl;
    outports[0] = loadToICache.ctrl;
    outports[1] = loadToDCache.ctrl;
    outports[2] = storeToDCache.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalControllerWithUncontrolled(inports, depports, outports);
    
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool)  stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple2#(Bool, Bool))) stage4Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(TOKEN)) stage5Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool)  stage6Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(TOKEN)) stage7Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple3#(TOKEN, Bool, MEM_ADDRESS))) stage8Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple2#(TOKEN,  MEM_ADDRESS))) stage9Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple3#(TOKEN, Bool, MEM_ADDRESS))) stage10Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(TOKEN)) stage11Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Bool) stage12Ctrl <- mkStageController();

    //********* Rules *********//

    // Mapping from cpu id to context ids and back.
    function CPU_INSTANCE_ID getCpuInstanceId(CONTEXT_ID ctx_id) = ctx_id;
    function CPU_INSTANCE_ID tokCpuInstanceId(TOKEN tok) = tokContextId(tok);
    function CPU_INSTANCE_ID storeTokCpuInstanceId(STORE_TOKEN st_tok) = storeTokContextId(st_tok);
    function CONTEXT_ID getContextId(CPU_INSTANCE_ID cpu_iid) = cpu_iid;


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
        
        Reg#(ISA_ADDRESS)      pc = pcPool.getRegWithWritePort(cpu_iid, 0);
        Reg#(STALL_FETCH) stalledFetch = stalledFetchPool.getRegWithWritePort(cpu_iid, 0);
        Reg#(STALL_LOAD)   stalledLoad = stalledLoadPool.getRegWithWritePort(cpu_iid, 0);
        Reg#(STALL_STORE)   stalledStore = stalledStorePool.getRegWithWritePort(cpu_iid, 0);

        // Check for delayed responses from the caches.
        // We assume that these are mutually exclusive, and only
        // happen when stalled.
        
        let m_irsp <- delRspFromICache.receive(cpu_iid);
        let m_ld_rsp <- delLoadRspFromDCache.receive(cpu_iid);
        let m_st_rsp <- delStoreRspFromDCache.receive(cpu_iid);
        
        if (stalledFetch matches tagged STALLED .inst &&& m_irsp matches tagged Valid .rsp)
        begin

            // Unstall.
            stalledFetch <= UNSTALLING(inst);
            debugLog.record_next_cycle(cpu_iid, $format("ICache responded from Miss."));
            
            // Unlike most functional methods, doITranslate requires
            // a no-message.  There will be no response.
            linkToITR.makeReq(tagged Invalid);

            // Skip translation, ICache load.
            stage2Ctrl.ready(cpu_iid, False);
        
        end
        else if (stalledLoad matches tagged STALLED .tok &&& m_ld_rsp matches tagged Valid .rsp)
        begin
        
            // Unstall.
            stalledLoad <= UNSTALLING(tok);
            debugLog.record_next_cycle(cpu_iid, $format("DCache responded from Load Miss."));
            
            linkToITR.makeReq(tagged Invalid);

            // Skip all stages except commit.
            stage2Ctrl.ready(cpu_iid, False);
        
        end
        else if (stalledStore matches tagged STALLED {.tok, .addr} &&& m_st_rsp matches tagged Valid .rsp)
        begin

            // Unstall.
            stalledStore <= UNSTALLING(tuple2(tok, addr));
            debugLog.record_next_cycle(cpu_iid, $format("DCache responded from Store Miss."));
            
            linkToITR.makeReq(tagged Invalid);

            // Skip all stages except store + commit.
            stage2Ctrl.ready(cpu_iid, tagged False);
        
        end
        else if (stalledFetch matches tagged UNSTALLED &&& stalledLoad matches tagged UNSTALLED &&& stalledStore matches tagged UNSTALLED)
        begin
        
            // Translate next pc.
            let ctx_id = getContextId(cpu_iid);
            linkToITR.makeReq(tagged Valid initFuncpReqDoITranslate(ctx_id, pc));
            debugLog.record_next_cycle(cpu_iid, $format("Translating virtual address: 0x%h", pc));
            stage2Ctrl.ready(cpu_iid, True);

        end
        else
        begin
        
            // Stalled, so just bubble.
            debugLog.record_next_cycle(cpu_iid, $format("STALL"));
            stage2Ctrl.ready(cpu_iid, False);
            
            linkToITR.makeReq(tagged Invalid);

        end

    endrule

    rule stage2_itrRsp_fetReq (True);

        match {.cpu_iid, .did_itr} <- stage2Ctrl.nextReadyInstance();
        Reg#(ISA_ADDRESS) pc = pcPool.getRegWithWritePort(cpu_iid, 0);

        if (did_itr)
        begin

            // Get the ITrans response started by stage1_itrReq
            let rsp = linkToITR.getResp();
            linkToITR.deq();

            debugLog.record(cpu_iid, $format("ITR Responded, hasMore: %0d", rsp.hasMore));

            // This model assumes the instruction is aligned.

            // Read the iCache to see if it hits.
            loadToICache.send(cpu_iid, tagged Valid initICacheLoad(initIMemBundle(?, pc, ?, cpu_iid)));

            // Fetch the next instruction
            linkToFET.makeReq(initFuncpReqGetInstruction(getContextId(cpu_iid), rsp.physicalAddress, rsp.offset));
            debugLog.record(cpu_iid, $format("Fetching physical address: 0x%h, offset: 0x%h", rsp.physicalAddress, rsp.offset));
            stage3Ctrl.ready(cpu_iid, True);

        end
        else
        begin

            // No request to ICache
            loadToICache.send(cpu_iid, tagged Invalid);

            // Propogate the bubble.
            stage3Ctrl.ready(cpu_iid, False);

        end

    endrule

    rule stage3_fetRsp_decReq (True);

        match {.cpu_iid, .did_fet} <- stage3Ctrl.nextReadyInstance();

        Reg#(ISA_ADDRESS)      pc = pcPool.getRegWithWritePort(cpu_iid, 1);
        Reg#(STALL_FETCH) stalledFetch = stalledFetchPool.getRegWithWritePort(cpu_iid, 1);

        let m_ld_rsp <- immRspFromICache.receive(cpu_iid);
            
        if (did_fet)
        begin
        
            // Get the instruction response
            let rsp = linkToFET.getResp();
            linkToFET.deq();

            debugLog.record(cpu_iid, $format("FET Responded."));
            
            // assert isValid m_ld_rsp
            let ld_rsp = validValue(m_ld_rsp);
            
            // See if the ICache hit or missed.
            case (ld_rsp.rspType) matches
                tagged ICACHE_hit:
                begin
                
                    // Tell the functional partition to decode the current instruction and place it in flight.
                    linkToDEC.makeReq(initFuncpReqGetDependencies(rsp.contextId, rsp.instruction, pc));
                    debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", rsp.instruction));
                    stage4Ctrl.ready(cpu_iid, tagged Valid tuple2(isaIsLoad(rsp.instruction), isaIsStore(rsp.instruction)));
                
                end
                tagged ICACHE_miss .miss_id:
                begin
                
                    // Stall for this load to come back.
                    stalledFetch <= tagged STALLED rsp.instruction;
                    stage4Ctrl.ready(cpu_iid, tagged Invalid);
                
                end
                tagged ICACHE_retry:
                begin
                    // Don't stall, but retry next cycle.
                    stage4Ctrl.ready(cpu_iid, tagged Invalid);
                end
            endcase
        
        end
        else if (stalledFetch matches tagged UNSTALLING .inst)
        begin
        
            debugLog.record(cpu_iid, $format("FET Unstall."));
            stalledFetch <= tagged UNSTALLED;

            // Tell the functional partition to decode the current instruction and place it in flight.
            linkToDEC.makeReq(initFuncpReqGetDependencies(getContextId(cpu_iid), inst, pc));
            debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", inst));

            stage4Ctrl.ready(cpu_iid, tagged Valid tuple2(isaIsLoad(inst), isaIsStore(inst)));

        end
        else
        begin
            
            // Propogate the bubble.
            stage4Ctrl.ready(cpu_iid, tagged Invalid);

        end


    endrule

    rule stage4_decRsp (True);
    
        
        match {.cpu_iid, .did_dec} <- stage4Ctrl.nextReadyInstance();
        
        case (did_dec) matches
            tagged Valid {.is_load, .is_store}:
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
                stage5Ctrl.ready(cpu_iid, tagged Valid tok);
            
            end
            tagged Invalid:
            begin

                // Propogate the bubble.
                stage5Ctrl.ready(cpu_iid, tagged Invalid);
            
            end

        endcase

    
    endrule

    rule stage5_exeReq (True);

        match {.cpu_iid, .m_tok} <- stage5Ctrl.nextReadyInstance();
        
        if (m_tok matches tagged Valid .tok)
        begin
        
            linkToEXE.makeReq(initFuncpReqGetResults(tok));
            debugLog.record(cpu_iid, $format("Executing token: %0d", tokTokenId(tok)));
            stage6Ctrl.ready(cpu_iid, True);

        end
        else
        begin
        
            // Propogate the bubble.
            stage6Ctrl.ready(cpu_iid, False);

        end
        
    endrule

    rule stage6_exeRsp (True);

        match {.cpu_iid, .did_exe} <- stage6Ctrl.nextReadyInstance();

        Reg#(ISA_ADDRESS) pc = pcPool.getRegWithWritePort(cpu_iid, 1);
        
        if (did_exe)
        begin

            // Get the execution result
            let exe_resp = linkToEXE.getResp();
            linkToEXE.deq();

            let tok = exe_resp.token;
            let res = exe_resp.result;

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
                linkToDTR.makeReq(tagged Valid initFuncpReqDoDTranslate(tok));
            end
            else
            begin
                // Unlike most functional methods, doITranslate requires
                // a no-message.  There will be no response.
                doDTranslate.makeReq(tagged Invalid);
            end

            stage7Ctrl.ready(cpu_iid, tagged Valid tok);

        end
        else
        begin
            doDTranslate.makeReq(tagged Invalid);
            stage7Ctrl.ready(cpu_iid, tagged Invalid);
        end

    endrule

    rule stage7_dtrRsp_loaReq (!isValid(dTransStall));
    
        match {.cpu_iid, .m_tok} <- stage7Ctrl.nextReadyInstance();
    
        if (m_tok matches tagged Valid .tok)
        begin

            if (tokIsLoad(tok) || tokIsStore(tok))
            begin

                // Get the response from dTranslate
                let rsp = linkToDTR.getResp();
                linkToDTR.deq();

                let new_tok = rsp.token;

                debugLog.record(cpu_iid, $format("DTR Responded for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

                if (! rsp.hasMore)
                begin

                    if (tokIsLoad(new_tok))
                    begin

                        // Request the load(s).
                        let bundle = initDMemBundle(tok, ?, ?, tokIsLoad(tok), tokIsStore(tok), Invalid, ?);
                        bundle.physicalAddress = rsp.physicalAddress;
                        loadToDCache.send(cpu_iid, tagged Valid initDCacheLoad(bundle));
                        linkToLOA.makeReq(initFuncpReqDoLoads(new_tok));
                        debugLog.record(cpu_iid, fshow(new_tok.index) + $format(": Do loads"));

                    end
                    else
                    begin
                        // No load.
                        loadToDCache.send(cpu_iid, tagged Invalid);
                    end

                    stage8Ctrl.ready(cpu_iid, tagged Valid tuple3(new_tok, tokIsLoad(new_tok), rsp.physicalAddress));
                end
                else
                begin

                    dTransStall <= tagged Valid cpu_iid;

                end

            end
            else
            begin

                // Non-memory operation.
                loadToDCache.send(cpu_iid, tagged Invalid);
                stage8Ctrl.ready(cpu_iid, tagged Valid tuple3(tok, False, ?));

            end
        end
        else
        begin
        
            // Bubble.
            loadToDCache.send(cpu_iid, tagged Invalid);
            stage8Ctrl.ready(cpu_iid, tagged Invalid);

        end
        
    endrule
    
    rule stage7_dtrRsp_unaligned (dTransStall matches tagged Valid .cpu_iid);
    
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

                let bundle = initDMemBundle(tok, ?, ?, tokIsLoad(tok), tokIsStore(tok), Invalid, ?);
                bundle.physicalAddress = rsp.physicalAddress;
                loadToDCache.send(cpu_iid, tagged Valid initDCacheLoad(bundle));
                linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));

            end
            else if (tokIsStore(tok))
            begin

                // Request the store(s)
                loadToDCache.send(cpu_iid, tagged Invalid);
                linkToSTO.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));

            end

            stage8Ctrl.ready(cpu_iid, tagged Valid tuple3(tok, True, rsp.physicalAddress));
            dTransStall <= tagged Invalid;

        end
        
    endrule


    rule stage8_loaRsp (True);

        match {.cpu_iid, .m_tok} <- stage8Ctrl.nextReadyInstance();

        Reg#(STALL_LOAD) stalledLoad = stalledLoadPool.getRegWithWritePort(cpu_iid, 1);

        let m_ld_rsp <- immLoadRspFromDCache.receive(cpu_iid);

        if (stalledLoad matches tagged UNSTALLING .tok)
        begin

            // Unstall
            stalledLoad <= tagged UNSTALLED;

            // Proceed as normal.
            stage9Ctrl.ready(cpu_iid, tagged Valid tuple2(tok, ?));

        end
        else if (m_tok matches tagged Valid {.tok, .need_rsp, .store_address})
        begin
        
            let new_tok = tok;

            if (tokIsLoad(tok))
            begin
            
            
                if (need_rsp)
                begin

                    // Get the load response
                    let rsp = linkToLOA.getResp();
                    linkToLOA.deq();

                    new_tok = rsp.token;

                    debugLog.record(cpu_iid, $format("Load ops responded for token: %0d", tokTokenId(new_tok)));

                end

                // assert isValid m_ld_rsp
                let ld_rsp = validValue(m_ld_rsp);

                case (ld_rsp.rspType) matches
                    tagged DCACHE_hit:
                    begin

                        // Well, the cache found it.
                        debugLog.record(cpu_iid, fshow("DCache hit."));

                        // Proceed as normal.
                        stage9Ctrl.ready(cpu_iid, tagged Valid tuple2(new_tok, store_address));

                    end
                    tagged DCACHE_miss .miss_id:
                    begin

                        // The cache missed, but is handling it. 
                        debugLog.record(cpu_iid, fshow("DCache miss."));

                        // Stall.
                        stalledLoad <= tagged STALLED new_tok;
                        stage9Ctrl.ready(cpu_iid, tagged Invalid);

                    end

                    tagged DCACHE_retry:
                    begin

                        // The cache needs us to retry.
                        debugLog.record(cpu_iid, fshow("DCache retry."));
                        stalledLoad <= tagged UNSTALLING new_tok;

                        stage9Ctrl.ready(cpu_iid, tagged Invalid);

                    end

                endcase

            end
            else
            begin
            
                // Non-memoy op.
                stage9Ctrl.ready(cpu_iid, tagged Valid tuple2(tok, store_address));
            
            end

        end
        else
        begin
        
            // Propogate the bubble.
            stage9Ctrl.ready(cpu_iid, tagged Invalid);
        
        end

        
    endrule



    rule stage9_stoReq (True);
    
        match {.cpu_iid, .m_tok} <- stage9Ctrl.nextReadyInstance();
        Reg#(STALL_STORE) stalledStore = stalledStorePool.getRegWithWritePort(cpu_iid, 1);
    
        if (stalledStore matches tagged UNSTALLING {.tok, .address})
        begin
        
            // It's OK to retry the store.
            debugLog.record(cpu_iid, $format("Unstall store."));
            stalledStore <= UNSTALLED;
            storeToDCache.send(cpu_iid, tagged Valid initDCacheStore(address));
            stage10Ctrl.ready(cpu_iid, tagged Valid tuple3(tok, False, address));
        
        end
        else if (m_tok matches tagged Valid {.tok, .store_address})
        begin

            if (tokIsStore(tok))
            begin

                // Request the store(s)
                storeToDCache.send(cpu_iid, tagged Valid initDCacheStore(store_address));
                linkToSTO.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));
            
            end
            else
            begin
            
                // Non-memory operation.
                storeToDCache.send(cpu_iid, tagged Invalid);

            end
            
            stage10Ctrl.ready(cpu_iid, tagged Valid tuple3(tok, tokIsStore(tok), store_address));

        end
        else
        begin
        
            // Bubble.
            storeToDCache.send(cpu_iid, tagged Invalid);
            stage10Ctrl.ready(cpu_iid, tagged Invalid);

        end
        
    endrule


    rule stage10_stoRsp (True);

        match {.cpu_iid, .m_tok} <- stage10Ctrl.nextReadyInstance();

        Reg#(STALL_STORE) stalledStore = stalledStorePool.getRegWithWritePort(cpu_iid, 2);

        let m_st_rsp <- immStoreRspFromDCache.receive(cpu_iid);

        if (m_tok matches tagged Valid {.tok, .need_rsp, .store_address})
        begin
        
            let new_tok = tok;

            if (tokIsStore(tok))
            begin

                if (need_rsp)
                begin

                    // Get the store response
                    let rsp = linkToSTO.getResp();
                    linkToSTO.deq();
 
                    new_tok = rsp.token;

                    debugLog.record(cpu_iid, $format(": Store ops responded for token: %0d", tokTokenId(new_tok)));
                
                end

                // Assert isValid m_st_rsp
                
                case (validValue(m_st_rsp)) matches

                    tagged DCACHE_ok:
                    begin

                        debugLog.record(cpu_iid, fshow("Store OK"));

                        // Locally commit the token.
                        linkToLCO.makeReq(initFuncpReqCommitResults(new_tok));
                        debugLog.record(cpu_iid, $format("Locally committing token: %0d", tokTokenId(new_tok)));

                        // Proceed as normal.
                        stage11Ctrl.ready(cpu_iid, tagged Valid new_tok);

                    end

                    tagged DCACHE_delay .miss_id:
                    begin

                        debugLog.record(cpu_iid, fshow("Store Delay."));

                        // Stall on a response
                        stalledStore <= tagged STALLED tuple2(new_tok, store_address);

                        // Bubble the rest of the pipeline.
                        stage11Ctrl.ready(cpu_iid, tagged Invalid);

                    end

                    tagged DCACHE_retryStore:
                    begin

                        debugLog.record(cpu_iid, fshow("Store Retry."));
                        // No change. Try again next cycle.
                        // Bubble the rest of the pipeline.
                        stalledStore <= tagged UNSTALLING tuple2(new_tok, store_address);
                        stage11Ctrl.ready(cpu_iid, tagged Invalid);

                    end

                endcase

            end
            else
            begin
            
                // Non-memoy op.
                // Locally commit the token.
                linkToLCO.makeReq(initFuncpReqCommitResults(tok));
                debugLog.record(cpu_iid, $format("Locally committing token: %0d", tokTokenId(tok)));

                stage11Ctrl.ready(cpu_iid, tagged Valid tok);
            
            end

        end
        else
        begin
        
            // Propogate the bubble.
            stage11Ctrl.ready(cpu_iid, tagged Invalid);
        
        end

        
    endrule

    rule stage11_lcoRsp_gcoReq (True);
    
        match {.cpu_iid, .m_tok} <- stage11Ctrl.nextReadyInstance();
        Reg#(ISA_ADDRESS) pc = pcPool.getRegWithWritePort(cpu_iid, 2);

        if (m_tok matches tagged Valid .tok)
        begin

            let rsp = linkToLCO.getResp();
            linkToLCO.deq();

            debugLog.record(cpu_iid, $format("LCO responded for token: %0d", tokTokenId(tok)));

            // Sample event & statistic (commit)
            eventCom.recordEvent(cpu_iid, tagged Valid truncate(pc));
            statCom.incr(cpu_iid);

            // Commit counter for heartbeat
            linkModelCommit.send(tuple2(cpu_iid, 1));
        
            if (rsp.faultRedirect matches tagged Valid .new_addr)
            begin

                debugLog.record(cpu_iid, $format("LCO responded with fault for token: %0d, new PC: 0x%h", tokTokenId(tok), new_addr));

                // Next PC following fault
                pc <= new_addr;

                stage12Ctrl.ready(cpu_iid, False);

            end
            else if (rsp.storeToken matches tagged Valid .st_tok)
            begin

                // Request global commit of stores.
                linkToGCO.makeReq(initFuncpReqCommitStores(st_tok));
                debugLog.record(cpu_iid, $format("Globally committing stores for token: %0d (store token: %0d)", tokTokenId(tok), storeTokTokenId(st_tok)));
                stage12Ctrl.ready(cpu_iid, True);

            end
            else
            begin

                stage12Ctrl.ready(cpu_iid, False);

            end
        
        end
        else
        begin
        
            eventCom.recordEvent(cpu_iid, tagged Invalid);
            stage12Ctrl.ready(cpu_iid, False);
        
        end

    endrule

    rule stage12_gcoRsp (True);

        match {.cpu_iid, .need_store_rsp} <- stage12Ctrl.nextReadyInstance();

        if (need_store_rsp)
        begin

            // Get the global commit response
            let rsp = linkToGCO.getResp();
            linkToGCO.deq();

            let st_tok = rsp.storeToken;

            debugLog.record(cpu_iid, fshow(st_tok.index) + $format("GCO Responded for store token: %0d", storeTokTokenId(st_tok)));

        end
        
        debugLog.record(cpu_iid, $format("Model cycle complete."));

        // End the model cycle.
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule


endmodule
