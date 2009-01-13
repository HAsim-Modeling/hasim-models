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

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"

`include "asim/dict/EVENTS_CPU.bsh"
`include "asim/dict/STATS_CPU.bsh"
`include "asim/dict/PARAMS_HASIM_PIPELINE.bsh"

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


typedef enum
{
    CTX_STATE_READY,            // Ready to fetch a new token
    CTX_STATE_BUSY              // Processing a token
}
CTX_STATE
    deriving(Eq, Bits);


module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_cpu.out");

    // ***** Dynamic parameters ********//

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();

    Param#(8) skewContextsParam <- mkDynamicParameter(`PARAMS_HASIM_PIPELINE_SKEW_CONTEXTS, paramNode);

    //********* State Elements *********//

    // Context status and program counters
    Vector#(NUM_CONTEXTS, Reg#(CTX_STATE)) ctxState = newVector();
    Vector#(NUM_CONTEXTS, Reg#(ISA_ADDRESS)) pc = newVector();
    for (Integer c = 0; c < valueOf(NUM_CONTEXTS); c = c + 1)
    begin
        ctxState[c] <- mkReg(CTX_STATE_READY);
        pc[c] <- mkReg(`PROGRAM_START_ADDR);
    end

    // Pipe from any stage that flows to local commit
    FIFO#(TOKEN) commitQ <- mkFIFO();

    //********* Connections *********//

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         link_model_cycle <- mkConnection_Send("model_cycle");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)        link_model_commit <- mkConnection_Send("model_commits");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT, 
                       FUNCP_RSP_NEW_IN_FLIGHT)       link_to_tok <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE, 
                       FUNCP_RSP_DO_ITRANSLATE)       link_to_itr <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)     link_to_fet <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES)    link_to_dec <- mkConnection_Client("funcp_getDependencies");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS)         link_to_exe <- mkConnection_Client("funcp_getResults");

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE,
                       FUNCP_RSP_DO_DTRANSLATE)       link_to_dtr <- mkConnection_Client("funcp_doDTranslate");

    Connection_Client#(FUNCP_REQ_DO_LOADS,
                       FUNCP_RSP_DO_LOADS)            link_to_loa <- mkConnection_Client("funcp_doLoads");

    Connection_Client#(FUNCP_REQ_DO_STORES,
                       FUNCP_RSP_DO_STORES)           link_to_sto <- mkConnection_Client("funcp_doSpeculativeStores");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS,
                       FUNCP_RSP_COMMIT_RESULTS)      link_to_lco <- mkConnection_Client("funcp_commitResults");

    Connection_Client#(FUNCP_REQ_COMMIT_STORES,
                       FUNCP_RSP_COMMIT_STORES)       link_to_gco <- mkConnection_Client("funcp_commitStores");

    Connection_Client#(FUNCP_REQ_HANDLE_FAULT, 
                       FUNCP_RSP_HANDLE_FAULT)   link_handleFault <- mkConnection_Client("funcp_handleFault");

    // For killing. UNUSED

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, 
                       FUNCP_RSP_REWIND_TO_TOKEN)     link_rewindToToken <- mkConnection_Client("funcp_rewindToToken");


    // Events
    EventRecorder event_com <- mkEventRecorder(`EVENTS_CPU_INSTRUCTION_COMMIT);

    // Stats
    Stat stat_com <- mkStatCounter(`STATS_CPU_INSTRUCTION_COMMIT);

    Vector#(0, Port_Control) inports = newVector();
    Vector#(0, Port_Control) outports = newVector();

    LocalController local_ctrl <- mkLocalController(inports, outports);

    //********* Rules *********//

    // Predicates used below
    function Bool ctxIsDone(CTX_STATE s) = (s == CTX_STATE_READY);

    //
    // endModelCycle --
    //     Invoked by any rule that is completely done with a token.
    //
    function Action endModelCycle(TOKEN tok);
    action
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": Model cycle complete"));

        // Sample event & statistic (commit)
        event_com.recordEvent(tagged Valid zeroExtend(pack(tok.index)));
        stat_com.incr();

        // Commit counter for heartbeat
        link_model_commit.send(tuple2(ctx_id, 1));
        
        // Update state
        ctxState[ctx_id] <= CTX_STATE_READY;
    endaction
    endfunction

    
    //
    // "Skew" is a start-up algorithm to keep identical workloads on multiple
    // contexts from running synchronized.  Synchronized programs would
    // likely hit the worst case functional pipeline behavior.  Skew delays
    // each context by a constant amount (dynamic parameter SKEW_CONTEXTS).
    //
    // This function is not a requirement for proper functioning of the model.
    // If it simply returned true all the time the model would run correctly,
    // but all contexts would start running instructions in the first cycle.
    //
    Reg#(CONTEXT_ID) skewPos <- mkReg(0);
    Reg#(Bit#(8)) skewCnt <- mkReg(0);

    function ActionValue#(Bool) skewPermitsContext(CONTEXT_ID ctxId);
    actionvalue
        Bool permitCtx;

        if (ctxId <= skewPos)
        begin
            // Context is fully active
            permitCtx = True;
        end
        else if (ctxId == (skewPos + 1))
        begin
            // Context is next to become active
            if (skewCnt == skewContextsParam)
            begin
                // Time to go
                skewPos <= skewPos + 1;
                skewCnt <= 0;
                permitCtx = True;
            end
            else
            begin
                // Not yet
                skewCnt <= skewCnt + 1;
                permitCtx = False;
            end
        end
        else
        begin
            // Context is not yet active
            permitCtx = False;
        end

        return permitCtx;
    endactionvalue
    endfunction


    //
    // Whether an instruction is a load or store is stored in token scratchpad
    // memory.  These are accessor functions...
    //
    function Bool tokIsLoad(TOKEN tok) = unpack(tok.timep_info.scratchpad[0]);
    function Bool tokIsStore(TOKEN tok) = unpack(tok.timep_info.scratchpad[1]);

    // Current tok_req context.  Go round robin to keep contexts within a
    // cycle of each other.
    Reg#(CONTEXT_ID) curTokCtx <- mkReg(0);

    rule tok_req (ctxState[curTokCtx] == CTX_STATE_READY);
        local_ctrl.startModelCC();

        let ctx_id = curTokCtx;

        // Request a Token
        if (local_ctrl.contextIsActive(ctx_id))
        begin
            debugLog.nextModelCycle(ctx_id);
            link_model_cycle.send(ctx_id);

            //
            // Optional skew keeps each context from executing identical instructions
            // when all contexts are running the same workload.  This model would
            // work perfectly well if skewPermitsContext() always returned True.
            //
            let permit_ctx <- skewPermitsContext(ctx_id);
            if (permit_ctx)
            begin
                debugLog.record_next_cycle(ctx_id, $format("Requesting a new token"));
                link_to_tok.makeReq(initFuncpReqNewInFlight(ctx_id));
                ctxState[ctx_id] <= CTX_STATE_BUSY;
            end
            else
            begin
                debugLog.record_next_cycle(ctx_id, $format("Skipping cycle for skewed start"));
            end
        end
        else
        begin
            debugLog.record(ctx_id, $format("Context is not active"));
        end
        
        curTokCtx <= curTokCtx + 1;
    endrule

    rule tok_rsp_itr_req (True);
        // Get the response from tok_req
        let rsp = link_to_tok.getResp();
        link_to_tok.deq();

        let tok = rsp.newToken;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": TOK Responded"));

        // Translate next pc.
        let ctx_pc = pc[ctx_id];
        link_to_itr.makeReq(initFuncpReqDoITranslate(tok, ctx_pc));
        debugLog.record(ctx_id, fshow(tok.index) + $format(": Translating at address 0x%h", ctx_pc));
    endrule

    rule itr_rsp_fet_req (True);
        // Get the ITrans response started by tok_rsp_itr_req
        let rsp = link_to_itr.getResp();
        link_to_itr.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": ITR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin
            // Fetch the next instruction
            link_to_fet.makeReq(initFuncpReqGetInstruction(tok));
            debugLog.record(ctx_id, fshow(tok.index) + $format(": Fetching at address 0x%h", pc[tokContextId(tok)]));
        end
    endrule

    rule fet_rsp_dec_req (True);
        // Get the instruction response
        let rsp = link_to_fet.getResp();
        link_to_fet.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": FET Responded"));

        // Record load and store properties for the instruction in the token
        tok.timep_info.scratchpad[0] = pack(isaIsLoad(rsp.instruction));
        tok.timep_info.scratchpad[1] = pack(isaIsStore(rsp.instruction));

        // Decode the current inst
        link_to_dec.makeReq(initFuncpReqGetDependencies(tok));
        debugLog.record(ctx_id, fshow(tok.index) + $format(": Decoding"));
    endrule

    rule dec_rsp_exe_req (True);
        // Get the decode response
        let rsp = link_to_dec.getResp();
        link_to_dec.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": DEC Responded"));

        // In a more complex processor we would use the dependencies 
        // to determine if we can issue the instruction.

        // Execute the instruction
        link_to_exe.makeReq(initFuncpReqGetResults(tok));
        debugLog.record(ctx_id, fshow(tok.index) + $format(": Executing"));
    endrule

    rule exe_rsp (True);
        // Get the execution result
        let exe_resp = link_to_exe.getResp();
        link_to_exe.deq();

        let tok = exe_resp.token;
        let res = exe_resp.result;

        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": EXE Responded"));

        // If it was a branch we must update the PC.
        case (res) matches
            tagged RBranchTaken .addr:
            begin
                debugLog.record(ctx_id, $format("Branch taken to address %h", addr));
                pc[ctx_id] <= addr;
            end

            tagged RBranchNotTaken .addr:
            begin
                debugLog.record(ctx_id, $format("Branch not taken"));
                pc[ctx_id] <= pc[ctx_id] + 4;
            end

            tagged RTerminate .pf:
            begin
                debugLog.record(ctx_id, $format("Terminating Execution"));
                local_ctrl.endProgram(pf);
            end

            default:
            begin
                pc[ctx_id] <= pc[ctx_id] + 4;
            end
        endcase

        if (tokIsLoad(tok) || tokIsStore(tok))
        begin
            // Memory ops require more work.
            debugLog.record(ctx_id, fshow(tok.index) + $format(": DTranslate"));

            // Get the physical address(es) of the memory access.
            link_to_dtr.makeReq(initFuncpReqDoDTranslate(tok));
        end
        else
        begin
            // Everything else should just be committed.
            commitQ.enq(tok);
        end
    endrule

    rule dtr_rsp_mem_req (True);
        // Get the response from dTranslate
        let rsp = link_to_dtr.getResp();
        link_to_dtr.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": DTR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin
            if (tokIsLoad(tok))
            begin
                // Request the load(s).
                link_to_loa.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record(ctx_id, fshow(tok.index) + $format(": Do loads"));
            end
            else
            begin
                // Request the store(s)
                link_to_sto.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(ctx_id, fshow(tok.index) + $format(": Do stores"));
            end
        end
    endrule

    rule loa_rsp (True);
        // Get the load response
        let rsp = link_to_loa.getResp();
        link_to_loa.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": Load ops responded"));

        commitQ.enq(tok);
    endrule

    rule sto_rsp (True);
        // Get the store response
        let rsp = link_to_sto.getResp();
        link_to_sto.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": Store ops responded"));

        commitQ.enq(tok);
    endrule

    rule lco_req (True);
        // Get the current token from previous rule
        let tok = commitQ.first();
        let ctx_id = tokContextId(tok);
        commitQ.deq();

        if (! tokIsPoisoned(tok))
        begin
            // Locally commit the token.
            link_to_lco.makeReq(initFuncpReqCommitResults(tok));
            debugLog.record(ctx_id, fshow(tok.index) + $format(": Locally committing"));
        end
        else
        begin
            // Token is poisoned.  Invoke fault handler.
            link_handleFault.makeReq(initFuncpReqHandleFault(tok));
            debugLog.record(ctx_id, fshow(tok.index) + $format(": Handling fault"));
        end
    endrule

    rule lco_rsp_gco_req (True);
        let rsp = link_to_lco.getResp();
        link_to_lco.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": LCO Responded"));

        if (tokIsStore(tok))
        begin
            // Request global commit of stores.
            link_to_gco.makeReq(initFuncpReqCommitStores(tok));
            debugLog.record(ctx_id, fshow(tok.index) + $format(": Globally committing"));
        end
        else
        begin
            endModelCycle(tok);
        end
    endrule

    rule gco_rsp (True);
        // Get the global commit response
        let rsp = link_to_gco.getResp();
        link_to_gco.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": GCO Responded"));

        endModelCycle(tok);
    endrule

    (* descending_urgency = "fault_rsp, gco_rsp, lco_rsp_gco_req, sto_rsp, loa_rsp, exe_rsp, tok_req" *)
    rule fault_rsp (True);
        // Fault response (started by lco_req)
        let rsp = link_handleFault.getResp();
        link_handleFault.deq();

        let tok = rsp.token;
        let ctx_id = tokContextId(tok);

        debugLog.record(ctx_id, fshow(tok.index) + $format(": Handle fault responded PC 0x%0x", rsp.nextInstructionAddress));

        // Next PC following fault
        pc[tokContextId(tok)] <= rsp.nextInstructionAddress;

        endModelCycle(tok);
    endrule
  
endmodule
