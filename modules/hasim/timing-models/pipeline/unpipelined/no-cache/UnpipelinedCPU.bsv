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

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_cpu.out");

    //********* State Elements *********//

    // Time to fetch new token
    Reg#(Bool) nextToken <- mkReg(True);

    // The Program Counter
    Reg#(ISA_ADDRESS) pc <- mkReg(`PROGRAM_START_ADDR);

    // Pipe from any stage that flows to local commit
    FIFO#(TOKEN) commitQ <- mkFIFO();

    //********* Connections *********//

    Connection_Send#(Bool)                            link_model_cycle <- mkConnection_Send("model_cycle");

    Connection_Send#(MODEL_NUM_COMMITS)               link_model_commit <- mkConnection_Send("model_commits");

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

    //
    // endModelCycle --
    //     Invoked by any rule that is completely done with a token.
    //
    function Action endModelCycle(TOKEN tok);
    action
        debugLog.record($format("Model cycle complete.", tok.index));

        // Sample event & statistic (commit)
        event_com.recordEvent(tagged Valid zeroExtend(tok.index));
        stat_com.incr();

        // Commit counter for heartbeat
        link_model_commit.send(1);
        
        // Request a new token
        nextToken <= True;
    endaction
    endfunction


    //
    // Whether an instruction is a load or store is stored in token scratchpad
    // memory.  These are accessor functions...
    //
    function Bool tokIsLoad(TOKEN tok) = unpack(tok.timep_info.scratchpad[0]);
    function Bool tokIsStore(TOKEN tok) = unpack(tok.timep_info.scratchpad[1]);


    rule tok_req (nextToken);
        local_ctrl.startModelCC();

        // Request a Token
        debugLog.record($format("Requesting a new token"));
        link_to_tok.makeReq(initFuncpReqNewInFlight());

        // Debug and heartbeat cycle counters
        debugLog.nextModelCycle();
        link_model_cycle.send(?);

        nextToken <= False;
    endrule

    rule tok_rsp_itr_req (True);
        // Get the response from tok_req
        let rsp = link_to_tok.getResp();
        link_to_tok.deq();

        let tok = rsp.newToken;
        debugLog.record($format("TOKEN %d: TOK Responded", tok.index));

        // Translate next pc.
        link_to_itr.makeReq(initFuncpReqDoITranslate(tok, pc));
        debugLog.record($format("TOKEN %d: Translating at address 0x%h", tok.index, pc));
    endrule

    rule itr_rsp_fet_req (True);
        // Get the ITrans response started by tok_rsp_itr_req
        let rsp = link_to_itr.getResp();
        link_to_itr.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: ITR Responded, hasMore: %d", tok.index, rsp.hasMore));

        if (! rsp.hasMore)
        begin
            // Fetch the next instruction
            link_to_fet.makeReq(initFuncpReqGetInstruction(tok));
            debugLog.record($format("TOKEN %d: Fetching at address 0x%h", tok.index, pc));
        end
    endrule

    rule fet_rsp_dec_req (True);
        // Get the instruction response
        let rsp = link_to_fet.getResp();
        link_to_fet.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: FET Responded", tok.index));

        // Record load and store properties for the instruction in the token
        tok.timep_info.scratchpad[0] = pack(isaIsLoad(rsp.instruction));
        tok.timep_info.scratchpad[1] = pack(isaIsStore(rsp.instruction));

        // Decode the current inst
        link_to_dec.makeReq(initFuncpReqGetDependencies(tok));
        debugLog.record($format("TOKEN %d: Decoding", tok.index));
    endrule

    rule dec_rsp_exe_req (True);
        // Get the decode response
        let rsp = link_to_dec.getResp();
        link_to_dec.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: DEC Responded", tok.index));

        // In a more complex processor we would use the dependencies 
        // to determine if we can issue the instruction.

        // Execute the instruction
        link_to_exe.makeReq(initFuncpReqGetResults(tok));
        debugLog.record($format("TOKEN %d: Executing", tok.index));
    endrule

    rule exe_rsp (True);
        // Get the execution result
        let exe_resp = link_to_exe.getResp();
        link_to_exe.deq();

        let tok = exe_resp.token;
        let res = exe_resp.result;

        debugLog.record($format("TOKEN %d: EXE Responded", tok.index));

        // If it was a branch we must update the PC.
        case (res) matches
            tagged RBranchTaken .addr:
            begin
                debugLog.record($format("Branch taken to address %h", addr));
                pc <= addr;
            end

            tagged RBranchNotTaken .addr:
            begin
                debugLog.record($format("Branch not taken"));
                pc <= pc + 4;
            end

            tagged RTerminate .pf:
            begin
                debugLog.record($format("Terminating Execution"));
                local_ctrl.endProgram(pf);
            end

            default:
            begin
                pc <= pc + 4;
            end
        endcase

        if (tokIsLoad(tok) || tokIsStore(tok))
        begin
            // Memory ops require more work.
            debugLog.record($format("TOKEN %d: DTranslate", tok.index));

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
        debugLog.record($format("TOKEN %d: DTR Responded, hasMore: %d", tok.index, rsp.hasMore));

        if (! rsp.hasMore)
        begin
            if (tokIsLoad(tok))
            begin
                // Request the load(s).
                link_to_loa.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record($format("TOKEN %d: Do loads", tok.index));
            end
            else
            begin
                // Request the store(s)
                link_to_sto.makeReq(initFuncpReqDoStores(tok));
                debugLog.record($format("TOKEN %d: Do stores", tok.index));
            end
        end
    endrule

    rule loa_rsp (True);
        // Get the load response
        let rsp = link_to_loa.getResp();
        link_to_loa.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: Load ops responded", tok.index));

        commitQ.enq(tok);
    endrule

    rule sto_rsp (True);
        // Get the store response
        let rsp = link_to_sto.getResp();
        link_to_sto.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: Store ops responded", tok.index));

        commitQ.enq(tok);
    endrule

    rule lco_req (True);
        // Get the current token from previous rule
        let tok = commitQ.first();
        commitQ.deq();

        if (! tokIsPoisoned(tok))
        begin
            // Locally commit the token.
            link_to_lco.makeReq(initFuncpReqCommitResults(tok));
            debugLog.record($format("TOKEN %d: Locally committing", tok.index));
        end
        else
        begin
            // Token is poisoned.  Invoke fault handler.
            link_handleFault.makeReq(initFuncpReqHandleFault(tok));
            debugLog.record($format("TOKEN %d: Handling fault", tok.index));
        end
    endrule

    rule lco_rsp_gco_req (True);
        let rsp = link_to_lco.getResp();
        link_to_lco.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: LCO Responded", tok.index));

        if (tokIsStore(tok))
        begin
            // Request global commit of stores.
            link_to_gco.makeReq(initFuncpReqCommitStores(tok));
            debugLog.record($format("TOKEN %d: Globally committing", tok.index));
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
        debugLog.record($format("TOKEN %d: GCO Responded", tok.index));

        endModelCycle(tok);
    endrule

    rule fault_rsp (True);
        // Fault response (started by lco_req)
        let rsp = link_handleFault.getResp();
        link_handleFault.deq();

        let tok = rsp.token;
        debugLog.record($format("TOKEN %d: Handle fault responded PC 0x%0x", tok.index, rsp.nextInstructionAddress));

        // Next PC following fault
        pc <= rsp.nextInstructionAddress;

        endModelCycle(tok);
    endrule
  
endmodule
