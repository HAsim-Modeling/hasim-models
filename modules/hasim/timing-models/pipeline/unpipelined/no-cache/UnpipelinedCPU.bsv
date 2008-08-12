//
// INTEL CONFIDENTIAL
// Copyright (c) 2008 Intel Corp.  Recipient is granted a non-sublicensable 
// copyright license under Intel copyrights to copy and distribute this code 
// internally only. This code is provided "AS IS" with no support and with no 
// warranties of any kind, including warranties of MERCHANTABILITY,
// FITNESS FOR ANY PARTICULAR PURPOSE or INTELLECTUAL PROPERTY INFRINGEMENT. 
// By making any use of this code, Recipient agrees that no other licenses 
// to any Intel patents, trade secrets, copyrights or other intellectual 
// property rights are granted herein, and no other licenses shall arise by 
// estoppel, implication or by operation of law. Recipient accepts all risks 
// of use.
//

import Vector::*;

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

// CPU state

// Current interaction of the CPU and the functional partition.

typedef enum 
{ 
    TOK_REQ, 
    TOK_RSP,
    ITR_REQ, 
    ITR_RSP,
    ITR_RSP2,
    FET_REQ,
    FET_RSP,
    DEC_REQ,
    DEC_RSP,
    EXE_REQ,
    EXE_RSP,
    DTR_REQ,
    DTR_RSP,
    DTR_RSP2,
    LOA_REQ,
    LOA_RSP,
    STO_REQ,
    STO_RSP,
    LCO_REQ,
    LCO_RSP,
    GCO_REQ,
    GCO_RSP
}
    CPU_STATE
        deriving (Eq, Bits);

module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    Reg#(File) debug_log <- mkReg(InvalidFile);

    //********* State Elements *********//

    //The current state
    Reg#(CPU_STATE) state <- mkReg(TOK_REQ);

    //Current TOKEN (response from TOK stage)
    Reg#(TOKEN) cur_tok <- mkRegU();

    //Current instruction (response from FET stage)
    Reg#(ISA_INSTRUCTION)  cur_inst <- mkRegU();

    //The Program Counter
    Reg#(ISA_ADDRESS) pc <- mkReg(`PROGRAM_START_ADDR);

    //The actual Clock Cycle, for debugging messages
    Reg#(Bit#(32)) fpgaCC <- mkReg(0);

    //The simulation Clock Cycle, or "tick"
    Reg#(Bit#(32)) modelCC <- mkReg(0);

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

    //For killing. UNUSED

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, 
                       FUNCP_RSP_REWIND_TO_TOKEN)     link_rewindToToken <- mkConnection_Client("funcp_rewindToToken");


    //Events
    EventRecorder event_com <- mkEventRecorder(`EVENTS_CPU_INSTRUCTION_COMMIT);

    //Stats
    Stat stat_com <- mkStatCounter(`STATS_CPU_INSTRUCTION_COMMIT);

    Vector#(0, Port_Control) inports = newVector();
    Vector#(0, Port_Control) outports = newVector();

    LocalController local_ctrl <- mkLocalController(inports, outports);

    //********* Rules *********//

    //count
    rule count (True);

        if (fpgaCC == 0)
        begin

            local_ctrl.startModelCC();

            let fd <- $fopen("hasim_cpu.out");
            if (fd == InvalidFile)
            begin
              $display("Error opening logfile!");
              $finish(1);
            end
            debug_log <= fd;

        end

        fpgaCC <= fpgaCC + 1;

    endrule

    //process

    rule process (local_ctrl.running());

        case (state)
            TOK_REQ:
            begin

                //Request a Token
                debug(2, $fdisplay(debug_log, "[%d] Requesting a new Token on model cycle %0d.", fpgaCC, modelCC));
                link_to_tok.makeReq(initFuncpReqNewInFlight());
                link_model_cycle.send(?);

                state <= TOK_RSP;

            end
            TOK_RSP:
            begin

                //Get the response
                let rsp = link_to_tok.getResp();
                link_to_tok.deq();

                let tok = rsp.newToken;
                
                // Set the timing partition information.
                tok.timep_info = TOKEN_TIMEP_INFO{epoch: 0, scratchpad: 0};

                debug(2, $fdisplay(debug_log, "[%d] TOK Responded with TOKEN %0d.", fpgaCC, tok.index));

                cur_tok <= tok;

                state <= ITR_REQ;
            end
            ITR_REQ:
            begin

                // Translate next pc.
                link_to_itr.makeReq(initFuncpReqDoITranslate(cur_tok, pc));

                debug(2, $fdisplay(debug_log, "[%d] Translating TOKEN %0d at address 0x%h.", fpgaCC, cur_tok.index, pc));

                state <= ITR_RSP;
            
            end
            ITR_RSP:
            begin

                // Get the ITrans response.
                let rsp = link_to_itr.getResp();
                link_to_itr.deq();

                debug(2, $fdisplay(debug_log, "[%d] ITR Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("ITR ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                if (rsp.hasMore)
                begin

                   // The instruction crossed a physical word. We have to get the second address serially.
                   state <= ITR_RSP2;

                end
                else
                begin
                
                    state <= FET_REQ;

                end

            end
            ITR_RSP2:
            begin

                // Get the second ITrans response.
                let rsp = link_to_itr.getResp();
                link_to_itr.deq();

                debug(2, $fdisplay(debug_log, "[%d] ITR Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("ITR ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                state <= FET_REQ;

            end
            FET_REQ:
            begin

                // Fetch the next instruction
                link_to_fet.makeReq(initFuncpReqGetInstruction(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Fetching TOKEN %0d at address 0x%h.", fpgaCC, cur_tok.index, pc));

                state <= FET_RSP;

            end
            FET_RSP:
            begin

                // Get the instruction response
                let rsp = link_to_fet.getResp();
                link_to_fet.deq();

                debug(2, $fdisplay(debug_log, "[%d] FET Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                // Record the current instruction.
                cur_inst <= rsp.instruction;
                
                if (rsp.token.index != cur_tok.index) $display ("FET ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                state <= DEC_REQ;
            end
            DEC_REQ:
            begin

                // Decode the current inst
                debug(2, $fdisplay(debug_log, "[%d] Decoding TOKEN %0d.", fpgaCC, cur_tok.index));

                link_to_dec.makeReq(initFuncpReqGetDependencies(cur_tok));

                state <= DEC_RSP;
            end
            DEC_RSP:
            begin

                //Get the response
                let rsp = link_to_dec.getResp();
                link_to_dec.deq();
                
                // In a more complex processor we would use the dependencies 
                // to determine if we can issue the instruction.

                debug(2, $fdisplay(debug_log, "[%d] DEC Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("DEC ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                state <= EXE_REQ;
                
            end
            EXE_REQ:
            begin

                // Execute the instruction
                link_to_exe.makeReq(initFuncpReqGetResults(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Executing TOKEN %0d", fpgaCC, cur_tok.index));

                state <= EXE_RSP;

            end
            EXE_RSP:
            begin

                //Get the execution result
                let exe_resp = link_to_exe.getResp();
                link_to_exe.deq();

                let tok = exe_resp.token;
                let res = exe_resp.result;

                debug(2, $fdisplay(debug_log, "[%d] EXE Responded with TOKEN %0d.", fpgaCC, tok.index));

                if (tok.index != cur_tok.index) $display ("EXE ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, tok.index);

                // If it was a branch we must update the PC.

                case (res) matches

                    tagged RBranchTaken .addr:
                    begin

                        debug(2, $fdisplay(debug_log, "Branch taken to address %h", addr));
                        pc <= addr;

                    end
                    tagged RBranchNotTaken .addr:
                    begin

                        debug(2, $fdisplay(debug_log, "Branch not taken"));
                        pc <= pc + 4;

                    end
                    tagged RTerminate .pf:
                    begin

                        debug(2, $fdisplay(debug_log, "Terminating Execution"));
                        local_ctrl.endProgram(pf);

                    end
                    default:
                    begin

                       pc <= pc + 4;

                    end

                endcase

                if (isaIsLoad(cur_inst) || isaIsStore(cur_inst))
                begin
                    // Memory ops require more work.
                    state <= DTR_REQ;
                end
                else
                begin
                    // Everything else should just be committed.
                    state <= LCO_REQ;
                end

            end
            DTR_REQ:
            begin

                // Get the physical address(es) of the memory access.
                link_to_dtr.makeReq(initFuncpReqDoDTranslate(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] DTranslate for TOKEN %0d", fpgaCC, cur_tok.index));


                state <= DTR_RSP;

            end
            DTR_RSP:
            begin

                // Get the response
                let rsp = link_to_dtr.getResp();
                link_to_dtr.deq();

                debug(2, $fdisplay(debug_log, "[%d] DTranslate responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("DTR ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                if (rsp.hasMore)
                begin
                    // The load/store spanned two memory locations.
                    state <= DTR_RSP2;
                end
                else if (isaIsLoad(cur_inst))
                begin
                    state <= LOA_REQ;
                end
                else
                begin
                    state <= STO_REQ;
                end

            end
            DTR_RSP2:
            begin

                // Get the response
                let rsp = link_to_dtr.getResp();
                link_to_dtr.deq();

                debug(2, $fdisplay(debug_log, "[%d] DTranslate responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("DTR ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                if (isaIsLoad(cur_inst))
                begin
                    state <= LOA_REQ;
                end
                else
                begin
                    state <= STO_REQ;
                end

            end
            LOA_REQ:
            begin

                // Request the load(s).
                link_to_loa.makeReq(initFuncpReqDoLoads(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Loads for TOKEN %0d", fpgaCC, cur_tok.index));

                state <= LOA_RSP;

            end
            LOA_RSP:
            begin

                // Get the load response
                let rsp = link_to_loa.getResp();
                link_to_loa.deq();

                debug(2, $fdisplay(debug_log, "[%d] Load ops responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("LOA ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                state <= LCO_REQ;

            end
            STO_REQ:
            begin

                // Request the store(s)
                link_to_sto.makeReq(initFuncpReqDoStores(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Stores for TOKEN %0d", fpgaCC, cur_tok.index));

                state <= STO_RSP;
            end
            STO_RSP:
            begin

                 // Get the store response
                let rsp = link_to_sto.getResp();
                link_to_sto.deq();

                debug(2, $fdisplay(debug_log, "[%d] Store ops responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("STO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                state <= LCO_REQ;
            end
            LCO_REQ:
            begin
            
                // Locally commit the token.
                link_to_lco.makeReq(initFuncpReqCommitResults(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Locally committing TOKEN %0d.", fpgaCC, cur_tok.index));
                
                state <= LCO_RSP;

            end
            LCO_RSP:
            begin

                //Get the commit response

                let rsp = link_to_lco.getResp();
                link_to_lco.deq();

                debug(2, $fdisplay(debug_log, "[%d] LCO Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("LCO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);

                if (isaIsStore(cur_inst))
                begin

                    state <= GCO_REQ;

                end
                else
                begin
                
                    // End the model cycle.
                    modelCC <= modelCC + 1;
                    debug(1, $fdisplay(debug_log, "Committed TOKEN %0d on model cycle %0d.", cur_tok.index, modelCC));
                    event_com.recordEvent(tagged Valid zeroExtend(cur_tok.index));
                    link_model_commit.send(1);
                    stat_com.incr();

                    state <= TOK_REQ;

                end

            end
            GCO_REQ:
            begin

                // Request global commit of stores.
                link_to_gco.makeReq(initFuncpReqCommitStores(cur_tok));

                debug(2, $fdisplay(debug_log, "[%d] Globally committing TOKEN %0d", fpgaCC, cur_tok.index));

                state <= GCO_RSP;
            end
            GCO_RSP:
            begin

                //Get the commit response
                let rsp = link_to_gco.getResp();
                link_to_gco.deq();

                debug(2, $fdisplay(debug_log, "[%d] GCO Responded with TOKEN %0d.", fpgaCC, rsp.token.index));

                if (rsp.token.index != cur_tok.index) $display ("GCO ERROR: TOKEN Mismatch. Expected: %0d Received: %0d", cur_tok.index, rsp.token.index);
 
                // End the model cycle.
                modelCC <= modelCC + 1;
                debug(1, $fdisplay(debug_log, "Committed TOKEN %0d on model cycle %0d.", cur_tok.index, modelCC));
                event_com.recordEvent(tagged Valid zeroExtend(cur_tok.index));
                link_model_commit.send(1);
                stat_com.incr();

                state <= TOK_REQ;

            end

        endcase    

    endrule
  
endmodule

