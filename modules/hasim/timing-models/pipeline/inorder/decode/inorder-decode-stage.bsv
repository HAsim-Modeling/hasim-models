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


// ****** Bluespec imports  *****

import FShow::*;
import Vector::*;
import FIFO::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_interface.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_DECODE.bsh"


// ****** Local types ******

// DEC2_STATE

// The second stage stalls when it asks for the dependencies.

typedef enum
{
    DEC2_ready,
    DEC2_depsRsp
}
    DEC2_STATE
        deriving (Bits, Eq);


// mkDecode

// Multi-context inorder decode module which stalls the first instruction until its
// source registers have been written. 

// Writes to registers can be reported by the Exe, Mem, or Com stages.

// Certain instructions must stall the pipeline either before or after their execution.

// This module is pipelined across contexts. Stages:

// Stage 1 -> Stage 2* -> Stage 3 
// * Stage 2 stalls once for each instruction until it gets a
//   response from the functional partition. Thus we only pay 
//   this penalty once if an instruction stalls in the InstQ.

// Possible ways the model cycle can end:
//   Path 1: InstQ is empty or the IssueQ is full, so we can't issue.
//   Path 2: We issue succesfully.
//   Path 3: The instruction must be stalled on a dependency.

module [HASIM_MODULE] mkDecode ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_decode.out");


    // ****** Ports *****

    PORT_STALL_RECV_MULTICTX#(FETCH_BUNDLE) bundleFromInstQ <- mkPortStallRecv_MultiCtx("InstQ");
    PORT_STALL_SEND_MULTICTX#(BUNDLE)        bundleToIssueQ <- mkPortStallSend_MultiCtx("IssueQ");
    
    PORT_RECV_MULTICTX#(BUS_MESSAGE) writebackFromExe <- mkPortRecv_MultiCtx("Exe_to_Dec_writeback", 1); //0?
    PORT_RECV_MULTICTX#(BUS_MESSAGE) writebackFromMem <- mkPortRecv_MultiCtx("Mem_to_Dec_writeback", 1); //0?
    PORT_RECV_MULTICTX#(TOKEN)       writebackFromCom <- mkPortRecv_MultiCtx("Com_to_Dec_writeback", 1); //0?


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

 
    // ****** Local Controller ******

    Vector#(5, PORT_CONTROLS) inports  = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0]  = bundleFromInstQ.ctrl;
    inports[1]  = writebackFromExe.ctrl;
    inports[2]  = writebackFromMem.ctrl;
    inports[3]  = writebackFromCom.ctrl;
    inports[4]  = bundleToIssueQ.ctrl;
    outports[0] = bundleToIssueQ.ctrl;
    outports[1] = bundleFromInstQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    // ****** Events ******
    EVENT_RECORDER_MULTICTX eventDec <- mkEventRecorder_MultiCtx(`EVENTS_DECODE_INSTRUCTION_DECODE);

    // ****** Model State (per Context) ******

    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prfValidInit = newVector();

    let maxInitReg = numIsaArchRegisters * valueOf(NUM_CONTEXTS);

    for (Integer i = 0; i < maxInitReg; i = i + 1)
        prfValidInit[i] = True;

    for (Integer i = maxInitReg; i < numFuncpPhyRegisters; i = i + 1)
        prfValidInit[i] = False;
    
    MULTICTX#(COUNTER#(TOKEN_INDEX_SIZE))              ctx_numInstrsInFlight <- mkMultiCtx(mkLCounter(0));
    MULTICTX#(Reg#(Bool))                                  ctx_drainingAfter <- mkMultiCtx(mkReg(False));
    MULTICTX#(Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES))) ctx_memoDependencies <- mkMultiCtx(mkReg(Invalid));
    MULTICTX#(Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)))      ctx_prfValid <- mkMultiCtx(mkReg(prfValidInit));

    // ****** UnModel Pipeline State ******
    
    Reg#(DEC2_STATE) stage2State <- mkReg(DEC2_ready);
    
    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();
    
    // ***** Helper Functions ******
    
    // readyToGo
    
    // Check if a token's sources are ready.

    function ActionValue#(Bool) readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);
    actionvalue

        // Extract local state from the context.
        let ctx = tokContextId(tok);
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = ctx_prfValid[ctx];

        // Check if each source register is ready.
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
            begin
                rdy = rdy && prfValid[pr];

                if (! prfValid[pr])
                begin
                    debugLog.record(ctx, fshow(tok) + $format(": PR %0d (AR %0d) not ready", pr, ar));
                end
            end
        end

        return rdy;

    endactionvalue
    endfunction

    // readyDrainBefore
    
    // If an instruction is marked drainBefore, then we must wait until
    // the instructions older than it have committed.

    function Bool readyDrainBefore(CONTEXT_ID ctx, ISA_INSTRUCTION inst);
    
        COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight = ctx_numInstrsInFlight[ctx];
    
        if (isaDrainBefore(inst))
            return numInstrsInFlight.value() == 0;
        else
            return True;
    
    endfunction
    
    // readyDrainAfter
    
    // If we had previously issued an instruction that was marked drainAfter,
    // then we must wait until instructions older than the next instruction
    // have committed.
    
    function Bool readyDrainAfter(CONTEXT_ID ctx);
    
        Reg#(Bool)                     drainingAfter = ctx_drainingAfter[ctx];
        COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight = ctx_numInstrsInFlight[ctx];
    
        if (drainingAfter)
            return numInstrsInFlight.value() == 0;
        else
            return True;
    
    endfunction

    // markPRFInvalid

    // When we issue an instruction we mark its destinations as unready.

    function Action markPRFInvalid(TOKEN tok, ISA_DST_MAPPING dstmap);
    action
        
        // Extract local state from the context.
        let ctx = tokContextId(tok);
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = ctx_prfValid[ctx];
        COUNTER#(TOKEN_INDEX_SIZE)         numInstrsInFlight = ctx_numInstrsInFlight[ctx];

        // Update the scoreboard.
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
            begin
                prf_valid[pr] = False;
                debugLog.record(ctx, fshow(tok) + $format(": PR %0d (AR %0d) locked", pr, ar));
            end
        end
        prfValid <= prf_valid;
        numInstrsInFlight.up();

    endaction
    endfunction

    // makeBundle
    
    // Marshall up a bundle of useful information to send to the rest of the pipeline.

    function BUNDLE makeBundle(TOKEN tok, FETCH_BUNDLE fbndl, ISA_DST_MAPPING dstmap);
        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end
        return BUNDLE { token:   tok,
                        isLoad:  isaIsLoad(fbndl.inst),
                        isStore: isaIsStore(fbndl.inst),
                        isTerminate: Invalid,
                        pc: fbndl.pc,
                        branchAttr: fbndl.branchAttr,
                        effAddr: ?,
                        dests: dests };
    endfunction

    // ****** Rules ******
    
    // stage1_commits
    
    // Begin simulating a new context.
    // Get any commits from Exe, Mem, or Com and update the scoreboard.
    // This stage never stalls.

    rule stage1_writebacks (True);

        // Begin model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
        
        
        // Extract our local state from the context.
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = ctx_prfValid[ctx];
        COUNTER#(TOKEN_INDEX_SIZE)         numInstrsInFlight = ctx_numInstrsInFlight[ctx];

        // Record how many registers have been written or instructions killed.
        Integer instrs_removed = 0;
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        // Process writes from EXE
        let bus_exe <- writebackFromExe.receive(ctx);
        if (bus_exe matches tagged Valid .msg)
        begin

            if (msg.tokKilled)
            begin

                // It was shot down because it was from the wrong epoch.
                debugLog.record_next_cycle(ctx, fshow(msg.token) + $format(": no longer in flight"));
                instrs_removed = instrs_removed + 1;

            end

            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin

                if (msg.destRegs[i] matches tagged Valid .pr)
                begin

                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(ctx, fshow(msg.token) + $format(": PR %0d is ready -- EXE", pr));

                end
            end
        end

        // Process writes from MEM
        let bus_mem <- writebackFromMem.receive(ctx);
        if (bus_mem matches tagged Valid .msg)
        begin
            if (msg.tokKilled)
            begin
                instrs_removed = instrs_removed + 1;
                debugLog.record_next_cycle(ctx, fshow(msg.token) + $format(": no longer in flight"));
            end

            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin

                if (msg.destRegs[i] matches tagged Valid .pr)
                begin

                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(ctx, fshow(msg.token) + $format(": PR %0d is ready -- MEM", pr));

                end

            end
        end

        // Process writes from Com. These can't have been retired.
        let commit <- writebackFromCom.receive(ctx);
        if (commit matches tagged Valid .commit_tok)
        begin

            debugLog.record_next_cycle(ctx, fshow(commit_tok) + $format(": Commit"));
            instrs_removed = instrs_removed + 1;

        end
        
        // Update the scoreboard.
        numInstrsInFlight.downBy(fromInteger(instrs_removed));
        prfValid <= prf_valid;

        // Pass to the next stage.
        stage2Q.enq(ctx);
        
    endrule

    // stage2_dependencies
    
    // Check if there's an instruction waiting to be issued. 
    // In order to issue we have to have the dependencies from the functional partition.
    // If we don't have them, stall to retrieve them.
    // If we previously got them, advance to the next stage.
    
    rule stage2_dependencies (stage2State == DEC2_ready);

        // Get the context from the previous stage.
        let ctx = stage2Q.first();
        
        // Extract local state from the context.
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = ctx_memoDependencies[ctx];

        if (bundleToIssueQ.canEnq(ctx) && bundleFromInstQ.canDeq(ctx))
        begin
            
            // Issue Q has space and there's an instruction waiting.
                
            // There is... 
            let bundle = bundleFromInstQ.peek(ctx);
            let tok = bundle.token;

            // Let's get the dependencies, if we haven't already.                
            if (!isValid(memoDependencies))
            begin

                // We need to retrieve dependencies from the functional partition.
                getDependencies.makeReq(initFuncpReqGetDependencies(tok));
                // Stall this stage until we get a response.
                stage2State <= DEC2_depsRsp;

            end
            else
            begin

               // We have the dependencies, advance this context to the next stage.
               stage2Q.deq();
               stage3Q.enq(ctx);

            end

        end
        else
        begin

            // There's a bubble. Nothing we can do.
            debugLog.record(ctx, fshow("BUBBLE"));
            eventDec.recordEvent(ctx, tagged Invalid);
            stage2Q.deq();

            // Don't dequeue the InstQ.
            bundleFromInstQ.noDeq(ctx);

            // Don't enqueue anything to the IssueQ. 
            bundleToIssueQ.noEnq(ctx);
            
            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(ctx, 1);

        end

    endrule

    // stage2_dependenciesRsp
    
    // Get the response from the functional partition and unstall this stage.

    rule stage2_dependenciesRsp (stage2State == DEC2_depsRsp);
    
        // Get the stalled context.
        let ctx = stage2Q.first();

        // Get the response from the functional partition.
        let rsp = getDependencies.getResp();
        getDependencies.deq();
        
        // Extract local state from the context.
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = ctx_memoDependencies[ctx];
    
        // Update dependencies.
        memoDependencies <= tagged Valid rsp;

        // Unstall this stage.
        stage2State <= DEC2_ready;
        stage2Q.deq();

        // Pass it to the next stage.
        stage3Q.enq(ctx);
        
    endrule

    // stage3_attemptIssue
    
    // Check to see if we can actually issue the youngest instruction.
    // If we get to this stage, we have previously checked that 
    // there's an instruction in the InstQ and that there's room in the IssueQ,
    // and we must have already gotten the dependencies from the functional partition.

    rule stage3_attemptIssue (True);
    
        // Extract our local state from the context.
        let ctx = stage3Q.first();
        stage3Q.deq();
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = ctx_memoDependencies[ctx];
        Reg#(Bool)                                  drainingAfter = ctx_drainingAfter[ctx];
        
        // assert memoDependencies == Valid
        let rsp = validValue(memoDependencies);
        // assert bundleFromInstQ.canDeq(ctx)
        let fetchbundle = bundleFromInstQ.peek(ctx);
        let tok = fetchbundle.token;
        
        // Check if we can issue.
        let data_ready <- readyToGo(tok, rsp.srcMap);

        if (data_ready && readyDrainAfter(ctx) && readyDrainBefore(ctx, fetchbundle.inst))
        begin

            // Yep... we're ready to send it. Make sure to send the newer token from the FP.
            let bundle = makeBundle(rsp.token, fetchbundle, rsp.dstMap);
            debugLog.record(ctx, fshow(tok) + fshow(": SEND INST:") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle));
            eventDec.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
            
            // Update the scoreboard to reflect this instruction issuing.
            // Mark its destination registers as unready until it commits.
            markPRFInvalid(tok, rsp.dstMap);
            memoDependencies <= Invalid;
            drainingAfter <= isaDrainAfter(fetchbundle.inst);

            // Dequeue the InstQ.
            bundleFromInstQ.doDeq(ctx);

            // Enqueue the decoded instruction in the IssueQ.
            bundleToIssueQ.doEnq(ctx, bundle);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(ctx, 2);

        end
        else
        begin

            // Nope, we're waiting on an older instruction to write its results.
            debugLog.record(ctx, fshow(tok) + fshow(": STALL ON DEPENDENCY"));
            eventDec.recordEvent(ctx, tagged Invalid);
            
            // Propogate the bubble.
            bundleFromInstQ.noDeq(ctx);
            bundleToIssueQ.noEnq(ctx);
            
            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(ctx, 3);

        end

    endrule

endmodule
