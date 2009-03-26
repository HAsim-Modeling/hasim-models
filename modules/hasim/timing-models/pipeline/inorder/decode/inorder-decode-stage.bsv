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
`include "asim/provides/chip_base_types.bsh"
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

// This module is pipelined across instance. Stages:

// Stage 1 -> Stage 2* -> Stage 3 
// * Stage 2 stalls once for each instruction until it gets a
//   response from the functional partition. Thus we only pay 
//   this penalty once if an instruction stalls in the InstQ.

// Possible ways the model cycle can end:
//   Path 1: InstQ is empty or the IssueQ is full, so we can't issue.
//   Path 2: We issue succesfully.
//   Path 3: The instruction must be stalled on a dependency.

module [HASIM_MODULE] mkDecode ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_decode.out");


    // ****** Ports *****

    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, FETCH_BUNDLE) bundleFromInstQ <- mkPortStallRecv_Multiplexed("DecQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, BUNDLE)        bundleToIssueQ <- mkPortStallSend_Multiplexed("IssueQ");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, TOKEN)                    allocToSB <- mkPortSend_Multiplexed("Dec_to_SB_alloc");
    
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE) writebackFromExe <- mkPortRecv_Multiplexed("Exe_to_Dec_writeback", 1); //0?
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE) writebackFromMemHit <- mkPortRecv_Multiplexed("DMem_to_Dec_hit_writeback", 1); //0?
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BUS_MESSAGE) writebackFromMemMiss <- mkPortRecv_Multiplexed("DMem_to_Dec_miss_writeback", 1); //0?
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, TOKEN)       writebackFromCom <- mkPortRecv_Multiplexed("Com_to_Dec_writeback", 1); //0?

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)        creditFromSB <- mkPortRecv_Multiplexed("SB_to_Dec_credit", 1); //0?


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

 
    // ****** Local Controller ******

    Vector#(7, PORT_CONTROLS#(NUM_CPUS)) inports  = newVector();
    Vector#(3, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = bundleFromInstQ.ctrl;
    inports[1]  = writebackFromExe.ctrl;
    inports[2]  = writebackFromMemHit.ctrl;
    inports[3]  = writebackFromMemMiss.ctrl;
    inports[4]  = writebackFromCom.ctrl;
    inports[5]  = bundleToIssueQ.ctrl;
    inports[6]  = creditFromSB.ctrl;
    outports[0] = bundleToIssueQ.ctrl;
    outports[1] = bundleFromInstQ.ctrl;
    outports[2] = allocToSB.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventDec <- mkEventRecorder_Multiplexed(`EVENTS_DECODE_INSTRUCTION_DECODE);

    // ****** Model State (per Context) ******

    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prfValidInit = newVector();

    let maxInitReg = numIsaArchRegisters * valueOf(NUM_CONTEXTS);

    for (Integer i = 0; i < maxInitReg; i = i + 1)
        prfValidInit[i] = True;

    for (Integer i = maxInitReg; i < numFuncpPhyRegisters; i = i + 1)
        prfValidInit[i] = False;
    
    MULTIPLEXED#(NUM_CPUS, COUNTER#(TOKEN_INDEX_SIZE))              numInstrsInFlightPool <- mkMultiplexed(mkLCounter(0));
    MULTIPLEXED#(NUM_CPUS, Reg#(Bool))                                  drainingAfterPool <- mkMultiplexed(mkReg(False));
    MULTIPLEXED#(NUM_CPUS, Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES))) memoDependenciesPool <- mkMultiplexed(mkReg(Invalid));
    MULTIPLEXED#(NUM_CPUS, Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)))      prfValidPool <- mkMultiplexed(mkReg(prfValidInit));

    // ****** UnModel Pipeline State ******
    
    Reg#(DEC2_STATE) stage2State <- mkReg(DEC2_ready);
    
    FIFO#(CPU_INSTANCE_ID) stage2Q <- mkFIFO();
    FIFO#(CPU_INSTANCE_ID) stage3Q <- mkFIFO();
    
    // ***** Helper Functions ******
    
    // readyToGo
    
    // Check if a token's sources are ready.

    function ActionValue#(Bool) readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);
    actionvalue

        // Extract local state from the context.
        let cpu_iid = tokCpuInstanceId(tok);
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = prfValidPool[cpu_iid];

        // Check if each source register is ready.
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
            begin
                rdy = rdy && prfValid[pr];

                if (! prfValid[pr])
                begin
                    debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) not ready", pr, ar));
                end
            end
        end

        return rdy;

    endactionvalue
    endfunction

    // readyDrainBefore
    
    // If an instruction is marked drainBefore, then we must wait until
    // the instructions older than it have committed.

    function Bool readyDrainBefore(CPU_INSTANCE_ID cpu_iid, ISA_INSTRUCTION inst);
    
        COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight = numInstrsInFlightPool[cpu_iid];
    
        if (isaDrainBefore(inst))
            return numInstrsInFlight.value() == 0;
        else
            return True;
    
    endfunction
    
    // readyDrainAfter
    
    // If we had previously issued an instruction that was marked drainAfter,
    // then we must wait until instructions older than the next instruction
    // have committed.
    
    function Bool readyDrainAfter(CPU_INSTANCE_ID cpu_iid);
    
        Reg#(Bool)                     drainingAfter = drainingAfterPool[cpu_iid];
        COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight = numInstrsInFlightPool[cpu_iid];
    
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
        let cpu_iid = tokCpuInstanceId(tok);
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = prfValidPool[cpu_iid];
        COUNTER#(TOKEN_INDEX_SIZE)         numInstrsInFlight = numInstrsInFlightPool[cpu_iid];

        // Update the scoreboard.
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
            begin
                prf_valid[pr] = False;
                debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) locked", pr, ar));
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
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        
        
        // Extract our local state from the context.
        Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid = prfValidPool[cpu_iid];
        COUNTER#(TOKEN_INDEX_SIZE)         numInstrsInFlight = numInstrsInFlightPool[cpu_iid];

        // Record how many registers have been written or instructions killed.
        Integer instrs_removed = 0;
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        // Process writes from EXE
        let bus_exe <- writebackFromExe.receive(cpu_iid);
        if (bus_exe matches tagged Valid .msg)
        begin
            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin

                if (msg.destRegs[i] matches tagged Valid .pr)
                begin

                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(cpu_iid, fshow(msg.token) + $format(": PR %0d is ready -- EXE", pr));

                end
            end
        end

        // Process writes from MEM hits
        let bus_memh <- writebackFromMemHit.receive(cpu_iid);
        if (bus_memh matches tagged Valid .msg)
        begin
            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin

                if (msg.destRegs[i] matches tagged Valid .pr)
                begin

                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(cpu_iid, fshow(msg.token) + $format(": PR %0d is ready -- MEM", pr));

                end

            end
        end

        // Process writes from MEM misses.
        let bus_memm <- writebackFromMemMiss.receive(cpu_iid);
        if (bus_memm matches tagged Valid .msg)
        begin
            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin

                if (msg.destRegs[i] matches tagged Valid .pr)
                begin

                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(cpu_iid, fshow(msg.token) + $format(": PR %0d is ready -- MEM", pr));

                end

            end
        end
    
        // Process writes from Com. These can't have been retired.
        let commit <- writebackFromCom.receive(cpu_iid);
        if (commit matches tagged Valid .commit_tok)
        begin

            debugLog.record_next_cycle(cpu_iid, fshow(commit_tok) + $format(": Commit"));
            numInstrsInFlight.down();

        end
        
        // Update the scoreboard.
        prfValid <= prf_valid;

        // Pass to the next stage.
        stage2Q.enq(cpu_iid);
        
    endrule

    // stage2_dependencies
    
    // Check if there's an instruction waiting to be issued. 
    // In order to issue we have to have the dependencies from the functional partition.
    // If we don't have them, stall to retrieve them.
    // If we previously got them, advance to the next stage.
    
    rule stage2_dependencies (stage2State == DEC2_ready);

        // Get the context from the previous stage.
        let cpu_iid = stage2Q.first();
        
        // Extract local state from the context.
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = memoDependenciesPool[cpu_iid];

        if (bundleToIssueQ.canEnq(cpu_iid) && bundleFromInstQ.canDeq(cpu_iid))
        begin
            
            // Issue Q has space and there's an instruction waiting.
                
            // There is... 
            let bundle = bundleFromInstQ.peek(cpu_iid);
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
               stage3Q.enq(cpu_iid);

            end

        end
        else
        begin

            // There's a bubble. Nothing we can do.
            debugLog.record(cpu_iid, fshow("BUBBLE"));
            eventDec.recordEvent(cpu_iid, tagged Invalid);
            stage2Q.deq();
            
            // No allocation to the store buffer.
            let m_credit <- creditFromSB.receive(cpu_iid);
            allocToSB.send(cpu_iid, tagged Invalid);

            // Don't dequeue the InstQ.
            bundleFromInstQ.noDeq(cpu_iid);

            // Don't enqueue anything to the IssueQ. 
            bundleToIssueQ.noEnq(cpu_iid);
            
            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

        end

    endrule

    // stage2_dependenciesRsp
    
    // Get the response from the functional partition and unstall this stage.

    rule stage2_dependenciesRsp (stage2State == DEC2_depsRsp);
    
        // Get the stalled context.
        let cpu_iid = stage2Q.first();

        // Get the response from the functional partition.
        let rsp = getDependencies.getResp();
        getDependencies.deq();
        
        // Extract local state from the context.
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = memoDependenciesPool[cpu_iid];
    
        // Update dependencies.
        memoDependencies <= tagged Valid rsp;

        // Unstall this stage.
        stage2State <= DEC2_ready;
        stage2Q.deq();

        // Pass it to the next stage.
        stage3Q.enq(cpu_iid);
        
    endrule

    // stage3_attemptIssue
    
    // Check to see if we can actually issue the youngest instruction.
    // If we get to this stage, we have previously checked that 
    // there's an instruction in the InstQ and that there's room in the IssueQ,
    // and we must have already gotten the dependencies from the functional partition.

    rule stage3_attemptIssue (True);
    
        // Extract our local state from the context.
        let cpu_iid = stage3Q.first();
        stage3Q.deq();
        Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies = memoDependenciesPool[cpu_iid];
        Reg#(Bool)                                  drainingAfter = drainingAfterPool[cpu_iid];
        
        // assert memoDependencies == Valid
        let rsp = validValue(memoDependencies);
        // assert bundleFromInstQ.canDeq(cpu_iid)
        let fetchbundle = bundleFromInstQ.peek(cpu_iid);
        let tok = fetchbundle.token;
        
        // Get the store buffer credit in case it's a store...
        let m_credit <- creditFromSB.receive(cpu_iid);
        let sb_has_credit = isValid(m_credit);
        
        // If it's a store, we need to be able to allocate a slot in the store buffer.
        let inst_is_store = isaIsStore(fetchbundle.inst);
        let store_ready =  inst_is_store ? sb_has_credit : True;

        // Check if we can issue.
        let data_ready <- readyToGo(tok, rsp.srcMap);

        if (data_ready && store_ready && readyDrainAfter(cpu_iid) && readyDrainBefore(cpu_iid, fetchbundle.inst))
        begin

            // Yep... we're ready to send it. Make sure to send the newer token from the FP.
            let bundle = makeBundle(rsp.token, fetchbundle, rsp.dstMap);
            debugLog.record(cpu_iid, fshow(tok) + fshow(": SEND INST:") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle));
            eventDec.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));
            
            // If it's a store, reserve a slot in the store buffer.
            if (inst_is_store)
            begin
                allocToSB.send(cpu_iid, tagged Valid tok);
            end
            else
            begin
                allocToSB.send(cpu_iid, tagged Invalid);
            end
            
            // Update the scoreboard to reflect this instruction issuing.
            // Mark its destination registers as unready until it commits.
            markPRFInvalid(tok, rsp.dstMap);
            memoDependencies <= Invalid;
            drainingAfter <= isaDrainAfter(fetchbundle.inst);

            // Dequeue the InstQ.
            bundleFromInstQ.doDeq(cpu_iid);

            // Enqueue the decoded instruction in the IssueQ.
            bundleToIssueQ.doEnq(cpu_iid, bundle);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        else
        begin

            // Nope, we're waiting on an older instruction to write its results.
            debugLog.record(cpu_iid, fshow(tok) + fshow(": STALL ON DEPENDENCY"));
            eventDec.recordEvent(cpu_iid, tagged Invalid);
            
            // Propogate the bubble.
            bundleFromInstQ.noDeq(cpu_iid);
            bundleToIssueQ.noEnq(cpu_iid);
            allocToSB.send(cpu_iid, tagged Invalid);
            
            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(cpu_iid, 3);

        end

    endrule

endmodule
