//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_interface.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_DECODE.bsh"

// ****** Local Datatypes ******

typedef union tagged
{
    void         STAGE3_bubble;
    void         STAGE3_rewindRsp;
    void         STAGE3_depsReady;
    FETCH_BUNDLE STAGE3_depsRsp;
}
DEC_STAGE3_STATE deriving (Bits, Eq);

typedef union tagged
{
    void  STAGE4_bubble;
    void  STAGE4_depsCheck;
}
DEC_STAGE4_STATE deriving (Bits, Eq);
 


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

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_decode.out");


    // ****** Ports *****
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BUNDLE)        bundleToIssueQ <- mkPortStallSend_Multiplexed("IssueQ");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                    deqToInstQ <- mkPortSend_Multiplexed("Dec_to_InstQ_deq");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)                    allocToSB <- mkPortSend_Multiplexed("Dec_to_SB_alloc");

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, FETCH_BUNDLE)      bundleFromInstQ      <- mkPortRecv_Multiplexed("InstQ_to_Dec_first", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromExe     <- mkPortRecv_Multiplexed("Exe_to_Dec_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromMemHit  <- mkPortRecv_Multiplexed("DMem_to_Dec_hit_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromMemMiss <- mkPortRecv_Multiplexed("DMem_to_Dec_miss_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)             writebackFromCom     <- mkPortRecv_Multiplexed("Com_to_Dec_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MULTITHREADED#(Bool))              creditFromSB         <- mkPortRecv_Multiplexed("SB_to_Dec_credit", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN_FAULT_EPOCH, THREAD_ID)) mispredictFromExe    <- mkPortRecv_Multiplexed("Exe_to_Dec_mispredict", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, THREAD_ID)              faultFromCom         <- mkPortRecv_Multiplexed("Com_to_Dec_fault", 1);


    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");
 
    // ****** Local Controller ******

    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbExeCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbHitCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbMissCtrl <- mkDependenceController();

    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inports  = newVector();
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  depports = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = bundleToIssueQ.ctrl.in;
    inports[1]  = creditFromSB.ctrl;
    inports[2]  = mispredictFromExe.ctrl;
    inports[3]  = faultFromCom.ctrl;
    inports[4]  = bundleFromInstQ.ctrl;
    inports[5]  = writebackFromCom.ctrl;
    depports[0] = writebackFromExe.ctrl;
    depports[1] = writebackFromMemHit.ctrl;
    depports[2] = writebackFromMemMiss.ctrl;
    depports[3] = wbExeCtrl.ctrl;
    depports[4] = wbHitCtrl.ctrl;
    depports[5] = wbMissCtrl.ctrl;
    outports[0] = bundleToIssueQ.ctrl.out;
    outports[1] = deqToInstQ.ctrl;
    outports[2] = allocToSB.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalControllerWithUncontrolled(inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DEC_STAGE3_STATE) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DEC_STAGE4_STATE) stage4Ctrl <- mkStageController();

    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventDec <- mkEventRecorder_Multiplexed(`EVENTS_DECODE_INSTRUCTION_DECODE);

    // ****** Model State (per Instance) ******

    MULTIPLEXED#(MAX_NUM_CPUS, COUNTER#(TOKEN_INDEX_SIZE))              numInstrsInFlightPool <- mkMultiplexed(mkLCounter(0));
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Bool)                                  drainingAfterPool <- mkMultiplexedReg(False);
    MULTIPLEXED_REG_MULTI_WRITE#(MAX_NUM_CPUS, 2, Maybe#(Tuple2#(FETCH_BUNDLE, FUNCP_RSP_GET_DEPENDENCIES))) instToIssuePool <- mkMultiplexedRegMultiWrite(Invalid);

    // PRF valid bits.

    DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardLUTRAM();
    // DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardMultiWrite(); // Can be switched for expensive version.


    MULTIPLEXED_REG#(MAX_NUM_CPUS, MULTITHREADED#(TOKEN_EPOCH)) epochsPool <- mkMultiplexedReg(initEpoch(0, 0));


    // ***** Helper Functions ******

    // readyToGo
    
    // Check if a token's sources are ready.

    function ActionValue#(Bool) readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);
    actionvalue

        // Extract local state from the context.
        let cpu_iid = tokCpuInstanceId(tok);

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
                        branchEpoch: fbndl.branchEpoch,
                        faultEpoch: fbndl.faultEpoch,
                        isLoad:  isaIsLoad(fbndl.inst),
                        isStore: isaIsStore(fbndl.inst),
                        isTerminate: Invalid,
                        pc: fbndl.pc,
                        branchAttr: fbndl.branchAttr,
                        effAddr: ?,
                        dests: dests };
    endfunction

    // ****** Rules ******
    
    // stage1_writebacks
    
    // Begin simulating a new context.
    // Get any writebacks from Exe, Mem, or Com and update the scoreboard.

    // Ports read:
    // * writebackFromExe
    // * writebackFromMemHit
    // * writebackFromMemMiss
    // * writebackFromCom
    
    // Ports written:
    // * None

    (* conservative_implicit_conditions *)
    rule stage1_writebackExe (writebackFromExe.ctrl.nextReadyInstance() matches tagged Valid .iid 
                                &&& wbExeCtrl.producerCanStart());
    
        // Process writes from EXE
        let bus_exe <- writebackFromExe.receive(iid);
        wbExeCtrl.producerStart();
        wbExeCtrl.producerDone();

        if (bus_exe matches tagged Valid .msg)
        begin
        
            for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
            begin
            
                if (msg.destRegs[x] matches tagged Valid .pr)
                begin
                    debugLog.record_next_cycle(iid, fshow(msg.token) + $format(": PR %0d is ready -- EXE", pr));
                    prfScoreboard.wbExe[x].ready(pr);
                end

            end
        
        end
        
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage1_writebackMemHit (writebackFromMemHit.ctrl.nextReadyInstance() matches tagged Valid .iid
                                &&& wbHitCtrl.producerCanStart());
    
        // Process writes from MEM Hit
        let bus_hit <- writebackFromMemHit.receive(iid);
        wbHitCtrl.producerStart();
        wbHitCtrl.producerDone();

        if (bus_hit matches tagged Valid .msg)
        begin
        
            for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
            begin
            
                if (msg.destRegs[x] matches tagged Valid .pr)
                begin
                    debugLog.record_next_cycle(iid, fshow(msg.token) + $format(": PR %0d is ready -- HIT", pr));
                    prfScoreboard.wbHit[x].ready(pr);
                end

            end
        
        end
        
    endrule

    (* conservative_implicit_conditions *)
    rule stage1_writebackMemMiss (writebackFromMemMiss.ctrl.nextReadyInstance() matches tagged Valid .iid
                                &&& wbMissCtrl.producerCanStart());
    
        // Process writes from MEM Miss
        let bus_miss <- writebackFromMemMiss.receive(iid);
        wbMissCtrl.producerStart();
        wbMissCtrl.producerDone();

        if (bus_miss matches tagged Valid .msg)
        begin
        
            for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
            begin
            
                if (msg.destRegs[x] matches tagged Valid .pr)
                begin
                    debugLog.record_next_cycle(iid, fshow(msg.token) + $format(": PR %0d is ready -- MISS", pr));
                    prfScoreboard.wbMiss[x].ready(pr);
                end

            end
        
        end
        
    endrule

    // stage2_dependencies
    
    // Check if there's an instruction waiting to be issued. 
    // In order to issue we have to have the dependencies from the functional partition.
    // If we don't have them, request them.
    // If we previously got them, just advance to the next stage.
    
    // Ports read:
    // * mispredictFromExe
    // * faultFromCom
    // * bundleFromInstQ
    
    // Ports written:
    // * None
    (* conservative_implicit_conditions *)
    rule stage2_dependencies (True);

        // Begin model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
        
        // Extract local state from the cpu instance.
        Reg#(Maybe#(Tuple2#(FETCH_BUNDLE, FUNCP_RSP_GET_DEPENDENCIES))) instToIssue = instToIssuePool.getRegWithWritePort(cpu_iid, 0);
        Reg#(MULTITHREADED#(TOKEN_EPOCH)) epochsReg = epochsPool.getReg(cpu_iid);
        COUNTER#(TOKEN_INDEX_SIZE)         numInstrsInFlight = numInstrsInFlightPool[cpu_iid];
        
        let m_mispred <- mispredictFromExe.receive(cpu_iid);
        let m_fault   <- faultFromCom.receive(cpu_iid);
        let m_bundle  <- bundleFromInstQ.receive(cpu_iid);
    
        // Process retired instructions from Com.
        let commit <- writebackFromCom.receive(cpu_iid);
        if (commit matches tagged Valid .commit_tok)
        begin

            debugLog.record_next_cycle(cpu_iid, fshow(commit_tok) + $format(": Commit"));
            numInstrsInFlight.down();

        end

        if (m_fault matches tagged Valid .tok)
        begin
            
            // A fault occurred.
            let thread = tokThreadId(tok);
            debugLog.record_next_cycle(cpu_iid, $format("2: FAULT: THREAD %0d: ", thread) + fshow(tok));
            
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));

            // Increment the epoch. Don't do anything with the queue. We'll start dropping instructions on the next cycle.
            epochs[thread].fault = epochs[thread].fault + 1;
            
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            
            // Tell the following stages it's a rewind.
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_rewindRsp);
        
        end
        else if (m_mispred matches tagged Valid {.tok, .fault_epoch} &&& fault_epoch == epochs[tokThreadId(tok)].fault)
        begin

            // A mispredict occurred.
            let thread = tokThreadId(tok);
            debugLog.record_next_cycle(cpu_iid, fshow("2: MISPREDICT"));
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));

            // Increment the epoch. Don't do anything with the queue. We'll start dropping instructions on the next cycle.
            epochs[thread].branch <= epochs[thread].branch + 1;
        
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            
            // Tell the following stages it's a bubble.
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_rewindRsp);

        end
        else if (isValid(instToIssue))
        begin
        
            // We have an instruction to issue, tell the next stage to just proceed.
            debugLog.record_next_cycle(cpu_iid, fshow("2: Deps ready."));

            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);

            stage3Ctrl.ready(cpu_iid, tagged STAGE3_depsReady);
            
        end
        else if (m_bundle matches tagged Valid .bundle)
        begin
            
            // There's a new instruction, and we're not stalled.
            // Dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Valid (?));
            
            //Is it of the correct epoch?
            if (bundle.branchEpoch == epoch.branch && bundle.faultEpoch == epochs[bundle.thread].fault)
            begin

                // We need to retrieve dependencies from the functional partition.
                debugLog.record_next_cycle(cpu_iid, fshow("2: Request Deps."));
                getDependencies.makeReq(initFuncpReqGetDependencies(getContextId(cpu_iid), bundle.inst, bundle.pc));

                // Tell the next stage to get the response.
                stage3Ctrl.ready(cpu_iid, tagged STAGE3_depsRsp bundle);

            end
            else
            begin
            
                // The instruction is from an old epoch, and we haven't gotten
                // a token yet. Tell the following stages to drop it.
                debugLog.record_next_cycle(cpu_iid, fshow("2: SILENT DROP"));

                stage3Ctrl.ready(cpu_iid, tagged STAGE3_bubble);
            
            end
        
        end
        else
        begin
        
            // There's a bubble. Just propogate it.
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            debugLog.record_next_cycle(cpu_iid, fshow("2: BUBBLE"));
            stage3Ctrl.ready(cpu_iid, tagged STAGE3_bubble);

        end
 
    endrule

    // stage3_dependenciesRsp
    
    // Get the response from the functional partition (if any).
    
    // Ports read:
    // * None
    
    // Ports written:
    // * None

    rule stage3_dependenciesRsp (True);
    
        // Get the next instance id.
        match {.cpu_iid, .state} <- stage3Ctrl.nextReadyInstance();

        if (state matches tagged STAGE3_bubble)
        begin
        
            // Just propogate the bubble to the next stage.
            stage4Ctrl.ready(cpu_iid, tagged STAGE4_bubble);
        
        end
        else if (state matches tagged STAGE3_rewindRsp)
        begin
        
            // Get the rewind response.
            let rsp = rewindToToken.getResp();
            rewindToToken.deq();
            debugLog.record(cpu_iid, $format("3: REWIND RSP"));
        
            // Propogate the bubble to the next stage.
            stage4Ctrl.ready(cpu_iid, tagged STAGE4_bubble);
        
        end
        else if (state matches tagged STAGE3_depsReady)
        begin

            // instToIssue is already up to date.

            // Pass it to the next stage to do the checking.
            stage4Ctrl.ready(cpu_iid, tagged STAGE4_depsCheck);
        
        end
        else if (state matches tagged STAGE3_depsRsp .bundle)
        begin
        
            // Get the response from the functional partition.
            let rsp = getDependencies.getResp();
            getDependencies.deq();

            // Extract local state from the context.
            Reg#(Maybe#(Tuple2#(FETCH_BUNDLE, FUNCP_RSP_GET_DEPENDENCIES))) instToIssue = instToIssuePool.getRegWithWritePort(cpu_iid, 0);

            // Update dependencies.
            instToIssue <= tagged Valid tuple2(bundle, rsp);

            // Pass it to the next stage to do the checking.
            stage4Ctrl.ready(cpu_iid, tagged STAGE4_depsCheck);
        
        end

    endrule

    // stage4_attemptIssue
    
    // Check to see if we can actually issue the youngest instruction (if any).

    // Ports read:
    // * creditFromSB
    
    // Ports written:
    // * bundleToIssueQ
    // * deqToInstQ
    // * allocToSB

    let writebacksFinished = wbExeCtrl.consumerCanStart() && wbHitCtrl.consumerCanStart() && wbMissCtrl.consumerCanStart();

    // Specify a rule urgency so that if the Scoreboard has less parallelism there are no
    // compiler warnings.

    (* conservative_implicit_conditions, descending_urgency = "stage1_writebackMemMiss, stage1_writebackMemHit, stage1_writebackExe, stage4_attemptIssue" *)
    rule stage4_attemptIssue (writebacksFinished);
    
        // Extract the next active instance.
        match {.cpu_iid, .state} <- stage4Ctrl.nextReadyInstance();

        // Get the state for this instance.
        Reg#(Maybe#(Tuple2#(FETCH_BUNDLE, FUNCP_RSP_GET_DEPENDENCIES))) instToIssue = instToIssuePool.getRegWithWritePort(cpu_iid, 1);
        Reg#(Bool) drainingAfter = drainingAfterPool.getReg(cpu_iid);
        COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight = numInstrsInFlightPool[cpu_iid];
        Reg#(MULTITHREADED#(TOKEN_EPOCH)) epochs = epochsPool.getReg(cpu_iid);
        
        wbExeCtrl.consumerStart();
        wbExeCtrl.consumerDone();
        wbHitCtrl.consumerStart();
        wbHitCtrl.consumerDone();
        wbMissCtrl.consumerStart();
        wbMissCtrl.consumerDone();

        // Get the store buffer credit in case we're dealing with a store...
        let m_credit <- creditFromSB.receive(cpu_iid);
        let credits = validValue(m_credit);
        
        // See if the issueQ has any room.
        let can_enq <- bundleToIssueQ.canEnq(cpu_iid);

        if (state matches tagged STAGE4_bubble)
        begin

            // No allocation to the store buffer.
            allocToSB.send(cpu_iid, tagged Invalid);

            // Don't enqueue anything to the IssueQ.
            eventDec.recordEvent(cpu_iid, tagged Invalid);
            bundleToIssueQ.noEnq(cpu_iid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);
                
        end
        else if (state matches tagged STAGE4_depsCheck)
        begin

            // assert instToIssue == Valid
            match {.fetchbundle, .rsp} = validValue(instToIssue);
            let tok = rsp.token;
            
            if (can_enq)
            begin
            
                // Check if we can issue.
                let data_ready <- readyToGo(tok, rsp.srcMap);

                // If it's a store, we need to be able to allocate a slot in the store buffer.
                let inst_is_store = isaIsStore(fetchbundle.inst);
                let store_ready =  inst_is_store ? credits[tokThreadId(tok)] : True;

                if (fetchbundle.branchEpoch != epochs[thread].branch || fetchbundle.faultEpoch != epoch[thread].fault)
                begin

                    // It's from an old epoch, so we must issue it as a "dummy" that 
                    // will not be executed, but simply reclaimed.
                    tok.dummy = True;
                    let bundle = makeBundle(tok, fetchbundle, replicate(Invalid));
                    bundle.isLoad = False;
                    bundle.isStore = False;
                    debugLog.record(cpu_iid, fshow(tok) + fshow(": SEND DUMMY."));
                    eventDec.recordEvent(cpu_iid, tagged Invalid);
                    allocToSB.send(cpu_iid, tagged Invalid);
                    instToIssue <= tagged Invalid;

                    // For general sanity dummy instructions count as being in flight
                    // for the purposes of drainBefore.
                    numInstrsInFlight.up();

                    // Enqueue the decoded instruction in the IssueQ.
                    bundleToIssueQ.doEnq(cpu_iid, bundle);

                    // End of model cycle. (Path 3)
                    localCtrl.endModelCycle(cpu_iid, 3);

                end
                else if (data_ready && store_ready && readyDrainAfter(cpu_iid) && readyDrainBefore(cpu_iid, fetchbundle.inst))
                begin

                    // Yep... we're ready to send it. 
                    let bundle = makeBundle(tok, fetchbundle, rsp.dstMap);
                    debugLog.record(cpu_iid, fshow(tok) + fshow(": SEND INST: ") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle));
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

                    for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
                    begin

                        if (rsp.dstMap[x] matches tagged Valid { .ar, .pr })
                        begin
                            prfScoreboard.issue[x].unready(pr);
                            debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) locked", pr, ar));
                        end

                    end

                    // Update the number of instructions in flight.
                    numInstrsInFlight.up();

                    instToIssue <= Invalid;
                    drainingAfter <= isaDrainAfter(fetchbundle.inst);

                    // Enqueue the decoded instruction in the IssueQ.
                    bundleToIssueQ.doEnq(cpu_iid, bundle);

                    // End of model cycle. (Path 4)
                    localCtrl.endModelCycle(cpu_iid, 4);

                end
                else
                begin

                    // Nope, we're waiting on an older instruction to write its results.
                    debugLog.record(cpu_iid, fshow(tok) + fshow(": STALL ON DEPENDENCY"));
                    eventDec.recordEvent(cpu_iid, tagged Invalid);

                    // Propogate the bubble.
                    bundleToIssueQ.noEnq(cpu_iid);
                    allocToSB.send(cpu_iid, tagged Invalid);

                    // End of model cycle. (Path 5)
                    localCtrl.endModelCycle(cpu_iid, 5);

                end
            end
            else
            begin
            
                // Nope, the issueQ is full.
                debugLog.record(cpu_iid, fshow(tok) + fshow(": STALL ON ISSUEQ"));
                eventDec.recordEvent(cpu_iid, tagged Invalid);

                // Propogate the bubble.
                bundleToIssueQ.noEnq(cpu_iid);
                allocToSB.send(cpu_iid, tagged Invalid);

                // End of model cycle. (Path 6)
                localCtrl.endModelCycle(cpu_iid, 6);

            end

        end

    endrule

endmodule
