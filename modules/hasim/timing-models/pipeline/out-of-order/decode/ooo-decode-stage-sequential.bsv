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
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_interface.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_DECODE.bsh"

// ****** Local Datatypes ******

typedef 16 NUM_ROB_ENTRIES;
typedef Bit#(TLog#(NUM_ROB_ENTRIES)) ROB_IDX;
typedef Vector#(NUM_ROB_ENTRIES, ROB_ENTRY) ROB;

typedef union tagged
{
    void STAGE3_bubble;
    void STAGE3_rewindRsp;
    void STAGE3_depsRsp;
}
DEC_STAGE3_STATE deriving (Bits, Eq);


typedef struct
{
    FETCH_BUNDLE bundle;
    FUNCP_RSP_GET_DEPENDENCIES deps;
    Bool valid;
    Bool issued;
    Bool completed;
    Bool mispredicted;
    Bool newEpoch;
    Maybe#(Bool) terminate;
}
ROB_ENTRY deriving (Eq, Bits);

ROB_ENTRY initialROBEntry =
    ROB_ENTRY
    {
        bundle: ?,
        deps: ?,
        valid: False,
        issued: False,
        completed: False,
        newEpoch: False,
        mispredicted: False,
        terminate: Invalid
    };

typedef struct
{
    Bool shifting;
    Bool seenBarrier;
    Bool seenMispredict;
    Bool addedMispredict;
    Bool consolidatingEpochs;
    Maybe#(Tuple2#(TOKEN, Maybe#(Bool))) mComplete;
    Maybe#(TOKEN) mMispredict;
    Maybe#(FETCH_BUNDLE) mEnq;
    FUNCP_RSP_GET_DEPENDENCIES newDeps;
    Maybe#(BUNDLE) mInstToIssue;
    Bool issueQHasRoom;
    Bool needSBSlot;
}
DECODE_ROB_INTERMEDIATE_STATE deriving (Eq, Bits);

DECODE_ROB_INTERMEDIATE_STATE initialDecodeIntermediateState =
    DECODE_ROB_INTERMEDIATE_STATE
    {
        shifting: False,
        seenBarrier: False,
        seenMispredict: False,
        addedMispredict: False,
        consolidatingEpochs: False,
        mComplete: Invalid,
        mMispredict: Invalid,
        mEnq: Invalid,
        newDeps: ?,
        mInstToIssue: Invalid,
        issueQHasRoom: False,
        needSBSlot: False
    };

typedef struct
{
    TOKEN_INDEX numInstrsInFlight;
    Bool drainingAfter;
    Bool drainingEpoch;
    Bool expectingNewEpoch;
    TOKEN_EPOCH epoch;
    Bit#(TAdd#(TLog#(NUM_ROB_ENTRIES), 1)) robOccupancy;
    DECODE_ROB_INTERMEDIATE_STATE tmps;
}
DECODE_STATE deriving (Eq, Bits);

DECODE_STATE initialDecodeState =
    DECODE_STATE
    {
        numInstrsInFlight: 0,
        drainingAfter: False,
        drainingEpoch: False,
        expectingNewEpoch: False,
        epoch: initEpoch(0, 0),
        robOccupancy: 0,
        tmps: initialDecodeIntermediateState
    };

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
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DMEM_BUNDLE)   bundleToRetireQ <- mkPortStallSend_Multiplexed("RetireQ");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                    deqToInstQ <- mkPortSend_Multiplexed("Dec_to_InstQ_deq");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)                    allocToSB <- mkPortSend_Multiplexed("Dec_to_SB_alloc");

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, FETCH_BUNDLE)      bundleFromInstQ      <- mkPortRecv_Multiplexed("InstQ_to_Dec_first", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID)              creditFromSB         <- mkPortRecv_Multiplexed("SB_to_Dec_credit", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN, TOKEN_FAULT_EPOCH)) mispredictFromExe    <- mkPortRecv_Multiplexed("Exe_to_Dec_mispredict", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)             faultFromCom         <- mkPortRecv_Multiplexed("Com_to_Dec_fault", 1);

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DMEM_BUNDLE) bundleFromCommitQ  <- mkPortRecv_Multiplexed("commitQ_first", 0);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID) deqToCommitQ <- mkPortSend_Multiplexed("commitQ_deq");


    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE) writebackFromExe     <- mkPortRecv_Multiplexed("Exe_to_Dec_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE) writebackFromMemHit  <- mkPortRecv_Multiplexed("DMem_to_Dec_hit_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE) writebackFromMemMiss <- mkPortRecv_Multiplexed("DMem_to_Dec_miss_writeback", 1);

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

 
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    // ****** Model State (per Instance) ******

    MULTIPLEXED_STATE_POOL#(MAX_NUM_CPUS, DECODE_STATE) statePool <- mkMultiplexedStatePool(initialDecodeState);
    MULTIPLEXED_LUTRAM#(MAX_NUM_CPUS, ROB_IDX, ROB_ENTRY) robPool <- mkMultiplexedLUTRAM(initialROBEntry);

    // PRF valid bits.
    DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardLUTRAM();
    // DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardMultiWrite(); // Can be switched for expensive version.

    // ****** Local Controller ******

    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbExeCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbHitCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbMissCtrl <- mkDependenceController();

    Vector#(8, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inports  = newVector();
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  depports = newVector();
    Vector#(5, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = bundleToIssueQ.ctrl.in;
    inports[1]  = creditFromSB.ctrl;
    inports[2]  = mispredictFromExe.ctrl;
    inports[3]  = faultFromCom.ctrl;
    inports[4]  = bundleFromInstQ.ctrl;
    inports[5]  = statePool.ctrl;
    inports[6]  = bundleToRetireQ.ctrl.in;
    inports[7]  = bundleFromCommitQ.ctrl;
    depports[0] = writebackFromExe.ctrl;
    depports[1] = writebackFromMemHit.ctrl;
    depports[2] = writebackFromMemMiss.ctrl;
    depports[3] = wbExeCtrl.ctrl;
    depports[4] = wbHitCtrl.ctrl;
    depports[5] = wbMissCtrl.ctrl;
    outports[0] = bundleToIssueQ.ctrl.out;
    outports[1] = deqToInstQ.ctrl;
    outports[2] = allocToSB.ctrl;
    outports[3] = bundleToRetireQ.ctrl.out;
    outports[4] = deqToCommitQ.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalControllerWithUncontrolled(inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(DEC_STAGE3_STATE, DECODE_STATE)) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage4Ctrl <- mkStageController();
    
    Reg#(Maybe#(Tuple3#(CPU_INSTANCE_ID, DECODE_STATE, ROB_IDX))) stage3Stall <- mkReg(tagged Invalid);

    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventDec <- mkEventRecorder_Multiplexed(`EVENTS_DECODE_INSTRUCTION_DECODE);


    // ***** Helper Functions ******

    // readyToGo
    
    // Check if a token's sources are ready.

    function Bool readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);

        // Extract local state from the context.
        let cpu_iid = tokCpuInstanceId(tok);

        // Check if each source register is ready.
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
            begin
                let is_rdy = prfScoreboard.isReady(pr);
                rdy = rdy && is_rdy;

//                if (!is_rdy)
//                begin
//                    debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) not ready", pr, ar));
//                end
            end
        end

        return rdy;

    endfunction

    // readyDrainBefore
    
    // If an instruction is marked drainBefore, then we must wait until
    // the instructions older than it have committed.
    // IE, the index in the ROB should be zero.

    function Bool readyDrainBefore(ROB_IDX idx, ISA_INSTRUCTION inst);
    
        if (isaDrainBefore(inst))
            return idx == 0;
        else
            return True;
    
    endfunction
    
    // readyDrainAfter
    
    // If we had previously issued an instruction that was marked drainAfter,
    // then we must wait until instructions older than the next instruction
    // have committed.
    // IE, the index in the ROB should be zero.
    
    function Bool readyDrainAfter(DECODE_STATE local_state, ROB_IDX idx);
    
        if (local_state.drainingAfter)
            return idx == 0;
        else
            return True;
    
    endfunction


    // makeIssueBundle
    
    // Marshall up a bundle of useful information to send to the rest of the pipeline.

    function BUNDLE makeIssueBundle(ROB_ENTRY entry);
    
        TOKEN tok = entry.deps.token;
        FETCH_BUNDLE fbndl = entry.bundle;
        ISA_DST_MAPPING dstmap = entry.deps.dstMap;

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

    function DMEM_BUNDLE makeCommitBundle(ROB_ENTRY entry);
    
        TOKEN tok = entry.deps.token;
        FETCH_BUNDLE fbndl = entry.bundle;
        ISA_DST_MAPPING dstmap = entry.deps.dstMap;

        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end
        return DMEM_BUNDLE { token:   tok,
                        virtualAddress: ?,
                        physicalAddress: ?,
                        faultEpoch: fbndl.faultEpoch,
                        isLoad:  isaIsLoad(fbndl.inst),
                        isStore: isaIsStore(fbndl.inst),
                        isTerminate: entry.terminate,
                        dests: dests };

    endfunction

    function Bool canIssue(DECODE_STATE local_state, ROB_ENTRY entry, ROB_IDX idx);
    
        let fetchbundle = entry.bundle;
        let deps = entry.deps;
        let tok = deps.token;
    
        // Check if we can issue.
        let data_ready = readyToGo(tok, deps.srcMap);

        // If it's a store, we need to be able to allocate a slot in the store buffer.
        let good_epoch = (!local_state.tmps.seenMispredict || entry.newEpoch) && fetchbundle.faultEpoch == local_state.epoch.fault;

        return (!local_state.tmps.seenBarrier && good_epoch && local_state.tmps.issueQHasRoom && data_ready && readyDrainAfter(local_state, idx) && readyDrainBefore(idx, fetchbundle.inst));

    endfunction

    // ****** Rules ******

    // stage1_writebacks
    
    // Begin simulating a new context.
    // Get any writebacks from Exe, Mem, or Com and update the scoreboard.

    // Ports read:
    // * writebackFromExe
    // * writebackFromMemHit
    // * writebackFromMemMiss
    
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
    // * deqToInstQ
    
    (* conservative_implicit_conditions *)
    rule stage2_retireAndDeps (True);

        // Begin model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Extract local state from the cpu instance.
        let local_state <- statePool.extractState(cpu_iid);
        // Reset the "non-registered" state.
        local_state.tmps = initialDecodeIntermediateState;
        
        let m_bundle <- bundleFromInstQ.receive(cpu_iid);
        let m_mispred <- mispredictFromExe.receive(cpu_iid);
        let m_fault   <- faultFromCom.receive(cpu_iid);

        let m_commit  <- bundleFromCommitQ.receive(cpu_iid);
        let can_retire <- bundleToRetireQ.canEnq(cpu_iid);
    
        // Process retired instructions from Com.
        if (m_commit matches tagged Valid .bundle)
        begin

            // Save the commit. We'll update the ROB when we scan it sequentially.
            local_state.tmps.mComplete = tagged Valid tuple2(bundle.token, bundle.isTerminate);
            
            // Dequeue the commitQ.
            deqToCommitQ.send(cpu_iid, tagged Valid (?));
        end
        else
        begin
            // No deq.
            deqToCommitQ.send(cpu_iid, tagged Invalid);
        end

        // Examine entry 0 to see if we should retire it. This is the oldest entry.
        LUTRAM#(ROB_IDX, ROB_ENTRY) rob = robPool.getRAM(cpu_iid);
        ROB_ENTRY entry = rob.sub(0);
        
        Bool doing_rewind = False;

        // See if we have a fault coming in.
        if (m_fault matches tagged Valid .fault_tok)
        begin

            // A fault occurred.
            debugLog.record_next_cycle(cpu_iid, fshow("2: FAULT: ") + fshow(fault_tok));
            
            rewindToToken.makeReq(initFuncpReqRewindToToken(fault_tok));
            doing_rewind = True;
            
            // Don't retire anything this cycle. We'll start dropping things next cycle.
            bundleToRetireQ.noEnq(cpu_iid);

            // Increment the epoch.
            local_state.expectingNewEpoch = True;
            local_state.epoch.fault = local_state.epoch.fault + 1;
            
        end      
        else if (m_mispred matches tagged Valid {.tok, .fault_epoch} &&& fault_epoch == local_state.epoch.fault)
        begin
        
            // A misprediction occurred.
            debugLog.record_next_cycle(cpu_iid, fshow("2: MISPREDICT: ") + fshow(tok));
            
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            doing_rewind = True;
            
            // Don't retire anything this cycle. We'll start dropping things next cycle.
            bundleToRetireQ.noEnq(cpu_iid);
            
            // Note the mispredict for sweeping the ROB.
            local_state.tmps.mMispredict = tagged Valid tok;
            local_state.epoch.branch = local_state.epoch.branch + 1;
            local_state.expectingNewEpoch = True;

        end
        else if (entry.valid)
        begin
            if ((local_state.drainingEpoch && !entry.newEpoch) || entry.bundle.faultEpoch != local_state.epoch.fault)
            begin

                if (!entry.issued && !isaIsStore(entry.bundle.inst))
                begin
                
                    // We can silently drop it since no one else knows about it.
                    debugLog.record_next_cycle(cpu_iid, fshow("2: SILENT DROP: ") + fshow(entry.deps.token));
                    bundleToRetireQ.noEnq(cpu_iid);
                
                end
                else
                begin

                    // We can't silently drop it because we need to reclaim the slot from the store buffer.
                    // Instead, mark it as a dummy and send it to commit.
                    let bundle = makeCommitBundle(entry);
                    bundle.token.dummy = True;
                    debugLog.record_next_cycle(cpu_iid, fshow("2: COMMIT DROP AS DUMMY: ") + fshow(bundle.token));
                    bundleToRetireQ.doEnq(cpu_iid, bundle);
                
                end

                // Dequeue the rob later when we sweep it.
                local_state.robOccupancy = local_state.robOccupancy - 1;
                // Shift every ROB entry down 1.
                local_state.tmps.shifting = True;

            end
            else if (entry.completed && can_retire)
            begin

                // Retire the oldest entry.
                bundleToRetireQ.doEnq(cpu_iid, makeCommitBundle(entry));
                local_state.robOccupancy = local_state.robOccupancy - 1;


                // Shift every ROB entry down 1.
                local_state.tmps.shifting = True;
                
                if (entry.mispredicted)
                begin
                    // A mispredict. Commit it, but discard the rest of this epoch.
                    debugLog.record_next_cycle(cpu_iid, fshow("2: COMMIT (MISPREDICTED): ") + fshow(entry.deps.token));
                    // Drain the rest of this epoch.
                    local_state.drainingEpoch = True;
                end
                else
                begin
                    // A normal commit. The commit stage will report back if a fault occurred.
                    debugLog.record_next_cycle(cpu_iid, fshow("2: COMMIT: ") + fshow(entry.deps.token));
                    // If we were draining an epoch, we can stop.
                    local_state.drainingEpoch = False;
                end

            end
            else
            begin
                // Entry is still in flight.
                debugLog.record_next_cycle(cpu_iid, fshow("2: COMMIT STALL: ") + fshow(entry.deps.token));
                bundleToRetireQ.noEnq(cpu_iid);
            end
        end
        else
        begin
            // Oldest entry is invalid, ROB is empty.
            debugLog.record_next_cycle(cpu_iid, fshow("2: NO COMMIT (EMPTY)"));
            bundleToRetireQ.noEnq(cpu_iid);
        end


        // Get the store buffer credit in case we're dealing with a store...
        let m_credit <- creditFromSB.receive(cpu_iid);
        let sb_has_credit = isValid(m_credit);

        if (doing_rewind)
        begin
            
            // Get the rewind response in the next stage.
            deqToInstQ.send(cpu_iid, tagged Invalid);

            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_rewindRsp, local_state));

        end
        else if (m_bundle matches tagged Valid .bundle)
        begin
        
            if (bundle.branchEpoch == local_state.epoch.branch &&& 
                bundle.faultEpoch == local_state.epoch.fault)
            begin

                // An enqueue is happening. Do we have room in the ROB?
                let rob_has_room = local_state.robOccupancy != fromInteger(valueof(NUM_ROB_ENTRIES));

                // For stores we also need a slot in the store buffer.
                // Running out of these should be rare enough that the performance impact is minimal.
                let inst_is_store = isaIsStore(bundle.inst);
                let store_ready = inst_is_store ? sb_has_credit : True;

                if (rob_has_room && store_ready)
                begin

                    // We're guaranteed to have space. Do the enq when we scan the ROB.
                    local_state.tmps.mEnq = tagged Valid bundle;
                    deqToInstQ.send(cpu_iid, tagged Valid (?));
                    local_state.robOccupancy = local_state.robOccupancy + 1;

                    // We need to retrieve dependencies from the functional partition.
                    debugLog.record_next_cycle(cpu_iid, $format("2: ENQ, REQUEST DEPS: PC=0x%0h, INST=0x%0h", bundle.pc, bundle.inst));
                    getDependencies.makeReq(initFuncpReqGetDependencies(getContextId(cpu_iid), bundle.inst, bundle.pc));
                    
                    // Possibly get a store buffer slot after we have the token.
                    local_state.tmps.needSBSlot = inst_is_store;
                    
                    // Actually do the enqueue later as we scan the ROB.
                    stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_depsRsp, local_state));
                end
                else
                begin

                   // ROB is full.
                   debugLog.record_next_cycle(cpu_iid, $format("2: NO ENQ: ROB FULL"));
                   deqToInstQ.send(cpu_iid, tagged Invalid);
                   stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_bubble, local_state));

                end
            end
            else
            begin
            
                // Drop the instruction from a bad epoch.
                debugLog.record_next_cycle(cpu_iid, $format("2: DROP WRONG EPOCH: PC=0x%0h, INST=0x%0h", bundle.pc, bundle.inst));            
                deqToInstQ.send(cpu_iid, tagged Valid (?));

                stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_bubble, local_state));
            end

        end
        else
        begin

            // No enqueue to the ROB.
            debugLog.record_next_cycle(cpu_iid, $format("2: NO ENQ: BUBBLE"));
            deqToInstQ.send(cpu_iid, tagged Invalid);
            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_bubble, local_state));

        end
 
    endrule

    // stage3_dependenciesRsp
    
    // Get the response from the functional partition (if any).
    
    // Ports read:
    // * None
    
    // Ports written:
    // * None

    let writebacksFinished = wbExeCtrl.consumerCanStart() && wbHitCtrl.consumerCanStart() && wbMissCtrl.consumerCanStart();

    rule stage3_dependenciesRsp (writebacksFinished && !isValid(stage3Stall));
    
        // Get the next instance id.
        match {.cpu_iid, {.stage3state, .local_state}} <- stage3Ctrl.nextReadyInstance();

        wbExeCtrl.consumerStart();
        wbHitCtrl.consumerStart();
        wbMissCtrl.consumerStart();

        case (stage3state) matches 
            tagged STAGE3_bubble:
            begin
                debugLog.record(cpu_iid, $format("3: BUBBLE"));
                // No alloc to the store buffer.
                allocToSB.send(cpu_iid, tagged Invalid);
            end
            tagged STAGE3_rewindRsp:
            begin

                // Get the rewind response.
                let rsp = rewindToToken.getResp();
                rewindToToken.deq();
                debugLog.record(cpu_iid, $format("3: REWIND RSP"));
                // No alloc to the store buffer.
                allocToSB.send(cpu_iid, tagged Invalid);

            end
            tagged STAGE3_depsRsp:
            begin

                // Get the response from the functional partition.
                let rsp = getDependencies.getResp();
                getDependencies.deq();

                // Update dependencies.
                local_state.tmps.newDeps = rsp;
                debugLog.record(cpu_iid, $format("3: DEPS RSP: ") + fshow(rsp.token));

                // Update the scoreboard to reflect this instruction entering the ROB.
                // Mark its destination registers as unready until it commits.
                for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
                begin

                    if (rsp.dstMap[x] matches tagged Valid {.ar, .pr} )
                    begin
                        prfScoreboard.issue[x].unready(pr);
                        debugLog.record(cpu_iid, fshow(rsp.token) + $format(": PR %0d (AR %0d) locked", pr, ar));
                    end

                end

                // Get a store buffer slot if neccesary.
                if (local_state.tmps.needSBSlot)
                begin

                    // A store needs to reserve a slot in the store buffer.
                    allocToSB.send(cpu_iid, tagged Valid rsp.token);

                end
                else
                begin

                    // Non-store instruction.
                    allocToSB.send(cpu_iid, tagged Invalid);

                end

            end
        endcase


        // When we scan the ROB, if we're draining the oldest epoch then we don't want to issue instructions
        // from this epoch.
        local_state.tmps.seenMispredict = local_state.drainingEpoch;

        // See if the issueQ has room.
        let can_enq <- bundleToIssueQ.canEnq(cpu_iid);
        local_state.tmps.issueQHasRoom = can_enq;
        
        // Start scanning the ROB to apply all updates.
        stage3Stall <= tagged Valid tuple3(cpu_iid, local_state, 0);

    endrule

    // stage3_ROBScan
    
    // scan the ROB in order, determining all updates, including which instruction to issue.
    (* conservative_implicit_conditions *)
    rule stage3_ROBScan (isValid(stage3Stall));

        match {.cpu_iid, .local_state, .idx} = validValue(stage3Stall);

        // If we are shifting we examine entry x+1, otherwise entry x.
        // The boundary conditions make this a bit ugly.
        // Note that we always write entry x, thus accomplishing the shift if we retired an entry this cycle.

        LUTRAM#(ROB_IDX, ROB_ENTRY) rob = robPool.getRAM(cpu_iid);
        
        ROB_ENTRY entry_cur = rob.sub(idx);
        ROB_ENTRY entry_next = rob.sub(idx + 1); // Can wrap to zero.

        // Shift in an initial entry in the last position.
        ROB_ENTRY new_entry = (idx == fromInteger(valueof(NUM_ROB_ENTRIES) - 1)) ? local_state.tmps.shifting ? initialROBEntry : entry_cur :
                                                                                   local_state.tmps.shifting ? entry_next : entry_cur;

        Bool enq_new_epoch = False;

        // Try to enq to/issue from the ROB.
        if (!new_entry.valid)
        begin

            // This slot is empty. See if we were enqueuing.
            if (local_state.tmps.mEnq matches tagged Valid .fetchbundle)
            begin
                // An enqueue.
                debugLog.record(cpu_iid, $format("3S: ENQ: SLOT %0d, ", idx) + fshow(local_state.tmps.newDeps.token));
                new_entry = initialROBEntry;
                new_entry.bundle = fetchbundle;
                new_entry.deps = local_state.tmps.newDeps;
                new_entry.valid = True;
                
                // If this is the first instruction of a new epoch, record that.
                if (local_state.expectingNewEpoch)
                begin
                    debugLog.record(cpu_iid, $format("3S: ENQ: NEW EPOCH"));
                    local_state.expectingNewEpoch = False;
                    new_entry.newEpoch = True;
                    enq_new_epoch = True;
                end

                // Invalidate the enqueue so later stages won't try to repeat it.
                local_state.tmps.mEnq = tagged Invalid;
            end
            else
            begin
                debugLog.record(cpu_iid, $format("3S: SKIP INVALID: SLOT %0d", idx));
            end

        end
        else if (!new_entry.issued)
        begin
            // Try to issue.
            if (canIssue(local_state, new_entry, idx))
            begin

                // We can issue it. But did we already find an older instruction?
                if (!isValid(local_state.tmps.mInstToIssue))
                begin
                    // Time to issue this entry. Since we favor older instructions no later stage will displace this.
                    debugLog.record(cpu_iid, $format("3S: ISSUE: SLOT %0d, ", idx) + fshow(new_entry.deps.token));
                    local_state.tmps.mInstToIssue = tagged Valid makeIssueBundle(new_entry);
                    new_entry.issued = True;
                    
                    // See if we need to drain the ROB after this instruction.
                    local_state.drainingAfter = isaDrainAfter(new_entry.bundle.inst);

                end
                else
                begin
                    // We already found an older instruction.
                    debugLog.record(cpu_iid, $format("3S: STRUCTURAL STALL: SLOT %0d, ", idx) + fshow(new_entry.deps.token));
                end
            end
            else
            begin
                // We're stalled for some reason.
                debugLog.record(cpu_iid, $format("3S: DEPENDENCY STALL: SLOT %0d, ", idx) + fshow(new_entry.deps.token));
            end
        end
        else
        begin
            // The entry has already been issued.
            debugLog.record(cpu_iid, $format("3S: SKIP IN-FLIGHT: SLOT %0d, ", idx) + fshow(new_entry.deps.token));
        end


        // Update scoreboarding based on incoming updates.
        if (new_entry.valid)
        begin
            if (isaDrainAfter(new_entry.bundle.inst))
            begin
                // Don't issue instructions after a drain after, until it's completed.
                local_state.tmps.seenBarrier = True;
            end
            if (new_entry.newEpoch)
            begin
                // Once we see a new epoch we can begin to issue again.
                if (local_state.tmps.consolidatingEpochs)
                begin
                    // When we consolidate epochs we merge all the existing epochs into the
                    // current (now dead) epoch. However don't do this with newly enq'd entries.
                    // They are truly new.
                    new_entry.newEpoch = enq_new_epoch;
                    debugLog.record(cpu_iid, $format("3S: EPOCH CONSOLIDATION: CONSOLIDATING EPOCH (UNLESS NEW)"));
                end
                else
                begin
                    // We're in a new epoch, so reset all mispredict record-keeping.
                    local_state.tmps.seenMispredict = False;
                    local_state.tmps.addedMispredict = False;
                end
            end
            if (new_entry.mispredicted)
            begin
                // After we see a mispredict, don't issue any more instructions until the new epoch.
                // This ensures that the front end will get redirects in the "optimal" order.
                // IE "The best information I have is that you should go this way"
                if (local_state.tmps.addedMispredict)
                begin
                    // If we just added a mispredict then this must be a younger mispredict in
                    // the same epoch. Uh oh. That means we've started to fetch instructions from
                    // what turned out to be the wrong path. Consolidate the speculative epochs into one.
                    debugLog.record(cpu_iid, $format("3S: BEGIN EPOCH CONSOLIDATION"));
                    local_state.tmps.consolidatingEpochs = True;
                end
                local_state.tmps.seenMispredict = True;
            end
            if (local_state.tmps.mComplete matches tagged Valid {.tok, .term} &&& tokTokenId(tok) == tokTokenId(new_entry.deps.token))
            begin
                // The entry is ready to complete. Update termination info.
                new_entry.completed = True;
                new_entry.terminate = term;
                // Get any updates to the token itself. (Like dummy status, etc)
                new_entry.deps.token = tok;
            end
            if (local_state.tmps.mMispredict matches tagged Valid .tok &&& tokTokenId(tok) == tokTokenId(new_entry.deps.token))
            begin
                new_entry.mispredicted = True;
                // Record that we've seen a mispredict. Therefore if we see any younger
                // mispredicts they're bad-path.
                local_state.tmps.seenMispredict = True;
                local_state.tmps.addedMispredict = True;
            end
        end

        // Update the /current/ index. This accomplishes the shift with only one write port on the RAM.
        rob.upd(idx, new_entry);
            
        // If we're not shifting, we can stop when we've handled all enqueues, 
        // and either found an instruction to issue or reached an invalid entry.
        let enq_done = !isValid(local_state.tmps.mEnq);
        let issue_done = isValid(local_state.tmps.mInstToIssue) || !new_entry.valid;
        // Also make sure all updates are finished.
        let updates_done = (!isValid(local_state.tmps.mComplete) && !isValid(local_state.tmps.mMispredict)) || !new_entry.valid;
        // If we've added a mispredict, we need to go until we see a new epoch, or an invalid.
        // If this lead to us considating epochs, we need to go until an invalid.
        let epochs_done = !new_entry.valid || (!local_state.tmps.addedMispredict && !local_state.tmps.consolidatingEpochs);
        // If we're shifting, we need to go until we find an invalid entry (if any)
        let shifting_done = !(local_state.tmps.shifting && new_entry.valid);
        
        let work_done = enq_done && issue_done && updates_done && shifting_done;
        
        // Alternatively, stop at the end of the ROB.
        if (work_done || idx == fromInteger(valueof(NUM_ROB_ENTRIES) - 1))
        begin

            // We can stop circulating.
            debugLog.record(cpu_iid, $format("3S: UNSTALL: SLOT %0d, ", idx));
            stage3Stall <= tagged Invalid;
            // Move on to the next stage.
            stage4Ctrl.ready(cpu_iid, local_state);

        end
        else
        begin
        
            // Repeat this stage again on the next index.
            stage3Stall <= tagged Valid tuple3(cpu_iid, local_state, idx + 1);

        end

    endrule


    // Specify a rule urgency so that if the Scoreboard has less parallelism there are no
    // compiler warnings.

    (* conservative_implicit_conditions, descending_urgency = "stage1_writebackMemMiss, stage1_writebackMemHit, stage1_writebackExe, stage3_dependenciesRsp, stage4_performIssue" *)
    rule stage4_performIssue (True);

        match {.cpu_iid, .local_state} <- stage4Ctrl.nextReadyInstance();

        // See if we found anything to issue.
        if (local_state.tmps.mInstToIssue matches tagged Valid .bundle)
        begin
            // A successful issue.
            debugLog.record(cpu_iid, $format("4: ISSUE: ") + fshow(bundle.token));
            bundleToIssueQ.doEnq(cpu_iid, bundle);
        end
        else
        begin
            // Didn't find anything to issue for some reason.
            debugLog.record(cpu_iid, $format("4: NO ISSUE"));
            bundleToIssueQ.noEnq(cpu_iid);
        end

        // End the model cycle.        
        wbExeCtrl.consumerDone();
        wbHitCtrl.consumerDone();
        wbMissCtrl.consumerDone();

        statePool.insertState(cpu_iid, local_state);
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

        
endmodule
