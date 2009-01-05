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

import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

//import PipelineTypes::*;
import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_DECODE.bsh"

typedef enum { DECODE_STATE_BUS, DECODE_STATE_INST, DECODE_STATE_DEP, DECODE_STATE_SEND } DECODE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDecode ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_decode.out");

    StallPort_Receive#(Tuple2#(TOKEN,FETCH_BUNDLE)) inQ <- mkStallPort_Receive("fet2dec");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))          outQ <- mkStallPort_Send   ("dec2exe");

    Vector#(2, Port_Receive#(BUS_MESSAGE)) busQ = newVector();
    busQ[0] <- mkPort_Receive("exe_bus", 0);
    busQ[1] <- mkPort_Receive("mem_bus", 0);

    Port_Receive#(TOKEN) commitQ <- mkPort_Receive("commit_bus", 0);

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    Reg#(DECODE_STATE) state <- mkReg(DECODE_STATE_BUS);
    Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies <- mkReg(Invalid);

    //Local Controller
    Vector#(4, Port_Control) inports  = newVector();
    Vector#(1, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = busQ[0].ctrl;
    inports[2]  = busQ[1].ctrl;
    inports[3]  = commitQ.ctrl;
    outports[0] = outQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_dec <- mkEventRecorder(`EVENTS_DECODE_INSTRUCTION_DECODE);

    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prfValid_init = newVector();

    for (Integer i = 0; i < numIsaArchRegisters; i = i + 1)
        prfValid_init[i] = True;

    for (Integer i = numIsaArchRegisters; i < numFuncpPhyRegisters; i = i + 1)
        prfValid_init[i] = False;

    COUNTER#(TOKEN_INDEX_SIZE) numInstrsInFlight <- mkLCounter(0);
    Reg#(Bool)    drainingAfter <- mkReg(False);

    Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool)) prfValid <- mkReg(prfValid_init);

    function ActionValue#(Bool) readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);
    actionvalue
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
            begin
                rdy = rdy && prfValid[pr];

                if (! prfValid[pr])
                begin
                    debugLog.record(fshow(tok) + $format(": PR %0d (AR %0d) not ready", pr, ar));
                end
            end
        end
        return rdy;
    endactionvalue
    endfunction

    function Bool readyDrainBefore(ISA_INSTRUCTION inst);
    
        if (isaDrainBefore(inst))
            return numInstrsInFlight.value() == 0;
        else
            return True;
    
    endfunction
    
    function Bool readyDrainAfter();
    
        if (drainingAfter)
            return numInstrsInFlight.value() == 0;
        else
            return True;
    
    endfunction

    function Action markPRFInvalid(TOKEN tok, ISA_DST_MAPPING dstmap);
      action
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
            begin
                prf_valid[pr] = False;
                debugLog.record(fshow(tok) + $format(": PR %0d (AR %0d) locked", pr, ar));
            end
        end
        prfValid <= prf_valid;
        numInstrsInFlight.up();
      endaction
    endfunction

    function BUNDLE makeBundle(FETCH_BUNDLE fbndl, ISA_DST_MAPPING dstmap);
        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end
        return BUNDLE { isLoad:  isaIsLoad(fbndl.inst),
                        isStore: isaIsStore(fbndl.inst),
                        isTerminate: Invalid,
                        pc: fbndl.pc,
                        branchAttr: fbndl.branchAttr,
                        effAddr: ?,
                        dests: dests };
    endfunction

    rule bus (state == DECODE_STATE_BUS);
        debugLog.nextModelCycle();
        local_ctrl.startModelCC();

        Integer instrs_removed = 0;
        Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prf_valid = prfValid;

        // Process writes from EXE
        let bus_exe <- busQ[0].receive();
        if (bus_exe matches tagged Valid .msg)
        begin
            if (msg.tokKilled)
            begin
                instrs_removed = instrs_removed + 1;
                debugLog.record_next_cycle(fshow(msg.token) + $format(": no longer in flight"));
            end

            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin
                if (msg.destRegs[i] matches tagged Valid .pr)
                begin
                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(fshow(msg.token) + $format(": PR %0d is ready -- EXE", pr));
                end
            end
        end

        // Process writes from MEM
        let bus_mem <- busQ[1].receive();
        if (bus_mem matches tagged Valid .msg)
        begin
            if (msg.tokKilled)
            begin
                instrs_removed = instrs_removed + 1;
                debugLog.record_next_cycle(fshow(msg.token) + $format(": no longer in flight"));
            end

            for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
            begin
                if (msg.destRegs[i] matches tagged Valid .pr)
                begin
                    prf_valid[pr] = True;
                    debugLog.record_next_cycle(fshow(msg.token) + $format(": PR %0d is ready -- MEM", pr));
                end
            end
        end

        // Track in-flight instructions
        let commit <- commitQ.receive();
        if (commit matches tagged Valid .commit_tok)
        begin
            debugLog.record_next_cycle(fshow(commit_tok) + $format(": Commit"));
            instrs_removed = instrs_removed + 1;
        end

        numInstrsInFlight.downBy(fromInteger(instrs_removed));
        prfValid <= prf_valid;

        state <= DECODE_STATE_INST;
    endrule

    rule stall (state == DECODE_STATE_INST && !outQ.canSend);
        debugLog.record(fshow("STALL PROPAGATED"));
        inQ.pass();
        outQ.pass();
        event_dec.recordEvent(Invalid);
        state <= DECODE_STATE_BUS;
    endrule

    rule bubble (state == DECODE_STATE_INST && outQ.canSend && !isValid(inQ.peek));
        debugLog.record(fshow("BUBBLE"));
        let x <- inQ.receive();
        outQ.send(Invalid);
        event_dec.recordEvent(Invalid);
        state <= DECODE_STATE_BUS;
    endrule

    rule inst (state == DECODE_STATE_INST &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .* });
        if (!isValid(memoDependencies))
            getDependencies.makeReq(initFuncpReqGetDependencies(tok));
        state <= DECODE_STATE_DEP;
    endrule

    rule dep (state == DECODE_STATE_DEP);
        if (!isValid(memoDependencies)) begin
            let rsp = getDependencies.getResp();
            memoDependencies <= Valid(rsp);
            getDependencies.deq();
        end
        state <= DECODE_STATE_SEND;
    endrule

    rule send (state == DECODE_STATE_SEND &&& memoDependencies matches tagged Valid .rsp
                                          &&& inQ.peek() matches tagged Valid { .tok, .fetchbundle });
    
        let tok = rsp.token;
        let data_ready <- readyToGo(tok, rsp.srcMap);
        if (data_ready && readyDrainAfter() && readyDrainBefore(fetchbundle.inst))
        begin

            markPRFInvalid(tok, rsp.dstMap);
            let mtup <- inQ.receive();
            let bundle = makeBundle(fetchbundle, rsp.dstMap);
            outQ.send(Valid(tuple2(tok,bundle)));
            event_dec.recordEvent(Valid(zeroExtend(pack(tok.index))));
            debugLog.record(fshow(tok) + fshow(": SEND INST:") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle));
            memoDependencies <= Invalid;
            drainingAfter <= isaDrainAfter(fetchbundle.inst);
            
        end
        else
        begin
            debugLog.record(fshow(tok) + fshow(": STALL ON DEPENDENCY"));
            inQ.pass();
            outQ.send(Invalid);
            event_dec.recordEvent(Invalid);
        end
        state <= DECODE_STATE_BUS;
    endrule

endmodule
