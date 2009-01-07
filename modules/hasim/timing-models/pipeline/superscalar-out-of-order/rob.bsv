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
import Counter::*;
import FShow::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_controller.bsh"

`include "asim/provides/hasim_pipeline_types.bsh"
`include "asim/provides/hasim_issue.bsh"

typedef enum {ROB_STATE_WRITEBACK_ALU, ROB_STATE_WRITEBACK_MEM, ROB_STATE_ADD, ROB_STATE_COMMIT_REQ, ROB_STATE_COMMIT_RESP, ROB_STATE_ISSUE_REQ, ROB_STATE_ISSUE_RESP} ROB_STATE deriving (Bits, Eq);

typedef Bit#(3) TIME_STAMP;
typedef enum {ALU, MEM} WRITE_TYPE deriving (Bits, Eq);

typedef DECODE_BUNDLE ROB_ENTRY;

REWIND_BUNDLE nullRewindBundle = REWIND_BUNDLE{robIndex: 0, mispredict: False, addr: 0, token: ?};

module [HASIM_MODULE] mkIssue();
    TIMEP_DEBUG_FILE                                                              debugLog <- mkTIMEPDebugFile("pipe_rob.out");

    PORT_CREDIT_SEND#(COMMIT_BUNDLE, `COMMIT_NUM, LOG_COMMIT_NUM)               commitPort <- mkPortCreditSend("commit");
    PORT_CREDIT_RECEIVE#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS) aluWritebackPort <- mkPortCreditReceive("aluWriteback");
    PORT_CREDIT_RECEIVE#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS) memWritebackPort <- mkPortCreditReceive("memWriteback");
    PORT_CREDIT_RECEIVE#(DECODE_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)        decodePort <- mkPortCreditReceive("decode");
    PORT_CREDIT_SEND#(MEM_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)                       memPort <- mkPortCreditSend("mem");
    PORT_CREDIT_SEND#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                       aluPort <- mkPortCreditSend("alu");
    PORT_CREDIT_SEND#(PREDICT_UPDATE_BUNDLE, `ALU_NUM, LOG_ALU_NUM)      predictUpdatePort <- mkPortCreditSend("predictUpdate");

    PORT_CREDIT_SEND#(REWIND_BUNDLE, 1, 1)                                     resteerPort <- mkPortCreditSend("resteer");
    PORT_CREDIT_SEND#(FAULT_BUNDLE, 1, 1)                                        faultPort <- mkPortCreditSend("fault");

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)                                   modelCycle <- mkConnection_Send("model_cycle");

    BRAM#(ROB_INDEX, ROB_ENTRY)                                                        rob <- mkBRAM();

    function prfInit(i) = (i < valueOf(TExp#(SizeOf#(ISA_REG_INDEX))));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool))                prfValids <- mkReg(genWith(prfInit));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP))     writeTimeStamp <- mkReg(replicate(1));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE)))  writeType <- mkReg(replicate(tagged Valid ALU));
    Reg#(TIME_STAMP)                                                             timeStamp <- mkReg(0);

    Reg#(ROB_STATE)                                                                  state <- mkReg(ROB_STATE_WRITEBACK_ALU);

    Reg#(ROB_PTR)                                                                commitPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                   addPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                 issuePtr <- mkReg(0);

    Reg#(Vector#(DECODE_CREDITS, Bool))                                           robValid <- mkReg(replicate(True));
    Reg#(Vector#(DECODE_CREDITS, Bool))                                           robEpoch <- mkReg(replicate(False));
    Reg#(Bool)                                                                 resteerWait <- mkReg(False);

    LUTRAM#(ROB_INDEX, Bool)                                                       robDone <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     robPoison <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     robIssued <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     terminate <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                      passFail <- mkLUTRAMU();

    Reg#(REWIND_BUNDLE)                                                       rewindBundle <- mkReg(nullRewindBundle);
    Reg#(FAULT_BUNDLE)                                                         faultBundle <- mkReg(makeNoFaultBundle());

    Reg#(Bool)                                                               allPrevIssued <- mkReg(False);
    Reg#(Bool)                                                            allPrevMemIssued <- mkReg(False);

    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    LocalController                                                        localController <- mkLocalController(inports, outports);

    ROB_INDEX commitIndex = truncate(commitPtr);
    ROB_INDEX addIndex = truncate(addPtr);
    ROB_INDEX issueIndex = truncate(issuePtr);

    ROB_PTR credits = fromInteger(valueOf(DECODE_CREDITS)) - (addPtr - commitPtr);

    function Vector#(DECODE_CREDITS, Bool) markValidOrInvalid(Bool which, Vector#(DECODE_CREDITS, Bool) inVec, ROB_INDEX startIndex);
        Bit#(DECODE_CREDITS) commitMask = (~0) << commitIndex;
        Bit#(DECODE_CREDITS) startMask = (~0) << startIndex;
        Bit#(DECODE_CREDITS) mask = (startIndex < commitIndex)? ((~commitMask) & startMask): ((~commitMask) | startMask);
        Vector#(DECODE_CREDITS, Bool) outVec = newVector();
        for(Integer i = 0; i < valueOf(DECODE_CREDITS); i = i + 1)
            outVec[i] = (mask[i] == 1)? which: inVec[i];
        return outVec;
    endfunction

    function Bool isReady(ROB_ENTRY entry);
        Vector#(ISA_MAX_SRCS, Bool) readys = newVector();
        for(Integer i = 0; i < valueOf(ISA_MAX_SRCS); i = i + 1)
        begin
            if(entry.srcs[i] matches tagged Valid .src)
                readys[i] = (isValid(writeType[src]) && (validValue(writeType[src]) == ALU && (timeStamp - writeTimeStamp[src]) > 1) || prfValids[src]);
            else
                readys[i] = True;
        end
        return fold(\&& , readys) && (!entry.drainBefore || issuePtr == commitPtr);
    endfunction

    function Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool) markPrfValidOrInvalid(Bool which, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs);
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool) newPrfValids = prfValids;
        for(Integer i = 0; i < valueOf(ISA_MAX_DSTS); i = i + 1)
        begin
            if(regs[i] matches tagged Valid .idx)
                newPrfValids[idx] = which;
        end
        return newPrfValids;
    endfunction

    function Action markWriteType(Maybe#(WRITE_TYPE) _writeType, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs);
    action
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE)) newWriteType = writeType;
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP)    newWriteTimeStamp = writeTimeStamp;
        for(Integer i = 0; i < valueOf(ISA_MAX_DSTS); i = i + 1)
        begin
            if(regs[i] matches tagged Valid .idx)
            begin
                newWriteType[idx] = _writeType;
                newWriteTimeStamp[idx] = timeStamp;
            end
        end
        writeType <= newWriteType;
        writeTimeStamp <= newWriteTimeStamp;
    endaction
    endfunction

    rule writebackAlu(state == ROB_STATE_WRITEBACK_ALU);
        if(aluWritebackPort.canReceive())
        begin
            let bundle <- aluWritebackPort.pop();
            debugLog.record($format("writebackAlu ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]));
            predictUpdatePort.enq(makePredictUpdateBundle(bundle));
            if(robValid[bundle.robIndex])
            begin
                debugLog.record($format("robValid"));
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);
                if(bundle.mispredict)
                begin
                    debugLog.record($format("mispredict"));
                    let newRobValid = markValidOrInvalid(False, robValid, bundle.robIndex);
                    newRobValid[bundle.robIndex] = True;
                    let newRobEpoch = markValidOrInvalid(False, robEpoch, bundle.robIndex);
                    newRobEpoch[bundle.robIndex] = True;
                    robValid <= newRobValid;
                    robEpoch <= newRobEpoch;
                    rewindBundle <= makeRewindBundle(bundle);
                    resteerWait <= True;
                end
            end

            terminate.upd(bundle.robIndex, bundle.terminate);
            passFail.upd(bundle.robIndex, bundle.passFail);
            robPoison.upd(bundle.robIndex, tokIsPoisoned(bundle.token));
            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            predictUpdatePort.done;
            aluWritebackPort.done(`ALU_CREDITS);
            state <= ROB_STATE_WRITEBACK_MEM;
            debugLog.record($format("writebackAlu resteer ") + fshow(rewindBundle));
            resteerPort.send(rewindBundle);
            rewindBundle <= nullRewindBundle;
        end
    endrule

    rule writebackMem(state == ROB_STATE_WRITEBACK_MEM);
        if(memWritebackPort.canReceive())
        begin
            let bundle <- memWritebackPort.pop();
            debugLog.record($format("writebackMem ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]));
            if(robValid[bundle.robIndex])
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);

            robPoison.upd(bundle.robIndex, tokIsPoisoned(bundle.token));
            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            debugLog.record($format("writeback mem done"));
            memWritebackPort.done(`MEM_CREDITS);
            state <= ROB_STATE_ADD;
        end
    endrule

    rule add(state == ROB_STATE_ADD);
        if(decodePort.canReceive())
        begin
            let entry <- decodePort.pop();
            debugLog.record($format("decode port dequeued"));
            if(robValid[addIndex] || entry.afterResteer && robEpoch[entry.epochRob])
            begin
                resteerWait <= False;
                debugLog.record($format("add ") + fshow(entry) + $format(" addPtr: %d commitPtr: %d credits: %d", addPtr, commitPtr, credits));
                robValid <= markValidOrInvalid(True, robValid, addIndex);
                robIssued.upd(addIndex, False);
                robDone.upd(addIndex, False);
                robPoison.upd(addIndex, tokIsPoisoned(entry.token));
                rob.write(addIndex, entry);
                prfValids <= markPrfValidOrInvalid(False, entry.dsts);
                markWriteType(Invalid, entry.dsts);
                terminate.upd(addIndex, False);
                addPtr <= addPtr + 1;
            end
        end
        else
        begin
            debugLog.record($format("add done"));
            state <= ROB_STATE_COMMIT_REQ;
            decodePort.done(credits);
        end
    endrule

    rule commitReq(state == ROB_STATE_COMMIT_REQ);
        debugLog.record($format("commitReq: commitPtr: %d robValid: %b robDone: %b robValid[]: %b addPtr: %d credits: %d", commitPtr, robValid, robDone.sub(commitIndex), robValid[commitIndex], addPtr, credits));
        if(!commitPort.canSend() || commitPtr == addPtr || !robDone.sub(commitIndex))
        begin
            debugLog.record($format("commit done"));
            commitPort.done();
            state <= ROB_STATE_ISSUE_REQ;
            issuePtr <= commitPtr;
            allPrevIssued <= True;
            allPrevMemIssued <= True;
            faultPort.send(faultBundle);
            faultBundle <= makeNoFaultBundle();
        end
        else if(!robValid[commitIndex])
        begin
            debugLog.record($format("robInValid %d", commitIndex));
            commitPtr <= commitPtr + 1;
        end
        else
        begin
            debugLog.record($format("commit req -> resp"));
            rob.readReq(commitIndex);
            state <= ROB_STATE_COMMIT_RESP;
        end
    endrule

    rule commitResp(state == ROB_STATE_COMMIT_RESP);
        let entry <- rob.readRsp();
        debugLog.record($format("commitResp ") + fshow(entry) + $format(" commitPtr: %d robValid: %b addPtr: %d credits: %d", commitPtr, robValid, addPtr, credits));

        if(terminate.sub(commitIndex))
        begin
            debugLog.record($format("terminate"));
            localController.endProgram(passFail.sub(commitIndex));
        end

        if (robPoison.sub(commitIndex))
        begin
            debugLog.record($format("robFault %d", commitIndex));

            let newRobValid = markValidOrInvalid(False, robValid, commitIndex);
            robValid <= newRobValid;

            let newRobEpoch = markValidOrInvalid(False, robEpoch, commitIndex);
            newRobEpoch[commitIndex] = True;
            robEpoch <= newRobEpoch;

            // Set the poison bit in the token sent back to the front end.
            // It was never updated in the ROB itself.
            let tok = entry.token;
            tok.poison = True;
            faultBundle <= makeFaultBundle(tok, True, commitIndex);
            resteerWait <= True;
        end
        else
        begin
            commitPort.enq(makeCommitBundle(entry));
            robValid[commitIndex] <= !resteerWait;
        end

        commitPtr <= commitPtr + 1;
        state <= ROB_STATE_COMMIT_REQ;
    endrule

    rule issueReq(state == ROB_STATE_ISSUE_REQ);
        if((!memPort.canSend() && !aluPort.canSend()) || issuePtr == addPtr || !allPrevIssued)
        begin
            debugLog.record($format("end cycle"));
            modelCycle.send(0);
            debugLog.nextModelCycle();
            timeStamp <= timeStamp + 1;
            memPort.done();
            aluPort.done();
            state <= ROB_STATE_WRITEBACK_ALU;
        end
        else if(robIssued.sub(issueIndex))
        begin
            debugLog.record($format("issued"));
            issuePtr <= issuePtr + 1;
        end
        else if(!robValid[issueIndex])
        begin
            debugLog.record($format("issue robInvalid %d", issueIndex));
            robIssued.upd(issueIndex, True);
            robDone.upd(issueIndex, True);
            issuePtr <= issuePtr + 1;
        end
        else
        begin
            debugLog.record($format("issue req -> issue resp"));
            rob.readReq(issueIndex);
            state <= ROB_STATE_ISSUE_RESP;
        end
    endrule

    rule issueResp(state == ROB_STATE_ISSUE_RESP);
        let entry <- rob.readRsp();
        Bool isMem = isaIsStore(entry.inst) || isaIsLoad(entry.inst);
        Bool newAllPrevMemIssued = allPrevMemIssued;
        Bool newAllPrevIssued = True;
        debugLog.record($format("issueResp check issuePtr: %d", issuePtr) + fshow(entry));
        if(isReady(entry))
        begin
            if(isMem)
            begin
                if(allPrevMemIssued && memPort.canSend())
                begin
                    debugLog.record($format("issueResp mem") + fshow(entry));
                    memPort.enq(makeMemBundle(entry, issuePtr));
                    markWriteType(tagged Valid MEM, entry.dsts);
                    robIssued.upd(issueIndex, True);
                end
                else
                begin
                    debugLog.record($format("mem not issued"));
                    newAllPrevMemIssued = False;
                    if(`INORDER_ISSUE == 1)
                        newAllPrevIssued = False;
                end
            end
            else
            begin
                if(aluPort.canSend())
                begin
                    debugLog.record($format("issueResp alu") + fshow(entry));
                    aluPort.enq(makeAluBundle(entry, issuePtr));
                    markWriteType(tagged Valid ALU, entry.dsts);
                    robIssued.upd(issueIndex, True);
                end
                else
                begin
                    debugLog.record($format("alu not issued"));
                    if(`INORDER_ISSUE == 1)
                        newAllPrevIssued = False;
                end
            end
        end
        else
        begin
            if(isMem)
                newAllPrevMemIssued = False;
            if(`INORDER_ISSUE == 1)
                newAllPrevIssued = False;
            debugLog.record($format("entry not ready for issue ") + fshow(entry));
        end
        issuePtr <= issuePtr + 1;
        state <= ROB_STATE_ISSUE_REQ;
        allPrevMemIssued <= newAllPrevMemIssued;
        allPrevIssued <= newAllPrevIssued;
    endrule
endmodule
