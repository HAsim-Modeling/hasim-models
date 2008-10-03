import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;
import fpga_components::*;
import module_local_controller::*;

import Vector::*;
import Counter::*;

`include "PipelineTypes.bsv"

typedef enum {ROB_STATE_WRITEBACK_ALU, ROB_STATE_WRITEBACK_MEM, ROB_STATE_ADD, ROB_STATE_COMMIT_REQ, ROB_STATE_COMMIT_RESP, ROB_STATE_ISSUE_REQ, ROB_STATE_ISSUE_RESP} ROB_STATE deriving (Bits, Eq);

typedef Bit#(3) TIME_STAMP;
typedef enum {ALU, MEM} WRITE_TYPE deriving (Bits, Eq);

typedef DECODE_BUNDLE ROB_ENTRY;

REWIND_BUNDLE nullRewindBundle = REWIND_BUNDLE{robIndex: 0, mispredict: False, addr: 0, token: ?};

module [HASIM_MODULE] mkRob();
    TIMEP_DEBUG_FILE                                                             debugLog <- mkTIMEPDebugFile("pipe_rob.out");

    PORT_CREDIT_SEND#(COMMIT_BUNDLE, `COMMIT_NUM, LOG_COMMIT_NUM)              commitPort <- mkPortCreditSend("commit");
    PORT_NO_STALL_RECEIVE#(ALU_WRITEBACK_BUNDLE, `ALU_NUM)               aluWritebackPort <- mkPortNoStallReceive("aluWriteback");
    PORT_NO_STALL_RECEIVE#(MEM_WRITEBACK_BUNDLE, `MEM_NUM)               memWritebackPort <- mkPortNoStallReceive("memWriteback");
    PORT_CREDIT_RECEIVE#(DECODE_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)       decodePort <- mkPortCreditReceive("decode");
    PORT_CREDIT_SEND#(MEM_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)                      memPort <- mkPortCreditSend("mem");
    PORT_CREDIT_SEND#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                      aluPort <- mkPortCreditSend("alu");
    PORT_CREDIT_SEND#(PREDICT_UPDATE_BUNDLE, `ALU_NUM, LOG_ALU_NUM)     predictUpdatePort <- mkPortCreditSend("predictUpdate");

    PORT_CREDIT_SEND#(REWIND_BUNDLE, 1, 1)                                    resteerPort <- mkPortCreditSend("resteer");

    Connection_Send#(Bool)                                                     modelCycle <- mkConnection_Send("model_cycle");

    BRAM#(ROB_INDEX, ROB_ENTRY)                                                       rob <- mkBRAM();

    function prfInit(i) = (i < valueOf(TExp#(SizeOf#(ISA_REG_INDEX))));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool))               prfValids <- mkReg(genWith(prfInit));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP))    writeTimeStamp <- mkReg(replicate(1));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE))) writeType <- mkReg(replicate(tagged Valid ALU));
    Reg#(TIME_STAMP)                                                            timeStamp <- mkReg(0);

    Reg#(ROB_STATE)                                                                 state <- mkReg(ROB_STATE_WRITEBACK_ALU);

    Reg#(ROB_PTR)                                                               commitPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                  addPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                issuePtr <- mkReg(0);

    Reg#(Vector#(DECODE_CREDITS, Bool))                                          robValid <- mkReg(replicate(True));
    Reg#(Vector#(DECODE_CREDITS, Bool))                                          robEpoch <- mkReg(replicate(False));
    Reg#(Bool)                                                                resteerWait <- mkReg(False);

    LUTRAM#(ROB_INDEX, Bool)                                                      robDone <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                    robIssued <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                    terminate <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     passFail <- mkLUTRAMU();

    Reg#(REWIND_BUNDLE)                                                      rewindBundle <- mkReg(nullRewindBundle);

    Reg#(Bool)                                                              allPrevIssued <- mkReg(False);
    Reg#(Bool)                                                           allPrevMemIssued <- mkReg(False);

    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    LocalController                                                       localController <- mkLocalController(inports, outports);

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
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);
                if(bundle.mispredict)
                begin
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
            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            predictUpdatePort.done;
            aluWritebackPort.done;
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

            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            memWritebackPort.done;
            state <= ROB_STATE_ADD;
        end
    endrule

    rule add(state == ROB_STATE_ADD);
        if(decodePort.canReceive())
        begin
            let entry <- decodePort.pop();
            if(robValid[addIndex] || entry.afterResteer && robEpoch[entry.epochRob])
            begin
                resteerWait <= False;
                debugLog.record($format("add ") + fshow(entry) + $format(" addPtr: %d commitPtr: %d credits: %d", addPtr, commitPtr, credits));
                robValid <= markValidOrInvalid(True, robValid, addIndex);
                robIssued.upd(addIndex, False);
                robDone.upd(addIndex, False);
                rob.write(addIndex, entry);
                prfValids <= markPrfValidOrInvalid(False, entry.dsts);
                markWriteType(Invalid, entry.dsts);
                terminate.upd(addIndex, False);
                addPtr <= addPtr + 1;
            end
        end
        else
        begin
            state <= ROB_STATE_COMMIT_REQ;
            decodePort.done(credits);
        end
    endrule

    rule commitReq(state == ROB_STATE_COMMIT_REQ);
        debugLog.record($format("commitReq: commitPtr: %d robValid: %b robDone: %b robValid[]: %b addPtr: %d credits: %d", commitPtr, robValid, robDone.sub(commitIndex), robValid[commitIndex], addPtr, credits));
        if(!commitPort.canSend() || commitPtr == addPtr || !robDone.sub(commitIndex))
        begin
            commitPort.done();
            state <= ROB_STATE_ISSUE_REQ;
            issuePtr <= commitPtr;
            allPrevIssued <= True;
            allPrevMemIssued <= True;
        end
        else if(!robValid[commitIndex])
            commitPtr <= commitPtr + 1;
        else
        begin
            rob.readReq(commitIndex);
            state <= ROB_STATE_COMMIT_RESP;
        end
    endrule

    rule commitResp(state == ROB_STATE_COMMIT_RESP);
        let entry <- rob.readRsp();
        debugLog.record($format("commitResp ") + fshow(entry) + $format(" commitPtr: %d robValid: %b addPtr: %d credits: %d", commitPtr, robValid, addPtr, credits));
        commitPort.enq(makeCommitBundle(entry));
        if(terminate.sub(commitIndex))
            localController.endProgram(passFail.sub(commitIndex));
        robValid[commitIndex] <= !resteerWait;
        commitPtr <= commitPtr + 1;
        state <= ROB_STATE_COMMIT_REQ;
    endrule

    rule issueReq(state == ROB_STATE_ISSUE_REQ);
        if((!memPort.canSend() && !aluPort.canSend()) || issuePtr == addPtr || !allPrevIssued)
        begin
            modelCycle.send(?);
            debugLog.nextModelCycle();
            timeStamp <= timeStamp + 1;
            memPort.done();
            aluPort.done();
            state <= ROB_STATE_WRITEBACK_ALU;
        end
        else if(robIssued.sub(issueIndex))
            issuePtr <= issuePtr + 1;
        else if(!robValid[issueIndex])
        begin
            robIssued.upd(issueIndex, True);
            robDone.upd(issueIndex, True);
            issuePtr <= issuePtr + 1;
        end
        else
        begin
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
        end
        issuePtr <= issuePtr + 1;
        state <= ROB_STATE_ISSUE_REQ;
        allPrevMemIssued <= newAllPrevMemIssued;
        allPrevIssued <= newAllPrevIssued;
    endrule
endmodule
