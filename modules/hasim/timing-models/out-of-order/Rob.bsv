import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;
import fpga_components::*;
import module_local_controller::*;

import RegFile::*;
import Vector::*;

`include "PipelineTypes.bsv"
`include "DebugFile.bsv"

typedef enum {ROB_STATE_WRITEBACK_ALU, ROB_STATE_WRITEBACK_MEM, ROB_STATE_ADD, ROB_STATE_COMMIT_REQ, ROB_STATE_COMMIT_RESP, ROB_STATE_ISSUE_REQ, ROB_STATE_ISSUE_RESP} ROB_STATE deriving (Bits, Eq);

typedef DECODE_BUNDLE ROB_ENTRY;

REWIND_BUNDLE nullRewindBundle = REWIND_BUNDLE{robIndex: 0, prediction: False, mispredict: False, addr: 0, token: ?};

module [HASIM_MODULE] mkRob();
    DebugFile                                                                           debug <- mkDebugFile("Rob.out");

    PORT_BANDWIDTH_CREDIT_SEND#(COMMIT_BUNDLE, `COMMIT_NUM, `COMMIT_NUM)           commitPort <- mkPortBandwidthCreditSend("commit");
    PORT_BANDWIDTH_CREDIT_RECEIVE#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, `ALU_NUM) aluWritebackPort <- mkPortBandwidthCreditReceive("aluWriteback", `ALU_NUM);
    PORT_BANDWIDTH_CREDIT_RECEIVE#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, `MEM_NUM) memWritebackPort <- mkPortBandwidthCreditReceive("memWriteback", `MEM_NUM);
    PORT_BANDWIDTH_CREDIT_RECEIVE#(DECODE_BUNDLE, `DECODE_NUM, DECODE_CREDITS)     decodePort <- mkPortBandwidthCreditReceive("decode", fromInteger(valueOf(DECODE_CREDITS)));
    PORT_BANDWIDTH_CREDIT_SEND#(MEM_BUNDLE, `MEM_NUM, `MEM_CREDITS)                   memPort <- mkPortBandwidthCreditSend("mem");
    PORT_BANDWIDTH_CREDIT_SEND#(ALU_BUNDLE, `ALU_NUM, `ALU_CREDITS)                   aluPort <- mkPortBandwidthCreditSend("alu");

    PORT_SEND#(REWIND_BUNDLE)                                                     resteerPort <- mkPortSend("resteer", nullRewindBundle);

    Connection_Send#(Bool)                                                         modelCycle <- mkConnection_Send("model_cycle");

    BRAM#(`LOG_DECODE_CREDITS, ROB_ENTRY)                                                 rob <- mkBram();

    function prfInit(i) = (i < valueOf(TExp#(SizeOf#(ISA_REG_INDEX))));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool))                   prfValids <- mkReg(genWith(prfInit));

    Reg#(ROB_STATE)                                                                     state <- mkReg(ROB_STATE_WRITEBACK_ALU);

    Reg#(ROB_PTR)                                                                   commitPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                      addPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                    issuePtr <- mkReg(0);

    Reg#(Vector#(DECODE_CREDITS, Bool))                                              robValid <- mkReg(replicate(True));
    Reg#(Vector#(DECODE_CREDITS, Bool))                                              robEpoch <- mkReg(replicate(False));
    Reg#(Bool)                                                                    resteerWait <- mkReg(False);

    RegFile#(ROB_INDEX, Bool)                                                         robDone <- mkRegFileFull();
    RegFile#(ROB_INDEX, Bool)                                                       robIssued <- mkRegFileFull();
    RegFile#(ROB_INDEX, Bool)                                                       terminate <- mkRegFileFull();
    RegFile#(ROB_INDEX, Bool)                                                        passFail <- mkRegFileFull();

    Reg#(REWIND_BUNDLE)                                                          rewindBundle <- mkReg(nullRewindBundle);

    Reg#(Bool)                                                              inorderIssueStall <- mkReg(False);

    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    LocalController                                                           localController <- mkLocalController(inports, outports);

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
                readys[i] = prfValids[src];
            else
                readys[i] = True;
        end
        function ands(x, y) = x && y;
        return fold(ands, readys);
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

    rule writebackAlu(state == ROB_STATE_WRITEBACK_ALU);
        if(aluWritebackPort.canReceive())
        begin
            let bundle <- aluWritebackPort.pop();
            debug <= $format("writebackAlu ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]);
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
            aluWritebackPort.done(`ALU_NUM);
            state <= ROB_STATE_WRITEBACK_MEM;
            debug <= $format("writebackAlu resteer ") + fshow(rewindBundle);
            resteerPort.enq(rewindBundle);
            rewindBundle <= REWIND_BUNDLE{robIndex: 0, prediction: False, mispredict: False, addr: 0, token: ?};
        end
    endrule

    rule writebackMem(state == ROB_STATE_WRITEBACK_MEM);
        if(memWritebackPort.canReceive())
        begin
            let bundle <- memWritebackPort.pop();
            debug <= $format("writebackMem ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]);
            if(robValid[bundle.robIndex])
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);

            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            memWritebackPort.done(`MEM_NUM);
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
                debug <= $format("add ") + fshow(entry) + $format(" addPtr: %d commitPtr: %d credits: %d", addPtr, commitPtr, credits);
                robValid <= markValidOrInvalid(True, robValid, addIndex);
                robIssued.upd(addIndex, False);
                robDone.upd(addIndex, False);
                rob.write(addIndex, entry);
                prfValids <= markPrfValidOrInvalid(False, entry.dsts);
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
        debug <= $format("commitReq: commitPtr: %d robValid: %b robDone: %b robValid[]: %b addPtr: %d credits: %d", commitPtr, robValid, robDone.sub(commitIndex), robValid[commitIndex], addPtr, credits);
        if(!commitPort.canSend() || commitPtr == addPtr || !robDone.sub(commitIndex))
        begin
            commitPort.done();
            state <= ROB_STATE_ISSUE_REQ;
            issuePtr <= commitPtr;
            inorderIssueStall <= False;
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
        let entry <- rob.readResp();
        debug <= $format("commitResp ") + fshow(entry) + $format(" commitPtr: %d robValid: %b addPtr: %d credits: %d", commitPtr, robValid, addPtr, credits);
        commitPort.enq(makeCommitBundle(entry));
        if(terminate.sub(commitIndex))
            localController.endProgram(passFail.sub(commitIndex));
        robValid[commitPtr] <= !resteerWait;
        commitPtr <= commitPtr + 1;
        state <= ROB_STATE_COMMIT_REQ;
    endrule

    rule issueReq(state == ROB_STATE_ISSUE_REQ);
        if((!memPort.canSend() && !aluPort.canSend()) || issuePtr == addPtr || inorderIssueStall)
        begin
            debug.endModelCC();
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
            modelCycle.send(?);
        end
    endrule

    rule issueResp(state == ROB_STATE_ISSUE_RESP);
        let entry <- rob.readResp();
        if(isReady(entry))
        begin
            Bool isMem = isaIsStore(entry.inst) || isaIsLoad(entry.inst);
            if(isMem && memPort.canSend())
            begin
                debug <= $format("issueResp mem") + fshow(entry);
                memPort.enq(makeMemBundle(entry, issuePtr));
                robIssued.upd(issueIndex, True);
            end
            else if(!isMem && aluPort.canSend())
            begin
                debug <= $format("issueResp alu") + fshow(entry);
                aluPort.enq(makeAluBundle(entry, issuePtr));
                robIssued.upd(issueIndex, True);
            end
            else if(`INORDER_ISSUE == 1)
                inorderIssueStall <= True;
        end
        else if(`INORDER_ISSUE == 1)
            inorderIssueStall <= True;
        issuePtr <= issuePtr + 1;
        state <= ROB_STATE_ISSUE_REQ;
    endrule
endmodule
