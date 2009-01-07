import Vector::*;
import Counter::*;
import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"
`include "module_local_controller.bsh"
`include "fpga_components.bsh"

`include "hasim_pipeline_types.bsh"

typedef Bit#(3) TIME_STAMP;
typedef enum {ALU, MEM} WRITE_TYPE deriving (Bits, Eq);

typedef DECODE_BUNDLE ROB_ENTRY;

REWIND_BUNDLE nullRewindBundle = REWIND_BUNDLE{robIndex: 0, mispredict: False, addr: 0, token: ?};

module [HASIM_MODULE] mkIssue();
    TIMEP_DEBUG_FILE                                                            debugLog1 <- mkTIMEPDebugFile("pipe_issue1.out");
    TIMEP_DEBUG_FILE                                                            debugLog2 <- mkTIMEPDebugFile("pipe_issue2.out");

    PORT_CREDIT_SEND#(COMMIT_BUNDLE, `COMMIT_NUM, LOG_COMMIT_NUM)              commitPort <- mkPortCreditSend("commit");
    PORT_FIFO_RECEIVE#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)  aluWritebackPort <- mkPortFifoReceive("aluWriteback", True, `ALU_CREDITS);
    PORT_FIFO_RECEIVE#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)  memWritebackPort <- mkPortFifoReceive("memWriteback", True, `MEM_CREDITS);
    PORT_FIFO_RECEIVE#(DECODE_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)         decodePort <- mkPortFifoReceive("decode", True, valueOf(DECODE_CREDITS));
    PORT_CREDIT_SEND#(MEM_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)                      memPort <- mkPortCreditSend("mem");
    PORT_CREDIT_SEND#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                      aluPort <- mkPortCreditSend("alu");
    PORT_CREDIT_SEND#(PREDICT_UPDATE_BUNDLE, `ALU_NUM, LOG_ALU_NUM)     predictUpdatePort <- mkPortCreditSend("predictUpdate");
    PORT_CREDIT_SEND#(REWIND_BUNDLE, 1, 1)                                    resteerPort <- mkPortCreditSend("resteer");
    PORT_CREDIT_SEND#(COMMIT_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)         aluWriteback2Send <- mkPortCreditSend("aluWriteback2");
    PORT_FIFO_RECEIVE#(COMMIT_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)        aluWriteback2Recv <- mkPortFifoReceive("aluWriteback2", True, `ALU_CREDITS);

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)                                  modelCycle <- mkConnection_Send("model_cycle");

    function prfInit(i) = (i < valueOf(TExp#(SizeOf#(ISA_REG_INDEX))));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool))               prfValids <- mkReg(genWith(prfInit));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP))    writeTimeStamp <- mkReg(replicate(1));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE))) writeType <- mkReg(replicate(tagged Valid ALU));
    Reg#(TIME_STAMP)                                                            timeStamp <- mkReg(0);

    Reg#(REWIND_BUNDLE)                                                      rewindBundle <- mkReg(nullRewindBundle);

    Reg#(Bool)                                                                resteerWait <- mkReg(False);
    Reg#(Bit#(TLog#(TAdd#(1, SizeOf#(TIME_STAMP)))))                            numIssued <- mkReg(0);

    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    LocalController                                                       localController <- mkLocalController(inports, outports);

    function Bool isReady(ROB_ENTRY entry);
        Vector#(ISA_MAX_SRCS, Bool) readys = newVector();
        for(Integer i = 0; i < valueOf(ISA_MAX_SRCS); i = i + 1)
        begin
            if(entry.srcs[i] matches tagged Valid .src)
                readys[i] = (isValid(writeType[src]) && (validValue(writeType[src]) == ALU && (timeStamp - writeTimeStamp[src]) > 1) || prfValids[src]);
            else
                readys[i] = True;
        end
        return fold(\&& , readys) && (!entry.drainBefore || numIssued == 0);
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

    rule commit;
        if(commitPort.canSend)
        begin
            debugLog2.record($format("try commit"));
            if(aluWriteback2Recv.canReceive)
            begin
                numIssued <= numIssued - 1;
                aluWriteback2Recv.deq;
                commitPort.enq(COMMIT_BUNDLE{token: aluWriteback2Recv.first.token, isStore: False});
                debugLog2.record($format("commitAlu ") + fshow(aluWriteback2Recv.first));
            end
            else if(memWritebackPort.canReceive)
            begin
                numIssued <= numIssued - 1;
                let bundle = memWritebackPort.first;
                memWritebackPort.deq;
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);
                commitPort.enq(COMMIT_BUNDLE{token: bundle.token, isStore: bundle.isStore});
                debugLog2.record($format("commitMem ") + fshow(bundle));
            end
        end
        else
        begin
            debugLog2.record($format("end cycle"));
            debugLog2.nextModelCycle;
            modelCycle.send(0);
            commitPort.done;
            aluWriteback2Recv.done;
            memWritebackPort.done;
        end
    endrule

    rule writebackAlu;
        if(aluWritebackPort.canReceive())
        begin
            let bundle = aluWritebackPort.first();
            aluWritebackPort.deq;
            debugLog1.record($format("writebackAlu ") + fshow(bundle));
            predictUpdatePort.enq(makePredictUpdateBundle(bundle));
            prfValids <= markPrfValidOrInvalid(True, bundle.dsts);
            if(!resteerWait)
            begin
                debugLog1.record($format("writeback done"));
                aluWriteback2Send.enq(COMMIT_BUNDLE{token: bundle.token, isStore: False});
                if(bundle.terminate)
                begin
                    localController.endProgram(bundle.passFail);
                    debugLog1.record($format("terminate ") + fshow(bundle.passFail));
                end
                if(bundle.mispredict)
                begin
                    rewindBundle <= makeRewindBundle(bundle);
                    debugLog1.record($format("rewind ") + fshow(bundle));
                    resteerWait <= True;
                end
            end
        end
        else
        begin
            predictUpdatePort.done;
            aluWritebackPort.done;
            aluWriteback2Send.done;
            debugLog1.record($format("writebackAlu resteer ") + fshow(rewindBundle));
            resteerPort.send(rewindBundle);
            rewindBundle <= nullRewindBundle;
        end
    endrule

    rule issue;
        if(decodePort.canReceive)
        begin
            let entry = decodePort.first;
            if(resteerWait && !entry.afterResteer)
            begin
                decodePort.deq;
                debugLog1.record($format("drop"));
            end
            else if(isReady(entry))
            begin
                Bool isMem = isaIsStore(entry.inst) || isaIsLoad(entry.inst);
                debugLog1.record($format("ready ") + fshow(entry));
                if(isMem && memPort.canSend)
                begin
                    numIssued <= numIssued + 1;
                    decodePort.deq;
                    debugLog1.record($format("issueResp mem") + fshow(entry));
                    memPort.enq(makeMemBundle(entry, ?));
                    markWriteType(tagged Valid MEM, entry.dsts);
                end
                else if(!isMem && aluPort.canSend)
                begin
                    numIssued <= numIssued + 1;
                    decodePort.deq;
                    debugLog1.record($format("issueResp alu") + fshow(entry));
                    aluPort.enq(makeAluBundle(entry, 0));
                    markWriteType(tagged Valid ALU, entry.dsts);
                end
            end
        end
        else
        begin
            debugLog1.record($format("end cycle"));
            modelCycle.send(0);
            decodePort.done;
            aluPort.done;
            memPort.done;
            debugLog1.nextModelCycle;
        end
    endrule
endmodule
