`ifndef PIPELINE_TYPES_BSV
`define PIPELINE_TYPE_BSV
import FShow::*;
import Vector::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"

typedef TLog#(TAdd#(`FETCH_CREDITS, 1)) LOG_FETCH_CREDITS;
typedef TAdd#(`ROB_INDEX_SIZE, 1) LOG_DECODE_CREDITS;
typedef TLog#(TAdd#(`ALU_CREDITS, 1)) LOG_ALU_CREDITS;
typedef TLog#(TAdd#(`ALU_NUM, 1)) LOG_ALU_NUM;
typedef TLog#(TAdd#(`MEM_CREDITS, 1)) LOG_MEM_CREDITS;
typedef TLog#(TAdd#(`MEM_NUM, 1)) LOG_MEM_NUM;
typedef TLog#(TAdd#(`COMMIT_NUM, 1)) LOG_COMMIT_NUM;

typedef TAdd#(`ALU_NUM, `MEM_NUM) WRITEBACK_NUM;
typedef Bit#(TLog#(TAdd#(WRITEBACK_NUM, 1))) WRITEBACK_INDEX;
typedef Bit#(TLog#(TAdd#(ISA_MAX_DSTS, 1))) ISA_DEST_REGS_INDEX;
typedef Bit#(LOG_FETCH_CREDITS) FETCH_BUFFER_INDEX;
typedef TExp#(`ROB_INDEX_SIZE) DECODE_CREDITS;

typedef Bit#(`ROB_INDEX_SIZE) ROB_INDEX;
typedef Bit#(LOG_DECODE_CREDITS) ROB_PTR;

typedef enum {PRED_TYPE_BRANCH_IMM, PRED_TYPE_BRANCH_REG, PRED_TYPE_JUMP_IMM, PRED_TYPE_JUMP_REG, PRED_TYPE_RET, PRED_TYPE_TARGET, PRED_TYPE_NONE} PRED_TYPE deriving (Bits, Eq); 

typedef struct {
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    PRED_TYPE predType;
    Bool prediction;
    ISA_ADDRESS predPc;
    Bool afterResteer;
    ROB_INDEX epochRob;
} FETCH_BUNDLE deriving (Bits, Eq);

instance FShow#(FETCH_BUNDLE);
    function Fmt fshow(FETCH_BUNDLE b);
        return $format("FETCH: pc: 0x%x predType: %d prediction: %b predPc: 0x%x afterResteer: %b resteerEpoch: %d ", b.pc, b.predType, b.prediction, b.predPc, b.afterResteer, b.epochRob);
    endfunction
endinstance

typedef struct {
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    PRED_TYPE predType;
    Bool prediction;
    ISA_ADDRESS predPc;
    Bool afterResteer;
    ROB_INDEX epochRob;
    Vector#(ISA_MAX_SRCS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) srcs;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    Bool drainBefore;
    Bool drainAfter;
    TOKEN token;
} DECODE_BUNDLE deriving (Bits, Eq);

instance FShow#(DECODE_BUNDLE);
    function Fmt fshow(DECODE_BUNDLE b);
        function Fmt combine(Fmt x, Fmt y);
            return x + $format(" ") + y;
        endfunction
        Vector#(ISA_MAX_SRCS, Fmt) srcFmt = newVector();
        Vector#(ISA_MAX_DSTS, Fmt) dstFmt = newVector();
        for(Integer i = 0; i < valueOf(ISA_MAX_SRCS); i = i + 1)
        begin
            if(b.srcs[i] matches tagged Valid .src)
                srcFmt[i] = $format("1.%d", src);
            else
                srcFmt[i] = $format("0.  0");
        end
        Fmt srcFmts = foldl1(combine, srcFmt);
        for(Integer i = 0; i < valueOf(ISA_MAX_DSTS); i = i + 1)
        begin
            if(b.dsts[i] matches tagged Valid .dst)
                dstFmt[i] = $format("1.%d", dst);
            else
                dstFmt[i] = $format("0.  0");
        end
        Fmt dstFmts = foldl1(combine, dstFmt);
        return $format("DECODE: pc: 0x%x srcs: ", b.pc) + srcFmts + $format(" dsts: ") + dstFmts + $format(" ") + fshow(b.token);
    endfunction
endinstance

function DECODE_BUNDLE makeDecodeBundle(TOKEN tok, FETCH_BUNDLE fetch, Vector#(ISA_MAX_SRCS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) srcs, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts);
    return DECODE_BUNDLE{inst: fetch.inst,
                         pc: fetch.pc,
                         predType: fetch.predType,
                         prediction: fetch.prediction,
                         predPc: fetch.predPc,
                         afterResteer: fetch.afterResteer,
                         epochRob: fetch.epochRob,
                         drainBefore: isaDrainBefore(fetch.inst),
                         drainAfter: isaDrainAfter(fetch.inst),
                         srcs: srcs,
                         dsts: dsts,
                         token: tok};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    PRED_TYPE predType;
    Bool prediction;
    ISA_ADDRESS predPc;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    Bool drainBefore;
    Bool drainAfter;
    TOKEN token;
} ALU_BUNDLE deriving (Bits, Eq);

instance FShow#(ALU_BUNDLE);
    function Fmt fshow(ALU_BUNDLE b);
        return $format("ALU: robIndex: %d ", b.robIndex) + fshow(b.token);
    endfunction
endinstance

function ALU_BUNDLE makeAluBundle(DECODE_BUNDLE decode, ROB_PTR robPtr);
    return ALU_BUNDLE{robIndex: truncate(robPtr),
                      inst: decode.inst,
                      predType: decode.predType,
                      pc: decode.pc,
                      predPc: decode.predPc,
                      prediction: decode.prediction,
                      dsts: decode.dsts,
                      drainBefore: decode.drainBefore,
                      drainAfter: decode.drainAfter,
                      token: decode.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    ISA_INSTRUCTION inst;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    TOKEN token;
} MEM_BUNDLE deriving (Bits, Eq);

instance FShow#(MEM_BUNDLE);
    function Fmt fshow(MEM_BUNDLE b);
        return $format("MEM: robIndex: %d ", b.robIndex) + fshow(b.token);
    endfunction
endinstance

function MEM_BUNDLE makeMemBundle(DECODE_BUNDLE decode, ROB_PTR robPtr);
    return MEM_BUNDLE{robIndex: truncate(robPtr),
                      inst: decode.inst,
                      dsts: decode.dsts,
                      token: decode.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    ISA_ADDRESS pc;
    PRED_TYPE predType;
    Bool prediction;
    ISA_ADDRESS predPc;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    Bool mispredict;
    ISA_ADDRESS addr;
    Bool terminate;
    Bool passFail;
    TOKEN token;
} ALU_WRITEBACK_BUNDLE deriving (Bits, Eq);

instance FShow#(ALU_WRITEBACK_BUNDLE);
    function Fmt fshow(ALU_WRITEBACK_BUNDLE b);
        return $format("ALU_WRITEBACK: robIndex: %d mispredict: %b newAddr: 0x%x terminate: %b passFail: %b ", b.robIndex, b.mispredict, b.addr, b.terminate, b.passFail) + fshow(b.token);
    endfunction
endinstance

function ALU_WRITEBACK_BUNDLE makeAluWritebackBundle(ALU_BUNDLE alu, FUNCP_RSP_GET_RESULTS res);
    Bool mispredict;
    ISA_ADDRESS addr;
    Bool terminate;
    Bool passFail;
    case (res.result) matches
        tagged RBranchTaken .address:
        begin
            mispredict = !alu.prediction || alu.drainAfter;
            addr = address;
            terminate = False;
            passFail = False;
        end
        tagged RBranchNotTaken .address:
        begin
            mispredict = alu.prediction || alu.drainAfter;
            addr = address;
            terminate = False;
            passFail = False;
        end
        tagged RTerminate .pf:
        begin
            mispredict = False;
            addr = 0;
            terminate = True;
            passFail = pf;
        end
        default:
        begin
            mispredict = alu.drainAfter;
            addr = res.instructionAddress + zeroExtend(res.instructionSize);
            terminate = False;
            passFail = False;
        end
    endcase
    return ALU_WRITEBACK_BUNDLE{robIndex: alu.robIndex,
                                pc: alu.pc,
                                predType: alu.predType,
                                prediction: alu.prediction,
                                predPc: alu.predPc,
                                dsts: alu.dsts,
                                mispredict: mispredict,
                                addr: addr,
                                terminate: terminate,
                                passFail: passFail,
                                token: res.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    ISA_INSTRUCTION inst;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    ISA_ADDRESS addr;
    TOKEN token;
} MEM_ADDRESS_BUNDLE deriving (Bits, Eq);

instance FShow#(MEM_ADDRESS_BUNDLE);
    function Fmt fshow(MEM_ADDRESS_BUNDLE b);
        return $format("MEM_ADDRESS: robIndex: %d", b.robIndex) + fshow(b.token);
    endfunction
endinstance

function MEM_ADDRESS_BUNDLE makeMemAddressBundle(MEM_BUNDLE mem, FUNCP_RSP_GET_RESULTS res);
    ISA_ADDRESS addr;
    if(res.result matches tagged REffectiveAddr .ea)
        addr = ea;
    else
        addr = 0;
    return MEM_ADDRESS_BUNDLE{robIndex: mem.robIndex,
                              inst: mem.inst,
                              dsts: mem.dsts,
                              addr: addr,
                              token: mem.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
    Bool isStore;
    TOKEN token;
} MEM_WRITEBACK_BUNDLE deriving (Bits, Eq);

instance FShow#(MEM_WRITEBACK_BUNDLE);
    function Fmt fshow(MEM_WRITEBACK_BUNDLE b);
        return $format("MEM_WRITEBACK: robIndex: %d ", b.robIndex) + fshow(b.token);
    endfunction
endinstance

function MEM_WRITEBACK_BUNDLE makeMemWritebackBundle(MEM_ADDRESS_BUNDLE mem, TOKEN tok);
    return MEM_WRITEBACK_BUNDLE{robIndex: mem.robIndex,
                                dsts: mem.dsts,
                                isStore: isaIsStore(mem.inst),
                                token: tok};
endfunction

typedef struct {
    PRED_TYPE predType;
    Bool pred;
    Bool actual;
    ISA_ADDRESS predPc;
    ISA_ADDRESS pc;
    ISA_ADDRESS actualAddr;
    TOKEN token;
} PREDICT_UPDATE_BUNDLE deriving (Bits, Eq);

instance FShow#(PREDICT_UPDATE_BUNDLE);
    function Fmt fshow(PREDICT_UPDATE_BUNDLE b);
        return $format("predType: %d pred: %b actual: %b predPc: 0x%x actualAddr: 0x%x pc: 0x%x", b.predType, b.pred, b.actual, b.predPc, b.actualAddr, b.pc) + fshow(b.token);
    endfunction
endinstance

function PREDICT_UPDATE_BUNDLE makePredictUpdateBundle(ALU_WRITEBACK_BUNDLE alu);
    return PREDICT_UPDATE_BUNDLE{predType: alu.predType,
                                 pred: alu.prediction,
                                 actual: alu.mispredict? !alu.prediction: alu.prediction,
                                 predPc: alu.predPc,
                                 actualAddr: alu.addr,
                                 pc: alu.pc,
                                 token: alu.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    Bool mispredict;
    ISA_ADDRESS addr;
    TOKEN token;
} REWIND_BUNDLE deriving (Bits, Eq);

instance FShow#(REWIND_BUNDLE);
    function Fmt fshow(REWIND_BUNDLE b);
        return $format("REWIND: robIndex: %d mispredict: %b addr: 0x%x ", b.robIndex, b.mispredict, b.addr) + fshow(b.token);
    endfunction
endinstance

function REWIND_BUNDLE makeRewindBundle(ALU_WRITEBACK_BUNDLE alu);
    return REWIND_BUNDLE{robIndex: alu.robIndex,
                         mispredict: alu.mispredict,
                         addr: alu.addr,
                         token: alu.token};
endfunction

typedef struct {
    TOKEN token;
    ROB_INDEX epochRob;
    Bool isStore;
} COMMIT_BUNDLE deriving (Bits, Eq);

instance FShow#(COMMIT_BUNDLE);
    function Fmt fshow(COMMIT_BUNDLE b);
        return $format("COMMIT: isStore: %b ", b.isStore) + fshow(b.token);
    endfunction
endinstance

function COMMIT_BUNDLE makeCommitBundle(DECODE_BUNDLE decode, ROB_INDEX epoch_rob);
    return COMMIT_BUNDLE{token: decode.token,
                         epochRob: epoch_rob,
                         isStore: isaIsStore(decode.inst)
                        };
endfunction

//
// FAULT bundle (from COMMIT bundle)
//
typedef struct {
    Bool fault;
    ROB_INDEX robIndex;
    TOKEN token;
    ISA_ADDRESS nextInstructionAddress;
} FAULT_BUNDLE deriving (Bits, Eq);

instance FShow#(FAULT_BUNDLE);
    function Fmt fshow(FAULT_BUNDLE b);
        return $format("FAULT: fault: %d robIndex: %d ", b.fault, b.robIndex) + fshow(b.token);
    endfunction
endinstance

function FAULT_BUNDLE makeFaultBundle(TOKEN tok, Bool fault, ROB_INDEX rob_index, ISA_ADDRESS next_addr);
    return FAULT_BUNDLE{fault: fault,
                        robIndex: rob_index,
                        nextInstructionAddress: next_addr,
                        token: tok};
endfunction

function FAULT_BUNDLE makeNoFaultBundle();
    return FAULT_BUNDLE{fault: False,
                        nextInstructionAddress: ?,
                        robIndex: ?,
                        token: ?};
endfunction

function Vector#(n, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) extractPhysReg(Vector#(n, Maybe#(Tuple2#(ISA_REG_INDEX, FUNCP_PHYSICAL_REG_INDEX))) regMap);
    Vector#(n, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs = newVector();
    for(Integer i = 0; i < valueOf(n); i = i + 1)
    begin
        case (regMap[i]) matches
            tagged Valid {.ar, .pr}: regs[i] = tagged Valid pr;
            tagged Invalid: regs[i] = tagged Invalid;
        endcase
    end
    return regs;
endfunction
`endif
