`ifndef PIPELINE_TYPES_BSV
`define PIPELINE_TYPES_BSV

import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

import Vector::*;

typedef TAdd#(`ALU_NUM, `MEM_NUM) WRITEBACK_NUM;
typedef Bit#(TLog#(TAdd#(WRITEBACK_NUM, 1))) WRITEBACK_INDEX;
typedef Bit#(TLog#(TAdd#(ISA_MAX_DSTS, 1))) ISA_DEST_REGS_INDEX;
typedef Bit#(TLog#(TAdd#(`FETCH_CREDITS, 1))) FETCH_BUFFER_INDEX;
typedef TExp#(`LOG_DECODE_CREDITS) DECODE_CREDITS;

typedef Bit#(`LOG_DECODE_CREDITS) ROB_INDEX;
typedef Bit#(TAdd#(`LOG_DECODE_CREDITS, 1)) ROB_PTR;

instance FShow#(TOKEN);
    function Fmt fshow(TOKEN tok);
        return $format("TOKEN: %d", tok.index);
    endfunction
endinstance

typedef struct {
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    Bool prediction;
    Bool afterResteer;
    ROB_INDEX epochRob;
    TOKEN token;
} FETCH_BUNDLE deriving (Bits, Eq);

instance FShow#(FETCH_BUNDLE);
    function Fmt fshow(FETCH_BUNDLE b);
        return $format("FETCH: pc: 0x%x prediction: %b afterResteer: %b resteerEpoch: %d ", b.pc, b.prediction, b.afterResteer, b.epochRob) + fshow(b.token);
    endfunction
endinstance

typedef struct {
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    Bool prediction;
    Bool afterResteer;
    ROB_INDEX epochRob;
    Vector#(ISA_MAX_SRCS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) srcs;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
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

function DECODE_BUNDLE makeDecodeBundle(FETCH_BUNDLE fetch, Vector#(ISA_MAX_SRCS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) srcs, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts);
    return DECODE_BUNDLE{inst: fetch.inst,
                         pc: fetch.pc,
                         prediction: fetch.prediction,
                         afterResteer: fetch.afterResteer,
                         epochRob: fetch.epochRob,
                         srcs: srcs,
                         dsts: dsts,
                         token: fetch.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    ISA_INSTRUCTION inst;
    ISA_ADDRESS pc;
    Bool prediction;
    Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dsts;
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
                      pc: decode.pc,
                      prediction: decode.prediction,
                      dsts: decode.dsts,
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
    Bool prediction;
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
            mispredict = !alu.prediction;
            addr = address;
            terminate = False;
            passFail = False;
        end
        tagged RBranchNotTaken .address:
        begin
            mispredict = alu.prediction;
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
            mispredict = False;
            addr = 0;
            terminate = False;
            passFail = False;
        end
    endcase
    return ALU_WRITEBACK_BUNDLE{robIndex: alu.robIndex,
                                prediction: alu.prediction,
                                dsts: alu.dsts,
                                mispredict: mispredict,
                                addr: addr,
                                terminate: terminate,
                                passFail: passFail,
                                token: alu.token};
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
    TOKEN token;
} MEM_WRITEBACK_BUNDLE deriving (Bits, Eq);

instance FShow#(MEM_WRITEBACK_BUNDLE);
    function Fmt fshow(MEM_WRITEBACK_BUNDLE b);
        return $format("MEM_WRITEBACK: robIndex: %d ", b.robIndex) + fshow(b.token);
    endfunction
endinstance

function MEM_WRITEBACK_BUNDLE makeMemWritebackBundle(MEM_ADDRESS_BUNDLE mem);
    return MEM_WRITEBACK_BUNDLE{robIndex: mem.robIndex,
                                dsts: mem.dsts,
                                token: mem.token};
endfunction

typedef struct {
    ROB_INDEX robIndex;
    Bool prediction;
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
                         prediction: alu.prediction,
                         mispredict: alu.mispredict,
                         addr: alu.addr,
                         token: alu.token};
endfunction

typedef struct {
    TOKEN token;
    Bool isStore;
} COMMIT_BUNDLE deriving (Bits, Eq);

instance FShow#(COMMIT_BUNDLE);
    function Fmt fshow(COMMIT_BUNDLE b);
        return $format("COMMIT: isStore: % b", b.isStore) + fshow(b.token);
    endfunction
endinstance

function COMMIT_BUNDLE makeCommitBundle(DECODE_BUNDLE decode);
    return COMMIT_BUNDLE{token: decode.token,
                         isStore: isaIsStore(decode.inst)
                        };
endfunction
`endif