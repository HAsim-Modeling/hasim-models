`include "PipelineTypes.bsv"
`include "hasim_common.bsh"
`include "hasim_isa.bsh"

import FIFOF::*;

interface BRANCH_PREDICTOR;
    method Action readReq(ISA_ADDRESS pc);
    method ActionValue#(BRANCH_BUNDLE) readResp();
    method Action update(ISA_ADDRESS pc, ISA_ADDRESS nextPc, Bool taken, FETCH_INDEX num);
endinterface

module mkBranchPredictor(BRANCH_PREDICTOR);
    TARGET_PREDICTOR       targetPredictor <- mkTargetPredictor;
    DIRECTION_PREDICTOR directionPredictor <- mkDirectionPredictor;

    method Action readReq(ISA_ADDRESS pc);
        targetPredictor.readReq(pc);
        directionPredictor.readReq(pc);
    endmethod

    method ActionValue#(BRANCH_BUNDLE) readResp();
        let target <- targetPredictor.readResp;
        let direction <- directionPredictor.readResp;
        if(target.valid && direction.taken)
            return makeBranchBundle(direction.num, target.addr, True);
        else
            return makeBranchBundle(`FETCH_NUM, target.addr, False);
    endmethod

    method Action update(ISA_ADDRESS pc, ISA_ADDRESS nextPc, Bool taken, FETCH_INDEX num);
        targetPredictor.update(pc, nextPc);
        directionPredictor.update(pc, taken, num);
    endmethod
endmodule

typedef struct {
    Bool valid;
    ISA_ADDRESS addr;
} TARGET_BUNDLE deriving (Bits, Eq);

function TARGET_BUNDLE makeTargetBundle(Bool valid, ISA_ADDRESS addr);
    return TARGET_BUNDLE{valid: valid, addr: addr};
endfunction

typedef struct {
    Bool taken;
    FETCH_INDEX num;
} DIRECTION_BUNDLE deriving (Bits, Eq);

function DIRECTION_BUNDLE makeDirectionBundle(Bool taken, FETCH_INDEX num);
    return DIRECTION_BUNDLE{taken: taken, num: num};
endfunction

interface TARGET_PREDICTOR;
    method Action readReq(ISA_ADDRESS pc);
    method ActionValue#(TARGET_BUNDLE) readResp();
    method Action update(ISA_ADDRESS pc, ISA_ADDRESS nextPc);
endinterface

interface DIRECTION_PREDICTOR;
    method Action readReq(ISA_ADDRESS pc);
    method ActionValue#(DIRECTION_BUNDLE) readResp();
    method Action update(ISA_ADDRESS pc, Bool taken, FETCH_INDEX num);
endinterface

module mkTargetPredictor(TARGET_PREDICTOR);
    FIFOF#(ISA_ADDRESS) pcFifo <- mkFIFOF();

    method Action readReq(ISA_ADDRESS pc);
        pcFifo.enq(pc);
    endmethod

    method ActionValue#(TARGET_BUNDLE) readResp();
        pcFifo.deq;
        return makeTargetBundle(False, pcFifo.first + `FETCH_NUM * 4);
    endmethod

    method Action update(ISA_ADDRESS pc, ISA_ADDRESS nextPc);
    endmethod
endmodule

module mkDirectionPredictor(DIRECTION_PREDICTOR);
    FIFOF#(ISA_ADDRESS) pcFifo <- mkFIFOF();

    method Action readReq(ISA_ADDRESS pc);
        pcFifo.enq(pc);
    endmethod

    method ActionValue#(DIRECTION_BUNDLE) readResp();
        pcFifo.deq;
        return makeDirectionBundle(False, ?);
    endmethod

    method Action update(ISA_ADDRESS pc, Bool taken, FETCH_INDEX num);
    endmethod
endmodule
