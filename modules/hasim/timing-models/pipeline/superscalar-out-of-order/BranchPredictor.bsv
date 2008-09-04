`include "PipelineTypes.bsv"
`include "hasim_common.bsh"
`include "hasim_isa.bsh"

import FIFOF::*;

interface BRANCH_PREDICTOR;
    method Action readReq(ISA_ADDRESS pc);
    method ActionValue#(BRANCH_BUNDLE) readResp();
    method Action update(ISA_ADDRESS pc, ISA_ADDRESS nextPc);
endinterface

module mkBranchPredictor(BRANCH_PREDICTOR);
    FIFOF#(ISA_ADDRESS) pc <- mkFIFOF();

    method Action readReq(ISA_ADDRESS _pc);
        pc.enq(_pc);
    endmethod

    method ActionValue#(BRANCH_BUNDLE) readResp();
        pc.deq;
        return makeBranchBundle(`FETCH_NUM, pc.first + `FETCH_NUM*4);
    endmethod

    method Action update(ISA_ADDRESS _pc, ISA_ADDRESS nextPc);
    endmethod
endmodule
