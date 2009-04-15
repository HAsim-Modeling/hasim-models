import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_isa.bsh"

interface BRANCH_PREDICTOR_ALG;
    method Action upd(ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(ISA_ADDRESS addr);
endinterface

module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTOR_ALG);

    FIFO#(VOID) respQ <- mkFIFO();
    
    method Action upd(ISA_ADDRESS addr, Bool pred, Bool actual);
        noAction;
    endmethod

    method Action  getPredReq(ISA_ADDRESS addr);
        respQ.enq(?);
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return True;
    endmethod
    
    method Action abort(ISA_ADDRESS addr);
        noAction;
    endmethod
endmodule
