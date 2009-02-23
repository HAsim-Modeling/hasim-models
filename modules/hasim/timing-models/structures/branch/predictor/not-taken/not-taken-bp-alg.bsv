import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_isa.bsh"

interface BRANCH_PREDICTION_ALG;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface

module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTION_ALG);

    FIFO#(VOID) respQ <- mkFIFO();
    
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        noAction;
    endmethod

    method Action  getPredReq(TOKEN token, ISA_ADDRESS addr);
        respQ.enq(?);
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return False;
    endmethod
    
    method Action abort(TOKEN token);
        noAction;
    endmethod

endmodule
