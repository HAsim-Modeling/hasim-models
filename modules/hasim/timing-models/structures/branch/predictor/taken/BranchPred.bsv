import FIFO::*;

import hasim_common::*;
import hasim_isa::*;

interface BranchPred;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface

module mkBranchPred(BranchPred);

    FIFO#(Bit#(0)) respQ <- mkFIFO();
    
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        noAction;
    endmethod

    method Action  getPredReq(TOKEN token, ISA_ADDRESS addr);
        respQ.enq(?);
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return True;
    endmethod
    
    method Action abort(TOKEN token);
        noAction;
    endmethod
endmodule
