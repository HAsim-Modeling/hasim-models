import hasim_common::*;
import hasim_isa::*;

import RegFile::*;
import FIFO::*;

typedef Bit#(`BRANCH_TABLE_SIZE) BranchIndex;

interface BranchPred;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface

module mkBranchPred(BranchPred);

    RegFile#(BranchIndex, Bool) branchRegFile <- mkRegFileFull();
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        branchRegFile.upd(truncate(addr), actual);
    endmethod

    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
        respQ.enq(branchRegFile.sub(truncate(addr)));
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return respQ.first();
    endmethod

    method Action abort(TOKEN token);
        noAction;
    endmethod
    
endmodule
