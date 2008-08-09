import hasim_common::*;
import hasim_isa::*;

import RegFile::*;
import FIFO::*;

typedef Bit#(`GLOBAL_HIST_SIZE) GlobalHist;
typedef Bit#(TAdd#(`GLOBAL_HIST_SIZE,`BRANCH_TABLE_SIZE)) BranchTableIndex;

interface BranchPred;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface


module mkBranchPred(BranchPred);
    Reg#(GlobalHist) globalHist <- mkReg(0);
    RegFile#(BranchTableIndex, Bool) branchTable <- mkRegFileFull();
    RegFile#(TokIndex, GlobalHist) screenShot <- mkRegFileFull();
    
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        branchTable.upd(truncate({addr, globalHist}), actual);
        screenShot.upd(token.index, globalHist);
        globalHist <= truncate({globalHist, pack(actual)});
    endmethod

    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
        let val = branchTable.sub(truncate({addr, globalHist}));
        respQ.enq(val);
    endmethod

    method ActionValue#(Bool) getPredResp();
    
        respQ.deq();
	return respQ.first();
    
    endmethod
    
    method Action abort(TOKEN token);
        globalHist <= screenShot.sub(token.index);
    endmethod
endmodule
