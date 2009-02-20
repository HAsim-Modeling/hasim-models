`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

import FIFO::*;

typedef Bit#(`GLOBAL_HIST_SIZE) GLOBAL_HIST;
typedef Bit#(TAdd#(`GLOBAL_HIST_SIZE,`BRANCH_TABLE_SIZE)) BRANCH_TABLE_INDEX;

interface BRANCH_PREDICTOR_ALG;

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);

endinterface


module mkBranchPredAlg
    //interface:
        (BRANCH_PREDICTOR_ALG);

    Reg#(GLOBAL_HIST) GLOBAL_HIST <- mkReg(0);
    LUTRAM#(BRANCH_TABLE_INDEX, Bool) branchTable <- mkLUTRAMU();
    LUTRAM#(TOKEN_INDEX, GLOBAL_HIST) screenShot <- mkLUTRAMU();
    
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        branchTable.upd(truncate({addr, GLOBAL_HIST}), actual);
        screenShot.upd(token.index, GLOBAL_HIST);
        GLOBAL_HIST <= truncate({GLOBAL_HIST, pack(actual)});
    endmethod

    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
        let val = branchTable.sub(truncate({addr, GLOBAL_HIST}));
        respQ.enq(val);
    endmethod

    method ActionValue#(Bool) getPredResp();
    
        respQ.deq();
	return respQ.first();
    
    endmethod
    
    method Action abort(TOKEN token);
        GLOBAL_HIST <= screenShot.sub(token.index);
    endmethod

endmodule
