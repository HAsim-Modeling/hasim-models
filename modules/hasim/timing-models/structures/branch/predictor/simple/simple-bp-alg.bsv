`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

import FIFO::*;

typedef Bit#(`BRANCH_TABLE_SIZE) BRANCH_INDEX;

interface BRANCH_PREDICTOR_ALG;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface

module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTOR_ALG);

    LUTRAM#(BRANCH_INDEX, Bool) branchTable <- mkLUTRAMU();
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        branchTable.upd(truncate(addr), actual);
    endmethod

    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
        respQ.enq(branchTable.sub(truncate(addr)));
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return respQ.first();
    endmethod

    method Action abort(TOKEN token);
        noAction;
    endmethod
    
endmodule
