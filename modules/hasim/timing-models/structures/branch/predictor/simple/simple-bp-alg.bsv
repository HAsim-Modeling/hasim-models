`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

import FIFO::*;

typedef Bit#(`BRANCH_TABLE_SIZE) BRANCH_INDEX;

interface BRANCH_PREDICTOR_ALG;
    method Action upd(ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(ISA_ADDRESS addr);
endinterface

module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTOR_ALG);

    LUTRAM#(BRANCH_INDEX, Bool) branchTable <- mkLUTRAMU();
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(ISA_ADDRESS addr, Bool pred, Bool actual);
        branchTable.upd(truncate(addr), actual);
    endmethod

    method Action getPredReq(ISA_ADDRESS addr);
        respQ.enq(branchTable.sub(truncate(addr)));
    endmethod

    method ActionValue#(Bool) getPredResp();
        respQ.deq();
        return respQ.first();
    endmethod

    method Action abort(ISA_ADDRESS addr);
        noAction;
    endmethod
    
endmodule
