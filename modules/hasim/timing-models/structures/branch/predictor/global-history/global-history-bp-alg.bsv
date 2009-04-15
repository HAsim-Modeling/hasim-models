`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

import FIFO::*;

typedef Bit#(`GLOBAL_HIST_SIZE) GLOBAL_HIST;
typedef Bit#(TAdd#(`GLOBAL_HIST_SIZE,`BRANCH_TABLE_SIZE)) BRANCH_TABLE_INDEX;
typedef Bit#(TSub#(ISA_ADDRESS_SIZE, TAdd#(`GLOBAL_HIST_SIZE,`BRANCH_TABLE_SIZE))) BRANCH_TABLE_TAG;

function BRANCH_TABLE_INDEX getAddrIdx(ISA_ADDRESS addr);
      return truncate(hashBits(addr[31:2]));
endfunction

function BRANCH_TAG getAddrTag(ISA_ADDRESS addr);

    return truncate(reverse(addr));

endfunction
    
interface BRANCH_PREDICTOR_ALG;

    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);

endinterface


module mkBranchPredAlg
    //interface:
        (BRANCH_PREDICTOR_ALG);

    Reg#(GLOBAL_HIST) globalHist <- mkReg(0);
    LUTRAM#(BRANCH_TABLE_INDEX, Bool) branchTable <- mkLUTRAMU();
    LUTRAM#(ADDR_INDEX, Tuple2#(GLOBAL_HIST, ADDR_TAG)) screenShot <- mkLUTRAMU();
    
    FIFO#(Bool) respQ <- mkFIFO();

    method Action upd(ISA_ADDRESS addr, Bool pred, Bool actual);
        branchTable.upd(truncate({addr, globalHist}), actual);
        screenShot.upd(getIdx(addr), tuple2(globalHist, getTag(addr));
        globalHist <= truncate({globalHist, pack(actual)});
    endmethod

    method Action getPredReq(ISA_ADDRESS addr);
        let val = branchTable.sub(truncate({addr, globalHist}));
        respQ.enq(val);
    endmethod

    method ActionValue#(Bool) getPredResp();
    
        respQ.deq();
	return respQ.first();
    
    endmethod
    
    method Action abort(ISA_ADDRESS addr);
        let idx = getAddrIdx(addr);
        match {.snap, .tag} = screenShot.sub(idx);

        if (tag == getAddrTag(addr))
        begin
            globalHist <= snap;
        end

    endmethod

endmodule
