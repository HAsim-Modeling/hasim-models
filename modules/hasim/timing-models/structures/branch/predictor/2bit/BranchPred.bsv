import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/hasim_isa.bsh"

typedef enum
{
    BPRED_PATH_UPDATE,
    BPRED_PATH_PREDICT
}
    BPRED_PATH
        deriving (Eq, Bits);


typedef Bit#(`BRANCH_TABLE_SIZE) BranchIndex;

interface BranchPred;
    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredResp();
    method Action abort(TOKEN token);
endinterface


module mkBranchPred(BranchPred);

    BRAM#(`BRANCH_TABLE_SIZE, Bit#(2)) branchPredTable <- mkBramInitialized(1);
    FIFO#(Tuple2#(BranchIndex, Bool)) respQ <- mkFIFO();

    // This queue records where prediction table responses should go
    FIFO#(BPRED_PATH) bpredPathQ <- mkSizedFIFO(16);

    function BranchIndex getIdx(ISA_ADDRESS addr);
        return truncate(hashTo32(addr[31:2]));
    endfunction

    //
    // updatePred -- receive old counter values and updates from the upd()
    //               method.  Update the prediction table.
    //
    rule updatePred (bpredPathQ.first() == BPRED_PATH_UPDATE);

        let counter <- branchPredTable.readResp();
        match { .idx, .actual } = respQ.first();
        respQ.deq();
        bpredPathQ.deq();

        let newCounter = 0;
        if (actual)
            newCounter = (counter == 3)? 3: counter + 1;
        else
            newCounter = (counter == 0)? 0: counter - 1;

        branchPredTable.write(idx, newCounter);

    endrule


    method Action upd(TOKEN token, ISA_ADDRESS addr, Bool pred, Bool actual);
        // Read current table value pass update info on to updatePred rule above.
        let idx = getIdx(addr);
        branchPredTable.readReq(idx);
        respQ.enq(tuple2(idx, actual));
        bpredPathQ.enq(BPRED_PATH_UPDATE);
    endmethod

    method Action getPredReq(TOKEN token, ISA_ADDRESS addr);
        let idx = getIdx(addr);
        branchPredTable.readReq(idx);
        bpredPathQ.enq(BPRED_PATH_PREDICT);
    endmethod
    
    method ActionValue#(Bool) getPredResp() if (bpredPathQ.first() == BPRED_PATH_PREDICT);
        bpredPathQ.deq();
        let counter <- branchPredTable.readResp();
        return (counter > 1);
    endmethod

    method Action abort(TOKEN token);
        noAction;
    endmethod
endmodule
