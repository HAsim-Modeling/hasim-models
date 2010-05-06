import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"

typedef enum
{
    BPRED_PATH_UPDATE,
    BPRED_PATH_PREDICT
}
    BPRED_PATH
        deriving (Eq, Bits);


typedef Bit#(`BRANCH_TABLE_SIZE) BRANCH_INDEX;

interface BRANCH_PREDICTOR_ALG;
    method Action upd(CPU_INSTANCE_ID iid, ISA_ADDRESS addr, Bool pred, Bool actual);
    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
    method ActionValue#(Bool) getPredRsp(CPU_INSTANCE_ID iid);
    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
endinterface


module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTOR_ALG);

    MULTIPLEXED_LUTRAM#(NUM_CPUS, BRANCH_INDEX, Bit#(2)) branchPredTablePool <- mkMultiplexedLUTRAM(1);
    FIFO#(Bool) rspQ <- mkSizedFIFO(valueof(NUM_CPUS));

    function BRANCH_INDEX getIdx(ISA_ADDRESS addr);
        return truncate(hashBits(addr[31:2]));
    endfunction

    method Action upd(CPU_INSTANCE_ID iid, ISA_ADDRESS addr, Bool pred, Bool actual);
    
        LUTRAM#(BRANCH_INDEX, Bit#(2)) branchPredTable = branchPredTablePool.getRAM(iid);

        // Read current table value pass update info on to updatePred rule above.
        let idx = getIdx(addr);
        let counter = branchPredTable.sub(idx);
        
        let new_counter = 0;
        if (actual)
            new_counter = (counter == 3) ? 3 : counter + 1;
        else
            new_counter = (counter == 0) ? 0 : counter - 1;

        branchPredTable.upd(idx, new_counter);

    endmethod

    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);

        LUTRAM#(BRANCH_INDEX, Bit#(2)) branchPredTable = branchPredTablePool.getRAM(iid);

        let idx = getIdx(addr);
        let counter = branchPredTable.sub(idx);
        rspQ.enq(counter > 1);

    endmethod

    method ActionValue#(Bool) getPredRsp(CPU_INSTANCE_ID iid);

        rspQ.deq();
        return rspQ.first();
        
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
    
        noAction;
        
    endmethod

endmodule
