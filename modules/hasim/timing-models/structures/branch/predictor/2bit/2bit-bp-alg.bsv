//
// Copyright (C) 2011 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

import FIFO::*;
import FIFOF::*;
import SpecialFIFOs::*;

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


//
// Branch predictor table read ports
//
`define PORT_UPD  0
`define PORT_PRED 1


module mkBranchPredAlg
    // interface:
        (BRANCH_PREDICTOR_ALG);

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(NUM_CPUS, 2, BRANCH_INDEX, Bit#(2))
        branchPredTablePool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, 1));

    FIFO#(Tuple4#(CPU_INSTANCE_ID, ISA_ADDRESS, Bool, Bool)) newUpdQ <- mkBypassFIFO();
    FIFOF#(Tuple4#(CPU_INSTANCE_ID, BRANCH_INDEX, Bool, Bool)) updQ <- mkFIFOF();

    FIFO#(Tuple2#(CPU_INSTANCE_ID, ISA_ADDRESS)) newReqQ <- mkBypassFIFO();
    FIFO#(CPU_INSTANCE_ID) reqQ <- mkFIFO();
    FIFO#(Bool) rspQ <- mkSizedFIFO(valueof(NUM_CPUS));

    function BRANCH_INDEX getIdx(ISA_ADDRESS addr);
        return truncate(hashBits(addr[31:2]));
    endfunction

    //
    // cpuIsLockedForUpdate --
    //     A CPU is locked if an if the prediction table has been read but not
    //     yet updated.  This code depends on updQ being a FIFO of normal
    //     size.
    //
    function cpuIsLockedForUpdate(CPU_INSTANCE_ID iid);
        return updQ.notEmpty() && (tpl_1(updQ.first()) == iid);
    endfunction


    //
    // predReq --
    //     New prediction request.  Block if an update is in progress for the
    //     same CPU.
    //
    rule predReq (! cpuIsLockedForUpdate(tpl_1(newReqQ.first())));
        match {.iid, .addr} = newReqQ.first();
        newReqQ.deq();

        let idx = getIdx(addr);
        branchPredTablePool.readPorts[`PORT_PRED].readReq(iid, idx);
        reqQ.enq(iid);
    endrule

    //
    // predRsp --
    //     Consume prediction request and generate a response.  While simple,
    //     the rule is required because the caller expects sufficient buffering
    //     in the response queue to hold one request for each CPU.  The branch
    //     table RAM read buffer is too small for this purpose.
    //
    rule predRsp (True);
        let iid = reqQ.first();
        reqQ.deq();

        let counter <- branchPredTablePool.readPorts[`PORT_PRED].readRsp(iid);
        rspQ.enq(counter > 1);
    endrule


    //
    // startUpd --
    //     Update prediction table.  Only one update per CPU is permitted.
    //
    rule startUpd (! cpuIsLockedForUpdate(tpl_1(newUpdQ.first())));
        match {.iid, .addr, .pred, .actual} = newUpdQ.first();
        newUpdQ.deq();

        // Read current table value
        let idx = getIdx(addr);
        branchPredTablePool.readPorts[`PORT_UPD].readReq(iid, idx);

        updQ.enq(tuple4(iid, idx, pred, actual));
    endrule

    rule finishUpd (True);
        match {.iid, .idx, .pred, .actual} = updQ.first();
        updQ.deq();

        let counter <- branchPredTablePool.readPorts[`PORT_UPD].readRsp(iid);
        
        let new_counter = 0;
        if (actual)
            new_counter = (counter == 3) ? 3 : counter + 1;
        else
            new_counter = (counter == 0) ? 0 : counter - 1;

        branchPredTablePool.write(iid, idx, new_counter);
    endrule


    method Action upd(CPU_INSTANCE_ID iid, ISA_ADDRESS addr, Bool pred, Bool actual);
    
        newUpdQ.enq(tuple4(iid, addr, pred, actual));

    endmethod

    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);

        newReqQ.enq(tuple2(iid, addr));

    endmethod

    method ActionValue#(Bool) getPredRsp(CPU_INSTANCE_ID iid);

        rspQ.deq();
        return rspQ.first();
        
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
    
        noAction;
        
    endmethod

endmodule
