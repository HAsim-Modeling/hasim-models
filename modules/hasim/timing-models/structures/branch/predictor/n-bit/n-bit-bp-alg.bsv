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

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/hasim_isa.bsh"

`include "awb/provides/model_structures_base_types.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/pipeline_base_types.bsh"


//
// Branch predictor table read ports
//
`define PORT_UPD  0
`define PORT_PRED 1


module mkBranchPredAlg
    // interface:
    (BRANCH_PREDICTOR_ALG)
    provisos (Alias#(t_PRED_CNT, Bit#(`BP_COUNTER_SIZE)),
              Alias#(t_TABLE_IDX, Bit#(`BP_TABLE_IDX_SIZE)));

    DEBUG_FILE debugLog <- mkDebugFile("alg_bp_n_bit.out");

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(NUM_CPUS, 2, t_TABLE_IDX, t_PRED_CNT)
        branchPredTablePool <- mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiReadInitialized(False, ~0 >> 1));

    FIFO#(Tuple3#(CPU_INSTANCE_ID, ISA_ADDRESS, Bool)) newUpdQ <- mkBypassFIFO();
    FIFOF#(Tuple3#(CPU_INSTANCE_ID, t_TABLE_IDX, Bool)) updQ <- mkFIFOF();

    FIFO#(Tuple2#(CPU_INSTANCE_ID, ISA_ADDRESS)) newReqQ <- mkBypassFIFO();
    FIFO#(CPU_INSTANCE_ID) reqQ <- mkFIFO();
    FIFO#(Bool) rspQ <- mkSizedFIFO(valueof(NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS));

    function t_TABLE_IDX getIdx(ISA_ADDRESS addr);
        return resize(hashBits(pcAddrWithoutAlignmentBits(addr)));
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
        let taken = (msb(counter) == 1);
        
        debugLog.record($format("<%0d>: pred=%d, cnt=%0d", iid, taken, counter));

        rspQ.enq(taken);
    endrule


    //
    // startUpd --
    //     Update prediction table.  Only one update per CPU is permitted.
    //
    rule startUpd (! cpuIsLockedForUpdate(tpl_1(newUpdQ.first())));
        match {.iid, .addr, .actual} = newUpdQ.first();
        newUpdQ.deq();

        // Read current table value
        let idx = getIdx(addr);
        branchPredTablePool.readPorts[`PORT_UPD].readReq(iid, idx);

        updQ.enq(tuple3(iid, idx, actual));
    endrule

    rule finishUpd (True);
        match {.iid, .idx, .actual} = updQ.first();
        updQ.deq();

        let counter <- branchPredTablePool.readPorts[`PORT_UPD].readRsp(iid);
        
        t_PRED_CNT new_counter;
        if (actual)
            new_counter = (counter == maxBound) ? maxBound : counter + 1;
        else
            new_counter = (counter == 0) ? 0 : counter - 1;

        branchPredTablePool.write(iid, idx, new_counter);

        debugLog.record($format("<%0d>: Upd idx=%0d, taken=%0d, was=%0d, new=%0d", iid, idx, actual, counter, new_counter));
    endrule


    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        newReqQ.enq(tuple2(iid, addr));
    endmethod

    method ActionValue#(Bool) getPredRsp(CPU_INSTANCE_ID iid);
        rspQ.deq();
        return rspQ.first();
    endmethod

    method Action upd(CPU_INSTANCE_ID iid,
                      ISA_ADDRESS addr,
                      Bool wasCorrect,
                      Bool actual);
        newUpdQ.enq(tuple3(iid, addr, actual));
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        noAction;
    endmethod

endmodule
