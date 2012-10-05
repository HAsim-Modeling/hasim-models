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
// mkBranchPredAlg --
//     Make a static prediction for branch direction.  Direction is determined
//     by AWB parameter.
//
module mkBranchPredAlg
    // interface:
    (BRANCH_PREDICTOR_ALG);

    FIFO#(VOID) rspQ <- mkSizedFIFO(valueOf(NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS));
    
    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        rspQ.enq(?);
    endmethod

    method ActionValue#(Bool) getPredRsp(CPU_INSTANCE_ID iid);
        rspQ.deq();
        return (`BP_STATIC_TAKEN != 0);
    endmethod

    method Action upd(CPU_INSTANCE_ID iid,
                      ISA_ADDRESS addr,
                      Bool wasCorrect,
                      Bool actual);
        noAction;
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        noAction;
    endmethod
endmodule
