//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
