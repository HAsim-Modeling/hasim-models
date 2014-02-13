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

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/hasim_isa.bsh"

`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"


//
// Standard interface for a next address prediction.  The interface is
// polymorphic to support a variety of predictor classes:
//
//  - IDX_SPACE is the space in which predictions are being made.  For most
//    predictors this will typicaly be the ISA_ADDRESS virtual address space.
//
//  - PRED_TYPE is the type of the returned prediction.  For a branch predictor
//    it will be a Bool.  For a line or BTB it will be an address.
//
interface NEXT_ADDR_PREDICTOR_ALG#(type t_IDX_SPACE, type t_PRED_TYPE);
    // Get a prediction request/response
    method Action getPredReq(CPU_INSTANCE_ID iid, t_IDX_SPACE idx);
    method ActionValue#(t_PRED_TYPE) getPredRsp(CPU_INSTANCE_ID iid);

    // Update the predictor given the actual value
    method Action upd(CPU_INSTANCE_ID iid,
                      t_IDX_SPACE idx,
                      Bool wasCorrect,
                      t_PRED_TYPE actual);

    // Abort some path.  For many predictors this is meaningless.  For some,
    // e.g. global history branch predictor, it can cause the state to revert
    // to some previously stored known-good state.
    method Action abort(CPU_INSTANCE_ID iid, t_IDX_SPACE idx);
endinterface


//
// Next address predictors have variable internal pipeline depth between
// getPredReq and getPredRsp.  The minimum buffer slots exists so multiple
// predictors may be called in parallel from the same rule pairs without
// introducing FPGA pipeline bubbles.
//
typedef `NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS;


//
// Classes of predictors.
//

typedef NEXT_ADDR_PREDICTOR_ALG#(ISA_ADDRESS, Bool) BRANCH_PREDICTOR_ALG;

typedef NEXT_ADDR_PREDICTOR_ALG#(ISA_ADDRESS, Maybe#(ISA_ADDRESS)) BRANCH_TARGET_BUFFER_ALG;
