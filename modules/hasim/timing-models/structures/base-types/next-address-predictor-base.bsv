//
// Copyright (C) 2012 Intel Corporation
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
