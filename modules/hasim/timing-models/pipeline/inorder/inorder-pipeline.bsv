//
// Copyright (C) 2008 Intel Corporation
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

`include "asim/provides/hasim_common.bsh"

`include "asim/provides/fetch_stage.bsh"
`include "asim/provides/imem_stage.bsh"
`include "asim/provides/pccalc_stage.bsh"
`include "asim/provides/instq_stage.bsh"
`include "asim/provides/decode_stage.bsh"
`include "asim/provides/execute_stage.bsh"
`include "asim/provides/dmem_stage.bsh"
`include "asim/provides/commit_stage.bsh"

`include "asim/provides/branch_predictor.bsh"
`include "asim/provides/store_buffer.bsh"

module [HASIM_MODULE] mkPipeline ();

    let fetch   <- mkFetch();
    let imem    <- mkIMem();
    let pccalc  <- mkPCCalc();
    let iq      <- mkInstructionQueue();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let dmem    <- mkDMem();
    let commit  <- mkCommit();

    let bp     <- mkBranchPredictor();
    let sb     <- mkStoreBuffer();

endmodule

