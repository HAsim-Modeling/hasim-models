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

`include "asim/provides/hasim_common.bsh"

`include "asim/rrr/remote_server_stub_SMT_PIPELINE.bsh"

`include "asim/provides/fetch_stage.bsh"
`include "asim/provides/imem_stage.bsh"
`include "asim/provides/pccalc_stage.bsh"
`include "asim/provides/instq_stage.bsh"
`include "asim/provides/decode_stage.bsh"
`include "asim/provides/execute_stage.bsh"
`include "asim/provides/dmem_stage.bsh"
`include "asim/provides/commitq_stage.bsh"
`include "asim/provides/commit_stage.bsh"

`include "asim/provides/line_predictor.bsh"
`include "asim/provides/branch_predictor.bsh"
`include "asim/provides/store_buffer.bsh"
`include "asim/provides/write_buffer.bsh"

module [HASIM_MODULE] mkPipeline ();

    let serverStub <- mkServerStub_SMT_PIPELINE();

    let fetch   <- mkFetch();
    let imem    <- mkIMem();
    let pccalc  <- mkPCCalc();
    let iq      <- mkInstructionQueue();
    let decode  <- mkDecode();
    let execute <- mkExecute();
    let dmem    <- mkDMem();
    let cq      <- mkCommitQueue();
    let commit  <- mkCommit();

    let lp     <- mkLinePredictor();
    let bp     <- mkBranchPredictor();
    let sb     <- mkStoreBuffer();
    let wb     <- mkWriteBuffer();

    rule setNumThreadsPerCore (True);
        let numthreads <- serverStub.acceptRequest_SetNumThreadsPerCore();
        fetch.setNumThreadsPerCore(numthreads);
        serverStub.sendResponse_SetNumThreadsPerCore(?);
    endrule

endmodule

