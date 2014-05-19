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
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/hasim_itlb.bsh"
`include "awb/provides/hasim_dtlb.bsh"
`include "awb/provides/hasim_l1_icache.bsh"
`include "awb/provides/hasim_l1_dcache.bsh"
`include "awb/provides/hasim_l1_arbiter.bsh"
`include "awb/provides/hasim_l2_cache.bsh"

module [HASIM_MODULE] mkPrivateCaches();
    let itlb    <- mkITLB();
    let dtlb    <- mkDTLB();
    let icache  <- mkL1ICache();
    let dcache  <- mkL1DCache();
    let arbiter <- mkL1CacheArbiter("L1toL2_ReqQ", "L2toL1_RspQ");

    let l2cache <- mkL2Cache("L1toL2_ReqQ", "L2toL1_RspQ",
                             "CorePvtCache_to_UncoreQ",
                             "Uncore_to_CorePvtCacheQ");
endmodule
