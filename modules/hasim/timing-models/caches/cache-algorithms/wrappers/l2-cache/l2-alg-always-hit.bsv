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

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/mem_services.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_cache_algorithms.bsh"


// mkL2DCacheAlg

// Always-Hit L2Cache algorithm.

// Instantiate an always-hit algorithm.

// Although the always-hit algorithm does not actually use indexing,
// other algorithms do, so we still need a dummy index size.

typedef CACHE_ALG#(t_NUM_INSTANCES, void, 0, 1)
    L2_CACHE_ALG#(numeric type t_NUM_INSTANCES, type t_OPAQUE);

typedef CACHE_ENTRY_IDX#(0, 1)
    L2_CACHE_IDX;

typedef CACHE_LOOKUP_RSP#(void, 0, 1)
    L2_CACHE_LOOKUP_RSP#(type t_OPAQUE);

module [HASIM_MODULE] mkL2CacheAlg#(function Bool mayEvict(void opaque))
    // interface:
    (L2_CACHE_ALG#(t_NUM_INSTANCES, t_OPAQUE));

    let _alg <- mkCacheAlgAlwaysHit(mayEvict);
    return _alg;
endmodule
