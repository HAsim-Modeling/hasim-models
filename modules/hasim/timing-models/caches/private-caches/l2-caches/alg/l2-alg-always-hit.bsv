//
// Copyright (C) 2013 Intel Corporation
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

module [HASIM_MODULE] mkL2CacheAlg
    // interface:
        (CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE))
    provisos
        (Bits#(t_OPAQUE, t_OPAQUE_SZ),
         Add#(t_OPAQUE_SZ, t_TMP, 8));

    CACHE_ALG#(MAX_NUM_CPUS, t_OPAQUE) alg <- mkCacheAlgAlwaysHit();

    return alg;
endmodule
