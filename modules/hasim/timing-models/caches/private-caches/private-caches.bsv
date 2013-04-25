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

`include "awb/provides/hasim_common.bsh"
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
