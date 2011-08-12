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

import Vector::*;
import FShow::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/memory_base_types.bsh"


typedef `NUM_CPUS NUM_CPUS;
typedef INSTANCE_ID#(NUM_CPUS) CPU_INSTANCE_ID;

typedef `NUM_LANES NUM_LANES;
typedef INSTANCE_ID#(NUM_LANES) LANE_IDX;

typedef `VCS_PER_LANE VCS_PER_LANE;
typedef INSTANCE_ID#(VCS_PER_LANE) VC_IDX;

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) VC_INFO#(parameter type t_DATA);

// First Bool is credit, second Bool is NotFull.
typedef VC_INFO#(Tuple2#(Bool, Bool)) VC_CREDIT_INFO;

// There is one more station because of the memory controller.
// Different topologies may handle this differently in the future.
typedef TAdd#(NUM_CPUS, 1) NUM_STATIONS;
typedef INSTANCE_ID#(NUM_STATIONS) STATION_IID;
typedef STATION_IID STATION_ID;


typedef union tagged
{
    struct {STATION_ID src; STATION_ID dst; Bool isStore;} FLIT_HEAD;
    struct {MEM_OPAQUE opaque; Bool isTail;} FLIT_BODY; // Address?
}
OCN_FLIT deriving (Eq, Bits);

typedef Tuple3#(LANE_IDX, VC_IDX, OCN_FLIT) OCN_MSG;


//
// Debug formatting
//

instance FShow#(OCN_FLIT);
    function Fmt fshow(OCN_FLIT ocnFlit);
        if (ocnFlit matches tagged FLIT_HEAD .flit)
        begin
            return $format("{HEAD %s (%0d:%0d)}",
                           flit.isStore ? "ST" : "LD",
                           flit.src, flit.dst);
        end
        else if (ocnFlit matches tagged FLIT_BODY .flit)
        begin
            return flit.isTail ? fshow("{TAIL}") : fshow("{BODY}");
        end
        else
        begin
            return fshow("{ILLEGAL FLIT}");
        end
    endfunction
endinstance

instance FShow#(OCN_MSG);
    function Fmt fshow(OCN_MSG msg);
        match {.ln, .vc, .flit} = msg;
        return fshow("[") + fshow(flit) + $format(" ln %0d vc %0d]", ln, vc);
    endfunction
endinstance
