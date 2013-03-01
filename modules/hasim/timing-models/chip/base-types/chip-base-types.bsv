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


typedef `MAX_NUM_CPUS MAX_NUM_CPUS;
typedef INSTANCE_ID#(MAX_NUM_CPUS) CPU_INSTANCE_ID;
typedef INSTANCE_ID_BITS#(MAX_NUM_CPUS) CPU_INSTANCE_ID_SZ;

typedef `MAX_NUM_MEM_CTRLS MAX_NUM_MEM_CTRLS;
typedef INSTANCE_ID#(MAX_NUM_MEM_CTRLS) MEM_CTRL_INSTANCE_ID;
typedef INSTANCE_ID_BITS#(MAX_NUM_MEM_CTRLS) MEM_CTRL_INSTANCE_ID_SZ;

typedef `NUM_LANES NUM_LANES;
typedef INSTANCE_ID#(NUM_LANES) LANE_IDX;
typedef INSTANCE_ID_BITS#(NUM_LANES) LANE_IDX_SZ;

typedef `VCS_PER_LANE VCS_PER_LANE;
typedef INSTANCE_ID#(VCS_PER_LANE) VC_IDX;
typedef INSTANCE_ID_BITS#(VCS_PER_LANE) VC_IDX_SZ;

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) VC_INFO#(parameter type t_DATA);

// First Bool is credit, second Bool is NotFull.
typedef VC_INFO#(Tuple2#(Bool, Bool)) VC_CREDIT_INFO;

//
// The number of stations is the number of CPUs plus the number of
// memory controllers.
//
typedef TAdd#(MAX_NUM_CPUS, MAX_NUM_MEM_CTRLS) NUM_STATIONS;
typedef INSTANCE_ID#(NUM_STATIONS) STATION_IID;
typedef STATION_IID STATION_ID;


typedef struct
{
    STATION_ID src;
    STATION_ID dst;
    Bool isStore;
}
OCN_FLIT_HEAD
    deriving (Eq, Bits);

typedef struct
{
    MEM_OPAQUE opaque;
    Bool isTail;
}
OCN_FLIT_BODY
    deriving (Eq, Bits);

typedef union tagged
{
    OCN_FLIT_HEAD FLIT_HEAD;
    OCN_FLIT_BODY FLIT_BODY; // Address?
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
