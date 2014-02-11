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


typedef `MAX_NUM_CPUS MAX_NUM_CPUS;
typedef INSTANCE_ID#(MAX_NUM_CPUS) CPU_INSTANCE_ID;
typedef INSTANCE_ID_BITS#(MAX_NUM_CPUS) CPU_INSTANCE_ID_SZ;

typedef `MAX_NUM_MEM_CTRLS MAX_NUM_MEM_CTRLS;
typedef INSTANCE_ID#(MAX_NUM_MEM_CTRLS) MEM_CTRL_INSTANCE_ID;
typedef INSTANCE_ID_BITS#(MAX_NUM_MEM_CTRLS) MEM_CTRL_INSTANCE_ID_SZ;

typedef `NUM_EXTRA_OCN_STATIONS NUM_EXTRA_OCN_STATIONS;

typedef `NUM_LANES NUM_LANES;
typedef INSTANCE_ID#(NUM_LANES) LANE_IDX;
typedef INSTANCE_ID_BITS#(NUM_LANES) LANE_IDX_SZ;

typedef `VCS_PER_LANE VCS_PER_LANE;
typedef INSTANCE_ID#(VCS_PER_LANE) VC_IDX;
typedef INSTANCE_ID_BITS#(VCS_PER_LANE) VC_IDX_SZ;

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) VC_INFO#(parameter type t_DATA);

// Bucket for tracking sender-side credits in on-chip network links.
// Credits are counted as packets, not flits!  The number of flit buffer
// slots is MAX_FLITS_PER_PACKET * credits.
typedef UInt#(4) VC_CREDIT_CNT;

// Message sent with credit updates.
typedef VC_INFO#(VC_CREDIT_CNT) VC_CREDIT_MSG;

typedef `MAX_FLITS_PER_PACKET MAX_FLITS_PER_PACKET;

//
// The number of stations is the number of CPUs plus the number of
// memory controllers plus the number of extra stations.  The extra
// stations allow more flexible topologies, such as rectangles with
// some memory controllers on the top and bottom rows, with other
// top/bottom slots empty.
//
typedef TAdd#(MAX_NUM_CPUS,
              TAdd#(MAX_NUM_MEM_CTRLS, NUM_EXTRA_OCN_STATIONS)) NUM_STATIONS;
typedef INSTANCE_ID#(NUM_STATIONS) STATION_IID;
typedef STATION_IID STATION_ID;

//
// An opaque storage container in flit bodies.  This is typically used to store
// a pointer to a packet descriptor.  The size is chosen because it matches
// the size of an OCN_FLIT_HEAD.
//
typedef Bit#(TMul#(2, INSTANCE_ID_BITS#(NUM_STATIONS))) OCN_FLIT_OPAQUE;


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
    OCN_FLIT_OPAQUE opaque;
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
            String b_type = flit.isTail ? "TAIL" : "BODY";
            return $format("{%s} flit_opaque 0x%0x", b_type, flit.opaque);
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
