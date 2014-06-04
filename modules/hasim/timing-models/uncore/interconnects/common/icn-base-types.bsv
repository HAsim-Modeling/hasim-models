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


import Vector::*;
import FShow::*;

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/chip_base_types.bsh"

`include "awb/dict/OCN_LANES.bsh"


typedef `NUM_EXTRA_OCN_STATIONS NUM_EXTRA_OCN_STATIONS;

// Lanes are allocated and assigned as dictionary entries.  This lets the
// model respond to configurable components wishing to connect to the network
// without a central module knowing the lane assignments.
typedef TAdd#(`OCN_LANES__NENTRIES, `OCN_LANES__NSUBGROUPS) NUM_LANES;
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
