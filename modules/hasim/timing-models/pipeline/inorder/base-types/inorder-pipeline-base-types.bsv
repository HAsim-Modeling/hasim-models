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

import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;
`include "asim/provides/funcp_interface.bsh"

import Vector::*;
import FShow::*;

typedef Bit#(1) UNIT;

typedef union tagged {
    void        NotBranch;
    ISA_ADDRESS BranchNotTaken;
    ISA_ADDRESS BranchTaken;
} BRANCH_ATTR deriving (Bits, Eq);


//
// Messages from various stages to DECODE
//
typedef struct
{
    TOKEN token;

    // Registers written (and now available)
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs;
    // Token killed and won't be seen again
    Bool tokKilled;
}
BUS_MESSAGE
    deriving (Bits, Eq);

function BUS_MESSAGE genBusMessage(TOKEN tok,
                                   Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs,
                                   Bool killed);
    return BUS_MESSAGE { token: tok, destRegs: destRegs, tokKilled: killed };
endfunction


//
// Message from EXE back to front end branch predictor
//
typedef struct {
    TOKEN token;
    ISA_ADDRESS branchPC;      // PC of branch instruction
    BRANCH_ATTR exeResult;     // True outcome of branch as computed by EXE stage
    Bool predCorrect;          // Was original prediction correct?
} BRANCH_PRED_TRAIN deriving (Bits, Eq);

typedef struct {
    TOKEN token;
    ISA_ADDRESS pc;
    ISA_INSTRUCTION inst;
    BRANCH_ATTR branchAttr;
} FETCH_BUNDLE deriving (Bits, Eq);

typedef struct {
    TOKEN token;
    ISA_ADDRESS pc;
    BRANCH_ATTR branchAttr;
    Bool isLoad;
    Bool isStore;
    ISA_ADDRESS effAddr;
    Maybe#(Bool) isTerminate;
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;
} BUNDLE deriving (Bits, Eq);

typedef enum { SB_HIT, SB_MISS, SB_STALL } SB_RESPONSE deriving (Bits, Eq);

instance FShow#(BRANCH_ATTR);
    function Fmt fshow(BRANCH_ATTR x) =
        case (x) matches
            tagged NotBranch: (fshow("NotBranch"));
            tagged BranchTaken .a: (fshow("BranchTaken: tgt=") + fshow(a));
            tagged BranchNotTaken .a: (fshow("BranchNotTaken: taken-tgt=") + fshow(a));
        endcase;
endinstance

instance FShow#(BUNDLE);
    function Fmt fshow(BUNDLE x);
        Fmt s = fshow("BUNDLE: pc = ") + fshow(x.pc);
        if (x.isLoad)
            s = s + fshow(" LOAD");
        if (x.isStore)
            s = s + fshow(" STORE");
        if (x.isTerminate matches tagged Valid .b)
            s = s + $format(" TERMINATE(%b)", b);
        s = s + fshow(" BRANCH-ATTR: ") + fshow(x.branchAttr);
        return s;
    endfunction
endinstance
