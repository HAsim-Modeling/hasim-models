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
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

import Vector::*;
import FShow::*;

// Mapping from cpu id to context ids and back.
function CPU_INSTANCE_ID tokCpuInstanceId(TOKEN tok) = tokContextId(tok);
function CPU_INSTANCE_ID storeTokCpuInstanceId(STORE_TOKEN st_tok) = storeTokContextId(st_tok);
function CONTEXT_ID getContextId(CPU_INSTANCE_ID cpu_iid) = cpu_iid;


typedef Bit#(1) UNIT;

typedef union tagged {
    void        NotBranch;
    ISA_ADDRESS BranchNotTaken;
    ISA_ADDRESS BranchTaken;
} BRANCH_ATTR deriving (Bits, Eq);


typedef union tagged {
    void IMEM_itlb_fault;
    void IMEM_icache_hit;
    ICACHE_MISS_ID IMEM_icache_miss;
    void IMEM_icache_retry;
    void IMEM_bad_epoch;
} IMEM_RESPONSE deriving (Bits, Eq);

typedef struct {
    IMEM_BUNDLE bundle;
    IMEM_RESPONSE response;
} IMEM_OUTPUT deriving (Bits, Eq);

//
// Messages from various stages to DECODE
//
typedef struct
{
    TOKEN token;

    // Registers written (and now available)
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs;
}
BUS_MESSAGE
    deriving (Bits, Eq);

function BUS_MESSAGE genBusMessage(TOKEN tok,
                                   Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs);
    return BUS_MESSAGE { token: tok, destRegs: destRegs };
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
    TOKEN_BRANCH_EPOCH branchEpoch;
    TOKEN_FAULT_EPOCH faultEpoch;
    ISA_ADDRESS pc;
    ISA_INSTRUCTION inst;
    BRANCH_ATTR branchAttr;
} FETCH_BUNDLE deriving (Bits, Eq);

typedef 16 NUM_INSTQ_SLOTS;
typedef TSub#(NUM_INSTQ_SLOTS, 1) NUM_INSTQ_CREDITS;
typedef Bit#(TLog#(NUM_INSTQ_SLOTS)) INSTQ_CREDIT_COUNT;

// INSTQ_ENQUEUE
// If missID is Invalid, there was no icache miss.
// If missID is Valid, then expect a delayed icache response associated with
// this bundle.
// If bundle is Invalid, then don't enqueue the bundle, but reclaim the credit,
// and if there is an icache miss, just drop the response when you get it.
typedef struct {
    Maybe#(FETCH_BUNDLE) bundle;
    Maybe#(ICACHE_MISS_ID) missID;
} INSTQ_ENQUEUE deriving(Bits, Eq);

typedef struct {
    TOKEN token;
    ISA_ADDRESS pc;
    BRANCH_ATTR branchAttr;
    TOKEN_BRANCH_EPOCH branchEpoch;
    TOKEN_FAULT_EPOCH faultEpoch;
    Bool isLoad;
    Bool isStore;
    ISA_ADDRESS effAddr;
    Maybe#(Bool) isTerminate;
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;
} BUNDLE deriving (Bits, Eq);

typedef union tagged 
{ 
    TOKEN SB_HIT;
    TOKEN SB_MISS; 
    TOKEN SB_STALL;
} 
SB_RESPONSE deriving (Bits, Eq);

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
