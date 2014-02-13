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

import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

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
    void IMEM_icache_req;
    void IMEM_icache_hit;
    L1_ICACHE_MISS_ID IMEM_icache_miss;
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
    TOKEN_EPOCH epoch;

    // Registers written (and now available)
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs;
}
BUS_MESSAGE
    deriving (Bits, Eq);

//
// genBusMessage --
//     Simplest generator for BUS_MESSAGE.  Just fill in the structure.
//
function BUS_MESSAGE genBusMessage(TOKEN tok,
                                   TOKEN_EPOCH epoch,
                                   Vector#(ISA_MAX_DSTS,
                                           Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs);

    return BUS_MESSAGE { token: tok, epoch: epoch, destRegs: destRegs };
endfunction


//
// genBusMessageMasked --
//     Fill in a BUS_MESSAGE, but apply writeMask to the set of destRegs.
//
function BUS_MESSAGE genBusMessageMasked(TOKEN tok,
                                         TOKEN_EPOCH epoch,
                                         Vector#(ISA_MAX_DSTS,
                                            Maybe#(FUNCP_PHYSICAL_REG_INDEX)) destRegs,
                                         ISA_INST_DSTS_MASK writeMask);
    // Send only the registers allowed by writeMask
    function Maybe#(FUNCP_PHYSICAL_REG_INDEX) destRegIfEnabled(Maybe#(FUNCP_PHYSICAL_REG_INDEX) dst, Bool mask) =
        (mask ? dst : tagged Invalid);

    return genBusMessage(tok,
                         epoch,
                         zipWith(destRegIfEnabled, destRegs, writeMask));
endfunction

//
// genBusMessageEXE --
//     Generate a BUS_MESSAGE for the EXE phase, given a bundle.
//
function BUS_MESSAGE genBusMessageEXE(BUNDLE bundle);
    return genBusMessageMasked(bundle.token,
                               initEpoch(bundle.branchEpoch, bundle.faultEpoch),
                               bundle.dests,
                               bundle.writtenAtEXE);
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
    Maybe#(L1_ICACHE_MISS_ID) missID;
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

    // All physical register destinations for instruction
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;

    // Masks indicating when a destination is written.  Each destination will appear
    // only once.
    ISA_INST_DSTS_MASK writtenAtEXE;
    ISA_INST_DSTS_MASK writtenAtMEM;
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
