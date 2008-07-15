import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import Vector::*;
import FShow::*;

typedef Bit#(1) UNIT;

typedef union tagged {
    void        NotBranch;
    ISA_ADDRESS BranchNotTaken;
    ISA_ADDRESS BranchTaken;
} BRANCH_ATTR deriving (Bits, Eq);

typedef struct {
    ISA_ADDRESS pc;
    ISA_INSTRUCTION inst;
    BRANCH_ATTR branchAttr;
} FETCH_BUNDLE deriving (Bits, Eq);

typedef struct {
    ISA_ADDRESS pc;
    BRANCH_ATTR branchAttr;
    Bool isLoad;
    Bool isStore;
    Maybe#(Bool) isTerminate;
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;
} BUNDLE deriving (Bits, Eq);

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
            s = s + fshow("LOAD");
        if (x.isStore)
            s = s + fshow("STORE");
        if (x.isTerminate matches tagged Valid .b)
            s = s + $format("TERMINATE(%b)", b);
        s = s + fshow(" BRANCH-ATTR: ") + fshow(x.branchAttr);
        return s;
    endfunction
endinstance

instance FShow#(TOKEN);
    function Fmt fshow (TOKEN tok);
        return $format("TOK:%d", tok.index);
    endfunction
endinstance
