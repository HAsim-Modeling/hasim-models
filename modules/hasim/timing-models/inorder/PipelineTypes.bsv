import hasim_isa::*;

import Vector::*;

typedef Bit#(1) UNIT;

typedef struct {
    Bool isLoad;
    Bool isStore;
    Maybe#(Bool) isTerminate;
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;
} BUNDLE deriving (Bits, Eq);
