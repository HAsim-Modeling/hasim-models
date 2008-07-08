import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import Vector::*;
import FShow::*;

typedef Bit#(1) UNIT;

typedef struct {
    Bool isLoad;
    Bool isStore;
    Maybe#(Bool) isTerminate;
    Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests;
} BUNDLE deriving (Bits, Eq);

instance FShow#(BUNDLE);
    function Fmt fshow(BUNDLE x);
        Fmt s = fshow("BUNDLE:");
        if (x.isLoad)
            s = s + fshow("LOAD");
        if (x.isStore)
            s = s + fshow("STORE");
        if (x.isTerminate matches tagged Valid .b)
            s = s + $format("TERMINATE(%b)", b);
        return s;
    endfunction
endinstance

instance FShow#(TOKEN);
    function Fmt fshow (TOKEN tok);
        return $format("TOK:%d", tok.index);
    endfunction
endinstance
