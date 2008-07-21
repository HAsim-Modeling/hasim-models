import hasim_isa::*;

import FIFO::*;

module mkTargetBuffer#(Addr startAddr)(FIFO#(Addr));
    Reg#(Addr) addr <- mkReg(startAddr);

    method Action enq(Addr _addr);
        addr <= _addr;
    endmethod

    method Action deq();
        noAction;
    endmethod

    method Addr first();
        return addr;
    endmethod

    method Action clear();
        noAction;
    endmethod
endmodule

