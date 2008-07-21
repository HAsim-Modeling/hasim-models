import hasim_isa::*;

import FIFO::*;
import FIFOF::*;

typedef 4 BufferSize;

module mkTargetBuffer#(ISA_ADDRESS startAddr)(FIFO#(ISA_ADDRESS));
    FIFOF#(ISA_ADDRESS) addrFifo <- mkSizedFIFOF(valueOf(BufferSize));

    //Really dumb buffer which drops the value if the fifo is full
    method Action enq(ISA_ADDRESS _addr);
        if(addrFifo.notFull())
            addrFifo.enq(_addr);
    endmethod

    method Action deq();
        if(addrFifo.notEmpty())
            addrFifo.deq();
    endmethod

    method ISA_ADDRESS first();
        if(addrFifo.notEmpty)
            return addrFifo.first();
        else
            return startAddr;
    endmethod

    method Action clear();
        addrFifo.clear();
    endmethod
endmodule

