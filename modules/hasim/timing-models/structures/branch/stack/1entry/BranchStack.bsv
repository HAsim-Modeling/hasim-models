import hasim_cpu_types::*;

interface BranchStack;
    method ActionValue#(BranchCount) add();
    method Action resolveRight(BranchCount index);
    method Action resolveWrong(BranchCount index);
    method Bool notFull();
endinterface

module mkBranchStack(BranchStack);
    Reg#(Bool) allotted <- mkReg(False);

    method ActionValue#(BranchCount) add();
        allotted <= True;
        return 0;
    endmethod

    method Action resolveRight(BranchCount index);
        allotted <= False;
    endmethod

    method Action resolveWrong(BranchCount index);
        allotted <= False;
    endmethod

    method Bool notFull();
        return !allotted;
    endmethod
endmodule
