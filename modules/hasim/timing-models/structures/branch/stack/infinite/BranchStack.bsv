import hasim_cpu_types::*;

interface BranchStack;
    method ActionValue#(BranchStackIndex) add();
    method Action resolveRight(BranchStackIndex index);
    method Action resolveWrong(BranchStackIndex index);
    method Bool notFull();
endinterface

module mkBranchStack(BranchStack);
    method ActionValue#(BranchStackIndex) add();
        return 0;
    endmethod

    method Action resolveRight(BranchStackIndex index);
        noAction;
    endmethod

    method Action resolveWrong(BranchStackIndex index);
        noAction;
    endmethod

    method Bool notFull();
        return True;
    endmethod
endmodule
