import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import PipelineTypes::*;
import Vector::*;

typedef enum { STATE_BUS, STATE_INST, STATE_DEP, STATE_SEND } STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDecode ();

    StallPort_Receive#(Tuple2#(TOKEN,ISA_INSTRUCTION)) inQ <- mkStallPort_Receive("fet2dec");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))            outQ <- mkStallPort_Send   ("dec2exe");

    Vector#(3,Port_Receive#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)))) busQ = newVector();
    busQ[0] <- mkPort_Receive("exe_bus", 1);
    busQ[1] <- mkPort_Receive("mem_bus", 1);
    busQ[2] <- mkPort_Receive("wb_bus", 1);

    Connection_Client#(TOKEN, Tuple2#(TOKEN, ISA_DEPENDENCY_INFO)) getDependencies <- mkConnection_Client("funcp_getDependencies");

    Reg#(STATE) state <- mkReg(STATE_BUS);
    Reg#(Maybe#(Tuple2#(TOKEN, ISA_DEPENDENCY_INFO))) memoDependencies <- mkReg(Invalid);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //inports[1]  = busQ[0];
    //inports[2]  = busQ[1];
    //inports[3]  = busQ[2];
    //outports[0] = outQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    Vector#(FUNCP_PHYSICAL_REGS, Reg#(Bool)) prfValid = newVector();

    Integer numIsaArchRegisters = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_PHYSICAL_REGS);

    for (Integer i = 0; i < numIsaArchRegisters; i = i + 1)
        prfValid[i] <- mkReg(True);

    for (Integer i = numIsaArchRegisters; i < numFuncpPhyRegisters; i = i + 1)
        prfValid[i] <- mkReg(False);

    function Bool readyToGo(ISA_SRC_MAPPING srcmap);
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
                rdy = rdy && prfValid[pr];
        end
        return rdy;
    endfunction

    function Action markPRFInvalid(ISA_DST_MAPPING dstmap);
      action
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                prfValid[i] <= False;
        end
      endaction
    endfunction

    function BUNDLE makeBundle(ISA_INSTRUCTION inst, ISA_DST_MAPPING dstmap);
        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end
        return BUNDLE { isLoad:  isaIsLoad(inst),
                        isStore: isaIsStore(inst),
                        isTerminate: Invalid,
                        dests: dests };
    endfunction

    rule bus (state == STATE_BUS);
        local_ctrl.startModelCC();
        for (Integer i = 0; i < 3; i = i + 1)
        begin
            let mpregs <- busQ[i].receive();
            if (mpregs matches tagged Valid .pregs)
            begin
                for (Integer j = 0; j < valueof(ISA_MAX_DSTS); j = j + 1)
                begin
                    if (pregs[j] matches tagged Valid .p)
                        prfValid[p] <= True;
                end
            end
        end
        state <= STATE_INST;
    endrule

    rule stall (state == STATE_INST && !outQ.canSend);
        inQ.pass();
        outQ.pass();
        state <= STATE_BUS;
    endrule

    rule bubble (state == STATE_INST && outQ.canSend && !isValid(inQ.peek));
        let x <- inQ.receive();
        outQ.send(Invalid);
        state <= STATE_BUS;
    endrule

    rule inst (state == STATE_INST &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .* });
        if (!isValid(memoDependencies))
            getDependencies.makeReq(tok);
        state <= STATE_DEP;
    endrule

    rule dep (state == STATE_DEP);
        if (!isValid(memoDependencies))
            memoDependencies <= Valid(getDependencies.getResp());
        state <= STATE_SEND;
    endrule

    rule send (state == STATE_SEND &&& memoDependencies matches tagged Valid { .tok, .inf });
        match { .srcmap, .dstmap } = inf;
        if (readyToGo(srcmap))
        begin
            markPRFInvalid(dstmap);
            let mtup <- inQ.receive();
            if (mtup matches tagged Valid { .tok2, .inst })
                outQ.send(Valid(tuple2(tok,makeBundle(inst, dstmap))));
            memoDependencies <= Invalid;
        end
        else
        begin
            inQ.pass();
            outQ.send(Invalid);
        end
        state <= STATE_BUS;
    endrule

endmodule
