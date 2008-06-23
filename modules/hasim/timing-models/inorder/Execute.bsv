import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

import PipelineTypes::*;
import Vector::*;

typedef enum { STATE_EXEC, STATE_WORK } STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkExecute ();

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("dec2exe");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("exe2mem");

    Port_Send#(Tuple2#(TOKEN, Maybe#(ISA_ADDRESS))) rewindQ <- mkPort_Send("rewind");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("exe_bus");

    Connection_Client#(TOKEN, Tuple2#(TOKEN, ISA_EXECUTION_RESULT)) getResults <- mkConnection_Client("funcp_getResults");

    Reg#(STATE) state <- mkReg(STATE_EXEC);

    Reg#(TOKEN_TIMEP_EPOCH) epoch <- mkReg(0);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //outports[0] = outQ.ctrl;
    //outports[1] = rewindQ.ctrl;
    //outports[2] = busQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    Vector#(FUNCP_PHYSICAL_REGS, Reg#(Bool)) prfValid = newVector();

    function Bool good_epoch (TOKEN tok) = tok.timep_info.epoch == epoch;

    rule flush (state == STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .* } &&& !good_epoch(tok));
        local_ctrl.startModelCC();
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        busQ.send(Invalid);
    endrule

    rule exec (state == STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .* } &&& good_epoch(tok));
        local_ctrl.startModelCC();
        if (outQ.canSend)
        begin
            getResults.makeReq(tok);
            state <= STATE_WORK;
        end
        else
        begin
           inQ.pass();
           outQ.pass();
           rewindQ.send(Invalid);
           busQ.send(Invalid);
        end
    endrule

    rule bubble (state == STATE_EXEC &&& inQ.peek() == Invalid);
        local_ctrl.startModelCC();
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        busQ.send(Invalid);
    endrule

    rule results (state == STATE_WORK);
        getResults.deq();
        match { .tok, .res } = getResults.getResp();
        let x <- inQ.receive();
        if (x matches tagged Valid { .tok2, .bndl })
        begin
            let bundle = bndl;
            case (res) matches
              tagged RBranchTaken .addr:
                begin
                    epoch <= epoch + 1;
                    rewindQ.send(Valid(tuple2(tok, Valid(addr))));
                end
              tagged RBranchNotTaken .addr:
                begin
                    rewindQ.send(Invalid);
                end
              tagged REffectiveAddr .ea:
                begin
                    rewindQ.send(Invalid);
                end
              tagged RNop:
                begin
                    rewindQ.send(Invalid);
                end
              tagged RTerminate .pf:
                begin
                    rewindQ.send(Invalid);
                    bundle.isTerminate = Valid(pf);
                end
            endcase
            outQ.send(Valid(tuple2(tok, bundle)));
            rewindQ.send(Invalid);
            busQ.send(Invalid);
            state <= STATE_EXEC;
        end
    endrule

endmodule
