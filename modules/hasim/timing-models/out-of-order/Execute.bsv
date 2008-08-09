import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

//import PipelineTypes::*;
import FShow::*;
import Vector::*;

typedef enum { EXECUTE_STATE_EXEC, EXECUTE_STATE_WORK } EXECUTE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkExecute ();

    DebugFile debug <- mkDebugFile("pipe_execute.out", "PIPE: EXECUTE:\t");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("dec2exe");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("exe2mem");

    Port_Send#(Tuple2#(TOKEN, Maybe#(ISA_ADDRESS))) rewindQ <- mkPort_Send("rewind");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("exe_bus");

    Connection_Client#(TOKEN, Tuple2#(TOKEN, ISA_EXECUTION_RESULT)) getResults <- mkConnection_Client("funcp_getResults");

    Reg#(EXECUTE_STATE) state <- mkReg(EXECUTE_STATE_EXEC);

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

    rule flush (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .bundle } &&& !good_epoch(tok));
        debug <= fshow("FLUSH: ") + fshow(tok);
        local_ctrl.startModelCC();
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        busQ.send(Valid(bundle.dests));
    endrule

    rule exec (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .* } &&& good_epoch(tok));
        local_ctrl.startModelCC();
        if (outQ.canSend)
        begin
            debug <= fshow("EXEC: ") + fshow(tok);
            getResults.makeReq(tok);
            state <= EXECUTE_STATE_WORK;
        end
        else
        begin
           debug <= fshow("STALL PROPAGATED");
           inQ.pass();
           outQ.pass();
           rewindQ.send(Invalid);
           busQ.send(Invalid);
        end
    endrule

    rule bubble (state == EXECUTE_STATE_EXEC &&& inQ.peek() == Invalid);
        local_ctrl.startModelCC();
        debug <= fshow("BUBBLE");
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        busQ.send(Invalid);
    endrule

    rule results (state == EXECUTE_STATE_WORK);
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
                    debug <= fshow("BRANCH TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h END-OF-EPOCH:%d", addr, epoch);
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
            busQ.send(Invalid);
            state <= EXECUTE_STATE_EXEC;
        end
    endrule

endmodule
