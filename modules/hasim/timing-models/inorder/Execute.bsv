import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

`include "asim/provides/funcp_interface.bsh"

//import PipelineTypes::*;
import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_EXECUTE.bsh"
`include "asim/dict/STATS_EXECUTE.bsh"

typedef enum { EXECUTE_STATE_EXEC, EXECUTE_STATE_WORK } EXECUTE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkExecute ();

    DebugFile debug <- mkDebugFile("pipe_execute.out");

    StallPort_Receive#(Tuple2#(TOKEN,BUNDLE)) inQ  <- mkStallPort_Receive("dec2exe");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))    outQ <- mkStallPort_Send   ("exe2mem");

    Port_Send#(Tuple2#(TOKEN, ISA_ADDRESS))      rewindQ <- mkPort_Send("rewind");
    Port_Send#(Tuple2#(ISA_ADDRESS,BRANCH_ATTR)) bptrainQ <- mkPort_Send("bp_train");

    Port_Send#(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX))) busQ <- mkPort_Send("exe_bus");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS) getResults <- mkConnection_Client("funcp_getResults");

    Reg#(EXECUTE_STATE) state <- mkReg(EXECUTE_STATE_EXEC);

    Reg#(TOKEN_TIMEP_EPOCH) epoch <- mkReg(0);

    //Local Controller
    Vector#(1, Port_Control) inports  = newVector();
    Vector#(4, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    outports[0] = outQ.ctrl;
    outports[1] = rewindQ.ctrl;
    outports[2] = busQ.ctrl;
    outports[3] = bptrainQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_exe <- mkEventRecorder(`EVENTS_EXECUTE_INSTRUCTION_EXECUTE);
    //Stats
    Stat stat_mpred <- mkStatCounter(`STATS_EXECUTE_BPRED_MISPREDS);

    Vector#(FUNCP_PHYSICAL_REGS, Reg#(Bool)) prfValid = newVector();

    function Bool good_epoch (TOKEN tok) = tok.timep_info.epoch == epoch;

    rule flush (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .bundle } &&& !good_epoch(tok));
        debug <= fshow("FLUSH: ") + fshow(tok);
        local_ctrl.startModelCC();
        debug.startModelCC();
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        bptrainQ.send(Invalid);
        busQ.send(Valid(bundle.dests));
        event_exe.recordEvent(Invalid);
    endrule

    rule exec (state == EXECUTE_STATE_EXEC &&& inQ.peek() matches tagged Valid { .tok, .* } &&& good_epoch(tok));
        local_ctrl.startModelCC();
        debug.startModelCC();
        if (outQ.canSend)
        begin
            debug <= fshow("EXEC: ") + fshow(tok);
            getResults.makeReq(initFuncpReqGetResults(tok));
            state <= EXECUTE_STATE_WORK;
        end
        else
        begin
           debug <= fshow("STALL PROPAGATED");
           inQ.pass();
           outQ.pass();
           rewindQ.send(Invalid);
           bptrainQ.send(Invalid);
           busQ.send(Invalid);
           event_exe.recordEvent(Invalid);
        end
    endrule

    rule bubble (state == EXECUTE_STATE_EXEC &&& inQ.peek() == Invalid);
        local_ctrl.startModelCC();
        debug.startModelCC();
        debug <= fshow("BUBBLE");
        let x <- inQ.receive();
        if (outQ.canSend)
            outQ.send(Invalid);
        else
            outQ.pass();
        rewindQ.send(Invalid);
        bptrainQ.send(Invalid);
        busQ.send(Invalid);
        event_exe.recordEvent(Invalid);
    endrule

    rule results (state == EXECUTE_STATE_WORK);
        getResults.deq();

        let rsp = getResults.getResp();
        let tok = rsp.token;
        let res = rsp.result;

        let x <- inQ.receive();
        if (x matches tagged Valid { .tok2, .bndl })
        begin
            let bundle = bndl;
            let pc = bundle.pc;
            case (res) matches
              tagged RBranchTaken .addr:
                begin
                    if (bundle.branchAttr matches tagged BranchTaken .tgt &&& tgt == addr)
                    begin
                        rewindQ.send(Invalid);
                        bptrainQ.send(Invalid);
                    end
                    else
                    begin
                        stat_mpred.incr();
                        epoch <= epoch + 1;
                        rewindQ.send(Valid(tuple2(tok,addr)));
                        bptrainQ.send(Valid(tuple2(pc,BranchTaken(addr))));
                    end
                    debug <= fshow("BRANCH TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h END-OF-EPOCH:%d", addr, epoch);
                end
              tagged RBranchNotTaken .addr:
                begin
                    case (bundle.branchAttr) matches
                        tagged BranchNotTaken .tgt:
                            begin
                                rewindQ.send(Invalid);
                                bptrainQ.send(Invalid);
                            end
                        tagged BranchTaken .tgt:
                            begin
                                stat_mpred.incr();
                                epoch <= epoch + 1;
                                rewindQ.send(Valid(tuple2(tok,addr)));
                                bptrainQ.send(Invalid);
                            end
                        tagged NotBranch:
                            begin
                                rewindQ.send(Invalid);
                                bptrainQ.send(Invalid); // XXX
                            end
                    endcase
                    debug <= fshow("BRANCH NOT-TAKEN: ") + fshow(tok) + $format(" ADDR:0x%h", addr);
                end
              tagged REffectiveAddr .ea:
                begin
                    rewindQ.send(Invalid);
                    bptrainQ.send(Invalid);
                end
              tagged RNop:
                begin
                    if (bundle.branchAttr matches tagged NotBranch)
                    begin
                        rewindQ.send(Invalid);
                        bptrainQ.send(Invalid);
                    end
                    else
                    begin
                        debug <= fshow("NON-BRANCH PREDICTED BRANCH: ") + fshow(tok);
                        stat_mpred.incr();
                        epoch <= epoch + 1; // XXX
                        rewindQ.send(Valid(tuple2(tok,pc+4)));
                        bptrainQ.send(Valid(tuple2(pc,NotBranch)));
                    end
                end
              tagged RTerminate .pf:
                begin
                    rewindQ.send(Invalid);
                    bundle.isTerminate = Valid(pf);
                    bptrainQ.send(Invalid);
                end
            endcase
            outQ.send(Valid(tuple2(tok, bundle)));
            busQ.send(Invalid);
            event_exe.recordEvent(Valid(zeroExtend(tok.index)));
            state <= EXECUTE_STATE_EXEC;
        end
    endrule

endmodule
