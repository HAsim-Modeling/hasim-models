import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

//import PipelineTypes::*;
import FShow::*;
import Vector::*;

`include "asim/dict/EVENTS_DECODE.bsh"

typedef enum { DECODE_STATE_BUS1, DECODE_STATE_BUS2, DECODE_STATE_BUS3, DECODE_STATE_INST, DECODE_STATE_DEP, DECODE_STATE_SEND } DECODE_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkDecode ();

    DebugFile debug <- mkDebugFile("pipe_decode.out");

    StallPort_Receive#(Tuple2#(TOKEN,FETCH_BUNDLE)) inQ <- mkStallPort_Receive("fet2dec");
    StallPort_Send#(Tuple2#(TOKEN,BUNDLE))          outQ <- mkStallPort_Send   ("dec2exe");

    Vector#(3,Port_Receive#(ISA_INST_DSTS)) busQ = newVector();
    busQ[0] <- mkPort_Receive("exe_bus", 1);
    busQ[1] <- mkPort_Receive("mem_bus", 1);
    busQ[2] <- mkPort_Receive("wb_bus", 1);

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    Reg#(DECODE_STATE) state <- mkReg(DECODE_STATE_BUS1);
    Reg#(Maybe#(FUNCP_RSP_GET_DEPENDENCIES)) memoDependencies <- mkReg(Invalid);

    //Local Controller
    Vector#(4, Port_Control) inports  = newVector();
    Vector#(1, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = busQ[0].ctrl;
    inports[2]  = busQ[1].ctrl;
    inports[3]  = busQ[2].ctrl;
    outports[0] = outQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    //Events
    EventRecorder event_dec <- mkEventRecorder(`EVENTS_DECODE_INSTRUCTION_DECODE);

    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_PHYSICAL_REGS);

    Vector#(FUNCP_PHYSICAL_REGS,Bool) prfValid_init = newVector();

    for (Integer i = 0; i < numIsaArchRegisters; i = i + 1)
        prfValid_init[i] = True;

    for (Integer i = numIsaArchRegisters; i < numFuncpPhyRegisters; i = i + 1)
        prfValid_init[i] = False;

    Reg#(Vector#(FUNCP_PHYSICAL_REGS,Bool)) prfValid <- mkReg(prfValid_init);

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
        Vector#(FUNCP_PHYSICAL_REGS,Bool) prf_valid = prfValid;
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr }) begin
                prf_valid[pr] = False;
                debug <= $format("PRF: PR %d <= 0 (alloc)", pr);
            end
        end
        prfValid <= prf_valid;
      endaction
    endfunction

    function Action markPRFValid(Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dst);
      action
        Vector#(FUNCP_PHYSICAL_REGS,Bool) prf_valid = prfValid;
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dst[i] matches tagged Valid .pr) begin
                prf_valid[pr] = True;
                debug <= $format("PRF: PR %d <= 1 (free)", pr);
            end
        end
        prfValid <= prf_valid;
      endaction
    endfunction

    function BUNDLE makeBundle(FETCH_BUNDLE fbndl, ISA_DST_MAPPING dstmap);
        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end
        return BUNDLE { isLoad:  isaIsLoad(fbndl.inst),
                        isStore: isaIsStore(fbndl.inst),
                        isTerminate: Invalid,
                        pc: fbndl.pc,
                        branchAttr: fbndl.branchAttr,
                        effAddr: ?,
                        dests: dests };
    endfunction

    rule bus1 (state == DECODE_STATE_BUS1);
        local_ctrl.startModelCC();
        debug.startModelCC();
        let mpregs <- busQ[0].receive();
        if (mpregs matches tagged Valid .pregs)
            markPRFValid(pregs);
        state <= DECODE_STATE_BUS2;
    endrule
    rule bus2 (state == DECODE_STATE_BUS2);
        let mpregs <- busQ[1].receive();
        if (mpregs matches tagged Valid .pregs)
            markPRFValid(pregs);
        state <= DECODE_STATE_BUS3;
    endrule
    rule bus3 (state == DECODE_STATE_BUS3);
        let mpregs <- busQ[2].receive();
        if (mpregs matches tagged Valid .pregs)
            markPRFValid(pregs);
        state <= DECODE_STATE_INST;
    endrule

    rule stall (state == DECODE_STATE_INST && !outQ.canSend);
        debug <= fshow("STALL PROPAGATED");
        inQ.pass();
        outQ.pass();
        event_dec.recordEvent(Invalid);
        state <= DECODE_STATE_BUS1;
    endrule

    rule bubble (state == DECODE_STATE_INST && outQ.canSend && !isValid(inQ.peek));
        debug <= fshow("BUBBLE");
        let x <- inQ.receive();
        outQ.send(Invalid);
        event_dec.recordEvent(Invalid);
        state <= DECODE_STATE_BUS1;
    endrule

    rule inst (state == DECODE_STATE_INST &&& outQ.canSend &&& inQ.peek() matches tagged Valid { .tok, .* });
        if (!isValid(memoDependencies))
            getDependencies.makeReq(initFuncpReqGetDependencies(tok));
        state <= DECODE_STATE_DEP;
    endrule

    rule dep (state == DECODE_STATE_DEP);
        if (!isValid(memoDependencies)) begin
            let rsp = getDependencies.getResp();
            memoDependencies <= Valid(rsp);
            getDependencies.deq();
        end
        state <= DECODE_STATE_SEND;
    endrule

    rule send (state == DECODE_STATE_SEND &&& memoDependencies matches tagged Valid .rsp);
    
        let tok = rsp.token;
        if (readyToGo(rsp.srcMap))
        begin
            markPRFInvalid(rsp.dstMap);
            let mtup <- inQ.receive();
            if (mtup matches tagged Valid { .tok2, .fetchbundle })
            begin
                let bundle = makeBundle(fetchbundle, rsp.dstMap);
                outQ.send(Valid(tuple2(tok,bundle)));
                event_dec.recordEvent(Valid(zeroExtend(tok.index)));
                debug <= fshow("SEND: ") + fshow(tok) + fshow(" INST:") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle);
            end
            memoDependencies <= Invalid;
        end
        else
        begin
            debug <= fshow("STALL ON DEPENDENCY: ") + fshow(tok);
            inQ.pass();
            outQ.send(Invalid);
            event_dec.recordEvent(Invalid);
        end
        state <= DECODE_STATE_BUS1;
    endrule

endmodule
