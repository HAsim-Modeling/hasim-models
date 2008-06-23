import hasim_common::*;
import soft_connections::*;
import hasim_modellib::*;
import hasim_isa::*;
import module_local_controller::*;

`include "asim/provides/funcp_simulated_memory.bsh"

import Vector::*;

typedef enum { STATE_REWIND, STATE_REW_RESP, STATE_TOKEN, STATE_INST, STATE_SEND } STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch ();

    Reg#(ISA_ADDRESS) pc <- mkReg(`PROGRAM_START_ADDR);

    StallPort_Send#(Tuple2#(TOKEN,ISA_INSTRUCTION))   outQ    <- mkStallPort_Send("fet2dec");
    Port_Receive#(Tuple2#(TOKEN,Maybe#(ISA_ADDRESS))) rewindQ <- mkPort_Receive  ("rewind", 1);

    Connection_Send#(Bool) model_cycle <- mkConnection_Send("model_cycle");

    Connection_Client#(UNIT,TOKEN)                     newInFlight    <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(Tuple2#(TOKEN,ISA_ADDRESS),
                       Tuple2#(TOKEN,ISA_INSTRUCTION)) getInstruction <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(TOKEN,UNIT)                     rewindToToken  <- mkConnection_Client("funcp_rewindToToken");

    Reg#(STATE) state <- mkReg(STATE_REWIND);

    Reg#(TOKEN_TIMEP_EPOCH) epoch <- mkReg(0);

    //Local Controller
    Vector#(0, Port_Control) inports  = newVector();
    Vector#(0, Port_Control) outports = newVector();
    //inports[0]  = inQ.ctrl;
    //outports[0] = outQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule rewind (state == STATE_REWIND);
        local_ctrl.startModelCC();
        model_cycle.send(?);
        let x <- rewindQ.receive();
        if (x matches tagged Valid { .tok, .ma })
        begin
            if (ma matches tagged Valid .a)
            begin
                rewindToToken.makeReq(tok);
                pc <= a;
                epoch <= epoch + 1;
                state <= STATE_REW_RESP;
            end
        end
        else
            state <= STATE_TOKEN;
    endrule

    rule rewindResp (state == STATE_REW_RESP);
        rewindToToken.deq();
        state <= STATE_TOKEN;
    endrule

    rule token (state == STATE_TOKEN && outQ.canSend);
        newInFlight.makeReq(?);
        state <= STATE_INST;
    endrule

    rule pass (state == STATE_TOKEN);
        outQ.pass();
        state <= STATE_REWIND;
    endrule

    rule inst (state == STATE_INST);
        newInFlight.deq();
        let tok = newInFlight.getResp();
        tok.timep_info = TIMEP_TokInfo { epoch: epoch, scratchpad: 0 };
        getInstruction.makeReq(tuple2(tok,pc));
        pc <= pc + 4;
        state <= STATE_SEND;
    endrule

    rule done (state == STATE_SEND);
        getInstruction.deq();
        let inst = getInstruction.getResp();
        outQ.send(Valid(inst));
        state <= STATE_REWIND;
    endrule

endmodule
