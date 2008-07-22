import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

`include "PipelineTypes.bsv"
`include "DebugFile.bsv"

`include "funcp_simulated_memory.bsh"

typedef enum { FETCH_STATE_REWIND_REQ, FETCH_STATE_REWIND_RESP, FETCH_STATE_TOKEN_REQ, FETCH_STATE_INST_REQ, FETCH_STATE_INST_RESP } FETCH_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch();
    DebugFile                                                                               debug <- mkDebugFile("Fetch.out");

    PORT_BANDWIDTH_CREDIT_SEND#(FETCH_BUNDLE, `FETCH_NUM, `FETCH_CREDITS)               fetchPort <- mkPortBandwidthCreditSend("fetch");

    PORT_RECEIVE#(REWIND_BUNDLE)                                                      resteerPort <- mkPortReceive("resteer");

    Connection_Client#(UNIT,TOKEN)                                                    newInFlight <- mkConnection_Client("funcp_newInFlight");
    Connection_Client#(Tuple2#(TOKEN,ISA_ADDRESS), Tuple2#(TOKEN,ISA_INSTRUCTION)) getInstruction <- mkConnection_Client("funcp_getInstruction");
    Connection_Client#(TOKEN,UNIT)                                                  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    Reg#(ISA_ADDRESS)                                                                          pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(FETCH_STATE)                                                                       state <- mkReg(FETCH_STATE_REWIND_REQ);
    Reg#(ROB_INDEX)                                                                      epochRob <- mkRegU();
    Reg#(Bool)                                                                       afterResteer <- mkReg(False);
    Reg#(TOKEN_TIMEP_EPOCH)                                                                 epoch <- mkReg(0);

    Reg#(Bit#(32))                                                                         fpgaCC <- mkReg(0);
    rule inc(True);
        fpgaCC <= fpgaCC + 1;
    endrule

    rule rewindReq(state == FETCH_STATE_REWIND_REQ);
        let bundle <- resteerPort.pop();
        debug <= $format("rewindReq %d", fpgaCC) + fshow(bundle);
        if(bundle.mispredict)
        begin
            pc <= bundle.addr;
            epochRob <= bundle.robIndex;
            afterResteer <= True;
            rewindToToken.makeReq(bundle.token);
            state <= FETCH_STATE_REWIND_RESP;
        end
        else
            state <= FETCH_STATE_TOKEN_REQ;
    endrule

    rule rewindResp(state == FETCH_STATE_REWIND_RESP);
        debug <= $format("rewindResp %d", fpgaCC);
        rewindToToken.deq();
        epoch <= epoch + 1;
        state <= FETCH_STATE_TOKEN_REQ;
    endrule

    rule tokenReq(state == FETCH_STATE_TOKEN_REQ);
        if(fetchPort.canSend())
        begin
            newInFlight.makeReq(?);
            state <= FETCH_STATE_INST_REQ;
        end
        else
        begin
            debug.endModelCC();
            state <= FETCH_STATE_REWIND_REQ;
            fetchPort.done();
        end
    endrule

    rule instReq(state == FETCH_STATE_INST_REQ);
        let token = newInFlight.getResp();
        token.timep_info.epoch = epoch;
        newInFlight.deq();
        getInstruction.makeReq(tuple2(token, pc));
        state <= FETCH_STATE_INST_RESP;
    endrule

    rule instResp(state == FETCH_STATE_INST_RESP);
        match {.token, .inst} = getInstruction.getResp();
        getInstruction.deq();
        let bundle = FETCH_BUNDLE{token: token, inst: inst, pc: pc, prediction: False, epochRob: epochRob, afterResteer: afterResteer};
        debug <= $format("instResp ") + fshow(bundle);
        fetchPort.enq(bundle);
        pc <= pc + 4;
        afterResteer <= False;
        state <= FETCH_STATE_TOKEN_REQ;
    endrule
endmodule
