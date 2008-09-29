import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

typedef enum {ALU_REQ, MEM_REQ} GET_RESULTS_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkGetResults();
    PORT_CREDIT_RECEIVE#(FUNCP_REQ_GET_RESULTS, `ALU_NUM, LOG_ALU_NUM)    aluIn <- mkPortCreditReceive("aluGetResultsReq");
    PORT_CREDIT_RECEIVE#(FUNCP_REQ_GET_RESULTS, `MEM_NUM, LOG_MEM_NUM)    memIn <- mkPortCreditReceive("memGetResultsReq");

    Connection_Send#(FUNCP_RSP_GET_RESULTS)                              aluOut <- mkConnection_Send("aluGetResultsResp");
    Connection_Send#(FUNCP_RSP_GET_RESULTS)                              memOut <- mkConnection_Send("memGetResultsResp");

    Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS) getResults <- mkConnection_Client("funcp_getResults");

    Reg#(GET_RESULTS_STATE)                                               state <- mkReg(ALU_REQ);

    rule aluReq(state == ALU_REQ);
        if(aluIn.canReceive)
        begin
            let req <- aluIn.pop;
            req.token.timep_info.scratchpad = 0;
            getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: req.token});
        end
        else
        begin
            aluIn.done(`ALU_NUM);
            state <= MEM_REQ;
        end
    endrule

    rule memReq(state == MEM_REQ);
        if(memIn.canReceive)
        begin
            let req <- memIn.pop;
            req.token.timep_info.scratchpad = 1;
            getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: req.token});
        end
        else
        begin
            memIn.done(`MEM_NUM);
            state <= ALU_REQ;
        end
    endrule

    rule deq;
        let res = getResults.getResp();
        getResults.deq();
        if(res.token.timep_info.scratchpad == 0)
            aluOut.send(res);
        else
            memOut.send(res);
    endrule
endmodule
