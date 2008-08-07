import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

module [HASIM_MODULE] mkGetResults();
    Connection_Server#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS) getResults0 <- mkConnection_Server("funcp_getResults0");
    Connection_Server#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS) getResults1 <- mkConnection_Server("funcp_getResults1");

    Connection_Client#(FUNCP_REQ_GET_RESULTS, FUNCP_RSP_GET_RESULTS)  getResults <- mkConnection_Client("funcp_getResults");

    rule enq0(True);
        let req = getResults0.getReq();
        getResults0.deq();
        req.token.timep_info.scratchpad = 0;
        getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: req.token});
    endrule

    rule enq1(True);
        let req = getResults1.getReq();
        getResults1.deq();
        req.token.timep_info.scratchpad = 1;
        getResults.makeReq(FUNCP_REQ_GET_RESULTS{token: req.token});
    endrule

    rule deq(True);
        let res = getResults.getResp();
        getResults.deq();
        if(res.token.timep_info.scratchpad == 0)
            getResults0.makeResp(res);
        else
            getResults1.makeResp(res);
    endrule
endmodule
