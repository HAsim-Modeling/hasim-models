import hasim_common::*;
import hasim_modellib::*;
import hasim_isa::*;

import funcp_interface::*;

module [HASIM_MODULE] mkGetResults();
    Connection_Server#(TOKEN, FUNCP_GET_RESULTS_MSG) getResults0 <- mkConnection_Server("funcp_getResults0");
    Connection_Server#(TOKEN, FUNCP_GET_RESULTS_MSG) getResults1 <- mkConnection_Server("funcp_getResults1");

    Connection_Client#(TOKEN, FUNCP_GET_RESULTS_MSG)  getResults <- mkConnection_Client("funcp_getResults");

    rule enq0(True);
        let token = getResults0.getReq();
        getResults0.deq();
        token.timep_info.scratchpad = 0;
        getResults.makeReq(token);
    endrule

    rule enq1(True);
        let token = getResults1.getReq();
        getResults1.deq();
        token.timep_info.scratchpad = 1;
        getResults.makeReq(token);
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
