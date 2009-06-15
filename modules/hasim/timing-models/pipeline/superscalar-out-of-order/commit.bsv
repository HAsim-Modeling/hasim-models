import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"
`include "hasim_controller.bsh"

`include "hasim_pipeline_types.bsh"

module [HASIM_MODULE] mkCommit();
    TIMEP_DEBUG_FILE                                                            debugLog <- mkTIMEPDebugFile("pipe_com.out");

    PORT_NO_STALL_RECEIVE#(COMMIT_BUNDLE, `COMMIT_NUM)                        commitPort <- mkPortNoStallReceive("commit");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES)    commitStores <- mkConnection_Client("funcp_commitStores");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)                               modelCommit <- mkConnection_Send("model_commits");

    rule commitResultsReq(True);
        if(commitPort.canReceive())
        begin
            modelCommit.send(tuple2(0, 1));
            let bundle <- commitPort.pop();
            
            debugLog.record($format("TOKEN %0d: commit REQ", bundle.token.index));
            commitResults.makeReq(FUNCP_REQ_COMMIT_RESULTS{token: bundle.token});
        end
        else
        begin
            debugLog.record($format("end cycle"));
            debugLog.nextModelCycle();
            commitPort.done;
        end
    endrule

    rule commitResultsResp(True);
        let resp = commitResults.getResp();
        commitResults.deq();

        debugLog.record($format("TOKEN %0d: commit RESP", resp.token.index));

        // Is token a store?
        if(resp.storeToken matches tagged Valid .st_tok)
        begin
            debugLog.record($format("TOKEN %0d: committing STORES (STORE TOKEN: %0d)", resp.token.index, st_tok));
            commitStores.makeReq(FUNCP_REQ_COMMIT_STORES{storeToken: st_tok});
        end
    endrule

    rule commitStoresResp(True);
        let resp = commitStores.getResp();
        debugLog.record($format("STORE TOKEN %0d: STORES done", resp.storeToken));
        commitStores.deq();
    endrule
endmodule
