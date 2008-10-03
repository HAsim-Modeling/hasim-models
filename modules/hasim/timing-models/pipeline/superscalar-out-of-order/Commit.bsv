import hasim_common::*;
import hasim_modellib::*;

`include "PipelineTypes.bsv"

`include "hasim_controller.bsh"

module [HASIM_MODULE] mkCommit();
    TIMEP_DEBUG_FILE                                                            debugLog <- mkTIMEPDebugFile("pipe_com.out");

    PORT_NO_STALL_RECEIVE#(COMMIT_BUNDLE, `COMMIT_NUM)                        commitPort <- mkPortNoStallReceive("commit");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES)    commitStores <- mkConnection_Client("funcp_commitStores");

    Connection_Send#(MODEL_NUM_COMMITS)                                      modelCommit <- mkConnection_Send("model_commits");

    rule commitResultsReq(True);
        if(commitPort.canReceive())
        begin
            modelCommit.send(1);
            let bundle <- commitPort.pop();
            
            // Flag stores in scratchpad
            bundle.token.timep_info.scratchpad = zeroExtend(pack(bundle.isStore));

            debugLog.record($format("TOKEN %0d: commit REQ", bundle.token.index));
            commitResults.makeReq(FUNCP_REQ_COMMIT_RESULTS{token: bundle.token});
        end
        else
        begin
            debugLog.nextModelCycle();
            commitPort.done;
        end
    endrule

    rule commitResultsResp(True);
        let resp = commitResults.getResp();
        commitResults.deq();

        debugLog.record($format("TOKEN %0d: commit RESP", resp.token.index));

        // Is token a store?
        if(resp.token.timep_info.scratchpad == 1)
        begin
            debugLog.record($format("TOKEN %0d: committing STORES", resp.token.index));
            commitStores.makeReq(FUNCP_REQ_COMMIT_STORES{token: resp.token});
        end
    endrule

    rule commitStoresResp(True);
        let resp = commitStores.getResp();
        debugLog.record($format("TOKEN %0d: STORES done", resp.token.index));
        commitStores.deq();
    endrule
endmodule
