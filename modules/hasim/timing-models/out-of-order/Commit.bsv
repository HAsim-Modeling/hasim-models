import hasim_common::*;
import hasim_modellib::*;

`include "PipelineTypes.bsv"

`include "hasim_controller.bsh"

module [HASIM_MODULE] mkCommit();
    DebugFile                                                                    debug <- mkDebugFile("Commit.out");

    PORT_BANDWIDTH_CREDIT_RECEIVE#(COMMIT_BUNDLE, `COMMIT_NUM, `COMMIT_NUM) commitPort <- mkPortBandwidthCreditReceive("commit", `COMMIT_NUM);

    Connection_Client#(TOKEN,TOKEN)                                      commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(TOKEN,TOKEN)                                       commitStores <- mkConnection_Client("funcp_commitStores");

    Connection_Send#(MODEL_NUM_COMMITS)                                    modelCommit <- mkConnection_Send("model_commits");

    rule commitResultsReq(True);
        if(commitPort.canReceive())
        begin
            modelCommit.send(1);
            let bundle <- commitPort.pop();
            bundle.token.timep_info.scratchpad = zeroExtend(pack(bundle.isStore));
            commitResults.makeReq(bundle.token);
        end
        else
        begin
            debug.endModelCC();
            commitPort.done(`COMMIT_NUM);
        end
    endrule

    rule commitResultsResp(True);
        let token = commitResults.getResp();
        commitResults.deq();
        if(token.timep_info.scratchpad == 1)
        begin
            debug <= $format("commiting stores");
            commitStores.makeReq(token);
        end
    endrule

    rule commitStoresResp(True);
        let token = commitStores.getResp();
        commitStores.deq();
    endrule
endmodule
