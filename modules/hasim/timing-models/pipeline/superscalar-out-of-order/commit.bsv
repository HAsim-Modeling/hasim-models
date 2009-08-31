import FShow::*;
import FIFO::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"
`include "hasim_model_services.bsh"

`include "hasim_pipeline_types.bsh"

module [HASIM_MODULE] mkCommit();
    TIMEP_DEBUG_FILE                                                            debugLog <- mkTIMEPDebugFile("pipe_com.out");

    PORT_NO_STALL_RECEIVE#(COMMIT_BUNDLE, `COMMIT_NUM)                        commitPort <- mkPortNoStallReceive("commit");
    PORT_CREDIT_SEND#(FAULT_BUNDLE, 1, 1)                                      faultPort <- mkPortCreditSend("fault");

    Reg#(FAULT_BUNDLE)                                                         faultBundle <- mkReg(makeNoFaultBundle());

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS, FUNCP_RSP_COMMIT_RESULTS) commitResults <- mkConnection_Client("funcp_commitResults");
    Connection_Client#(FUNCP_REQ_COMMIT_STORES, FUNCP_RSP_COMMIT_STORES)    commitStores <- mkConnection_Client("funcp_commitStores");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)                               modelCommit <- mkConnection_Send("model_commits");

    FIFO#(ROB_INDEX) stage2Q <- mkFIFO();

    rule commitResultsReq(True);
        if(commitPort.canReceive())
        begin
            modelCommit.send(tuple2(0, 1));
            let bundle <- commitPort.pop();
            
            debugLog.record($format("TOKEN %0d: commit REQ", bundle.token.index));
            commitResults.makeReq(FUNCP_REQ_COMMIT_RESULTS{token: bundle.token, abort: False});
            stage2Q.enq(bundle.epochRob);
        end
        else
        begin

            faultPort.send(faultBundle);
            faultBundle <= makeNoFaultBundle();

            debugLog.record($format("end cycle"));
            debugLog.nextModelCycle();
            commitPort.done;
        end
    endrule

    rule commitResultsResp(True);
        let resp = commitResults.getResp();
        commitResults.deq();

        let rob_epoch = stage2Q.first();
        stage2Q.deq();

        debugLog.record($format("TOKEN %0d: commit RESP", resp.token.index));

        if (resp.faultRedirect matches tagged Valid .new_addr)
        begin        
            faultBundle <= makeFaultBundle(resp.token, True, rob_epoch, new_addr);
        end
        else if(resp.storeToken matches tagged Valid .st_tok)  // Is token a store?
        begin
            debugLog.record($format("TOKEN %0d: committing STORES", resp.token.index));
            commitStores.makeReq(FUNCP_REQ_COMMIT_STORES{storeToken: st_tok});
        end
    endrule

    rule commitStoresResp(True);
        let resp = commitStores.getResp();
        debugLog.record($format("STORE TOKEN %0d: STORES done", resp.storeToken));
        commitStores.deq();
    endrule
endmodule
