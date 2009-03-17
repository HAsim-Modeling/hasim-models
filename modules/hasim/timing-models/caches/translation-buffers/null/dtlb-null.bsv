import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/memory_base_types.bsh"

// mkDTLB

// An DTLB module that always hits.

module [HASIM_MODULE] mkDTLB();


    // UnModel State
    
    FIFO#(DMEM_BUNDLE) stage2Q <- mkFIFO();


    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE,
                       FUNCP_RSP_DO_DTRANSLATE)      doDTranslate   <- mkConnection_Client("funcp_doDTranslate");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_STALL_RECV_MULTICTX#(DMEM_BUNDLE) reqFromInQ <- mkPortStallRecv_MultiCtx("DTLBQ");

    // Outgoing ports to CPU
    PORT_STALL_SEND_MULTICTX#(DMEM_BUNDLE) rspToOutQ <- mkPortStallSend_MultiCtx("DMemQ");


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0] = reqFromInQ.ctrl;
    inports[1] = rspToOutQ.ctrl;
    outports[0] = rspToOutQ.ctrl;
    outports[1] = reqFromInQ.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    rule stage1_instReq (True);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // check request type
        if (reqFromInQ.canDeq(ctx) && rspToOutQ.canEnq(ctx))
	begin
        
            // See if it's a memory operation.
            let req = reqFromInQ.peek(ctx);
            
            if ((req.isLoad || req.isStore) && !tokIsPoisoned(req.token) && !req.isJunk)
            begin

                // An actual TLB would do something with the virtual
                // address to determine hit or miss. We always hit.
            
                // Pass it to the the functional partition, 
                // which actually translates the address.
                doDTranslate.makeReq(initFuncpReqDoDTranslate(req.token));
            
                // We assume the responses come back in order. Is this bad?
                stage2Q.enq(req);
            
            end
            else
            begin
            
                // It's not a memory operation, or it's junk. Just send it on.
	        rspToOutQ.doEnq(ctx, initDTLBHit(req, ?));
                reqFromInQ.doDeq(ctx);
                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(ctx, 1);
            
            end

	end
        else
	begin

            // Propogate the bubble.
	    rspToOutQ.noEnq(ctx);
            reqFromInQ.noDeq(ctx);
            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(ctx, 2);

	end
         

    endrule
     
    rule stage2_instRsp (True);
        
        let req = stage2Q.first();
        stage2Q.deq();
        
        // Get the response from the functional partition.
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();
        
        // Update with the latest token.
        req.token = rsp.token;

        // Get our context from the token.
        let ctx = tokContextId(rsp.token);

        // Always hit.
	rspToOutQ.doEnq(ctx, initDTLBHit(req, rsp.physicalAddress));
        reqFromInQ.doDeq(ctx);

        // End of model cycle. (Path 3)
        localCtrl.endModelCycle(ctx, 3);
     
    endrule

endmodule
