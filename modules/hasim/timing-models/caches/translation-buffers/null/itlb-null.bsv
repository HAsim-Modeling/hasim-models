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

// mkITLB

// An ITLB module that always hits.

module [HASIM_MODULE] mkITLB();


    // UnModel State
    
    FIFO#(IMEM_BUNDLE) stage2Q <- mkFIFO();


    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE,
                       FUNCP_RSP_DO_ITRANSLATE)      doITranslate   <- mkConnection_Client("funcp_doITranslate");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_RECV_MULTICTX#(IMEM_BUNDLE) reqFromFet <- mkPortRecv_MultiCtx("CPU_to_ITLB_req", 0);

    // Outgoing ports to CPU
    PORT_SEND_MULTICTX#(ITLB_OUTPUT) rspToIMem <- mkPortSend_MultiCtx("ITLB_to_CPU_rsp");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS) inports = newVector();
    Vector#(1, PORT_CONTROLS) outports = newVector();
    inports[0] = reqFromFet.ctrl;
    outports[0] = rspToIMem.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);

    rule stage1_instReq (True);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();

        // read input port
        let req_from_cpu <- reqFromFet.receive(ctx);

        // check request type
        case (req_from_cpu) matches
	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        rspToIMem.send(ctx, tagged Invalid);
                localCtrl.endModelCycle(ctx, 1);

	    end
	    tagged Valid .req:
	    begin


                // An actual TLB would do something with the virtual
                // address to determine hit or miss. We always hit.

                // Pass it to the the functional partition, 
                // which actually translates the address.
                doITranslate.makeReq(initFuncpReqDoITranslate(req.token, req.virtualAddress));
                
                // We assume the responses come back in order. Is this bad?
                stage2Q.enq(req);

	    end

        endcase
         

    endrule
     
    rule stage2_instRsp (True);
        
        let req = stage2Q.first();
        stage2Q.deq();
        
        // Get the response from the functional partition.
        let rsp = doITranslate.getResp();
        doITranslate.deq();
        
        // Update with the latest token.
        req.token = rsp.token;

        // Get our context from the token.
        let ctx = tokContextId(rsp.token);

        // Always hit.
	rspToIMem.send(ctx, tagged Valid initITLBHit(req, rsp.physicalAddress));

        // End of model cycle. (Path 2)
        localCtrl.endModelCycle(ctx, 2);
     
    endrule

endmodule
