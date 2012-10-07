import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

typedef union tagged
{
    void        STAGE2_bubble;
    IMEM_BUNDLE STAGE2_iTransRsp;
}
ITLB_STAGE2_STATE deriving (Eq, Bits);

// mkITLB

// An ITLB module that always hits.

module [HASIM_MODULE] mkITLB();


    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE,
                       FUNCP_RSP_DO_ITRANSLATE) doITranslate <- mkConnection_Client("funcp_doITranslate");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, IMEM_BUNDLE) reqFromFet <- mkPortRecv_Multiplexed("CPU_to_ITLB_req", `CPU_TO_ITLB_LATENCY);

    // Outgoing ports to CPU
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ITLB_OUTPUT) rspToIMem <- mkPortSend_Multiplexed("ITLB_to_CPU_rsp");


    // ****** Local Controller ******

    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(1, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0] = reqFromFet.ctrl;
    outports[0] = rspToIMem.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkNamedLocalController("ITLB", inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, ITLB_STAGE2_STATE) stage2Ctrl <- mkBufferedStageController();

    (* conservative_implicit_conditions *)
    rule stage1_instReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // read input port
        let req_from_cpu <- reqFromFet.receive(cpu_iid);

        // check request type
        case (req_from_cpu) matches
	    tagged Invalid:
	    begin

                // Tell the next stage to finish the bubble.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

	    end
	    tagged Valid .req:
	    begin


                // An actual TLB would do something with the virtual
                // address to determine hit or miss. We always hit.

                // Pass it to the the functional partition, 
                // which actually translates the address.
                doITranslate.makeReq(initFuncpReqDoITranslate(req.ctx_id, req.virtualAddress));
                
                // Tell the next stage to get the response.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_iTransRsp req);

	    end

        endcase
         

    endrule
     
    rule stage2_instRsp (True);
        
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        if (state matches tagged STAGE2_bubble)
        begin

            // Propogate the bubble.
	    rspToIMem.send(cpu_iid, tagged Invalid);
            localCtrl.endModelCycle(cpu_iid, 1);
        
        end
        else if (state matches tagged STAGE2_iTransRsp .req)
        begin
        
            // TODO: handle rsp.hasMore

            // Get the response from the functional partition.
            let rsp = doITranslate.getResp();
            doITranslate.deq();

            // Always hit.
	    rspToIMem.send(cpu_iid, tagged Valid initITLBHit(req, rsp.physicalAddress, rsp.offset));

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);
        
        end
     
    endrule

endmodule
