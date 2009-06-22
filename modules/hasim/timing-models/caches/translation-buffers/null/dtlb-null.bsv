import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

typedef union tagged
{
    DMEM_BUNDLE STAGE2_nonMemOp;
    DMEM_BUNDLE STAGE2_dTransRsp;
    void STAGE2_bubble;
}
DTLB_STAGE2_STATE deriving (Bits, Eq);


// mkDTLB

// An DTLB module that always hits.

module [HASIM_MODULE] mkDTLB();


    // ****** UnModel State ******
    
    Reg#(Maybe#(Tuple2#(INSTANCE_ID#(NUM_CPUS), DMEM_BUNDLE))) dTransStall <- mkReg(tagged Invalid);

    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_DO_DTRANSLATE,
                       FUNCP_RSP_DO_DTRANSLATE)      doDTranslate   <- mkConnection_Client("funcp_doDTranslate");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE) reqFromInQ <- mkPortStallRecv_Multiplexed("DTLBQ");

    // Outgoing ports to CPU
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, DMEM_BUNDLE) rspToOutQ <- mkPortStallSend_Multiplexed("DMemQ");


    // ****** Local Controller ******

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0] = reqFromInQ.ctrl.in;
    inports[1] = rspToOutQ.ctrl.in;
    outports[0] = rspToOutQ.ctrl.out;
    outports[1] = reqFromInQ.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    STAGE_CONTROLLER#(NUM_CPUS, DTLB_STAGE2_STATE) stage2Ctrl <- mkStageController();


    (* conservative_implicit_conditions *)
    rule stage1_dTransReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();


        let m_req <-reqFromInQ.receive(cpu_iid);

        // check request type
        let can_enq <- rspToOutQ.canEnq(cpu_iid);
        if (m_req matches tagged Valid .req &&& can_enq)
	begin
        
            // See if it's a memory operation.

            // Dequeue the FIFO.
            reqFromInQ.doDeq(cpu_iid);
            
            if ((req.isLoad || req.isStore) && !tokIsPoisoned(req.token) && !req.token.dummy)
            begin

                // An actual TLB would do something with the virtual
                // address to determine hit or miss. We always hit.
            
                // Pass it to the the functional partition, 
                // which actually translates the address.
                doDTranslate.makeReq(initFuncpReqDoDTranslate(req.token));
            
                // Tell the next stage to get the response.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_dTransRsp req);
            
            end
            else
            begin
            
                // It's not a memory operation, or it's junk. Just send it on.

                // Tell the next stage to just send the request on.
                stage2Ctrl.ready(cpu_iid, tagged STAGE2_nonMemOp req);
            
            end

	end
        else
	begin

            // No dequeue to do.
            reqFromInQ.noDeq(cpu_iid);
            
            // Finish the bubble in the next stage.
            stage2Ctrl.ready(cpu_iid, tagged STAGE2_bubble);

	end
         

    endrule
     
    rule stage2_dTransRsp (!isValid(dTransStall));
        
        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();
        
        if (state matches tagged STAGE2_bubble)
        begin

            // Propogate the bubble.
            rspToOutQ.noEnq(cpu_iid);
            localCtrl.endModelCycle(cpu_iid, 1);
        
        end
        else if (state matches tagged STAGE2_nonMemOp .req)
        begin
        
            rspToOutQ.doEnq(cpu_iid, initDTLBHit(req, ?));
            localCtrl.endModelCycle(cpu_iid, 2);
        
        end
        else if (state matches tagged STAGE2_dTransRsp .req)
        begin

            // Get the response from the functional partition.
            let rsp = doDTranslate.getResp();
            doDTranslate.deq();

            if (!rsp.hasMore)
            begin

                // Update with the latest token.
                let new_req = req;
                new_req.token = rsp.token;

                // Always hit.
                rspToOutQ.doEnq(cpu_iid, initDTLBHit(new_req, rsp.physicalAddress));

                // End of model cycle. (Path 3)
                localCtrl.endModelCycle(cpu_iid, 3);
            
            end
            else
            begin
            
                dTransStall <= tagged Valid tuple2(cpu_iid, req);
            
            end
        
        end
     
    endrule
    
    rule stage2_dTransUnaligned (dTransStall matches tagged Valid {.cpu_iid, .req});
    
        // Get the response from the functional partition.
        let rsp = doDTranslate.getResp();
        doDTranslate.deq();

        if (!rsp.hasMore)
        begin

            // Update with the latest token.
            let new_req = req;
            new_req.token = rsp.token;

            // Always hit.
            rspToOutQ.doEnq(cpu_iid, initDTLBHit(new_req, rsp.physicalAddress));

            // End of model cycle. (Path 4)
            localCtrl.endModelCycle(cpu_iid, 4);
            dTransStall <= tagged Invalid;

        end

    endrule

endmodule
