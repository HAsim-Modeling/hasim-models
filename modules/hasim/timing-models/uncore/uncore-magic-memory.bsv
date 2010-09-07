import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"

module [HASIM_MODULE] mkUncore 
    // interface:
        ();

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqFromCore <- mkPortStallRecv_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspToCore   <- mkPortStallSend_Multiplexed("L1_Cache_InQ");

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    outctrls[0]  = reqFromCore.ctrl.out;
    outctrls[1]  = rspToCore.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    rule stage1_delay (True);
    
        let cpu_iid <- localCtrl.startModelCycle();
    
        let m_req <- reqFromCore.receive(cpu_iid);
        let can_enq <- rspToCore.canEnq(cpu_iid);

        if (m_req matches tagged Valid .req)
        begin

            if (req.isStore)
            begin

                reqFromCore.doDeq(cpu_iid);
                rspToCore.noEnq(cpu_iid);

            end
            else if (can_enq)
            begin

                reqFromCore.doDeq(cpu_iid);
                let rsp = initMemRsp(req.physicalAddress, req.opaque);
                rspToCore.doEnq(cpu_iid, rsp);

            end
            else
            begin

                reqFromCore.noDeq(cpu_iid);
                rspToCore.noEnq(cpu_iid);

            end
        end
        else
        begin

            reqFromCore.noDeq(cpu_iid);
            rspToCore.noEnq(cpu_iid);

        end

        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

endmodule
