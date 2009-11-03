import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_interconnect.bsh"
`include "asim/provides/hasim_last_level_cache.bsh"
`include "asim/provides/hasim_memory_controller.bsh"

module [HASIM_MODULE] mkUncore 
    // interface:
        ();

    // Instantiate submodules
    let ic <- mkInterconnect();
   
    let memCtrl <- mkMemoryController();
    
    let lastLevelCache <- mkLastLevelCache();

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqFromCore <- mkPortStallRecv_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspToCore   <- mkPortStallSend_Multiplexed("L1_Cache_InQ");
    
    // Queues to/from on-chip interconnect
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, CORE_IC_REQ) reqToIC   <- mkPortStallSend_Multiplexed("CoreMemOutQ");
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, CORE_IC_RSP) rspFromIC <- mkPortStallRecv_Multiplexed("CoreMemInQ");
    
    Vector#(4, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    inctrls[2]  = reqToIC.ctrl.in;
    inctrls[3]  = rspFromIC.ctrl.in;
    outctrls[0]  = reqFromCore.ctrl.out;
    outctrls[1]  = rspToCore.ctrl.out;
    outctrls[2]  = reqToIC.ctrl.out;
    outctrls[3]  = rspFromIC.ctrl.out;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    rule stage1_coreReq (True);
    
        let cpu_iid <- localCtrl.startModelCycle();
    
        let m_req <- reqFromCore.receive(cpu_iid);
        let can_enq <- reqToIC.canEnq(cpu_iid);

        if (m_req matches tagged Valid .req &&& can_enq)
        begin
        
            reqFromCore.doDeq(cpu_iid);
            reqToIC.doEnq(cpu_iid, req);
        
        end
        else
        begin

            reqFromCore.noDeq(cpu_iid);
            reqToIC.noEnq(cpu_iid);

        end

        // Now check for responses from the interconnect station.
        let m_rsp <- rspFromIC.receive(cpu_iid);
        let can_rsp <- rspToCore.canEnq(cpu_iid);

        if (m_rsp matches tagged Valid .req &&& can_rsp)
        begin

            rspFromIC.doDeq(cpu_iid);
            let rsp = initMemRsp(req.physicalAddress, req.opaque);
            rspToCore.doEnq(cpu_iid, rsp);

        end
        else
        begin

            rspFromIC.noDeq(cpu_iid);
            rspToCore.noEnq(cpu_iid);

        end
        
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

endmodule
