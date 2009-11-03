import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"
`include "asim/provides/fpga_components.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_memory_controller.bsh"
`include "asim/provides/hasim_last_level_cache.bsh"

`include "asim/dict/RINGID.bsh"

typedef enum
{

    READ_MEM_CTRL,
    READ_CORES,
    WRITE_MEM_CTRL
}
    CROSSBAR_STATE deriving (Eq, Bits);

module [HASIM_MODULE] mkInterconnect
    // interface:
        ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("interconnect_xbar.out");

    // ******* Ports *******

    // Queues to/from core
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, CORE_IC_REQ) reqFromCores <- mkPortStallRecv_Multiplexed("CoreMemOutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, CORE_IC_RSP) rspToCores   <- mkPortStallSend_Multiplexed("CoreMemInQ");
    
    // Queue to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_STALL_RECV#(MEM_CTRL_RSP) rspFromMemCtrl <- mkPortStallRecv("MemCtrlOutQ");
    PORT_STALL_SEND#(MEM_CTRL_REQ) reqToMemCtrl   <- mkPortStallSend("MemCtrlInQ");

    // NOTE: The module does not use a local controller, as it does not 
    // follow standard port-simulation scheduling. Instead it uses a 
    // multiplex controller which interacts with commands like a local controller,
    // but simulates the instances in serial.

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    
    inports[0] = reqFromCores.ctrl.in;
    inports[1] = rspToCores.ctrl.in;

    MULTIPLEX_CONTROLLER#(NUM_CPUS) multiplexCtrl <- mkMultiplexController(inports);
    
    
    // ******** Local State ********
    
    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.
    
    // Because this interconnect is just a static priority, we don't actually have to wait for every input
    // before giving answers. Instead we just control to the first CPU which asks for it, and deny all
    // other requests. This, of course, could impact CPU performance via starvation.

    // Identify which core the memory controller response should go to.
    Reg#(Maybe#(CPU_INSTANCE_ID)) coreDst <- mkReg(tagged Invalid);

    // Store the memory controller message to transfer to 
    // the virtual core when we get to it.
    Reg#(Maybe#(MEM_CTRL_RSP)) corePayload <- mkRegU();

    // Store the payload for transfer to the memory controller.
    Reg#(MEM_CTRL_REQ) memCtrlPayload <- mkRegU();

    // A bit which tracks if we've already granted the crossbar to someone.
    Reg#(Bool) crossbarInUse <- mkReg(False);

    // A bit which tracks if the controller has room.
    Reg#(Bool) ctrlCanEnq <- mkRegU();

    // Track where we are in the simulation flow.
    Reg#(CROSSBAR_STATE) state <- mkReg(READ_MEM_CTRL);


    // ****** Helper Functions ******
    
    // toMemCtrlReq

    function MEM_CTRL_REQ toMemCtrlReq(CORE_IC_REQ req, CPU_INSTANCE_ID iid);
    
        return MEM_CTRL_REQ
        {
            physicalAddress: req.physicalAddress,
            isStore: req.isStore,
            opaque: req.opaque,
            destination: iid
        };
    
    endfunction

    // toCoreMemRsp

    function CORE_IC_RSP toCoreMemRsp(MEM_CTRL_RSP rsp);
    
        return CORE_IC_RSP
        {
            physicalAddress: rsp.physicalAddress,
            opaque: rsp.opaque
        };
    
    endfunction


    // ********* Rules *********

    // stage1_readMemCtrl
    
    // Read any output from the memory controller and record which virtual
    // core it should go to.
    
    // Ports Read: 
    // * rspFromMemCtrl
    // * reqToMemCtrl.canEnq
    
    // Ports Written:
    // * None

    rule stage1_readMemCtrl (state == READ_MEM_CTRL && multiplexCtrl.running);
    
        
        debugLog.nextModelCycle();
    
        // Start by reading requests from the mem controller. 
        // If it has data to move we will try to transmit it to a core.
        
        let m_rsp <- rspFromMemCtrl.receive();
        
        if (m_rsp matches tagged Valid .rsp)
        begin

            // Record the message and where it should go for the next stage.
            corePayload <= m_rsp;
            debugLog.record_next_cycle($format("1: MEM CTRL RSP"));
            // Don't dequeue yet since the core may not have room.

        end
        else
        begin
        
            // Nothing from the memory controller at the moment.
            debugLog.record_next_cycle($format("1: NO RSP"));
            rspFromMemCtrl.noDeq();
        
        end
        
        // Record if the mem controller incoming queue has room.
        // If it is full, no core can use the crossbar.
        let ctrl_can_enq <- reqToMemCtrl.canEnq();
        ctrlCanEnq <= ctrl_can_enq;
        
        // Move on to the next stage.
        state <= READ_CORES;
        
    endrule
    
    rule stage2_readCores (state == READ_CORES);
    
        // Read each virtual core in succession.
        
        match {.cpu_iid, .done} <- multiplexCtrl.nextReadyInstance();
        
        let m_req <- reqFromCores.receive(cpu_iid);

        // The first core with a message for the
        // memory controller will win control of the crossbar.
        // (If the controller has room.)

        if (m_req matches tagged Valid .req)
        begin
        
            debugLog.record($format("2: CORE %0d REQ", cpu_iid));
            
            if (!crossbarInUse && ctrlCanEnq)
            begin
        
                debugLog.record($format("2: CORE %0d GRANT", cpu_iid));
                // This core can have the crossbar.
                crossbarInUse <= True;
                memCtrlPayload <= toMemCtrlReq(req, cpu_iid);

                reqFromCores.doDeq(cpu_iid);

            end
            else
            begin

                debugLog.record($format("2: CORE %0d RETRY. CTRL FULL: %0d", cpu_iid, pack(!ctrlCanEnq)));
                reqFromCores.noDeq(cpu_iid);
            
            end

        end
        else
        begin

            reqFromCores.noDeq(cpu_iid);
        
        end

        // See if the controller is trying to send a message to this core,
        // and if it has room to receive it.

        let core_can_enq <- rspToCores.canEnq(cpu_iid);
        
        if (corePayload matches tagged Valid .rsp &&& cpu_iid == rsp.destination)
        begin
        
            // assert: this path is taken at most once per model cycle.
        

            if (core_can_enq)
            begin

                debugLog.record($format("2: CORE %0d RSP", cpu_iid));
                // Yep, this is the destination, and it has room. Go ahead and transfer data.
                rspToCores.doEnq(cpu_iid, toCoreMemRsp(rsp));
                rspFromMemCtrl.doDeq();

            end
            else
            begin

                debugLog.record($format("2: CORE %0d RSP RETRY", cpu_iid));
                // The core has no room. We gotta stall the controller.
                rspToCores.noEnq(cpu_iid);
                rspFromMemCtrl.noDeq();
            
            end
        
        end
        else
        begin
        
            // No attempt to send anything to this core.
            rspToCores.noEnq(cpu_iid);
        
        end

        // See if we're done processing all the cores.
        if (done)
        begin
        
            state <= WRITE_MEM_CTRL;
        
        end
        
    endrule

    rule stage3_writeMemCtrl (state == WRITE_MEM_CTRL);
    
        // If someone got the crossbar, then we
        // should actually move the data.
    
        if (crossbarInUse)
        begin

            reqToMemCtrl.doEnq(memCtrlPayload);
        
        end
        else
        begin
        
            reqToMemCtrl.noEnq();
        
        end
        
        // Refresh all the state for the next model cycle.
        corePayload <= tagged Invalid;
        crossbarInUse <= False;
        
        state <= READ_MEM_CTRL;
    
    endrule

endmodule

