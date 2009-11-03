import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/hasim_memory_controller.bsh"

typedef enum
{

    MEM_CTRL,
    CORES
}
RINGSTOP_STATE deriving (Eq, Bits);

typedef struct
{
    CPU_INSTANCE_ID coreID;
    union tagged
    {
        MEM_CTRL_RSP PKT_rsp;
        CORE_IC_REQ PKT_req;
    }
    payload;
}
RING_PACKET deriving (Eq, Bits);

module [HASIM_MODULE] mkInterconnect
    // interface:
        ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("interconnect_ring.out");

    // ******** Ports *******

    // Queues to/from cores
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, CORE_IC_REQ) reqFromCores <- mkPortStallRecv_Multiplexed("CoreMemOutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, CORE_IC_RSP) rspToCores   <- mkPortStallSend_Multiplexed("CoreMemInQ");

    // Links to next/prev ring stop.
    // Note: These ports actually connect together (they're the same port).
    // This is the main technique which makes this module work.
    // The token reordering keeps things in the correct order.
    // Note: We need an extra instance here for the memory controller's ring stop.
    PORT_SEND_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), RING_PACKET) toNextRingStop   <- mkPortSend_Multiplexed("ring_interconnect");
    PORT_RECV_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), RING_PACKET) fromPrevRingStop <- mkPortRecv_Multiplexed_ReorderFirstToLast("ring_interconnect", 1);
    
    // Queues to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_STALL_RECV#(MEM_CTRL_RSP) rspFromMemCtrl <- mkPortStallRecv("MemCtrlOutQ");
    PORT_STALL_SEND#(MEM_CTRL_REQ) reqToMemCtrl   <- mkPortStallSend("MemCtrlInQ");

    // NOTE: The module does not use a local controller, as it does not 
    // follow standard port-simulation scheduling. 
    // Instead this module uses a multiplex controller to read/write the virtual CPU instances in sequence.

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    inports[0] = reqFromCores.ctrl.in;
    inports[1] = rspToCores.ctrl.in;
    // inports[2] = fromPrevRingStop.ctrl;

    MULTIPLEX_CONTROLLER#(NUM_CPUS) multiplexCtrl <- mkMultiplexController(inports);
    
    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    // A bit where we are in the simulation flow.
    Reg#(RINGSTOP_STATE) state <- mkReg(MEM_CTRL);

    function RING_PACKET reqToPacket(CORE_IC_REQ req, CPU_INSTANCE_ID iid);
    
        return RING_PACKET
        {
            coreID: iid,
            payload: tagged PKT_req req
        };

    endfunction

    function RING_PACKET rspToPacket(MEM_CTRL_RSP rsp);
    
        return RING_PACKET
        {
            coreID: rsp.destination,
            payload: tagged PKT_rsp rsp
        };

    endfunction

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
    

    // ******* Rules *******


    rule stage1_MemCtrl (state == MEM_CTRL && multiplexCtrl.running());
    
        // Start with the mem controller's ring stop.
        debugLog.nextModelCycle();
        
        // Check for messages inbound to the memory controller.
        
        let m_inc <- fromPrevRingStop.receive(0);
        let can_enq <- reqToMemCtrl.canEnq();
        
        // Track which message we're sending to the next ring stop, if any.
        let message_out = tagged Invalid;
    
        if (m_inc matches tagged Valid .inc)
        begin
        
            if (inc.payload matches tagged PKT_req .req)
            begin

                debugLog.record_next_cycle($format("1: MEM CTRL INC REQ"));
                // assert reqToMemCtrl.canEnq
                reqToMemCtrl.doEnq(toMemCtrlReq(req, inc.coreID));

            end
            else
            begin

                debugLog.record_next_cycle($format("1: MEM CTRL PASS THROUGH"));
                message_out = m_inc;
                reqToMemCtrl.noEnq();

            end
        
        end
        else
        begin
            
            debugLog.record_next_cycle($format("1: MEM CTRL NO INC"));
            reqToMemCtrl.noEnq();
            
        end
        
        // Check for messages outbound from the memory controller.
        
        let m_rsp <- rspFromMemCtrl.receive();
        
        if (m_rsp matches tagged Valid .rsp &&& !isValid(message_out))
        begin

            debugLog.record_next_cycle($format("1: MEM CTRL RSP"));
            message_out = tagged Valid rspToPacket(rsp);
            rspFromMemCtrl.doDeq();

        end
        else
        begin
        
            debugLog.record_next_cycle($format("1: MEM CTRL NO RSP"));
            // Nothing from the memory controller at the moment.
            rspFromMemCtrl.noDeq();
        
        end

        toNextRingStop.send(0, message_out);
        
        // Move on to the next stage.
        state <= CORES;
        
    endrule
    
    rule stage2_Cores (state == CORES);
    
        // Read each core in succession.
    
        match {.cpu_iid, .done} <- multiplexCtrl.nextReadyInstance();
    
        // Track which message we're sending to the next ring stop, if any.
        let message_out = tagged Invalid;
    
        // See if the previous ring stop has something for us.
        let core_can_enq <- rspToCores.canEnq(cpu_iid);
        let m_inc <- fromPrevRingStop.receive(0);

        if (m_inc matches tagged Valid .packet)
        begin

            if (packet.coreID == cpu_iid &&& packet.payload matches tagged PKT_rsp .rsp)
            begin

                // It's going to this local processor.
                // assert core_can_enq
                debugLog.record($format("2: CORE %0d INC RSP", cpu_iid));
                rspToCores.doEnq(cpu_iid, toCoreMemRsp(rsp));

            end
            else
            begin

                // It's bound elsewhere.
                debugLog.record($format("2: CORE %0d PASS THROUGH", cpu_iid));
                rspToCores.noEnq(cpu_iid);
                message_out = m_inc;

            end

        end
        else
        begin
        
            // No incoming requests.
            debugLog.record($format("2: CORE %0d BUBBLE", cpu_iid));
            rspToCores.noEnq(cpu_iid);
            
        end
        
        // Now check for incoming requests from the core.
        let m_req <- reqFromCores.receive(cpu_iid);

        if (m_req matches tagged Valid .req &&& !isValid(message_out))
        begin
        
            // Send it around the ring.
            debugLog.record($format("2: CORE %0d REQ", cpu_iid));
            message_out = tagged Valid reqToPacket(req, cpu_iid);
            reqFromCores.doDeq(cpu_iid);

        end
        else
        begin
            // Don't pass it on.
            debugLog.record($format("2: CORE %0d REQ BUBBLE", cpu_iid));
            reqFromCores.noDeq(cpu_iid);

        end

        // Send whatever message we decided.
        toNextRingStop.send(0, message_out);

        // See if we're done processing all the cores.
        if (done)
        begin
        
            state <= MEM_CTRL;
        
        end
        
    endrule

endmodule

module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderFirstToLast#(String portname, Integer latency)
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));

    Connection_Receive#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))) con <- mkConnection_Receive(portname);
    
    let maxInstance = fromInteger(valueof(t_NUM_INSTANCES) - 1); // activeInstances - 1; // XXX this needs to know about dynamically active instances somehow.

    Integer rMax = (latency * valueof(t_NUM_INSTANCES)) + 1;

    if (rMax > 64)
        error("Total Port buffering cannot currently exceed 64. Port: " + portname);

    function Maybe#(t_MSG) initfunc(Bit#(6) idx);
        return tagged Invalid;
    endfunction

    LUTRAM#(Bit#(6), Maybe#(t_MSG)) rs <- mkLUTRAMWith(initfunc);
    Reg#(Maybe#(t_MSG)) sideBuffer <- mkReg(tagged Invalid);

    COUNTER#(6) head <- mkLCounter(0);
    COUNTER#(6) tail <- mkLCounter((fromInteger(latency * (valueof(t_NUM_INSTANCES) - 1))));
    Reg#(INSTANCE_ID#(t_NUM_INSTANCES)) curEnq <- mkReg(0);
    Reg#(INSTANCE_ID#(t_NUM_INSTANCES)) curDeq <- mkReg(0);

    Bool fullQ  = tail.value() + 1 == head.value();
    Bool emptyQ = head.value() == tail.value();


    rule shift (!fullQ && con.notEmpty());

        match {.iid, .msg} = con.receive();
        con.deq();

        if (curEnq == 0)
        begin
            
            sideBuffer <= msg;
        
        end
        else
        begin
        
            rs.upd(tail.value(), msg);
            tail.up();
        
        end
        
        if (curEnq == maxInstance)
        begin
        
            curEnq <= 0;
        
        end
        else
        begin

            curEnq <= curEnq + 1;
        
        end

    endrule
    
    interface INSTANCE_CONTROL_IN ctrl;


        method Bool empty() = emptyQ && curDeq != maxInstance;
        method Bool balanced() = True;
        method Bool light() = False;
        
        method Maybe#(INSTANCE_ID#(t_NUM_INSTANCES)) nextReadyInstance();
        
            return (emptyQ && curDeq != maxInstance) ? tagged Invalid : tagged Valid curDeq;
        
        endmethod
        
        method Action drop();
        
            // Take the side buffer into account.
            if (head.value() != fromInteger(valueOf(t_NUM_INSTANCES) - 2))
            begin
            
                head.up();

            end

        endmethod

    endinterface

    method ActionValue#(Maybe#(t_MSG)) receive(INSTANCE_ID#(t_NUM_INSTANCES) dummy) if (!emptyQ || curDeq == maxInstance);

        if (curDeq == maxInstance)
        begin
        
            // Return the side buffer.
            curDeq <= 0;
            return sideBuffer;
        
        end
        else
        begin
        
            // Return the main buffer.
            let res = rs.sub(head.value());
            head.up();
            curDeq <= curDeq + 1;
            return res;
        
        end

    endmethod

endmodule
