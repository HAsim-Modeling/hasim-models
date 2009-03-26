import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_interface.bsh"


`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

// mkICache

// An ICache module that always hits.

module [HASIM_MODULE] mkICache();


    // ***** Unmodel State ******
    
    FIFO#(Tuple2#(CPU_INSTANCE_ID, IMEM_BUNDLE)) stage2Q <- mkFIFO();


    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)    getInstruction <- mkConnection_Client("funcp_getInstruction");


    // ****** Ports ******

    // Incoming port from CPU
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_INPUT) pcFromFet <- mkPortRecv_Multiplexed("CPU_to_ICache_req", 0);

    // Outgoing ports to CPU
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immToFet <- mkPortSend_Multiplexed("ICache_to_CPU_immediate");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_DELAYED)   delToFet <- mkPortSend_Multiplexed("ICache_to_CPU_delayed");


    // ****** Local Controller ******

    Vector#(1, PORT_CONTROLS#(NUM_CPUS)) inports = newVector();
    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0] = pcFromFet.ctrl;
    outports[0] = immToFet.ctrl;
    outports[1] = delToFet.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);

    rule stage1_instReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // read input port
        let msg_from_cpu <- pcFromFet.receive(cpu_iid);

        // check request type
        case (msg_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        immToFet.send(cpu_iid, tagged Invalid);
	        delToFet.send(cpu_iid, tagged Invalid);
                localCtrl.endModelCycle(cpu_iid, 1);

	    end

	    tagged Valid .req:
	    begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We always hit.

                // Pass it to the next stage through the functional partition, 
                // which actually retrieves the instruction.
                getInstruction.makeReq(initFuncpReqGetInstruction(req.token));
                stage2Q.enq(tuple2(cpu_iid, req));
                        

	    end

        endcase
         

    endrule
     
    rule stage2_instRsp (True);
        
        // Get the response from the functional partition.
        let rsp = getInstruction.getResp();
        getInstruction.deq();

        match {.cpu_iid, .bundle} = stage2Q.first();
        stage2Q.deq();
        
        // Update the bundle with the latest token info.
        bundle.token = rsp.token;

        // Always hit.
	immToFet.send(cpu_iid, tagged Valid initICacheHit(bundle, rsp.instruction));
	delToFet.send(cpu_iid, tagged Invalid);

        // End of model cycle. (Path 2)
        localCtrl.endModelCycle(cpu_iid, 2);
     
    endrule

endmodule
