import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"

typedef struct
{
    LINE_ADDRESS physicalAddress;
    Bool isStore;
    MEM_OPAQUE opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_REQ deriving (Eq, Bits);

typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_RSP deriving (Eq, Bits);


function CORE_MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);

    return CORE_MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };

endfunction

function MEM_CTRL_RSP initMemCtrlRsp(MEM_CTRL_REQ req);

    return MEM_CTRL_RSP
    {
        physicalAddress: req.physicalAddress,
        opaque: req.opaque,
        destination: req.destination
    };

endfunction

module [HASIM_MODULE] mkMemoryController();

    // Queues to/from interconnect network.
    // Note: not multiplexed, as there is only one memory controller.
    PORT_STALL_RECV#(MEM_CTRL_REQ) reqFromIC <- mkPortStallRecv("MemCtrlInQ");
    PORT_STALL_SEND#(MEM_CTRL_RSP) rspToIC   <- mkPortStallSend("MemCtrlOutQ");
    
    rule stage1_delay (True);
    
        let m_req <- reqFromIC.receive();
        let can_enq <- rspToIC.canEnq();

        if (m_req matches tagged Valid .req)
        begin

            if (req.isStore)
            begin

                reqFromIC.doDeq();
                rspToIC.noEnq();

            end
            else if (can_enq)
            begin

                reqFromIC.doDeq();
                let rsp = initMemCtrlRsp(req);
                rspToIC.doEnq(rsp);

            end
            else
            begin

                reqFromIC.noDeq();
                rspToIC.noEnq();

            end
        end
        else
        begin

            reqFromIC.noDeq();
            rspToIC.noEnq();

        end

    endrule
    
endmodule
