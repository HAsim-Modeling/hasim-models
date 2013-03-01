import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/chip_base_types.bsh"

module [HASIM_MODULE] mkMemory();

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqFromCore <- mkPortStallRecv_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspToCore   <- mkPortStallSend_Multiplexed("L1_Cache_InQ");
    
    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    outctrls[0]  = reqFromCore.ctrl.out;
    outctrls[1]  = rspToCore.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

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

typedef 5 MEM_WORD_OFFSET_SIZE;
typedef Bit#(MEM_WORD_OFFSET_SIZE) MEM_WORD_OFFSET;
typedef TSub#(MEM_ADDRESS_SIZE, MEM_WORD_OFFSET_SIZE) LINE_ADDRESS_SIZE;
typedef Bit#(LINE_ADDRESS_SIZE) LINE_ADDRESS;

function LINE_ADDRESS toLineAddress(MEM_ADDRESS mem_addr);
    return truncateLSB(mem_addr);
endfunction

function MEM_ADDRESS fromLineAddress(LINE_ADDRESS line_addr);
    return {line_addr, 0};
endfunction

typedef 5 MEM_OPAQUE_SIZE;
typedef Bit#(MEM_OPAQUE_SIZE) MEM_OPAQUE;

typedef struct
{
    LINE_ADDRESS physicalAddress;
    Bool isStore;
    MEM_OPAQUE opaque;
}
MEMORY_REQ deriving (Eq, Bits);

function MEMORY_REQ initMemLoad(LINE_ADDRESS addr);

    return MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: False,
        opaque: 0
    };

endfunction

function MEMORY_REQ initMemStore(LINE_ADDRESS addr);

    return MEMORY_REQ
    {
        physicalAddress: addr,
        isStore: True,
        opaque: 0
    };

endfunction

typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
}
MEMORY_RSP deriving (Eq, Bits);

function MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);

    return MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };

endfunction

function MEM_OPAQUE toMemOpaque(t_ANY x)
    provisos
        (Bits#(t_ANY, t_SZ),
         Add#(t_SZ, t_TMP, MEM_OPAQUE_SIZE));
    
    return zeroExtend(pack(x));

endfunction

function t_ANY fromMemOpaque(MEM_OPAQUE x)
    provisos
        (Bits#(t_ANY, t_SZ),
         Add#(t_SZ, t_TMP, MEM_OPAQUE_SIZE));
        
    return unpack(truncate(x));

endfunction
