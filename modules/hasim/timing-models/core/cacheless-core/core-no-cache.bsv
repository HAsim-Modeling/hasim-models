`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_pipeline.bsh"

module [HASIM_MODULE] mkCore();
    let pipeline <- mkPipeline;
    // Queues to/from the the memory system, which are never used.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqToMem <- mkPortStallSend_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspFromMem <- mkPortStallRecv_Multiplexed("L1_Cache_InQ");
    
endmodule
