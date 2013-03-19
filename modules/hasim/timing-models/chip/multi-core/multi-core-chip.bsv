`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_core.bsh"
`include "asim/provides/hasim_uncore.bsh"
`include "asim/provides/hasim_chip_topology.bsh"

module [HASIM_MODULE] mkChip();
    let core <- mkCore();
    let uncore <- mkUncore();
    let topology <- mkTopology();
endmodule
