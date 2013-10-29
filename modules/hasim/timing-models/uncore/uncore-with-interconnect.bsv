import Vector::*;

// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/funcp_memstate_base_types.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_interconnect.bsh"
`include "awb/provides/hasim_last_level_cache.bsh"
`include "awb/provides/hasim_memory_controller.bsh"
`include "awb/provides/hasim_interconnect_common.bsh"

module [HASIM_MODULE] mkUncore 
    // interface:
        ();

    // Instantiate submodules
    let ic <- mkInterconnect();
    let memCtrl <- mkMemoryController();
    let lastLevelCache <- mkLastLevelCache();

    //
    // Storage (side buffers) in which the actual network messages are stored
    // instead of building the interconnect's data paths wide enough to carry
    // them.  The interconnect still simulates time, but we use a simple
    // global structure for transferring the data instead of simulated
    // network FIFOs.
    //
    // Two ports are needed.  One for memory controllers and one for the LLC.
    //
    NumTypeParam#(2) numClients = ?;
    let packetStorage <- mkNetworkPacketPayloadStorage(numClients);
endmodule
