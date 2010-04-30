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

endmodule
