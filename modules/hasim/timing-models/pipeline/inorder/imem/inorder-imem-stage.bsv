//
// Copyright (C) 2009 Massachusetts Institute of Technology
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

// ****** Bluespec imports ******

import FShow::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/hasim_controller.bsh"
`include "asim/provides/fpga_components.bsh"


// ****** Timing Model imports *****

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"


// ****** Modules ******

// mkIMem

// Inorder IMem stage. Gets the response from the ITLB. On a page fault it redirects to the handler.
// On a Hit it sends the response to the ICache.

// Expected Normal Flow
// stage1: No page fault. Make ICache request.

// Possible Ways a Model Cycle Can End
// Path 1: Page Fault
// Path 2: ITLB Hit, ICache request
// Path 3: Bubble or request from wrong epoch.


module [HASIM_MODULE] mkIMem
    // interface:
        ();


    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(IMEM_ITLB_EPOCH)) ctx_iTLBEpoch <- mkMultiCtx(mkReg(0));


    // ****** Ports ******

    PORT_RECV_MULTICTX#(ITLB_OUTPUT) rspFromITLB <- mkPortRecv_MultiCtx("ITLB_to_CPU_rsp", 0);

    PORT_SEND_MULTICTX#(ICACHE_INPUT)              physAddrToICache <- mkPortSend_MultiCtx("CPU_to_ICache_req");
    PORT_SEND_MULTICTX#(Tuple3#(TOKEN, IMEM_ITLB_EPOCH, ISA_ADDRESS)) faultToPCCalc    <- mkPortSend_MultiCtx("IMem_to_Fet_fault");


    // ****** Local Controller ******
        
    Vector#(1, PORT_CONTROLS) inports  = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0]  = rspFromITLB.ctrl;
    outports[0] = physAddrToICache.ctrl;
    outports[1] = faultToPCCalc.ctrl;
    
    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    // stage1_iTLBRsp
    
    // Gets the ITLB response and checks for any page faults.
    // An epoch check is important here since it is likely that there would
    // be page faults back to back and we only want to redirect on the
    // earliest fault.

    rule stage1_iTLBRsp (True);

        // Start a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        
        // Get our local state using the current context.
        Reg#(IMEM_ITLB_EPOCH) iTLBEpoch = ctx_iTLBEpoch[ctx];
        
        // See if there's a response from the ITLB.
        let m_rsp <- rspFromITLB.receive(ctx);
        
        if (m_rsp matches tagged Valid .rsp &&& rsp.bundle.iTLBEpoch == iTLBEpoch)
        begin

            // Epoch check ensures that we haven't already redirected from a previous page fault.

            // Check if we received a valid translation.
            if (rsp.rspType == ITLB_pageFault)
            begin

                // There was a page fault. :(

                // Increment the epoch so we'll drop any following faults (which occur quite commonly).
                iTLBEpoch <= iTLBEpoch + 1;

                // No physical address to load.
                physAddrToICache.send(ctx, tagged Invalid);

                // Redirect the PC to the handler.
                // TODO: We have no way of getting a handler address currently. 
                //       Instead just redirect back to the faulting PC.
                //       This at least works for pseudo-random TLB models.
                faultToPCCalc.send(ctx, tagged Valid tuple3(rsp.bundle.token, rsp.bundle.iCacheEpoch, rsp.bundle.virtualAddress));

                // End of model cycle. (Path 1)
                localCtrl.endModelCycle(ctx, 1);

            end
            else
            begin

                // It was a successful translation. No need to resteer.
                faultToPCCalc.send(ctx, tagged Invalid);

                // Send the physical address on to the ICache.
                physAddrToICache.send(ctx, tagged Valid initICacheLoad(rsp.bundle));

                // End of model cycle. (Path 2)
                localCtrl.endModelCycle(ctx, 2);

            end

        end
        else
        begin
        
            // It's a bubble or a request from the wrong epoch.
            faultToPCCalc.send(ctx, tagged Invalid);

            // Send the physical address on to the ICache.
            physAddrToICache.send(ctx, tagged Invalid);

            // End of model cycle. (Path 3)
            localCtrl.endModelCycle(ctx, 3);

        end
    
    endrule

endmodule
