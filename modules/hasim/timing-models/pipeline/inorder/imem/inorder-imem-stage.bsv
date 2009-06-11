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

import FIFO::*;
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
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"


// ****** Modules ******

// mkIMem

// Inorder IMem stage.
// Gets the response from the ITLB.
// If there was no fault and epoch is right, make an ICACHE request.
// Get the ICACHE response, forward results to pccalc stage.

// Expected Normal Flow
// stage1: Check for page fault, make ICache request.
// stage2: Get ICache response.

module [HASIM_MODULE] mkIMem
    // interface:
        ();


    // ****** Model State (per instance) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(IMEM_ITLB_EPOCH)) iTLBEpochPool <- mkMultiplexed(mkReg(0));

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ITLB_OUTPUT)                           rspFromITLB <- mkPortRecv_Multiplexed("ITLB_to_CPU_rsp", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_INPUT)                     physAddrToICache <- mkPortSend_Multiplexed("CPU_to_ICache_req");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, IMEM_OUTPUT)                       iMemToPCCalc    <- mkPortSend_Multiplexed("IMem_to_Fet_response");

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immRspFromICache <- mkPortRecvDependent_Multiplexed("ICache_to_CPU_immediate");

    // ****** Local Controller ******
        
    Vector#(1, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    inctrls[0]  = rspFromITLB.ctrl;
    outctrls[0] = physAddrToICache.ctrl;
    outctrls[1] = iMemToPCCalc.ctrl;
    
    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    // Stage 2 data.
    // If Invalid, means there was a bubble input to this stage.
    // Otherwise contains output that should be sent to pccalc if icache
    // responds with a bubble.
    STAGE_CONTROLLER#(NUM_CPUS, Maybe#(IMEM_OUTPUT)) stage2Ctrl <- mkStageController();

    // ****** Rules ******

    // stage1_iCacheReq
    
    // Gets the ITLB response, verifies the epoch and checks for page faults.
    // If all is well, makes a an icache request.
    // We perform the epoch check only to avoid making an uneccessary icache
    // request.
    //
    // Ports read:
    // * rspFromITLB
    //
    // Ports written:
    // * physAddrToICache

    (* conservative_implicit_conditions *)
    rule stage1_iCacheReq (True);

        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        
        // Get our local state using the current context.
        Reg#(IMEM_ITLB_EPOCH) iTLBEpoch = iTLBEpochPool[cpu_iid];

        Maybe#(IMEM_OUTPUT) stage_data = Invalid;
        
        // See if there's a response from the ITLB.
        let m_rsp <- rspFromITLB.receive(cpu_iid);

        if (m_rsp matches tagged Valid .rsp)
        begin

            if (rsp.bundle.epoch.iTLB == iTLBEpoch)
            begin
                // Check if we received a valid translation.
                if (rsp.rspType == ITLB_pageFault)
                begin

                    // There was a page fault. :(

                    // Increment the epoch so we'll drop any following faults (which occur quite commonly).
                    iTLBEpoch <= iTLBEpoch + 1;

                    // No physical address to load from icache.
                    physAddrToICache.send(cpu_iid, tagged Invalid);

                    stage_data = tagged Valid IMEM_OUTPUT {
                        bundle: rsp.bundle,
                        response: IMEM_itlb_fault
                    };
                end
                else
                begin
                    // ITLB was successful
                    // Send the physical address on to the ICache.
                    physAddrToICache.send(cpu_iid, tagged Valid initICacheLoad(rsp.bundle));

                    stage_data = tagged Valid IMEM_OUTPUT {
                        bundle: rsp.bundle,
                        response: ?
                    };
                end
            end
            else
            begin
                // Epoch check failed. Don't make an icache request
                physAddrToICache.send(cpu_iid, tagged Invalid);

                stage_data = tagged Valid IMEM_OUTPUT {
                    bundle: rsp.bundle,
                    response: IMEM_bad_epoch
                };
            end
        end
        else
        begin
            // Bubble. Don't make any icache request.
            physAddrToICache.send(cpu_iid, tagged Invalid);
            stage_data = Invalid;
        end

        stage2Ctrl.ready(cpu_iid, stage_data);
    endrule

    // Ports read:
    // * immRspFromICache
    //
    // Ports written:
    // * iMemToPCCalc
    //
    rule stage2_iCacheRsp;
        match {.cpu_iid, .m_stage_data} <- stage2Ctrl.nextReadyInstance();

        let m_icache_rsp <- immRspFromICache.receive(cpu_iid); 

        if (m_stage_data matches tagged Valid .stage_data)
        begin
            if (m_icache_rsp matches tagged Valid .icache_rsp)
            begin
                // Response from icache. Use the updated bundle, and
                // corresponding imem response.
                IMEM_RESPONSE response = case (icache_rsp.rspType) matches
                    tagged ICACHE_hit: tagged IMEM_icache_hit;
                    tagged ICACHE_miss .miss_id: tagged IMEM_icache_miss miss_id;
                    tagged ICACHE_retry: tagged IMEM_icache_retry;
                endcase;

                iMemToPCCalc.send(cpu_iid, tagged Valid IMEM_OUTPUT {
                    bundle: icache_rsp.bundle,
                    response: response
                });
            end
            else
            begin
                // Nothing from the icache, so there must have been an itlb
                // fault or bad epoch. Use the stage data as is.
                iMemToPCCalc.send(cpu_iid, tagged Valid stage_data);
            end
        end
        else
        begin
            // Bubble from ITLB. Send bubble onto pccalc.
            iMemToPCCalc.send(cpu_iid, Invalid);
        end

        // End the model cycle.
        localCtrl.endModelCycle(cpu_iid, 1);
    endrule

endmodule

