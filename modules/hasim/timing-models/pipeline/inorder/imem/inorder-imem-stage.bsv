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
`include "asim/provides/fpga_components.bsh"


// ****** Timing Model imports *****

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_model_services.bsh"


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

    MULTIPLEXED_STATE_POOL#(NUM_CPUS, IMEM_EPOCH) statePool <- mkMultiplexedStatePool(initialIMemEpoch);

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ITLB_OUTPUT)                           rspFromITLB <- mkPortRecv_Multiplexed("ITLB_to_CPU_rsp", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ICACHE_INPUT)                     physAddrToICache <- mkPortSend_Multiplexed("CPU_to_ICache_load");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, IMEM_OUTPUT)                       iMemToPCCalc    <- mkPortSend_Multiplexed("IMem_to_Fet_response");

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) immRspFromICache <- mkPortRecvDependent_Multiplexed("ICache_to_CPU_load_immediate");

    // ****** Soft Connections *******

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)    getInstruction <- mkConnection_Client("funcp_getInstruction");


    // ****** Local Controller ******
        
    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls  = newVector();
    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    inctrls[0]  = rspFromITLB.ctrl;
    inctrls[1]  = statePool.ctrl;
    outctrls[0] = physAddrToICache.ctrl;
    outctrls[1] = iMemToPCCalc.ctrl;
    
    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    // Stage 2 data.
    // If Invalid, means there was a bubble input to this stage.
    // Otherwise contains output that should be sent to pccalc if icache
    // responds with a bubble.
    STAGE_CONTROLLER#(NUM_CPUS, Tuple2#(Maybe#(IMEM_OUTPUT), IMEM_EPOCH)) stage2Ctrl <- mkBufferedStageController();

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
        IMEM_EPOCH epoch <- statePool.extractState(cpu_iid);
        Maybe#(IMEM_OUTPUT) stage_data = Invalid;
        
        // See if there's a response from the ITLB.
        let m_rsp <- rspFromITLB.receive(cpu_iid);

        if (m_rsp matches tagged Valid .rsp)
        begin

            Bool good_epoch;

            if (rsp.bundle.epoch.prediction != epoch.prediction ||
                rsp.bundle.epoch.branch != epoch.branch ||
                rsp.bundle.epoch.fault != epoch.fault)
            begin
                good_epoch = True;
                epoch = rsp.bundle.epoch;
            end
            else
            begin
                good_epoch = rsp.bundle.epoch.iTLB == epoch.iTLB && 
                             rsp.bundle.epoch.iCache == epoch.iCache;
            end

            if (good_epoch)
            begin
                // Check if we received a valid translation.
                if (rsp.rspType == ITLB_pageFault)
                begin

                    // There was a page fault. :(

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

                    // Simultaneously get the actual instruction from the 
                    // functional partition.
                    let req = initFuncpReqGetInstruction(rsp.bundle.ctx_id, rsp.bundle.physicalAddress, rsp.bundle.offset);
                    getInstruction.makeReq(req);

                    stage_data = tagged Valid IMEM_OUTPUT {
                        bundle: rsp.bundle,
                        response: IMEM_icache_req
                    };
                end
            end
            else
            begin
                // Epoch check failed. Don't make an icache request.
                physAddrToICache.send(cpu_iid, tagged Invalid);
                
                // Send the request on so PCCalc can reclaim the instQ slot.
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

        stage2Ctrl.ready(cpu_iid, tuple2(stage_data, epoch));
    endrule

    // Ports read:
    // * immRspFromICache
    //
    // Ports written:
    // * iMemToPCCalc
    //

    rule stage2_iCacheRsp;
        match {.cpu_iid, {.m_stage_data, .epoch}} <- stage2Ctrl.nextReadyInstance();

        let m_icache_rsp <- immRspFromICache.receive(cpu_iid); 

        if (m_stage_data matches tagged Valid .stage_data)
        begin
            case (stage_data.response) matches
                tagged IMEM_icache_req:
                begin

                    // assert isValid icache_rsp
                    let icache_rsp = validValue(m_icache_rsp);

                    // Response from icache. Use the updated bundle, and
                    // corresponding imem response.
                    IMEM_RESPONSE response = case (icache_rsp.rspType) matches
                        tagged ICACHE_hit: tagged IMEM_icache_hit;
                        tagged ICACHE_miss .miss_id: tagged IMEM_icache_miss miss_id;
                        tagged ICACHE_retry: tagged IMEM_icache_retry;
                    endcase;

                    // Fill in the instruction from the functional partition.
                    let rsp = getInstruction.getResp();
                    let new_bundle = icache_rsp.bundle;
                    new_bundle.instruction = rsp.instruction;
                    getInstruction.deq();

                    if (icache_rsp.rspType matches tagged ICACHE_retry)
                    begin
                        epoch.iCache = epoch.iCache + 1;
                    end

                    iMemToPCCalc.send(cpu_iid, tagged Valid IMEM_OUTPUT {
                        bundle: new_bundle,
                        response: response
                    });
                end
                tagged IMEM_itlb_fault:
                begin
                    epoch.iTLB = epoch.iTLB + 1;
                end
                default:
                begin
                    // Propogate whatever happened in the first stage.
                    iMemToPCCalc.send(cpu_iid, tagged Valid stage_data);
                end
            endcase
        end
        else
        begin
            // Bubble from ITLB. Send bubble onto pccalc.
            iMemToPCCalc.send(cpu_iid, Invalid);
        end
        

        // End the model cycle.
        statePool.insertState(cpu_iid, epoch);
        localCtrl.endModelCycle(cpu_iid, 1);
    endrule

endmodule

