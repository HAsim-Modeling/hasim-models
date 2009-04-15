//
// Copyright (C) 2008 Intel Corporation
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
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/memory_base_types.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"

// ****** Modules ******

// mkFetch

// Inorder fetch module. If we have room in the instructionQ, get a token from
// the functional partition and send it to the ITLB for translation and the
// branch predictor for calculation.

// Normal flow: 
// stage1: Have credit, token req -> stage2: token response.

// Possible ways to end a model cycle:
// Path 1: We have no credit, so emit a bubble.
// Path 2: We have a credit, so we got a token and sent it to the TLB/BP.


module [HASIM_MODULE] mkFetch ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_fetch.out");


    // ****** Model State (per Context) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(ISA_ADDRESS))    pcPool <- mkMultiplexed(mkReg(`PROGRAM_START_ADDR));
    MULTIPLEXED#(NUM_CPUS, Reg#(IMEM_EPOCH))  epochPool <- mkMultiplexed(mkReg(initIMemEpoch(0, 0, 0, 0)));


    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, INSTQ_SLOT_ID)                     creditFromInstQ <- mkPortRecv_Multiplexed("InstQ_to_Fet_credit", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH))  newPCFromPCCalc <- mkPortRecv_Multiplexed("PCCalc_to_Fet_newpc", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ITLB_INPUT) pcToITLB <- mkPortSend_Multiplexed("CPU_to_ITLB_req");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) pcToBP <- mkPortSend_Multiplexed("Fet_to_BP_pc");


    // ****** Local Controller ******
        
    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) inports  = newVector();
    Vector#(2, PORT_CONTROLS#(NUM_CPUS)) outports = newVector();
    inports[0]  = creditFromInstQ.ctrl;
    inports[1]  = newPCFromPCCalc.ctrl;
    outports[0] = pcToITLB.ctrl;
    outports[1] = pcToBP.ctrl;
    
    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTIPLEXED#(NUM_CPUS) eventFet <- mkEventRecorder_Multiplexed(`EVENTS_FETCH_INSTRUCTION_FET);

    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statCycles  <- mkStatCounter_Multiplexed(`STATS_FETCH_TOTAL_CYCLES);
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statFet     <- mkStatCounter_Multiplexed(`STATS_FETCH_INSTS_FETCHED);


    // ****** Rules ******
    
    // stage1_fetchReq
    
    // Update the PC with the new calculation.
    // If the InstQ has room then request a instruction from the IMemory.

    rule stage1_fetchReq (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        statCycles.incr(cpu_iid);
        debugLog.nextModelCycle(cpu_iid);
        modelCycle.send(cpu_iid);
        
        // Get our local state using the context.
        Reg#(ISA_ADDRESS)         pc = pcPool[cpu_iid];
        Reg#(IMEM_EPOCH)       epoch = epochPool[cpu_iid];
        
        // Get the next PC
        let m_newPC <- newPCFromPCCalc.receive(cpu_iid);
        let pc_to_fetch = pc;
        let epoch_to_fetch = epoch;
        
        // Update the PC and front end epochs.
        if (m_newPC matches tagged Valid {.new_pc, .new_epoch})
        begin

            pc_to_fetch = new_pc;
            epoch_to_fetch = new_epoch;

        end

        // See if we have room in the instructionQ.
        let m_credit <- creditFromInstQ.receive(cpu_iid);
        
        if (m_credit matches tagged Valid .slot)
        begin
        
            // The instructionQ still has room...
            // Send the current PC to the ITLB and Branch predictor.
            pcToITLB.send(cpu_iid, tagged Valid initIMemBundle(epoch_to_fetch, slot, pc_to_fetch));
            pcToBP.send(cpu_iid, tagged Valid pc_to_fetch);
        
            // End of model cycle. (Path 1)
            eventFet.recordEvent(cpu_iid, tagged Valid truncate(pc_to_fetch));
            statFet.incr(cpu_iid);
            debugLog.record_next_cycle(cpu_iid, $format("FETCH ADDR:0x%h", pc_to_fetch) + $format(" SLOT:%0d", slot));
            localCtrl.endModelCycle(cpu_iid, 1);

        end
        else
        begin

            // The instructionQ is full... Nothing we can do.
            debugLog.record_next_cycle(cpu_iid, $format("BUBBLE"));
            eventFet.recordEvent(cpu_iid, tagged Invalid);
            
            // Don't send anything to the ITLB.
            
            // Don't request a new address translation or branch prediction.
            pcToITLB.send(cpu_iid, tagged Invalid);
            pcToBP.send(cpu_iid, tagged Invalid);

            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

        end
        
        pc <= pc_to_fetch;
        epoch <= epoch_to_fetch;

    endrule

endmodule
