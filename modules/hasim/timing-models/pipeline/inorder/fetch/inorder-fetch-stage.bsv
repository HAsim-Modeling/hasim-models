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

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_fetch.out");


    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(ISA_ADDRESS))        ctx_pc          <- mkMultiCtx(mkReg(`PROGRAM_START_ADDR));
    MULTICTX#(Reg#(IMEM_ITLB_EPOCH))   ctx_iTLBEpoch   <- mkMultiCtx(mkReg(0));
    MULTICTX#(Reg#(IMEM_ICACHE_EPOCH)) ctx_iCacheEpoch <- mkMultiCtx(mkReg(0));


    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT,
                       FUNCP_RSP_NEW_IN_FLIGHT)      newInFlight    <- mkConnection_Client("funcp_newInFlight");


    // ****** Ports ******

    PORT_RECV_MULTICTX#(VOID)                              creditFromInstQ <- mkPortRecvInitial_MultiCtx("InstQ_to_Fet_credit", 1, (?));
    PORT_RECV_MULTICTX#(Tuple3#(ISA_ADDRESS, IMEM_ITLB_EPOCH, IMEM_ICACHE_EPOCH)) newPCFromPCCalc <- mkPortRecv_MultiCtx("PCCalc_to_Fet_newpc", 1);

    PORT_SEND_MULTICTX#(ITLB_INPUT) pcToITLB <- mkPortSend_MultiCtx("CPU_to_ITLB_req");
    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, ISA_ADDRESS)) pcToBP <- mkPortSend_MultiCtx("Fet_to_BP_pc");


    // ****** Local Controller ******
        
    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(2, PORT_CONTROLS) outports = newVector();
    inports[0]  = creditFromInstQ.ctrl;
    inports[1]  = newPCFromPCCalc.ctrl;
    outports[0] = pcToITLB.ctrl;
    outports[1] = pcToBP.ctrl;
    
    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTICTX eventFet <- mkEventRecorder_MultiCtx(`EVENTS_FETCH_INSTRUCTION_FET);

    STAT_RECORDER_MULTICTX statCycles  <- mkStatCounter_MultiCtx(`STATS_FETCH_TOTAL_CYCLES);
    STAT_RECORDER_MULTICTX statFet     <- mkStatCounter_MultiCtx(`STATS_FETCH_INSTS_FETCHED);


    // ****** Rules ******
    
    // stage1_tokenReq
    
    // Update the PC with the new calculation.
    // If the InstQ has room then request a new token from the functional partition.

    rule stage1_tokenReq (True);

        // Start a new model cycle
        let ctx <- localCtrl.startModelCycle();
        statCycles.incr(ctx);
        debugLog.nextModelCycle(ctx);
        modelCycle.send(ctx);
        
        // Get our local state using the context.
        Reg#(ISA_ADDRESS)                pc = ctx_pc[ctx];
        Reg#(IMEM_ITLB_EPOCH)     iTLBEpoch = ctx_iTLBEpoch[ctx];
        Reg#(IMEM_ICACHE_EPOCH) iCacheEpoch = ctx_iCacheEpoch[ctx];
        
        // Get the next PC
        let m_newPC <- newPCFromPCCalc.receive(ctx);
        
        // Update the PC and front end epochs.
        if (m_newPC matches tagged Valid {.new_pc, .itlb_epoch, .icache_epoch})
        begin

            pc <= new_pc;
            iTLBEpoch   <= itlb_epoch;
            iCacheEpoch <= icache_epoch;

        end

        // See if we have room in the instructionQ.
        let m_credit <- creditFromInstQ.receive(ctx);
        
        if (isValid(m_credit))
        begin
        
            // The instructionQ still has room...
            // Request a new token which we can send to the ICache.
            newInFlight.makeReq(initFuncpReqNewInFlight(ctx));

        end
        else
        begin

            // The instructionQ is full... Nothing we can do.
            debugLog.record_next_cycle(ctx, $format("BUBBLE"));
            eventFet.recordEvent(ctx, tagged Invalid);
            
            // Don't send anything to the ITLB.
            
            // Don't request a new address translation or branch prediction.
            pcToITLB.send(ctx, tagged Invalid);
            pcToBP.send(ctx, tagged Invalid);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(ctx, 1);

        end

    endrule


    // stage2_iTLBReq
    
    // Get a new token from the functional partition and
    // send the it and the current PC to the ITLB and Branch Predictor.
    
    rule stage2_iTLBReq (True);

        // Get the response.
        let rsp = newInFlight.getResp();
        newInFlight.deq();

        // Get our context from the token.
        let tok = rsp.newToken;
        let ctx = tokContextId(tok);

        // Get our local state using the current context id.
        Reg#(ISA_ADDRESS)                pc = ctx_pc[ctx];
        Reg#(IMEM_ITLB_EPOCH)     iTLBEpoch = ctx_iTLBEpoch[ctx];
        Reg#(IMEM_ICACHE_EPOCH) iCacheEpoch = ctx_iCacheEpoch[ctx];

        // Send the current PC to the ITLB and Branch predictor.
        pcToITLB.send(ctx, tagged Valid initIMemBundle(tok, iTLBEpoch, iCacheEpoch, pc));
        pcToBP.send(ctx, tagged Valid tuple2(tok, pc));
        
        // End of model cycle. (Path 2)
        eventFet.recordEvent(ctx, tagged Valid truncate(pc));
        statFet.incr(ctx);
        debugLog.record(ctx, fshow("FETCH: ") + fshow(tok) + $format(" ADDR:0x%h", pc));
        localCtrl.endModelCycle(ctx, 2);

    endrule

endmodule
