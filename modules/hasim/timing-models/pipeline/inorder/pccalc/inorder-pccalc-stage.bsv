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
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model imports *****

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/memory_base_types.bsh"


// ****** Generated files ******

`include "asim/dict/STATS_PCCALC.bsh"


// ****** Local Datatypes ******


// PCC1_STATE

// Record if the PCCAlc stage stalls for a rewind or fault.

typedef union tagged
{
    void        STAGE2_noredirect;
    ISA_ADDRESS STAGE2_redirect;
    void        STAGE2_faultRsp;
    ISA_ADDRESS STAGE2_rewindRsp;
}
PCC_STAGE2_STATE deriving (Bits, Eq);


// ASSUMPTIONS made by pccalc:
//   The input from BranchPrediction refers to the same instruction bundle as
//   the input from IMem.
// and probably more I don't realize yet.

// PATHS:
// 1. No fault from back end.
// 2. Fault from back end.
// 3. Rewind from back end.

// ****** Modules ******


module [HASIM_MODULE] mkPCCalc
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_pccalc.out");

    // ****** Model State (per instance) ******

    MULTIPLEXED#(NUM_CPUS, Reg#(IMEM_EPOCH))  epochPool <- mkMultiplexed(mkReg(initIMemEpoch(0, 0, 0, 0)));

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");
 
    Connection_Client#(FUNCP_REQ_HANDLE_FAULT,
                       FUNCP_RSP_HANDLE_FAULT)       handleFault <- mkConnection_Client("funcp_handleFault");
 

    // ****** Ports ******


    PORT_SEND_MULTIPLEXED#(NUM_CPUS, INSTQ_ALLOCATION)                  bundleToInstQ <- mkPortSend_Multiplexed("Fet_to_InstQ_allocate");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VOID)                              clearToInstQ  <- mkPortSend_Multiplexed("Fet_to_InstQ_clear");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Bool)                              creditResetToInstQ  <- mkPortSend_Multiplexed("Fet_to_InstQ_creditReset");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, Tuple2#(ISA_ADDRESS, IMEM_EPOCH))  nextPCToFetch <- mkPortSend_Multiplexed("PCCalc_to_Fet_newpc");

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, TOKEN)                                    faultFromCom  <- mkPortRecv_Multiplexed("Com_to_Fet_fault", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, Tuple3#(TOKEN, TOKEN_FAULT_EPOCH, ISA_ADDRESS)) rewindFromExe <- mkPortRecv_Multiplexed("Exe_to_Fet_rewind", 1);

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, IMEM_OUTPUT) rspFromIMem <- mkPortRecv_Multiplexed("IMem_to_Fet_response", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, ISA_ADDRESS) predFromBP  <- mkPortRecv_Multiplexed("BP_to_Fet_pred", 2);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, BRANCH_ATTR) attrFromBP  <- mkPortRecv_Multiplexed("BP_to_Fet_attr", 2);


    // ****** Local Controller ******

    Vector#(5, INSTANCE_CONTROL_IN#(NUM_CPUS)) inctrls  = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();

    inctrls[0] = faultFromCom.ctrl;
    inctrls[1] = rewindFromExe.ctrl;
    inctrls[2] = rspFromIMem.ctrl;
    inctrls[3] = predFromBP.ctrl;
    inctrls[4] = attrFromBP.ctrl;
    outctrls[0]  = bundleToInstQ.ctrl;
    outctrls[1]  = nextPCToFetch.ctrl;
    outctrls[2]  = clearToInstQ.ctrl;
    outctrls[3]  = creditResetToInstQ.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    STAGE_CONTROLLER#(NUM_CPUS, PCC_STAGE2_STATE) stage2Ctrl <- mkStageController();


    // ****** Stats ******
    STAT_RECORDER_MULTIPLEXED#(NUM_CPUS) statLpBpMismatches <- mkStatCounter_Multiplexed(`STATS_PCCALC_LP_BP_MISMATCHES);

    // ****** Rules ******
    
    // Ports read:
    // * rewindFromExe
    // * faultFromCom
    // * rspFromIMem
    // * predFromBP
    // * attrFromBP
    //
    // Ports written:
    // * bundleToInstQ
    // * clearToInstQ
    // * creditResetToInstQ

    rule stage1_nextPC (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our state from the instance.
        Reg#(IMEM_EPOCH) epochReg = epochPool[cpu_iid];
        IMEM_EPOCH epoch = epochReg;

        // Receive potential new PCs from our incoming ports.
        let m_rewind     <- rewindFromExe.receive(cpu_iid);
        let m_fault      <- faultFromCom.receive(cpu_iid);
        let m_imem_rsp  <- rspFromIMem.receive(cpu_iid);
        let m_pred_pc    <- predFromBP.receive(cpu_iid);
        let m_attr <- attrFromBP.receive(cpu_iid);

        PCC_STAGE2_STATE stage2_redirect_state = tagged STAGE2_noredirect;
        Bool credit_reset = False;
        Bool keep_reset_allocation = False;

        Bool back_end_fault = False;

        // Check for faults and rewinds from the back end.
        // This is only to see if we need to redirect the pc. It has nothing
        // to do with the instruction queue allocation or poisoning.
        //
        // If there is a fault or a rewind we increment the epoch, so that the
        // next stage will never try to overide redirect, because the epoch is
        // gaurenteed not to match.
        if (m_fault matches tagged Valid { .tok })
        begin
            // A fault occurred. Get the handler PC from the functional
            // partition.
            debugLog.record_next_cycle(cpu_iid, fshow("FAULT: ") + fshow(tok));
            handleFault.makeReq(initFuncpReqHandleFault(tok));
            epoch.fault = epoch.fault + 1;

            back_end_fault = True;
            stage2_redirect_state = tagged STAGE2_faultRsp;

        end
        else if (m_rewind matches tagged Valid { .tok, .fault_epoch, .addr} &&&
                 fault_epoch == epoch.fault)
        begin

            // A branch misprediction occured.
            // Epoch check ensures we haven't already redirected from a fault.
            debugLog.record_next_cycle(cpu_iid, fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            epoch.branch = epoch.branch + 1;

            back_end_fault = True;
            stage2_redirect_state = tagged STAGE2_rewindRsp addr;
        end

        // If we need to redirect, we put the pc to redirect to in here.
        Maybe#(ISA_ADDRESS) redirect = Invalid;

        // Figure out what to do with the INSTQ allocation.
        if (m_imem_rsp matches tagged Valid .imem_rsp)
        begin

            // If m_imem_rsp is valid, it means m_pred_pc and m_attr must be
            // valid.
            // assert isValid(m_pred_pc);
            let pred_pc = validValue(m_pred_pc);
            let attr = validValue(m_attr);

            // Check for problems with this bundle.
            if (imem_rsp.bundle.epoch != epoch)
            begin
                // Epoch is wrong. No need to redirect.
                debugLog.record_next_cycle(cpu_iid, $format("WRONG EPOCH"));
            end
            else if (imem_rsp.response matches tagged IMEM_itlb_fault)
            begin
                // An ITLB fault occured. Increment the itlb epoch, redirect
                // pc to the handler address, and credit_reset the INSTQ.
                debugLog.record_next_cycle(cpu_iid, $format("ITLB FAULT"));
                credit_reset = True;                
        
                // For now we just go to whatever address we were trying to
                // execute. This will likely have to change in the future.
                redirect = tagged Valid imem_rsp.bundle.virtualAddress;
                epoch.iTLB = epoch.iTLB + 1;
            
            end
            else if (imem_rsp.response matches tagged IMEM_icache_retry)
            begin
                // ICACHE Retry. We need to increment the iCache epoch,
                // redirect the PC and credit_reset the INSTQ.
                debugLog.record_next_cycle(cpu_iid, $format("ICACHE RETRY"));
                credit_reset = True;

                redirect = tagged Valid imem_rsp.bundle.virtualAddress;
                epoch.iCache = epoch.iCache + 1;
            end
            else if (imem_rsp.bundle.linePrediction != pred_pc)
            begin
                // Line prediction doesn't match branch prediction.
                // Increment the epoch (we use iCache epoch for this).
                // Redirect to the branch prediction.
                // Even though the line prediction was wrong, this instruction
                // is still correct, so don't credit_reset the INSTQ now, but
                // remeber that we need to do it next model cycle.
                statLpBpMismatches.incr(cpu_iid);
                debugLog.record_next_cycle(cpu_iid, $format("LP BP Mismatch.  LP: 0x%0h, BP: 0x%0h", imem_rsp.bundle.linePrediction, pred_pc));
                credit_reset = True;
                keep_reset_allocation = True;

                redirect = tagged Valid pred_pc;
                epoch.iCache = epoch.iCache+1;
            end

            Maybe#(ICACHE_MISS_ID) miss_id = tagged Invalid;
            if (imem_rsp.response matches tagged IMEM_icache_miss .id)
            begin 
                miss_id = tagged Valid id;
            end

            // Make appropriate INSTQ_ALLOCATION
            let bundle = FETCH_BUNDLE {
                branchEpoch: imem_rsp.bundle.epoch.branch,
                faultEpoch: imem_rsp.bundle.epoch.fault,
                pc: imem_rsp.bundle.virtualAddress,
                inst: imem_rsp.bundle.instruction,
                branchAttr: attr
            };

            let instq_alloc = INSTQ_ALLOCATION {
                credit: imem_rsp.bundle.instQCredit,
                bundle: bundle,
                missID: miss_id
            };

            bundleToInstQ.send(cpu_iid, tagged Valid instq_alloc);

        end
        else
        begin
            // There's a bubble from the front end.
            bundleToInstQ.send(cpu_iid, Invalid);
        end

        // Now we deal with pc redirection.
        if (!back_end_fault &&& redirect matches tagged Valid .new_pc)
        begin
            stage2_redirect_state = tagged STAGE2_redirect new_pc;
        end

        clearToInstQ.send(cpu_iid, back_end_fault ? tagged Valid (?) : Invalid);

        // Do a credit reset if needed.
        if (credit_reset)
        begin
            creditResetToInstQ.send(cpu_iid, tagged Valid keep_reset_allocation);
        end
        else
        begin
            creditResetToInstQ.send(cpu_iid, Invalid);
        end

        stage2Ctrl.ready(cpu_iid, stage2_redirect_state);

        epochReg <= epoch;
    endrule
    

    // stage2_redirect
    
    // Get fault response, rewind response, or do pc redirection.
    // Ports read:
    //  (none)
    // Ports written:
    // * nextPCToFetch

    rule stage2_redirect (True);

        match {.cpu_iid, .state} <- stage2Ctrl.nextReadyInstance();

        // Get our state based on the current cpu instance.
        let epoch = epochPool[cpu_iid];

            
        // Redirect as appropriate.
        if (state matches tagged STAGE2_noredirect) 
        begin
            nextPCToFetch.send(cpu_iid, tagged Invalid);
        end
        else if (state matches tagged STAGE2_redirect .new_pc)
        begin
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, epoch));
        end
        else if (state matches tagged STAGE2_faultRsp)
        begin
        
            // Get the fault response.
            let rsp = handleFault.getResp();
            handleFault.deq();

            // Resteer to the handler.
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(rsp.nextInstructionAddress, epoch));
        end
        else if (state matches tagged STAGE2_rewindRsp .new_pc)
        begin
            let rsp = rewindToToken.getResp();
            rewindToToken.deq();
            nextPCToFetch.send(cpu_iid, tagged Valid tuple2(new_pc, epoch));

        end

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule

endmodule

