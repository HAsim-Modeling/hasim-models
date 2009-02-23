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
import FIFO::*;
import Vector::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/hasim_controller.bsh"


// ****** Timing Model imports *****
`include "asim/provides/hasim_icache.bsh"


// ****** Generated files ******

`include "asim/dict/EVENTS_FETCH.bsh"
`include "asim/dict/STATS_FETCH.bsh"


// ****** Local types ******

// FET1_STATE

// The first stage may stall for either a rewind response or fault response.

typedef enum
{
    FET1_ready,
    FET1_rewindRsp,
    FET1_faultRsp
}
    FET1_STATE
        deriving (Bits, Eq);


// FET2_STATE

// The second stage may stall if it needs a new token from the functional partition.

typedef enum
{
    FET2_ready,
    FET2_iCacheReq,
    FET2_stall
}
    FET2_STATE
        deriving (Bits, Eq);


// mkFetch

// Multi-context fetch module which interacts with an instruction cache. Note that this version
// assumes that the cache is blocking.

// Branch mispredicts may be reported by the Exe module.
// Committing a faulting instruction may be reported by the Com module.

// This module is pipelined across contexts. Stages:

// Stage 1* -> Stage 2** -> Stage 3 -> Stage 4 -> Stage 5
// * Stage 1 stalls when a rewind or fault is reported, 
//   until the functional partition reports that the fault has been handled.
// ** Stage 2 stalls when it needs to request a new token, until the response comes back.

// Possible ways the model cycle can end:
///  Path 1: The InstQ is full, so stall.
//   Path 1: Blocking for the ICache, but it hasn't responded yet.
//   Path 2: We asked the ICache for an instruction, and it missed.
//   Path 3: The ICache told us to retry our request next cycle.
//   Path 4: The instruction was sent to the InstQ.


module [HASIM_MODULE] mkFetch ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_fetch.out");

    // ****** Model State (per Context) ******

    MULTICTX#(Reg#(ISA_ADDRESS))               ctx_pc <- mkMultiCtx(mkReg(`PROGRAM_START_ADDR));
    MULTICTX#(Reg#(Bool))           ctx_waitForICache <- mkMultiCtx(mkReg(False));
    MULTICTX#(Reg#(TOKEN_FAULT_EPOCH)) ctx_faultEpoch <- mkMultiCtx(mkReg(0));
    MULTICTX#(FIFO#(ISA_ADDRESS))             ctx_pcQ <- mkMultiCtx(mkFIFO());


    // ****** Ports ******

    PORT_STALL_SEND_MULTICTX#(FETCH_BUNDLE)                bundleToInstQ <- mkPortStallSend_MultiCtx("InstQ");
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN,ISA_ADDRESS))        rewindFromExe <- mkPortRecv_MultiCtx("Exe_to_Fet_rewind", 1);
    PORT_RECV_MULTICTX#(TOKEN)                              faultFromCom <- mkPortRecv_MultiCtx("Com_to_Fet_fault", 1);

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN,ISA_ADDRESS)) pcToBP <- mkPortSend_MultiCtx("Fet_to_BP_pc");
    PORT_RECV_MULTICTX#(ISA_ADDRESS)            predFromBP <- mkPortRecv_MultiCtx("BP_to_Fet_pred", 1);
    PORT_RECV_MULTICTX#(BRANCH_ATTR)            attrFromBP <- mkPortRecvGuarded_MultiCtx("BP_to_Fet_attr", 0);

    PORT_SEND_MULTICTX#(Tuple2#(TOKEN, CacheInput))                        pcToICache <- mkPortSend_MultiCtx("CPU_to_ICache_req");
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputImmediate))  rspFromICacheImmediate <- mkPortRecvGuarded_MultiCtx("ICache_to_CPU_immediate", 0);
    PORT_RECV_MULTICTX#(Tuple2#(TOKEN, CacheOutputDelayed))      rspFromICacheDelayed <- mkPortRecvGuarded_MultiCtx("ICache_to_CPU_delayed", 0);


    // ****** Soft Connections ******

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         modelCycle <- mkConnection_Send("model_cycle");

    Connection_Client#(FUNCP_REQ_NEW_IN_FLIGHT,
                       FUNCP_RSP_NEW_IN_FLIGHT)      newInFlight <- mkConnection_Client("funcp_newInFlight");

    Connection_Client#(FUNCP_REQ_DO_ITRANSLATE,
                       FUNCP_RSP_DO_ITRANSLATE)     doITranslate <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION) getInstruction <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    Connection_Client#(FUNCP_REQ_HANDLE_FAULT,
                       FUNCP_RSP_HANDLE_FAULT)       handleFault <- mkConnection_Client("funcp_handleFault");


    // ****** UnModel Pipeline State ******

    Reg#(FET1_STATE) stage1State <- mkReg(FET1_ready);
    Reg#(FET2_STATE) stage2State <- mkReg(FET2_ready);

    FIFO#(CONTEXT_ID)  stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID)  stage3Q <- mkFIFO();


    // ****** Local Controller ******

    Vector#(4, PORT_CONTROLS) inports  = newVector();
    Vector#(3, PORT_CONTROLS) outports = newVector();
    inports[0]  = rewindFromExe.ctrl;
    // inports[1]  = rspFromICacheImmediate.ctrl;
    // inports[2]  = rspFromICacheDelayed.ctrl;
    inports[1]  = predFromBP.ctrl;
    // inports[4]  = attrFromBP.ctrl;
    inports[2]  = faultFromCom.ctrl;
    inports[3]  = bundleToInstQ.ctrl;
    outports[0] = bundleToInstQ.ctrl;
    outports[1] = pcToICache.ctrl;
    outports[2] = pcToBP.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Events and Stats ******

    EVENT_RECORDER_MULTICTX eventFet <- mkEventRecorder_MultiCtx(`EVENTS_FETCH_INSTRUCTION_FET);

    STAT_RECORDER_MULTICTX statCycles  <- mkStatCounter_MultiCtx(`STATS_FETCH_TOTAL_CYCLES);
    STAT_RECORDER_MULTICTX statFet     <- mkStatCounter_MultiCtx(`STATS_FETCH_INSTS_FETCHED);
    STAT_RECORDER_MULTICTX statIMisses <- mkStatCounter_MultiCtx(`STATS_FETCH_ICACHE_MISSES);


    // ****** Rules ******
    
    // stage1_nextPC
    
    // Begin a new model cycle for a given context.
    // Update the PC with any faults, rewinds, or branch predictions.
    // If a fault or rewind occurs, stall until it is handled.
    // Note: this may degrade performance, since no other contexts are being handled 
    // in this time. We should ensure that this is not a performance bottleneck.
    
    rule stage1_nextPC (stage1State == FET1_ready);

        // Begin a new model cycle.
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);
        statCycles.incr(ctx);
        modelCycle.send(ctx);

        // Get our state from the contexts.
        Reg#(ISA_ADDRESS)               pc = ctx_pc[ctx];
        Reg#(TOKEN_FAULT_EPOCH) faultEpoch = ctx_faultEpoch[ctx];

        // Receive potential new PCs from our incoming ports.
        let m_pred_pc <- predFromBP.receive(ctx);
        let m_rewind  <- rewindFromExe.receive(ctx);
        let m_fault   <- faultFromCom.receive(ctx);
        
        // Later pipeline stages are given higher priority.
        // fault > rewind > prediction
        
        // Branch epochs are stored in the token and the functional partition.
        // We track the fault epoch ourselves here.
        // Assumed invariant:
        // We will get no more redirects from that stage until an instruction
        // of the new epoch reaches it. 
        // (IE the stage has a local copy of the epoch and silently drops 
        //  until it sees the new epoch.)
                
        if (m_fault matches tagged Valid { .tok })
        begin
            
            // A fault occurred. Get the handler PC from the functional partition.
            debugLog.record_next_cycle(ctx, fshow("FAULT: ") + fshow(tok));
            handleFault.makeReq(initFuncpReqHandleFault(tok));
            faultEpoch <= faultEpoch + 1;
            // Stall this stage until we get the response from the functional partition.
            stage1State <= FET1_faultRsp;

        end
        else if (m_rewind matches tagged Valid { .tok, .addr} &&&
                 tokFaultEpoch(tok) == faultEpoch)
        begin

            // A branch misprediction occured.
            // Epoch check ensures we haven't already redirected from a fault.
            debugLog.record_next_cycle(ctx, fshow("REWIND: ") + fshow(tok) + $format(" ADDR:0x%h", addr));
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            pc <= addr;
            // Stall this stage until the functional partition indicates the rewind is complete.
            stage1State <= FET1_rewindRsp;

        end
        else if (m_pred_pc matches tagged Valid .pred_pc)
        begin

            // We got a new response from the branch predictor.
            pc <= pred_pc;
            // Send to the next stage.
            stage2Q.enq(ctx);

        end
        else
        begin
            
            // There's no prediction, so keep the PC the same.
            // assert waitForICache == True
            stage2Q.enq(ctx);

        end

    endrule


    // stage1_faultRsp
    
    // Get a fault response, redirect to the handler, and unstall the pipeline.

    rule stage1_faultRsp (stage1State == FET1_faultRsp);

        // Get the fault response.
        let rsp = handleFault.getResp();
        handleFault.deq();

        // Get our state from the contexts.
        let tok = rsp.token;
        let ctx = tokContextId(tok);
        Reg#(ISA_ADDRESS) pc = ctx_pc[ctx];

        // Resteer to the handler.
        pc <= rsp.nextInstructionAddress;

        // Unstall this stage.
        stage1State <= FET1_ready;

        // Send to the next stage.
        stage2Q.enq(ctx);

    endrule


    // stage1_rewind
    
    // Get a rewind response and unstall the pipeline.

    rule stage1_rewindRsp (stage1State == FET1_rewindRsp);

        // Get the rewind response.
        let rsp = rewindToToken.getResp();
        let tok = rsp.token;
        let ctx = tokContextId(tok);
        rewindToToken.deq();
        
        // Unstall this stage.
        stage1State <= FET1_ready;

        // Send to the next stage.
        stage2Q.enq(ctx);

    endrule


    // stage2_token
    
    // If the InstQ has room and we're not stalled for the ICache,
    // then request a new token from the functional partition.

    rule stage2_token (stage2State == FET2_ready);

        // Get the context from the previous stage.
        let ctx = stage2Q.first();
        
        // Get our local state from the context.
        Reg#(Bool) waitForICache = ctx_waitForICache[ctx];
        
        if (bundleToInstQ.canEnq(ctx))
        begin
            
            // The instructionQ still has room...
            if (waitForICache)
            begin
                
                // We're waiting for an ICache response.
                // We already have a token, so just go on.
                
                // Don't request a new instruction load or branch prediction.
                pcToICache.send(ctx, tagged Invalid);
                pcToBP.send(ctx, tagged Invalid);

                // Send to the next stage.
                stage3Q.enq(ctx);
                stage2Q.deq();

            end
            else
            begin

                // Request a new token which we can send to the ICache.
                newInFlight.makeReq(initFuncpReqNewInFlight(ctx));
                
                // Stall on the response.
                stage2State <= FET2_iCacheReq;

	    end

        end
        else
        begin

            // The instructionQ is full... Nothing we can do.
            eventFet.recordEvent(ctx, tagged Invalid);
            
            // Don't enqueue to the InstQ.
            bundleToInstQ.noEnq(ctx);
            
            // Don't request a new instruction load or branch prediction.
            pcToICache.send(ctx, tagged Invalid);
            pcToBP.send(ctx, tagged Invalid);

            // Stall to handle the cache response.
            stage2State <= FET2_stall;

        end

    endrule

    // stage2_stall
    
    // Since the ports to/from the ICache are zero-latency, 
    // handle the response in a separate rule.
    
    // Note: Does this really need to stall the stage if no 
    // downstream values depend on the response? If we could ensure
    // that the local controller would not issue the same Context again
    // than this could be a separate stage.

    rule stage2_stall (stage2State == FET2_stall);
        
        // Get the stalled context.
        let ctx = stage2Q.first();
        debugLog.record(ctx, fshow("STALL"));

        // Drop the (presumably) invalid responses from the ICache.
        // assert icache_resp == Invalid.
        let icache_resp_imm <- rspFromICacheImmediate.receive(ctx);
        let icache_resp_del <- rspFromICacheDelayed.receive(ctx);
        let attr <- attrFromBP.receive(ctx);

        // Unstall the pipeline.
        stage2State <= FET2_ready;
        stage2Q.deq();
        
        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(ctx, 1);

    endrule


    // stage2_iCacheReq
    
    // Get a new token from the functional partition and
    // send the it and the current PC to the ICache and Branch Predictor.s
    
    rule stage2_iCacheReq (stage2State == FET2_iCacheReq);

        // Get the response.
        let rsp = newInFlight.getResp();
        newInFlight.deq();

        // Get our local state from the context.
        let tok = rsp.newToken;
        let ctx = tokContextId(tok);
        Reg#(ISA_ADDRESS) pc = ctx_pc[ctx];

        // Send the current PC to the cache and Branch predictor.
        pcToICache.send(ctx, tagged Valid tuple2(tok, tagged Inst_mem_ref pc));
        pcToBP.send(ctx, tagged Valid tuple2(tok, pc));

        // Unstall this stage.
        stage2Q.deq();
        stage2State <= FET2_ready;

        // Send this context to the next stage.
        stage3Q.enq(ctx);

    endrule

    // stage3_iCacheRsp
    
    // When the instruction cache responds it's either a hit or a miss.
    // If it's a miss, send bubbles until the miss response comes back.
    // If it's a hit or a miss response, start translating the address 
    // in the functional partition.
    
    // Note: In a more complex memory hierarchy the Itranslate should
    // probably be done by the simulated TLB, not by the Fetch module.

    rule stage3_iCacheRsp (True);

        // Get the new context.
        let ctx = stage3Q.first();
        stage3Q.deq();

        // Get our local state from the context.
        Reg#(Bool) waitForICache = ctx_waitForICache[ctx];
        FIFO#(ISA_ADDRESS)   pcQ = ctx_pcQ[ctx];

        // Get the responses from the ICache.
        let icache_ret_imm <- rspFromICacheImmediate.receive(ctx);
        let icache_ret_del <- rspFromICacheDelayed.receive(ctx);

        // assert !isValid(imm) && !isValid(del)

        case (icache_ret_del) matches
            tagged Invalid:
            begin

                // No delayed response came back. Check the immediate response.

                case (icache_ret_imm) matches

                    tagged Invalid:
                    begin

                        // Nothing... we're stalled.
                        debugLog.record(ctx, fshow("MISS: STALL"));
                        eventFet.recordEvent(ctx, tagged Invalid);
                        
                        // Note to send bubbles until a response comes back.
                        waitForICache <= True;

                        // Don't enqueue anything into the InstQ.
	                bundleToInstQ.noEnq(ctx);
                        
                        // End of model cycle. (Path 2)
                        localCtrl.endModelCycle(ctx, 2);

                    end

                    tagged Valid {.icachetok, .icachemsg}:
                    begin

                        case (icachemsg) matches

                            tagged Hit .reqpc:
                            begin
                                
                                // A hit!
                                debugLog.record(ctx, fshow("HIT: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc));
                                statFet.incr(ctx);

                                // Get the physical address from the functional partition.
                                doITranslate.makeReq(initFuncpReqDoITranslate(icachetok, reqpc));

                                // Unstall the Fetch module.
                                waitForICache <= False;

                                // Pass the PC to the next stage so it can go to the DEC module.
                                pcQ.enq(reqpc);

                            end

                            tagged Miss_servicing .reqpc:
                            begin

                                // A miss, but the cache is handling it.
                                debugLog.record(ctx, fshow("MISS: STALL"));
                                eventFet.recordEvent(ctx, tagged Invalid);
                                statIMisses.incr(ctx);

                                // Note that we should send bubbles until the response comes back.
                                waitForICache <= True;

                                // Don't enqueue anything into the InstQ.
	                        bundleToInstQ.noEnq(ctx);
                                
                                // End of model cycle. (Path 3)
                                localCtrl.endModelCycle(ctx, 3);

                            end

                            tagged Miss_retry .reqpc:
                            begin

                                // A miss, and the cache needs us to retry.
                                debugLog.record(ctx, fshow("MISS: RETRY"));

                                // Don't enqueue anything into the InstQ.
	                        bundleToInstQ.noEnq(ctx);
                                
                                // This is not acutally implemented, since we have no
                                // way to resteer the PC.
                                $display("WARNING: Fetch module encountered unimplemented Miss_retry command.");
                                
                                // End of model cycle. (Path 4)
                                localCtrl.endModelCycle(ctx, 4);

                            end

                        endcase

                    end

                endcase

            end

            tagged Valid { .icachetok, .icachemsg }:
            begin
            
                // A delayed response came back.
                case (icachemsg) matches

                    tagged Miss_response .reqpc:
                    begin
                        
                        // A miss came back. 
                        debugLog.record(ctx, fshow("MISS: RESP: ") + fshow(icachetok) + $format(" ADDR:0x%h", reqpc));
                        statFet.incr(ctx);
                        
                        // Get the physical address
                        // from the functional partition.
                        doITranslate.makeReq(initFuncpReqDoITranslate(icachetok, reqpc));
                        
                        // Stop sending bubbles.
                        waitForICache <= False;
                        
                        // Pass the current PC to the next stage so it can go to the DEC module.
                        pcQ.enq(reqpc);

                    end

                endcase

            end

        endcase
        
    endrule

    // stage4_getInst
    
    // Get the physical address from the from functional partition.
    // Now we can get the actual instruction from the functional partition.
    
    // Note: The functional partition could respond with multiple physical
    // addresses, since the PC could have been unaligned. This would probably
    // indicate that we're speculatively executing a garbage region.
    // Such a stall should be exceedingly rare.

    rule stage4_getInst (True);

        let rsp = doITranslate.getResp();
        doITranslate.deq();

        if (!rsp.hasMore)
        begin

            // We've got the complete physical address. Get the actual instruction.
            // This passes the token to the next stage via the functional partition.
            getInstruction.makeReq(initFuncpReqGetInstruction(rsp.token));

        end

    endrule
    
    // stage5_done
    
    // Get the actual instruction and pass it, the PC, and the branch prediction
    // to the DEC module.
    
    rule stage5_done (True);
        
        // Get the instruction from the functional partition.
        let rsp = getInstruction.getResp();
        getInstruction.deq();

        // Get our context from the token.
        let tok = rsp.token;
        let ctx = tokContextId(tok);
        FIFO#(ISA_ADDRESS) pcQ = ctx_pcQ[ctx];
        
        // Get the PC from the previous stages.
        let pc = pcQ.first();
        pcQ.deq();

        // Get the branch prediction attributes.
        let m_attr <- attrFromBP.receive(ctx);
        // assert m_attr = Valid
        let attr = validValue(m_attr);
        
        // Marshall up a bundle and enqueue it in the InstQ.
        let bundle = FETCH_BUNDLE {token: tok, pc: pc, inst: rsp.instruction, branchAttr: attr};
        bundleToInstQ.doEnq(ctx, bundle);
        
        // End of model cycle. (Path 5)
        eventFet.recordEvent(ctx, tagged Valid zeroExtend(pack(tok.index)));
        localCtrl.endModelCycle(ctx, 5);

    endrule

endmodule
