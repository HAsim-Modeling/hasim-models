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

import Vector::*;
import FShow::*;
import FIFO::*;


// ****** Project imports ******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/funcp_interface.bsh"

// ****** Timing Model Imports ******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/l1_cache_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/funcp_memstate_base_types.bsh"



typedef Bit#(TLog#(`SB_NUM_ENTRIES)) SB_INDEX;

// mkStoreBuffer

// A simple head/tail circular buffer store buffer.

// This uses an associative memory. Therefore it is best for small sizes. 
// Larger sizes would want to use BRAM or LUTRAM and sequentially search the RAMs.

// This module is pipelined across instances. Stages:
// Stage 1 -> Stage 2
// These stages will never stall.

// There is only one way that a model cycle can end.


module [HASIM_MODULE] mkStoreBuffer ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_storebuffer.out");


    // ****** Model State (per Context) ******
    
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))))) tokIDPool       <- mkMultiplexed(mkReg(multithreaded(replicate(Invalid))));
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, STORE_TOKEN))))   storeTokenPool  <- mkMultiplexed(mkRegU());
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(MEM_ADDRESS))))) physAddressPool <- mkMultiplexed(mkReg(multithreaded(replicate(Invalid))));

    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(SB_INDEX))) oldestCommittedPool   <- mkMultiplexed(mkReg(multithreaded(0)));
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(SB_INDEX))) numToCommitPool <- mkMultiplexed(mkReg(multithreaded(0)));
    MULTIPLEXED#(NUM_CPUS, Reg#(MULTITHREADED#(SB_INDEX))) nextFreeSlotPool      <- mkMultiplexed(mkReg(multithreaded(0)));

    // The thread which well look at next for storing to the writeQ.
    MULTIPLEXED#(NUM_CPUS, Reg#(THREAD_ID)) headThreadPool <- mkMultiplexed(mkReg(0));

    function Bool empty(CPU_INSTANCE_ID cpu_iid, THREAD_ID thread) = nextFreeSlotPool[cpu_iid][thread] == oldestCommittedPool[cpu_iid][thread];
    function Bool full(CPU_INSTANCE_ID cpu_iid, THREAD_ID thread)  = oldestCommittedPool[cpu_iid][thread] == nextFreeSlotPool[cpu_iid][thread] + 1;


    // ****** UnModel Pipeline State ******

    FIFO#(CPU_INSTANCE_ID) stage2Q <- mkFIFO();
    FIFO#(CPU_INSTANCE_ID) stage3Q <- mkFIFO();
    FIFO#(CPU_INSTANCE_ID) stage4Q <- mkFIFO();
    
    Reg#(Vector#(NUM_CPUS, Bool)) stallForStoreRsp <- mkReg(replicate(False));

    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(NUM_CPUS, TOKEN)             allocFromDec    <- mkPortRecv_Multiplexed("Dec_to_SB_alloc", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, SB_INPUT)          reqFromDMem     <- mkPortRecv_Multiplexed("DMem_to_SB_req", 0);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, SB_DEALLOC_INPUT)  deallocFromCom  <- mkPortRecv_Multiplexed("Com_to_SB_dealloc", 1);
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VOID)            creditFromWriteQ  <- mkPortRecv_Multiplexed("WB_to_SB_credit", 1);

    PORT_SEND_MULTIPLEXED#(NUM_CPUS, SB_OUTPUT)      rspToDMem     <- mkPortSend_Multiplexed("SB_to_DMem_rsp");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, MULTITHREADED#(Bool))          creditToDecode <- mkPortSend_Multiplexed("SB_to_Dec_credit");
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, WB_ENTRY)       storeToWriteQ <- mkPortSend_Multiplexed("SB_to_WB_enq");

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_DO_STORES, FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports  = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outports = newVector();
    inports[0]  = reqFromDMem.ctrl;
    inports[1]  = allocFromDec.ctrl;
    inports[2]  = deallocFromCom.ctrl;
    inports[3]  = creditFromWriteQ.ctrl;
    outports[0] = rspToDMem.ctrl;
    outports[1] = creditToDecode.ctrl;
    outports[2] = storeToWriteQ.ctrl;

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    (* conservative_implicit_conditions *)
    rule stage1_alloc (True);
    
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Get our local state based on the current context.
        Reg#(MULTITHREADED#(SB_INDEX)) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(MULTITHREADED#(SB_INDEX)) oldestCommitted = oldestCommittedPool[cpu_iid];

        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))))             tokID = tokIDPool[cpu_iid];
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(MEM_ADDRESS)))) physAddress = physAddressPool[cpu_iid];

        // Check if the decode is allocating a new slot.
        let m_alloc <- allocFromDec.receive(cpu_iid);
        
        let new_free = nextFreeSlot;
        
        if (m_alloc matches tagged Valid .tok)
        begin

            let thread = tokThreadId(tok);
        
            // Allocate a new slot.
            // assert !full(cpu_iid)
            debugLog.record(cpu_iid, fshow("ALLOC ") + fshow(tok));
            tokID[thread][nextFreeSlot[thread]] <= tagged Valid tok;

            // We don't know its effective address yet.
            physAddress[thread][nextFreeSlot[thread]] <= tagged Invalid;

            new_free[thread] = new_free[thread] + 1;
        
        end
        
        // Calculate the credits for decode.
        function Bool hasNoCredits(SB_INDEX newfree, SB_INDEX oldest);
            return (((newfree + 2) == oldest) || ((newfree + 1) == oldest)); // Plus 2 because we assume it takes a cycle for the credit to arrive. Should really be +L+1 where L is latency of credit.
        endfunction

        function Bool hasCredits(SB_INDEX newfree, SB_INDEX oldest) = !hasNoCredits(newfree, oldest);
            
        MULTITHREADED#(Bool) credits = zipWith(hasCredits, new_free, oldestCommitted);
        debugLog.record_next_cycle(cpu_iid, fshow("SEND CREDIT: ") + fshow(credits));
        creditToDecode.send(cpu_iid, tagged Valid credits);
        
        // Update the tail.        
        nextFreeSlot <= new_free;


        // Continue to the next stage.
        stage2Q.enq(cpu_iid);

    endrule


    // stage2_search
    
    (* conservative_implicit_conditions *)
    rule stage2_search (True);

        let cpu_iid = stage2Q.first();
        stage2Q.deq();

        // Get our local state based on the current context.
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))))             tokID = tokIDPool[cpu_iid];
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(MEM_ADDRESS)))) physAddress = physAddressPool[cpu_iid];

        // See if the DMem is completing or searching.
        let m_req <- reqFromDMem.receive(cpu_iid);

        case (m_req) matches
            tagged Invalid:
            begin

                // Propogate the bubble.
                debugLog.record(cpu_iid, fshow("NO SEARCH"));
                rspToDMem.send(cpu_iid, Invalid);

            end
            tagged Valid .req:
            case (req.reqType) matches
                tagged SB_search:
                begin


                    let target_addr = req.bundle.physicalAddress;
                    let thread = tokThreadId(req.bundle.token);

                    // Luckily, since we're a simulation, we don't actually 
                    // need to retrieve the value, which makes the hardware a LOT simpler
                    // as we don't need to get the "youngest store older than this load"
                    // Instead, just tell the DMem module that we have the value.
                
                    Bool hit = False;
                
                    for (Integer x = 0; x < `SB_NUM_ENTRIES; x = x + 1)
                    begin

                        // It's a hit if it's a store to the same address which is older than the load.
                        let addr_match = case (physAddress[thread][x]) matches 
                                            tagged Valid .addr: return addr == target_addr;
                                            tagged Invalid: return False;
                                         endcase;

                        let older_store = case (tokID[thread][x]) matches 
                                                tagged Valid .tok: return tokenIsOlderOrEq(tok.index.token_id, req.bundle.token.index.token_id);
                                                tagged Invalid: return False;
                                            endcase;

                        hit = hit || (addr_match && older_store);
                    end

                    if (hit)
                    begin

                        // We've got that address in the store buffer.
                        debugLog.record(cpu_iid, fshow("LOAD HIT ") + fshow(req.bundle.token));

                        rspToDMem.send(cpu_iid, tagged Valid initSBHit(req.bundle));

                    end
                    else
                    begin

                        // We don't have it.
                        debugLog.record(cpu_iid, fshow("LOAD MISS ") + fshow(req.bundle.token));
                        rspToDMem.send(cpu_iid, tagged Valid initSBMiss(req.bundle));

                    end

                end
                tagged SB_complete:
                begin

                    // A completion of a previously allocated store.
                    debugLog.record(cpu_iid, fshow("COMPLETE STORE ") + fshow(req.bundle.token));

                    // Update with the actual physical address.
                    // (A real store buffer would also record the value.)
                    let tok_id = tokTokenId(req.bundle.token);
                    let thread = tokThreadId(req.bundle.token);
                    
                    // We find the index for this token using a CAM Write
                    
                    SB_INDEX sb_idx = 0;
                    
                    for (Integer x = 0; x < `SB_NUM_ENTRIES; x = x + 1)
                    begin
                    
                        if (tokID[thread][x] matches tagged Valid .tok &&& tokTokenId(tok) == tok_id)
                            sb_idx = fromInteger(x);
                    
                    end
                    
                    physAddress[thread][sb_idx] <= tagged Valid req.bundle.physicalAddress;

                    // Tell the functional partition to make the store locally visible.
                    doStores.makeReq(initFuncpReqDoStores(req.bundle.token));

                    // Don't end the model cycle until the store response has come in.
                    stallForStoreRsp[cpu_iid] <= True;

                    // No need for a response.
                    rspToDMem.send(cpu_iid, tagged Invalid);

                end

            endcase

        endcase
        
        // Continue to the next stage.
        stage3Q.enq(cpu_iid);

    endrule
    
    (* conservative_implicit_conditions *)
    rule stage3_dealloc (True);
    
        // Get our context from the previous stage.
        let cpu_iid = stage3Q.first();
        stage3Q.deq();
    
        // Get our local state based on the current context.
        Reg#(MULTITHREADED#(SB_INDEX)) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(MULTITHREADED#(SB_INDEX)) oldestCommitted = oldestCommittedPool[cpu_iid];
        Reg#(MULTITHREADED#(SB_INDEX)) numToCommit = numToCommitPool[cpu_iid];

        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN)))) tokID = tokIDPool[cpu_iid];
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, STORE_TOKEN))) storeToken = storeTokenPool[cpu_iid];

        // See if we're getting a deallocation request.
        let m_dealloc <- deallocFromCom.receive(cpu_iid);
        
        if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_drop)
        begin

            // Invalidate the requested entry. We assume drop/dealloc requests come in allocation order.
            let thread = tokThreadId(req.token);
            debugLog.record(cpu_iid, fshow("DROP REQ ") + fshow(req.token));
            tokID[thread][oldestCommitted[thread] + numToCommit[thread]] <= tagged Invalid;

            // Record that the commit path has work to do.
            numToCommit[thread] <= numToCommit[thread] + 1;
        
        end
        else if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_writeback)
        begin

            // Update the token with the latest value.
            let thread = tokThreadId(req.token);
            debugLog.record(cpu_iid, fshow("DEALLOC REQ ") + fshow(req.token));
            tokID[thread][oldestCommitted[thread] + numToCommit[thread]] <= tagged Valid req.token;
            storeToken[thread][oldestCommitted[thread] + numToCommit[thread]] <= req.storeToken;

            // Record that the commit path has work to do.
            numToCommit[thread] <= numToCommit[thread] + 1;
        
        end
        
        // Finish up in the next stage.
        stage4Q.enq(cpu_iid);
        
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage4_commit (!stallForStoreRsp[stage4Q.first()]);
    
        // Get our context from the previous stage.
        let cpu_iid = stage4Q.first();
        stage4Q.deq();
    
        // Get our local state based on the current context.
        Reg#(MULTITHREADED#(SB_INDEX)) nextFreeSlot = nextFreeSlotPool[cpu_iid];
        Reg#(MULTITHREADED#(SB_INDEX)) oldestCommitted = oldestCommittedPool[cpu_iid];
        Reg#(MULTITHREADED#(SB_INDEX)) numToCommit = numToCommitPool[cpu_iid];

        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN))))       tokID = tokIDPool[cpu_iid];
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, Maybe#(MEM_ADDRESS)))) physAddress = physAddressPool[cpu_iid];
        Reg#(MULTITHREADED#(Vector#(`SB_NUM_ENTRIES, STORE_TOKEN))) storeToken = storeTokenPool[cpu_iid];
        Reg#(THREAD_ID) headThread = headThreadPool[cpu_iid];

        // See if the Write Buffer has room.
        let m_credit <- creditFromWriteQ.receive(cpu_iid);
        let write_buff_has_credit = isValid(m_credit);

        // We need to dealloc if we have pending commmits.
        if (numToCommit[headThread] != 0)
        begin
            case (tokID[headThread][oldestCommitted[headThread]]) matches
                tagged Invalid:
                begin
                
                    // If the oldest committed token is invalid then it was dropped. Just move over it.
                    debugLog.record(cpu_iid, fshow("JUNK DROPPED"));
                    oldestCommitted[headThread] <= oldestCommitted[headThread] + 1;
                    numToCommit[headThread] <= numToCommit[headThread] - 1;
                    
                    // No guys to commit.
                    storeToWriteQ.send(cpu_iid, tagged Invalid);

                end
                tagged Valid .tok:
                begin
                    // The oldest token has been committed. Let's see if we can send it to the write buffer.
                    if (physAddress[headThread][oldestCommitted[headThread]] matches tagged Valid .phys_addr &&& write_buff_has_credit)
                    begin

                        // It's got room. Let's send the oldest store.
                        debugLog.record(cpu_iid, fshow("DEALLOC ") + fshow(tok));

                        // Dequeue the old entry.
                        tokID[headThread][oldestCommitted[headThread]] <= tagged Invalid;
                        oldestCommitted[headThread] <= oldestCommitted[headThread] + 1;
                        numToCommit[headThread] <= numToCommit[headThread] - 1;

                        // Send it to the writeBuffer.
                        let store_tok = storeToken[headThread][oldestCommitted[headThread]];
                        storeToWriteQ.send(cpu_iid, tagged Valid tuple2(store_tok, phys_addr));

                    end
                    else
                    begin
                    
                        // No room to commit this guy.
                        debugLog.record(cpu_iid, fshow("DEALLOC STALL ") + fshow(tok));
                        storeToWriteQ.send(cpu_iid, tagged Invalid);
                    
                    end
                    
                end
            
            endcase
        
        end
        else
        begin

            // No guys to commit.
            debugLog.record(cpu_iid, fshow("NO DEALLOC"));
            storeToWriteQ.send(cpu_iid, tagged Invalid);
        
        end
        headThread <= headThread+1;
        
        localCtrl.endModelCycle(cpu_iid, 1);

    endrule
    
    (* conservative_implicit_conditions *)
    rule storeRsp (True);
    
        let rsp = doStores.getResp();
        doStores.deq();
        let tok = rsp.token;
        
        let cpu_iid = tokCpuInstanceId(tok);
        stallForStoreRsp[cpu_iid] <= False;
    
    endrule

endmodule
