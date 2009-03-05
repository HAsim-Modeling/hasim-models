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
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"
`include "asim/provides/memory_base_types.bsh"

`define STORE_BUFF_SIZE 4

// mkStoreBuffer

// A simple INORDER head/tail circular buffer store buffer.

// INORDER means that it assumes that stores are committed in the same order they're allocated.
// Thus this could not be used for an out-of-order issue processor.

// This uses an associative memory. Therefore it is best for small sizes. 
// Larger sizes would want to use BRAM or LUTRAM and sequentially search the RAMs.

// This module is pipelined across contexts. Stages:
// Stage 1 -> Stage 2
// These stages will never stall.

// There is only one way that a model cycle can end.


module [HASIM_MODULE] mkStoreBuffer ();

    TIMEP_DEBUG_FILE_MULTICTX debugLog <- mkTIMEPDebugFile_MultiCtx("pipe_storebuffer.out");


    // ****** Model State (per Context) ******
    
    MULTICTX#(Vector#(`STORE_BUFF_SIZE, Reg#(Maybe#(ISA_ADDRESS)))) ctx_buff <- mkMultiCtx(replicateM(mkReg(Invalid)));
    MULTICTX#(Reg#(Bit#(2))) ctx_head <- mkMultiCtx(mkReg(0));
    MULTICTX#(Reg#(Bit#(2))) ctx_tail <- mkMultiCtx(mkReg(0));

    function Bool empty(CONTEXT_ID ctx) = ctx_head[ctx] == ctx_tail[ctx];
    function Bool full(CONTEXT_ID ctx)  = ctx_head[ctx] == ctx_tail[ctx] + 1;


    // ****** UnModel Pipeline State ******

    FIFO#(CONTEXT_ID) stage2Q <- mkFIFO();
    FIFO#(CONTEXT_ID) stage3Q <- mkFIFO();

    // ****** Ports ******

    PORT_RECV_MULTICTX#(CACHE_INPUT) reqFromMem    <- mkPortRecv_MultiCtx("DMem_to_SB_req", 0);
    PORT_RECV_MULTICTX#(TOKEN)      deallocFromCom <- mkPortRecv_MultiCtx("Com_to_SB_dealloc", 1);
    PORT_SEND_MULTICTX#(SB_RESPONSE)   rspToMem    <- mkPortSend_MultiCtx("SB_to_DMem_rsp");


    // ****** Local Controller ******

    Vector#(2, PORT_CONTROLS) inports  = newVector();
    Vector#(1, PORT_CONTROLS) outports = newVector();
    inports[0]  = reqFromMem.ctrl;
    inports[1]  = deallocFromCom.ctrl;
    outports[0] = rspToMem.ctrl;

    LOCAL_CONTROLLER localCtrl <- mkLocalController(inports, outports);


    // ****** Rules ******

    rule stage1_dealloc (True);
    
        let ctx <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(ctx);

        Reg#(Bit#(2)) head = ctx_head[ctx];
        let buff = ctx_buff[ctx];

        let m <- deallocFromCom.receive(ctx);

        if (m matches tagged Valid .tok)
        begin

            // assert !empty(ctx)

            // A deallocation request.
            debugLog.record_next_cycle(ctx, fshow("DEALLOC ") + fshow(tok));

            // Invalidate the oldest entry and update the head pointer.
            head <= head + 1;
            buff[head] <= Invalid;

        end

        stage2Q.enq(ctx);

    endrule


    // stage2_search
    

    rule stage2_search (True);

        let ctx = stage2Q.first();
        stage2Q.deq();

        Reg#(Bit#(2)) tail = ctx_tail[ctx];

        let buff = ctx_buff[ctx];

        let m_req <- reqFromMem.receive(ctx);

        case (m_req) matches
            tagged Invalid:
            begin
                // Propogate the bubble.
                debugLog.record(ctx, fshow("BUBBLE"));
                rspToMem.send(ctx, Invalid);
            end
            tagged Valid .req:
            case (req.reqType) matches
                tagged CACHE_loadData { .pc, .addr }:
                begin
                    if (elem(tagged Valid addr, readVReg(buff)))
                    begin

                        // We've got that address in the store buffer.
                        debugLog.record(ctx, fshow("LOAD HIT ") + fshow(req.token));

                        // Luckily, since we're a simulation, we don't actually 
                        // need to retrieve the value, which makes the hardware a LOT simpler
                        // as we don't need to get the "youngest store older than this load"
                        // Instead, just tell the Mem module that we have the value.
                        rspToMem.send(ctx, tagged Valid (tagged SB_HIT req.token));

                    end
                    else
                    begin

                        // We don't have it.
                        debugLog.record(ctx, fshow("LOAD MISS ") + fshow(req.token));
                        rspToMem.send(ctx, tagged Valid (tagged SB_MISS req.token));

                    end
                end
                tagged CACHE_writeData { .pc, .a }:
                begin

                    if (full(ctx))
                    begin
                    
                        // We're full, the requester will have to wait until the pipeline drains.
                        debugLog.record(ctx, fshow("SB STORE RETRY (SB FULL!) ") + fshow(req.token));
                        rspToMem.send(ctx, tagged Valid (tagged SB_STALL req.token));

                    end
                    else
                    begin

                        // Do the allocation.
                        debugLog.record(ctx, fshow("SB STORE ALLOC ") + fshow(req.token));
                        tail <= tail + 1;
                        buff[tail] <= Valid(a);
                        // No need for a response.
                        rspToMem.send(ctx, tagged Invalid);

                    end

                end

            endcase

        endcase
        
        localCtrl.endModelCycle(ctx, 1);

    endrule
    
endmodule
