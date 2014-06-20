//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

import Vector::*;
import FShow::*;

// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/l1_cache_base_types.bsh"

//
// mkL1CacheArbiter --
//   Merge an L1 I-cache and D-cache into what appears to be a single interface
//   from the perspective of lower down in the hierarchy.  In this code, lower
//   down the hierarchy is called "memory", though it is likely the L2.
//
//   The number of ports (channels) is assumed to be the same for each L1 and
//   for the path to memory.
//
module [HASIM_MODULE] mkL1CacheArbiter#(NumTypeParam#(n_TO_MEM_CHANNELS) nToMemChannels,
                                        String toMemoryName,
                                        NumTypeParam#(n_FROM_MEM_CHANNELS) nFromMemChannels,
                                        String fromMemoryName)
    // Interface
    (Empty);

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_arbiter.out");

    // Eventually this could be a dynamic parameter.
    Bool favorICache = `L1_ARBITER_FAVOR_ICACHE;

    // Queues to/from DCache
    Vector#(n_TO_MEM_CHANNELS,
            PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) fromDCache = newVector();

    Vector#(n_FROM_MEM_CHANNELS,
            PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) toDCache = newVector();

    // Queues to/from ICache
    Vector#(n_TO_MEM_CHANNELS,
            PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) fromICache = newVector();

    Vector#(n_FROM_MEM_CHANNELS,
            PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) toICache = newVector();


    // Queues to/from Memory
    Vector#(n_TO_MEM_CHANNELS,
            PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) toMemory = newVector();

    Vector#(n_FROM_MEM_CHANNELS,
            PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) fromMemory = newVector();

    for (Integer c = 0; c < valueOf(n_TO_MEM_CHANNELS); c = c + 1)
    begin
        let n = integerToString(c);
        fromDCache[c] <- mkPortStallRecv_Multiplexed("L1_DCache_OutQ_" + n);
        fromICache[c] <- mkPortStallRecv_Multiplexed("L1_ICache_OutQ_" + n);
        toMemory[c] <- mkPortStallSend_Multiplexed(toMemoryName + "_" + n);
    end

    for (Integer c = 0; c < valueOf(n_FROM_MEM_CHANNELS); c = c + 1)
    begin
        let n = integerToString(c);
        toDCache[c] <- mkPortStallSend_Multiplexed("L1_DCache_InQ_" + n);
        toICache[c] <- mkPortStallSend_Multiplexed("L1_ICache_InQ_" + n);
        fromMemory[c] <- mkPortStallRecv_Multiplexed(fromMemoryName + "_" + n);
    end

    
    // ******* Local Controller *******
    
    Vector#(TAdd#(TMul#(3, n_TO_MEM_CHANNELS), TMul#(3, n_FROM_MEM_CHANNELS)),
            INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls = newVector();
    Vector#(TAdd#(TMul#(3, n_TO_MEM_CHANNELS), TMul#(3, n_FROM_MEM_CHANNELS)),
            INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    Integer c_idx = 0;

    for (Integer c = 0; c < valueOf(n_TO_MEM_CHANNELS); c = c + 1)
    begin
        inctrls[c_idx] = fromDCache[c].ctrl.in;
        inctrls[c_idx + 1] = fromICache[c].ctrl.in;
        inctrls[c_idx + 2] = toMemory[c].ctrl.in;

        outctrls[c_idx] = fromDCache[c].ctrl.out;
        outctrls[c_idx + 1] = fromICache[c].ctrl.out;
        outctrls[c_idx + 2] = toMemory[c].ctrl.out;

        c_idx = c_idx + 3;
    end

    for (Integer c = 0; c < valueOf(n_FROM_MEM_CHANNELS); c = c + 1)
    begin
        inctrls[c_idx] = toDCache[c].ctrl.in;
        inctrls[c_idx + 1] = toICache[c].ctrl.in;
        inctrls[c_idx + 2] = fromMemory[c].ctrl.in;

        outctrls[c_idx] = toDCache[c].ctrl.out;
        outctrls[c_idx + 1] = toICache[c].ctrl.out;
        outctrls[c_idx + 2] = fromMemory[c].ctrl.out;

        c_idx = c_idx + 3;
    end

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L1 Cache Arbiter", inctrls, outctrls);
    

    //
    // Tagging and lookup of the opaque value to route messages from memory
    // toward the L1.  The top two bits of opaque are used.  If MSB is clear
    // then the message should be forwarded to both the I- and the D-caches.
    // This is to make it easy for a lower level cache to forward a writeback
    // or invalidation request to both halves of the L1 cache.  An opaque
    // value of 0 is routed to both.  If the MSB is set then the next bit is
    // examined.  A value of 1 is routed to the I-cache and 0 to the D-cache.
    //

    function Bool forBoth(MEM_OPAQUE opaque);
        return ! unpack(msb(opaque));
    endfunction

    function Bool forIStream(MEM_OPAQUE opaque);
        return ! forBoth(opaque) && unpack(opaque[valueOf(MEM_OPAQUE_SIZE) - 2]);
    endfunction

    function Bool forDStream(MEM_OPAQUE opaque);
        return ! forBoth(opaque) && ! unpack(opaque[valueOf(MEM_OPAQUE_SIZE) - 2]);
    endfunction

    function MEM_OPAQUE setForIStream(MEM_OPAQUE opaque);
        let new_opaque = opaque;
        new_opaque[valueOf(MEM_OPAQUE_SIZE) - 1] = 1;
        new_opaque[valueOf(MEM_OPAQUE_SIZE) - 2] = 1;
        return new_opaque;
    endfunction


    function MEM_OPAQUE setForDStream(MEM_OPAQUE opaque);
        let new_opaque = opaque;
        new_opaque[valueOf(MEM_OPAQUE_SIZE) - 1] = 1;
        new_opaque[valueOf(MEM_OPAQUE_SIZE) - 2] = 0;
        return new_opaque;
    endfunction


    (* conservative_implicit_conditions *)
    rule stage1_arbitrate (True);
        // Get the next instance to simulate.
        let cpu_iid <- localCtrl.startModelCycle();
    
        // ================================================================
        //
        // Route responses from memory to the right L1 cache.
        //
        // ================================================================

        for (Integer c = 0; c < valueOf(n_FROM_MEM_CHANNELS); c = c + 1)
        begin
            let memory_out <- fromMemory[c].receive(cpu_iid);
            let icache_has_room <- toICache[c].canEnq(cpu_iid);
            let dcache_has_room <- toDCache[c].canEnq(cpu_iid);
        
            if (memory_out matches tagged Valid .memory_msg)
            begin
                // There's a response. Where should it go?
                if (forBoth(memory_msg.opaque))
                begin
                    if (icache_has_room && dcache_has_room)
                    begin
                        debugLog.record(cpu_iid, $format("TO BOTH[%0d]: ", c) + fshow(memory_msg));
                        toICache[c].doEnq(cpu_iid, memory_msg);
                        toDCache[c].doEnq(cpu_iid, memory_msg);
                        fromMemory[c].doDeq(cpu_iid);
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("TO BOTH[%0d] RETRY", c));
                        toDCache[c].noEnq(cpu_iid);
                        toICache[c].noEnq(cpu_iid);
                        fromMemory[c].noDeq(cpu_iid);
                    end
                end
                else if (forIStream(memory_msg.opaque))
                begin
                    // It's going to the ICacheQ, if it has room.
                    if (icache_has_room)
                    begin
                        debugLog.record(cpu_iid, $format("TO ICACHE[%0d]: ", c) + fshow(memory_msg));
                        toICache[c].doEnq(cpu_iid, memory_msg);
                        toDCache[c].noEnq(cpu_iid);
                        fromMemory[c].doDeq(cpu_iid);
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("TO ICACHE[%0d] RETRY", c));
                        // No room, so just leave it.
                        toICache[c].noEnq(cpu_iid);
                        toDCache[c].noEnq(cpu_iid);
                        fromMemory[c].noDeq(cpu_iid);
                    end
                end
                else
                begin
                    // It's going to the DCacheQ, if it has room.
                    if (dcache_has_room)
                    begin
                        debugLog.record(cpu_iid, $format("TO DCACHE[%0d]: ", c) + fshow(memory_msg));
                        toDCache[c].doEnq(cpu_iid, memory_msg);
                        toICache[c].noEnq(cpu_iid);
                        fromMemory[c].doDeq(cpu_iid);
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("TO DCACHE[%0d] RETRY", c));
                        // No room, so just leave it.
                        toDCache[c].noEnq(cpu_iid);
                        toICache[c].noEnq(cpu_iid);
                        fromMemory[c].noDeq(cpu_iid);
                    end
                end
            end
            else
            begin
                debugLog.record(cpu_iid, $format("NO TO L1[%0d]", c));
                // No response to route.
                toDCache[c].noEnq(cpu_iid);
                toICache[c].noEnq(cpu_iid);
                fromMemory[c].noDeq(cpu_iid);
            end
        end
    
    
        // ================================================================
        //
        // Route messages from L1 caches to memory.
        //
        // ================================================================
    
        for (Integer c = 0; c < valueOf(n_TO_MEM_CHANNELS); c = c + 1)
        begin
            let memory_has_room <- toMemory[c].canEnq(cpu_iid);

            let dcache_out <- fromDCache[c].receive(cpu_iid);
            let icache_out <- fromICache[c].receive(cpu_iid);

            let dcache_valid = isValid(dcache_out);
            let icache_valid = isValid(icache_out);

            if (! memory_has_room)
            begin
                // Stall
                debugLog.record(cpu_iid, $format("MEM FULL [%0d] STALL", c));
                fromDCache[c].noDeq(cpu_iid);
                fromICache[c].noDeq(cpu_iid);
                toMemory[c].noEnq(cpu_iid);
            end
            else
            begin
                // Arbitrate based on favorite.
                if (icache_out matches tagged Valid .icache_value)
                begin
                    if (dcache_out matches tagged Valid .dcache_value)
                    begin
                        // They both want it. Resolve based on dynamic parameter.
                        if (favorICache())
                        begin
                            let final_value = icache_value;
                            final_value.opaque = setForIStream(final_value.opaque);

                            debugLog.record(cpu_iid, $format("FROM ICACHE & DCACHE: ICACHE[%0d] GRANT: ", c) + fshow(final_value));

                            toMemory[c].doEnq(cpu_iid, final_value);
                            fromICache[c].doDeq(cpu_iid);
                            fromDCache[c].noDeq(cpu_iid);
                        end
                        else
                        begin
                            let final_value = dcache_value;
                            final_value.opaque = setForDStream(final_value.opaque);

                            debugLog.record(cpu_iid, $format("FROM ICACHE & DCACHE: DCACHE[%0d] GRANT:", c) + fshow(final_value));

                            toMemory[c].doEnq(cpu_iid, final_value);
                            fromDCache[c].doDeq(cpu_iid);
                            fromICache[c].noDeq(cpu_iid);
                        end
                    end
                    else
                    begin
                        // Only the ICache wants it, so they get it.
                        let final_value = icache_value;
                        final_value.opaque = setForIStream(final_value.opaque);

                        debugLog.record(cpu_iid, $format("FROM ICACHE[%0d]: ", c) + fshow(final_value));

                        toMemory[c].doEnq(cpu_iid, final_value);
                        fromICache[c].doDeq(cpu_iid);
                        fromDCache[c].noDeq(cpu_iid);
                    end
                end
                else if (dcache_out matches tagged Valid .dcache_value)
                begin
                    // Only the DCache wants it, so they get it.
                    let final_value = dcache_value;
                    final_value.opaque = setForDStream(final_value.opaque);

                    debugLog.record(cpu_iid, $format("FROM DCACHE[%0d]: ", c) + fshow(final_value));

                    toMemory[c].doEnq(cpu_iid, final_value);
                    fromDCache[c].doDeq(cpu_iid);
                    fromICache[c].noDeq(cpu_iid);
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("NO FROM L1[%0d]", c));
                    // Neither want it.
                    toMemory[c].noEnq(cpu_iid);
                    fromDCache[c].noDeq(cpu_iid);
                    fromICache[c].noDeq(cpu_iid);
                end
            end
        end

        localCtrl.endModelCycle(cpu_iid, 1);
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
