import Vector::*;

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

module [HASIM_MODULE] mkL1CacheArbiter#(String reqToMemoryName,
                                        String rspFromMemoryName)
    // Interface
    (Empty);

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_arbiter.out");

    // Eventually this could be a dynamic parameter.
    Bool favorICache = `L1_ARBITER_FAVOR_ICACHE;

    // Queues to/from DCache
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqFromDCache <-
        mkPortStallRecv_Multiplexed("L1_DCache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToDCache <-
        mkPortStallSend_Multiplexed("L1_DCache_InQ");

    // Queues to/from ICache
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqFromICache <-
        mkPortStallRecv_Multiplexed("L1_ICache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToICache <-
        mkPortStallSend_Multiplexed("L1_ICache_InQ");

    // Queues to/from Memory
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) reqToMemory <-
        mkPortStallSend_Multiplexed(reqToMemoryName);
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspFromMemory <-
        mkPortStallRecv_Multiplexed(rspFromMemoryName);
    
    // ******* Local Controller *******
    
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromDCache.ctrl.in;
    inctrls[1]  = rspToDCache.ctrl.in;
    inctrls[2]  = reqFromICache.ctrl.in;
    inctrls[3]  = rspToICache.ctrl.in;
    inctrls[4]  = reqToMemory.ctrl.in;
    inctrls[5]  = rspFromMemory.ctrl.in;
    outctrls[0]  = reqFromDCache.ctrl.out;
    outctrls[1]  = rspToDCache.ctrl.out;
    outctrls[2]  = reqFromICache.ctrl.out;
    outctrls[3]  = rspToICache.ctrl.out;
    outctrls[4]  = reqToMemory.ctrl.out;
    outctrls[5]  = rspFromMemory.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L1 Cache Arbiter", inctrls, outctrls);
    

    function Bool rspForIStream(MEM_OPAQUE opaque);
        return unpack(msb(opaque));
    endfunction


    function MEM_OPAQUE setRspForIStream(MEM_OPAQUE opaque);
        let new_opaque = opaque;
        new_opaque[valueof(MEM_OPAQUE_SIZE) - 1] = 1;
        return new_opaque;
    endfunction


    function MEM_OPAQUE setRspForDStream(MEM_OPAQUE opaque);
        let new_opaque = opaque;
        new_opaque[valueof(MEM_OPAQUE_SIZE) - 1] = 0;
        return new_opaque;
    endfunction


    rule stage1_arbitrate (True);
        // Get the next instance to simulate.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);
    
        // Route responses to the right place.
        let memory_out <- rspFromMemory.receive(cpu_iid);
        let icache_has_room <- rspToICache.canEnq(cpu_iid);
        let dcache_has_room <- rspToDCache.canEnq(cpu_iid);
        
        if (memory_out matches tagged Valid .memory_rsp)
        begin
            // There's a response. Where should it go?
            if (rspForIStream(memory_rsp.opaque))
            begin
                // It's going to the ICacheQ, if it has room.
                if (icache_has_room)
                begin
                    debugLog.record_next_cycle(cpu_iid, $format("RSP TO ICACHE"));
                    rspToICache.doEnq(cpu_iid, memory_rsp);
                    rspToDCache.noEnq(cpu_iid);
                    rspFromMemory.doDeq(cpu_iid);
                end
                else
                begin
                    debugLog.record_next_cycle(cpu_iid, $format("RSP TO ICACHE RETRY"));
                    // No room, so just leave it.
                    rspToICache.noEnq(cpu_iid);
                    rspToDCache.noEnq(cpu_iid);
                    rspFromMemory.noDeq(cpu_iid);
                end
            end
            else
            begin
                // It's going to the DCacheQ, if it has room.
                if (dcache_has_room)
                begin
                    debugLog.record_next_cycle(cpu_iid, $format("RSP TO DCACHE"));
                    rspToDCache.doEnq(cpu_iid, memory_rsp);
                    rspToICache.noEnq(cpu_iid);
                    rspFromMemory.doDeq(cpu_iid);
                end
                else
                begin
                    debugLog.record_next_cycle(cpu_iid, $format("RSP TO DCACHE RETRY"));
                    // No room, so just leave it.
                    rspToDCache.noEnq(cpu_iid);
                    rspToICache.noEnq(cpu_iid);
                    rspFromMemory.noDeq(cpu_iid);
                end
            end
        end
        else
        begin
            debugLog.record_next_cycle(cpu_iid, $format("NO RSP"));
            // No response to route.
            rspToDCache.noEnq(cpu_iid);
            rspToICache.noEnq(cpu_iid);
            rspFromMemory.noDeq(cpu_iid);
        end
    
        // Arbitrate the outgoing queue.
    
        let memory_has_room <- reqToMemory.canEnq(cpu_iid);

        let dcache_out <- reqFromDCache.receive(cpu_iid);
        let icache_out <- reqFromICache.receive(cpu_iid);
        
        let dcache_valid = isValid(dcache_out);
        let icache_valid = isValid(icache_out);
        
        if (!memory_has_room)
        begin
            // Stall
            debugLog.record_next_cycle(cpu_iid, $format("REQ STALL"));
            reqFromDCache.noDeq(cpu_iid);
            reqFromICache.noDeq(cpu_iid);
            reqToMemory.noEnq(cpu_iid);
        end
        else
        begin
            // Arbitrate based on favorite.
            if (icache_out matches tagged Valid .icache_value)
            begin
                if (dcache_out matches tagged Valid .dcache_value)
                begin
                    // The both want it. Resolve based on dynamic parameter.
                    if (favorICache())
                    begin
                        debugLog.record_next_cycle(cpu_iid, $format("REQ FROM ICACHE & DCACHE: ICACHE GRANT"));
                        let final_value = icache_value;
                        final_value.opaque = setRspForIStream(final_value.opaque);

                        reqToMemory.doEnq(cpu_iid, final_value);
                        reqFromICache.doDeq(cpu_iid);
                        reqFromDCache.noDeq(cpu_iid);
                    end
                    else
                    begin
                        debugLog.record_next_cycle(cpu_iid, $format("REQ FROM ICACHE & DCACHE: DCACHE GRANT"));

                        let final_value = dcache_value;
                        final_value.opaque = setRspForDStream(final_value.opaque);

                        reqToMemory.doEnq(cpu_iid, final_value);
                        reqFromDCache.doDeq(cpu_iid);
                        reqFromICache.noDeq(cpu_iid);
                    end
                end
                else
                begin
                    debugLog.record_next_cycle(cpu_iid, $format("REQ FROM ICACHE"));

                    // Only the ICache wants it, so they get it.
                    let final_value = icache_value;
                    final_value.opaque = setRspForIStream(final_value.opaque);

                    reqToMemory.doEnq(cpu_iid, final_value);
                    reqFromICache.doDeq(cpu_iid);
                    reqFromDCache.noDeq(cpu_iid);
                end
            end
            else if (dcache_out matches tagged Valid .dcache_value)
            begin
                debugLog.record_next_cycle(cpu_iid, $format("REQ FROM DCACHE"));

                // Only the DCache wants it, so they get it.
                let final_value = dcache_value;
                final_value.opaque = setRspForDStream(final_value.opaque);

                reqToMemory.doEnq(cpu_iid, final_value);
                reqFromDCache.doDeq(cpu_iid);
                reqFromICache.noDeq(cpu_iid);
            end
            else
            begin
                debugLog.record_next_cycle(cpu_iid, $format("NO REQ"));
                // Neither want it.
                reqToMemory.noEnq(cpu_iid);
                reqFromDCache.noDeq(cpu_iid);
                reqFromICache.noDeq(cpu_iid);
            end
        end

        localCtrl.endModelCycle(cpu_iid, 1);
    endrule

endmodule
