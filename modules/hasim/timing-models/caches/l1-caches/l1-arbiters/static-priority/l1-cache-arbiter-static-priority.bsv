import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"
`include "asim/provides/hasim_memory.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/l1_cache_base_types.bsh"

module [HASIM_MODULE] mkL1CacheArbiter ();

    // Eventually this could be a dynamic parameter.
    Bool favorICache = `L1_ARBITER_FAVOR_ICACHE;

    // Queues to/from DCache
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqFromDCache <- mkPortStallRecv_Multiplexed("L1_DCache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspToDCache   <- mkPortStallSend_Multiplexed("L1_DCache_InQ");

    // Queues to/from ICache
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqFromICache <- mkPortStallRecv_Multiplexed("L1_ICache_OutQ");
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspToICache   <- mkPortStallSend_Multiplexed("L1_ICache_InQ");

    // Queues to/from Memory
    PORT_STALL_SEND_MULTIPLEXED#(NUM_CPUS, MEMORY_REQ) reqToMemory   <- mkPortStallSend_Multiplexed("L1_Cache_OutQ");
    PORT_STALL_RECV_MULTIPLEXED#(NUM_CPUS, MEMORY_RSP) rspFromMemory <- mkPortStallRecv_Multiplexed("L1_Cache_InQ");
    
    // ******* Local Controller *******
    
    Vector#(6, INSTANCE_CONTROL_IN#(NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(NUM_CPUS)) outctrls = newVector();
    
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

    LOCAL_CONTROLLER#(NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);
    
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
                
                    rspToICache.doEnq(cpu_iid, memory_rsp);
                    rspToDCache.noEnq(cpu_iid);
                    rspFromMemory.doDeq(cpu_iid);
                
                end
                else
                begin
                
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
                
                    rspToDCache.doEnq(cpu_iid, memory_rsp);
                    rspToICache.noEnq(cpu_iid);
                    rspFromMemory.doDeq(cpu_iid);
                
                end
                else
                begin
                
                    // No room, so just leave it.
                    rspToDCache.noEnq(cpu_iid);
                    rspToICache.noEnq(cpu_iid);
                    rspFromMemory.noDeq(cpu_iid);
                
                end
            
            end
        
        end
        else
        begin
        
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
            reqFromDCache.noDeq(cpu_iid);
            reqFromICache.noDeq(cpu_iid);

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

                        let final_value = icache_value;
                        final_value.opaque = setRspForIStream(final_value.opaque);

                        reqToMemory.doEnq(cpu_iid, final_value);
                        reqFromICache.doDeq(cpu_iid);
                        reqFromDCache.noDeq(cpu_iid);

                    end
                    else
                    begin

                        let final_value = dcache_value;
                        final_value.opaque = setRspForDStream(final_value.opaque);

                        reqToMemory.doEnq(cpu_iid, final_value);
                        reqFromDCache.doDeq(cpu_iid);
                        reqFromICache.noDeq(cpu_iid);

                    end

                end
                else
                begin
                
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
            
                // Only the DCache wants it, so they get it.
                let final_value = dcache_value;
                final_value.opaque = setRspForDStream(final_value.opaque);

                reqToMemory.doEnq(cpu_iid, final_value);
                reqFromDCache.doDeq(cpu_iid);
                reqFromICache.noDeq(cpu_iid);
            
            end
            else
            begin
            
                // Neither want it.
                reqToMemory.noEnq(cpu_iid);
                reqFromDCache.noDeq(cpu_iid);
                reqFromICache.noDeq(cpu_iid);
            
            end
            
        end

        localCtrl.endModelCycle(cpu_iid, 1);
    
    endrule

endmodule
 
