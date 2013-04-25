import Vector::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/hasim_interconnect.bsh"

typedef enum
{
    LLC_CC_REQ_WB,
    LLC_CC_REQ_INVALIDATE
}
LLC_CC_REQ deriving (Eq, Bits);

typedef CORE_MEMORY_REQ LLC_CC_RSP;

typedef CORE_MEMORY_REQ CC_REQ;
typedef CORE_MEMORY_RSP CC_RSP;

module [HASIM_MODULE] mkLastLevelCache();

    // Make an interface to the cache coherence protocol.
    let ccifc <- mkCacheCoherenceInterface();

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_REQ) reqFromCore <- mkPortStallRecv_Multiplexed("CorePvtCache_to_UncoreQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, MEMORY_RSP) rspToCore   <- mkPortStallSend_Multiplexed("Uncore_to_CorePvtCacheQ");
    
    // Queues to/from coherence engine.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, LLC_CC_REQ) reqFromCC <- mkPortStallRecv_Multiplexed("CC_to_LLC_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CC_REQ)     reqToCC   <- mkPortStallSend_Multiplexed("LLC_to_CC_req");
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CC_RSP)     rspFromCC <- mkPortStallRecv_Multiplexed("CC_to_LLC_rsp");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, LLC_CC_RSP) rspToCC   <- mkPortStallSend_Multiplexed("LLC_to_CC_rsp");
    
    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    
    inctrls[0]  = reqFromCore.ctrl.in;
    inctrls[1]  = rspToCore.ctrl.in;
    inctrls[2]  = reqFromCC.ctrl.in;
    inctrls[3]  = reqToCC.ctrl.in;
    inctrls[4]  = rspFromCC.ctrl.in;
    inctrls[5]  = rspToCC.ctrl.in;
    outctrls[0]  = reqFromCore.ctrl.out;
    outctrls[1]  = rspToCore.ctrl.out;
    outctrls[2]  = reqFromCC.ctrl.out;
    outctrls[3]  = reqToCC.ctrl.out;
    outctrls[4]  = rspFromCC.ctrl.out;
    outctrls[5]  = rspToCC.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    
    (* conservative_implicit_conditions *)
    rule stage1_coreReq (True);
    
        let iid <- localCtrl.startModelCycle();
    
        // Start by checking for new Loads/Stores from the core.
        let m_core_req <- reqFromCore.receive(iid);
        
        let can_enq_cc_req <- reqToCC.canEnq(iid);
        let can_enq_cc_rsp <- rspToCC.canEnq(iid);

        if (m_core_req matches tagged Valid .req &&& can_enq_cc_req)
        begin
        
            //let cc_req = (req.isStore) ? toExcReq(req) : toShrReq(req);

            reqFromCore.doDeq(iid);
            reqToCC.doEnq(iid, req);

        end
        else
        begin

            reqFromCore.noDeq(iid);
            reqToCC.noEnq(iid);

        end
        
        // Unused by Null LLC. Should be handling invalidation writebacks.
        rspToCC.noEnq(iid);

        // Now check for responses from the cache coherence engine.
        let m_cc_rsp <- rspFromCC.receive(iid);

        // Also check for new requests from the cache coherence engine.
        let m_cc_req <- reqFromCC.receive(iid);

        let can_enq_core_rsp <- rspToCore.canEnq(iid);

        // To avoid deadlock, responses are higher priority than new requests.
        if (m_cc_req matches tagged Valid .req &&& can_enq_core_rsp)
        begin
        
            reqFromCC.doDeq(iid);
            rspFromCC.noDeq(iid);
            
            // Null LLC drops invalidates at this point. 
            // TODO: They should be passed on to L1C via SEPARATE fifos.
            rspToCore.noEnq(iid);

        end
        else if (m_cc_rsp matches tagged Valid .rsp &&& can_enq_core_rsp)
        begin

            reqFromCC.noDeq(iid);
            rspFromCC.doDeq(iid);
            let ocn_rsp = initMemRsp(rsp.physicalAddress, rsp.opaque);
            rspToCore.doEnq(iid, ocn_rsp);

        end
        else
        begin

            reqFromCC.noDeq(iid);
            rspFromCC.noDeq(iid);
            rspToCore.noEnq(iid);

        end
        
        
        localCtrl.endModelCycle(iid, 1);

    endrule

endmodule

`define LANE_LLC_REQ 0
`define LANE_LLC_RSP 1

module [HASIM_MODULE] mkCacheCoherenceInterface();

    // Queues to/from last level cache.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, LLC_CC_REQ) reqToLLC   <- mkPortStallSend_Multiplexed("CC_to_LLC_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CC_REQ)     reqFromLLC <- mkPortStallRecv_Multiplexed("LLC_to_CC_req");
    
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, LLC_CC_RSP) rspFromLLC <- mkPortStallRecv_Multiplexed("LLC_to_CC_rsp");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CC_RSP)     rspToLLC   <- mkPortStallSend_Multiplexed("CC_to_LLC_rsp");
    
    // Interface to OCN looks like lanes and virtual channels.   
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqFromOCN    <- mkPortRecv_Multiplexed("CoreMemInQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqToOCN      <- mkPortSend_Multiplexed("CoreMemOutQ_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditFromOCN <- mkPortRecv_Multiplexed("CoreMemInQ_credit", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditToOCN   <- mkPortSend_Multiplexed("CoreMemOutQ_credit");

    Vector#(6, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    Vector#(6, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();

    inctrls[0]  = reqToLLC.ctrl.in;
    inctrls[1]  = reqFromLLC.ctrl.in;
    inctrls[2]  = rspFromLLC.ctrl.in;
    inctrls[3]  = rspToLLC.ctrl.in;
    inctrls[4]  = enqFromOCN.ctrl;
    inctrls[5]  = creditFromOCN.ctrl;
    outctrls[0]  = reqToLLC.ctrl.out;
    outctrls[1]  = reqFromLLC.ctrl.out;
    outctrls[2]  = rspFromLLC.ctrl.out;
    outctrls[3]  = rspToLLC.ctrl.out;
    outctrls[4]  = enqToOCN.ctrl;
    outctrls[5]  = creditToOCN.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);
    STAGE_CONTROLLER_VOID#(MAX_NUM_CPUS) stage2Ctrl <- mkStageControllerVoid();

    MULTIPLEXED_REG#(MAX_NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCreditsPool  <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputNotFullsPool <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRspPool <- mkMultiplexedReg(tagged Invalid);
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReqPool <- mkMultiplexedReg(tagged Invalid);
    MULTIPLEXED_LUTRAM#(MAX_NUM_CPUS, MEM_OPAQUE, LINE_ADDRESS)         physAddrPool       <- mkMultiplexedLUTRAM(~0);

    function Maybe#(VC_IDX) vcToEnq(INSTANCE_ID#(MAX_NUM_CPUS) iid, LANE_IDX ln);
    
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(iid);
        Maybe#(VC_IDX) res = tagged Invalid;
        
        for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
        begin
            res = (notFulls[ln][vc]) ? tagged Valid fromInteger(vc) : res;
        end
        
        return res;
    
    endfunction
    
    function STATION_ID getDst(LINE_ADDRESS addr);
    
        // TODO: have home nodes for caches?
        // For now just send everything to the memory controller.
        return 0; // XXX
    
    endfunction

    (* conservative_implicit_conditions *)
    rule stage1_updateCredits (True);

        let iid <- localCtrl.startModelCycle();
        let m_credit <- creditFromOCN.receive(iid);
        
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls = outputNotFullsPool.getReg(iid);
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCredits = outputCreditsPool.getReg(iid);
        
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_not_fulls = notFulls;
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_credits   = outputCredits;
        
        if (m_credit matches tagged Valid .creds)
        begin
        
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                
                    match {.credit, .not_full} = creds[ln][vc];
                    new_not_fulls[ln][vc] = not_full;
                    new_credits[ln][vc] = credit;
                
                end
                
            end
        
        end
        
        notFulls <= new_not_fulls;
        outputCredits <= new_credits;
        
        stage2Ctrl.ready(iid);
    
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage2_LLCReq (True);
    
        let iid <- stage2Ctrl.nextReadyInstance();

        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingRsp = packetizingRspPool.getReg(iid);
        Reg#(Maybe#(Tuple2#(MEM_OPAQUE, VC_IDX))) packetizingReq = packetizingReqPool.getReg(iid);
        LUTRAM#(MEM_OPAQUE, LINE_ADDRESS)         physAddr       = physAddrPool.getRAM(iid);

        // Start by checking for new responses from the LLC.
        // These are higher priority.
        let m_llc_rsp <- rspFromLLC.receive(iid);

        // Also check for new ShReq/ExcReq from the LLC.
        let m_llc_req <- reqFromLLC.receive(iid);

        if (packetizingRsp matches tagged Valid {.op, .vc_idx})
        begin
        
            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Invalid;
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (packetizingReq matches tagged Valid {.op, .vc_idx})
        begin

            let msg = tagged FLIT_BODY {opaque: op, isTail: True};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            packetizingReq <= tagged Invalid;
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (m_llc_rsp matches tagged Valid .rsp &&& vcToEnq(iid, `LANE_LLC_RSP) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(iid), dst: getDst(rsp.physicalAddress), isStore: False};
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_RSP, vc_idx, msg));
            packetizingRsp <= tagged Valid tuple2(rsp.opaque, vc_idx);
            rspFromLLC.doDeq(iid);
            reqFromLLC.noDeq(iid);

        end
        else if (m_llc_req matches tagged Valid .req &&& vcToEnq(iid, `LANE_LLC_REQ) matches tagged Valid .vc_idx)
        begin
        
            let msg = tagged FLIT_HEAD {src: zeroExtend(iid), dst: getDst(req.physicalAddress), isStore: req.isStore};
            packetizingReq <= tagged Valid tuple2(req.opaque, vc_idx);
            enqToOCN.send(iid, tagged Valid tuple3(`LANE_LLC_REQ, vc_idx, msg));
            rspFromLLC.noDeq(iid);
            reqFromLLC.doDeq(iid);
            physAddr.upd(req.opaque, req.physicalAddress);

        end
        else
        begin

            enqToOCN.send(iid, tagged Invalid);
            rspFromLLC.noDeq(iid);
            reqFromLLC.noDeq(iid);
        
        end
        
        // Route enqueues from the OCN to the correct place. This ignores virtual channels - just lanes.
        let m_enq <- enqFromOCN.receive(iid);

        let can_enq_req <- reqToLLC.canEnq(iid);
        let can_enq_rsp <- rspToLLC.canEnq(iid);
        
        if (m_enq matches tagged Valid {.ln, .vc_idx, .msg})
        begin
        
            if (ln == `LANE_LLC_REQ)
            begin

                // assert can_enq_req
                case (msg) matches
                    tagged FLIT_HEAD .info:
                    begin
                        // Drop heads at this point.
                        reqToLLC.noEnq(iid);
                        rspToLLC.noEnq(iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.doEnq(iid, LLC_CC_REQ_WB); //TODO: actually distinguish.
                       rspToLLC.noEnq(iid);
                    end
                endcase

            end
            else if (ln == `LANE_LLC_RSP)
            begin
            
                // assert can_enq_rsp
                case (msg) matches
                    tagged FLIT_HEAD .info:
                    begin
                        // Drop heads at this point.
                        reqToLLC.noEnq(iid);
                        rspToLLC.noEnq(iid);
                    end
                    tagged FLIT_BODY .info:
                    begin
                       reqToLLC.noEnq(iid);
                       rspToLLC.doEnq(iid, initMemRsp(physAddr.sub(info.opaque), info.opaque));
                    end

                endcase

            end
            else
            begin

                reqToLLC.noEnq(iid);
                rspToLLC.noEnq(iid);

            end

        end
        else
        begin
        
            reqToLLC.noEnq(iid);
            rspToLLC.noEnq(iid);
        
        end
        
        VC_CREDIT_INFO creds = newVector();
        creds[`LANE_LLC_REQ] = newVector();
        creds[`LANE_LLC_RSP] = newVector();

        for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
        begin

            let have_credit_req = can_enq_req; // XXX capacity - occupancy > round-trip latency.
            let not_full_req = can_enq_req;
            creds[`LANE_LLC_REQ][vc] = tuple2(have_credit_req, not_full_req);

            let have_credit_rsp = can_enq_rsp; // XXX capacity - occupancy > round-trip latency.
            let not_full_rsp = can_enq_rsp;
            creds[`LANE_LLC_RSP][vc] = tuple2(have_credit_rsp, not_full_rsp);

        end
        
        creditToOCN.send(iid, tagged Valid creds);
        localCtrl.endModelCycle(iid, 0);
        
    endrule

endmodule
