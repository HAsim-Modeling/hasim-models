// TODO: Evaluate, possibly switch to BlockRAM.

import Vector::*;
import FIFO::*;
import FIFOF::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/common_services.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/hasim_memory_controller.bsh"

typedef STATION_IID RING_COORD; // Since coordinates can vary dynamically, we need to be able to hold the worst case in each direction, which is a ring network.
typedef TLog#(NUM_STATIONS) RING_COORD_SZ;

typedef OCN_FLIT RING_FLIT;
typedef OCN_MSG  RING_MSG;

typedef 3 NUM_PORTS;
typedef Bit#(TLog#(NUM_PORTS)) PORT_IDX;

PORT_IDX portEast   = 0;
PORT_IDX portWest   = 1;
PORT_IDX portLocal  = 2;

Integer numPorts = 3;

function String portShow(PORT_IDX p);

    return case (p)
        0: "east";
        1: "west";
        2: "local";
        default: "UNKNOWN";
    endcase;

endfunction

typedef Vector#(NUM_PORTS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA))) VC_STATE#(parameter type t_DATA);


typedef struct
{
    LANE_IDX lane;
    VC_IDX   inputVC;
    PORT_IDX outputPort;
    VC_IDX   outputVC;
    OCN_FLIT message;
}
WINNER_INFO 
    deriving (Eq, Bits);
    
`define MEM_CTRL_LOCATION 0

module [HASIM_MODULE] mkInterconnect
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_STATIONS) debugLog <- mkTIMEPDebugFile_Multiplexed("interconnect_ring.out");

    // ******** Ports *******

    // Queues to/from cores
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

    // Queues to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_RECV#(OCN_MSG)        enqFromMemCtrl    <- mkPortRecv("memctrl_to_ocn_enq", 1);
    PORT_SEND#(OCN_MSG)        enqToMemCtrl      <- mkPortSend("ocn_to_memctrl_enq");
    PORT_RECV#(VC_CREDIT_INFO) creditFromMemCtrl <- mkPortRecv("memctrl_to_ocn_credit", 1);
    PORT_SEND#(VC_CREDIT_INFO) creditToMemCtrl   <- mkPortSend("ocn_to_memctrl_credit");

    // Links to/from neighboring routers
    // Note: These ports actually connect together (they're the same port).
    // This is the main technique which makes this module work.
    // The token reordering keeps things in the correct order.
    // Note: We need an extra instance here for the memory controller's router.
    // Note: We have to control these ourselves since they have more instances than normal.
    
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, RING_MSG))       enqTo      = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, RING_MSG))       enqFrom    = newVector();
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO)) creditTo   = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO)) creditFrom = newVector();

    enqTo[portEast]       <- mkPortSend_Multiplexed("ring_interconnect_enq_E");
    enqFrom[portWest]     <- mkPortRecv_Multiplexed_ReorderLastToFirst("ring_interconnect_enq_E", 1);

    enqTo[portWest]       <- mkPortSend_Multiplexed("ring_interconnect_enq_W");
    enqFrom[portEast]     <- mkPortRecv_Multiplexed_ReorderFirstToLast("ring_interconnect_enq_W", 1);

    enqTo[portLocal]      <- mkPortSend_Multiplexed_Split(enqToCores, enqToMemCtrl, `MEM_CTRL_LOCATION);
    enqFrom[portLocal]    <- mkPortRecv_Multiplexed_Join(enqFromCores, enqFromMemCtrl, `MEM_CTRL_LOCATION);
    
    creditTo[portEast]    <- mkPortSend_Multiplexed("ring_interconnect_credit_E");
    creditFrom[portWest]  <- mkPortRecv_Multiplexed_ReorderLastToFirst("ring_interconnect_credit_E", 1);

    creditTo[portWest]    <- mkPortSend_Multiplexed("ring_interconnect_credit_W");
    creditFrom[portEast]  <- mkPortRecv_Multiplexed_ReorderFirstToLast("ring_interconnect_credit_W", 1);

    creditTo[portLocal]   <- mkPortSend_Multiplexed_Split(creditToCores, creditToMemCtrl, `MEM_CTRL_LOCATION);
    creditFrom[portLocal] <- mkPortRecv_Multiplexed_Join(creditFromCores, creditFromMemCtrl, `MEM_CTRL_LOCATION);

    // NOTE: The module does not use a local controller, as it has two sets of ports,
    // one set is MAX_NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    inports[0] = enqFromCores.ctrl;
    inports[1] = creditFromCores.ctrl;

    Vector#(6, INSTANCE_CONTROL_IN#(NUM_STATIONS)) inportsR = newVector();
    inportsR[0] = enqFrom[portEast].ctrl;
    inportsR[1] = enqFrom[portWest].ctrl;
    inportsR[2] = enqFrom[portLocal].ctrl;
    inportsR[3] = creditFrom[portEast].ctrl;
    inportsR[4] = creditFrom[portWest].ctrl;
    inportsR[5] = creditFrom[portLocal].ctrl;

    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_STATIONS)) outportsR = newVector();
    outportsR[0] = enqTo[portLocal].ctrl;
    outportsR[1] = creditTo[portLocal].ctrl;

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <- mkLocalControllerPlusN(inports, inportsR, outportsR);
    
    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    // The actual virtual channels. Currently actual FIFOs, but could be implemented
    // in BlockRAM or whatever.
    MULTIPLEXED#(NUM_STATIONS, VC_STATE#(FIFOF#(RING_FLIT)))    virtualChannelsPool    <- mkMultiplexed(replicateM(replicateM(replicateM(mkUGSizedFIFOF(4)))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(PORT_IDX))) routesPool             <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(VC_IDX)))   outputVCsPool          <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             usedVCsPool            <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputCreditsPool      <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputNotFullsPool     <- mkMultiplexedReg(replicate(replicate(replicate(False))));

    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER#(NUM_STATIONS, Vector#(NUM_PORTS, Maybe#(WINNER_INFO))) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage4Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage5Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage6Ctrl <- mkStageControllerVoid();

    // ******** Helper Functions *********
    
    // Calculate the 2D position for each router.
    Reg#(Vector#(NUM_STATIONS, RING_COORD)) routerRowPosition <- mkRegU();
    Reg#(Vector#(NUM_STATIONS, RING_COORD)) routerColPosition <- mkRegU();
    
    COUNTER#(RING_COORD_SZ) curInitID  <- mkLCounter(0);
    COUNTER#(RING_COORD_SZ) curInitRow <- mkLCounter(0);
    COUNTER#(RING_COORD_SZ) curInitCol <- mkLCounter(0);
        
    function PORT_IDX route(STATION_ID my_id, STATION_ID dst);

        STATION_ID dst_minus_src = dst - my_id;
        STATION_ID src_minus_dst = my_id - dst;
        STATION_ID n = fromInteger(valueof(NUM_STATIONS) - 1) + 1; // This looks silly, but works for powers of 2.
        let should_route_east = (dst > my_id) ? dst_minus_src < (src_minus_dst + n) : (dst_minus_src + n) < src_minus_dst;

        return (dst == my_id) ? portLocal : 
                 (should_route_east) ? portEast : portWest;
        
    endfunction


    // ******* Rules *******

    rule stage1_updateCreditsIn (True);
    
        // Get the next IID to simulate.
        let iid <- localCtrl.startModelCycle();
        
        // Get our state from the pools.
        Reg#(VC_STATE#(Bool)) outputCredits  = outputCreditsPool.getReg(iid);
        Reg#(VC_STATE#(Bool)) outputNotFulls = outputNotFullsPool.getReg(iid);
        
        // Update our notions of our neighbor's credits.
        VC_STATE#(Bool) new_credits = outputCredits;
        VC_STATE#(Bool) new_not_fulls = outputNotFulls;
        
        for (Integer p = 0 ; p < numPorts; p = p + 1)
        begin

            // Get the credits for this neighbor.
            let m_credits <- creditFrom[p].receive(iid);

            if (m_credits matches tagged Valid .vcinfo)
            begin
                
                // New credit info has arrived.
                for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
                begin

                    for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                    begin

                        match {.cred, .not_full} = vcinfo[ln][vc];
                        new_credits[p][ln][vc] = cred;
                        new_not_fulls[p][ln][vc] = not_full;

                    end

                end

            end
  
        end
        
        debugLog.record(iid, $format("1: Update input credits"));
        
        // Do the actual update.
        outputCredits <= new_credits;
        outputNotFulls <= new_not_fulls;
        
        // Move on to the next stage.
        stage2Ctrl.ready(iid);
    
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage2_multiplexVCs (True);
        
        // Get the info from the previous stage.
        let iid <- stage2Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(FIFOF#(RING_FLIT)) virtualChannels = virtualChannelsPool[iid];
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes         = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs      = outputVCsPool.getReg(iid);

        // This simulates the fact that the router only has one VC allocator.
        Bool vc_alloc_in_use = False;
        
        // This simulates the fact that only one VC from each port gets to even ATTEMPT
        // to send a message on the crossbar.
        Vector#(NUM_PORTS, Maybe#(WINNER_INFO)) vc_winners = replicate(tagged Invalid);
        
        debugLog.record(iid, $format("2: VCA Begin."));
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin

            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin

                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin

                    if (virtualChannels[p][ln][vc].notEmpty())
                    begin

                        if (routes[p][ln][vc] matches tagged Valid .rt)
                        begin

                            if (outputVCs[p][ln][vc] matches tagged Valid .out_vc)
                            begin

                                debugLog.record(iid, $format("2: VCA: Multiplexor for port %s chose lane %0d, virtual channel %0d, destination %s.", portShow(fromInteger(p)), ln, vc, portShow(rt)));
                                vc_winners[p] = tagged Valid WINNER_INFO 
                                                            {
                                                                lane: fromInteger(ln),
                                                                inputVC: fromInteger(vc), 
                                                                outputPort: rt, 
                                                                outputVC: out_vc, 
                                                                message: virtualChannels[p][ln][vc].first()
                                                            };
                            end
                        end
                    end
                end
            end
        end

        stage3Ctrl.ready(iid, vc_winners);

    endrule

    (* conservative_implicit_conditions *)
    rule stage3_crossbar (True);
        
        // Get the info from the previous stage.
        match {.iid, .vc_winners} <- stage3Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(FIFOF#(RING_FLIT))      virtualChannels = virtualChannelsPool[iid];
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes          = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs       = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             usedVCs         = usedVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputCredits   = outputCreditsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputNotFulls  = outputNotFullsPool.getReg(iid);

        // This is the vector of output messages that the virtual channels contend for.
        Vector#(NUM_PORTS, Maybe#(RING_MSG)) msg_to = replicate(tagged Invalid);
        
        // Vectors to update our registers with.
        VC_STATE#(Maybe#(PORT_IDX))   new_routes = routes;
        VC_STATE#(Bool)             new_used_vcs = usedVCs;
        VC_STATE#(Maybe#(VC_IDX)) new_output_vcs = outputVCs;

        debugLog.record(iid, $format("3: SA Begin."));

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin

            if (vc_winners[p] matches tagged Valid .info)
            begin

                if (!isValid(msg_to[info.outputPort]) && outputNotFulls[info.outputPort][info.lane][info.outputVC])
                begin

                    debugLog.record(iid, $format("3: SA: Gave crossbar %s output port to %s input port, lane %0d, virtual channel %0d", portShow(info.outputPort), portShow(fromInteger(p)), info.lane, info.inputVC));
                    msg_to[info.outputPort] = tagged Valid tuple3(info.lane, info.outputVC, info.message);
                    virtualChannels[p][info.lane][info.inputVC].deq();

                    if (info.message matches tagged FLIT_BODY .body_info &&& body_info.isTail)
                    begin

                        debugLog.record(iid, $format("3: SA: Detected tail flit. Tearing down routing info."));
                        new_routes[p][info.lane][info.inputVC] = tagged Invalid;
                        new_output_vcs[p][info.lane][info.inputVC] = tagged Invalid;
                        new_used_vcs[info.outputPort][info.lane][info.outputVC] = False;

                    end

                end

            end

        end

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin

            // Send out our output enqueues in each direction.
            enqTo[p].send(iid, msg_to[p]);
            
        end
        
        if (msg_to[portLocal] matches tagged Valid {.ln, .vc, .msg} &&& msg matches tagged FLIT_HEAD .info)
        begin
        
            debugLog.record(iid, $format("3: SA: MESSAGE EXIT: src %0d, dst %0d, isStore %0d, lane %0d, virtual channel %0d", info.src, info.dst, pack(info.isStore), ln, vc));
        end

        routes    <= new_routes;
        usedVCs   <= new_used_vcs;
        outputVCs <= new_output_vcs;

        stage4Ctrl.ready(iid);
    
    endrule

    Wire#(STATION_IID) stage4IID <- mkWire();
    PulseWire stage4Running <- mkPulseWire();
    VC_STATE#(RWire#(PORT_IDX)) newRoutesW <- replicateM(replicateM(replicateM(mkRWire())));
    RWire#(Tuple5#(PORT_IDX, LANE_IDX, VC_IDX, PORT_IDX, VC_IDX))   newOutputVCW <- mkRWire();
    
    rule beginStage4 (True);

        // Get the info from the previous stage.
        let iid <- stage4Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("4: Begin."));
        stage4IID <= iid;
        stage4Running.send();
        stage5Ctrl.ready(iid);
        
    endrule

    // Read our local state from the pools.
    VC_STATE#(FIFOF#(RING_FLIT)) stage4_virtualChannels = virtualChannelsPool[stage4IID];
    Reg#(VC_STATE#(Maybe#(PORT_IDX))) stage4_routes     = routesPool.getReg(stage4IID);
    Reg#(VC_STATE#(Maybe#(VC_IDX)))   stage4_outputVCs  = outputVCsPool.getReg(stage4IID);
    Reg#(VC_STATE#(Bool))             stage4_usedVCs       = usedVCsPool.getReg(stage4IID);
    Reg#(VC_STATE#(Bool))             stage4_outputCredits = outputCreditsPool.getReg(stage4IID);

    for (Integer p = 0; p < numPorts; p = p + 1)
    begin

        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                (* conservative_implicit_conditions *)
                rule stage4_routeVCs (stage4_virtualChannels[p][ln][vc].first() matches tagged FLIT_HEAD .info &&&
                                      stage4_virtualChannels[p][ln][vc].notEmpty() &&&
                                      !isValid(stage4_routes[p][ln][vc]) &&& stage4Running);

                    // Need to obtain a route.
                    let rt = route(stage4IID, info.dst);
                    debugLog.record(stage4IID, $format("4: RC: Got route for %s input port, lane %0d, virtual channel %0d, src %0d, dst %0d: %s output port.", portShow(fromInteger(p)), ln, vc, info.src, info.dst, portShow(rt)));
                    newRoutesW[p][ln][vc].wset(rt);

                endrule
            
                for (Integer vcx = 0; vcx < valueof(VCS_PER_LANE); vcx = vcx + 1)
                begin
    
                    (* conservative_implicit_conditions *)
                    rule stage4_getOuputVCs (stage4_virtualChannels[p][ln][vc].first() matches tagged FLIT_HEAD .info &&&
                                             stage4_routes[p][ln][vc] matches tagged Valid .rt &&&
                                             stage4_virtualChannels[p][ln][vc].notEmpty() &&&
                                             !isValid(stage4_outputVCs[p][ln][vc]) &&&
                                             !stage4_usedVCs[rt][ln][vcx] &&& 
                                             stage4_outputCredits[rt][ln][vcx] &&&
                                             stage4Running);


                        debugLog.record(stage4IID, $format("4: VA: Got Output VC %0d for %s input port, lane %0d, virtual channel %0d, output port %s.", vcx, portShow(fromInteger(p)), ln, vc, portShow(rt)));
                        newOutputVCW.wset(tuple5(fromInteger(p), fromInteger(ln), fromInteger(vc), rt, fromInteger(vcx)));

                    endrule
                end
            end
        end
    end
    
    (* fire_when_enabled *)
    rule endStage4 (stage4Running);

        // Vector to update our registers with.
        VC_STATE#(Maybe#(PORT_IDX))   new_routes = stage4_routes;
        VC_STATE#(Maybe#(VC_IDX)) new_output_vcs = stage4_outputVCs;
        VC_STATE#(Bool)             new_used_vcs = stage4_usedVCs;

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                    if (newRoutesW[p][ln][vc].wget() matches tagged Valid .new_rt)
                    begin
                        new_routes[p][ln][vc] = tagged Valid new_rt;
                    end
                end
            end
        end

        if (newOutputVCW.wget() matches tagged Valid {.p, .ln, .vc, .out_rt, .out_vc})
        begin
            new_output_vcs[p][ln][vc] = tagged Valid out_vc;
            new_used_vcs[out_rt][ln][out_vc] = True;
        end
        debugLog.record(stage4IID, $format("4: End."));

        stage4_outputVCs <= new_output_vcs;
        stage4_usedVCs <= new_used_vcs;
        stage4_routes <= new_routes;
        
    endrule

    rule stage5_enqs (True);

        // Get the current IID from the previous stage.    
        let iid <- stage5Ctrl.nextReadyInstance();
       
        // Read our local state from the pools.
        VC_STATE#(FIFOF#(RING_FLIT))      virtualChannels = virtualChannelsPool[iid];

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            
            // Deal with input enqueues from each direction.
            let m_enq <- enqFrom[p].receive(iid);
            if (m_enq matches tagged Valid {.ln, .vc, .flit})
            begin

                let new_flit = flit;
                if (flit matches tagged FLIT_HEAD .info &&& fromInteger(p) == portLocal)
                begin
                    let new_info = info;
                    new_info.src = iid;
                    if (iid != `MEM_CTRL_LOCATION)
                    begin
                        // For now we assume all core traffic goes to the mem controller.
                        new_info.dst = `MEM_CTRL_LOCATION;
                    end
                    debugLog.record(iid, $format("5: BW: MESSAGE ENTER: src %0d, dst %0d, isStore %0d, lane %0d, virtual channel %0d", new_info.src, new_info.dst, pack(new_info.isStore), ln, vc));
                    new_flit = tagged FLIT_HEAD new_info;
                end
                else
                begin
                    debugLog.record(iid, $format("5: BW: Enqueuing into %s input port, lane %0d, virtual channel %0d", portShow(fromInteger(p)), ln, vc));
                end
                virtualChannels[p][ln][vc].enq(new_flit);

            end
            else
            begin

                debugLog.record(iid, $format("5: BW: No enqueue for %s input port.", portShow(fromInteger(p))));

            end

        end

        stage6Ctrl.ready(iid);

    endrule

    (* conservative_implicit_conditions, descending_urgency="endStage4, beginStage4, stage6_creditsOut, stage5_enqs, stage3_crossbar, stage2_multiplexVCs, stage1_updateCreditsIn" *)
    rule stage6_creditsOut (True);
    
        let iid <- stage6Ctrl.nextReadyInstance();

        debugLog.record(iid, $format("6: Calculating output credits."));

        VC_STATE#(FIFOF#(RING_FLIT)) virtualChannels = virtualChannelsPool[iid];
        
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin

            VC_CREDIT_INFO creds = newVector();
            
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
            
                creds[ln] = newVector();

                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin

                    let have_credit = !virtualChannels[p][ln][vc].notEmpty(); // XXX capacity - occupancy > round-trip latency.
                    let not_full = !virtualChannels[p][ln][vc].notEmpty(); // virtualChannels[p][ln][vc].notFull();
                    creds[ln][vc] = tuple2(have_credit, not_full);

                end
            
            end
        
            creditTo[p].send(iid, tagged Valid creds);
        
        end
        
        // End of model cycle (path 1)
        localCtrl.endModelCycle(iid, 0);
        debugLog.nextModelCycle(iid);
    endrule

endmodule
