// TODO: Evaluate, possibly switch to BlockRAM.

`include "asim/dict/PARAMS_HASIM_INTERCONNECT.bsh"

import Vector::*;
import FIFO::*;
import FIFOF::*;

// TEMPORARY:
`include "asim/dict/RINGID.bsh"

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

typedef STATION_IID MESH_COORD; // Since coordinates can vary dynamically, we need to be able to hold the worst case in each direction, which is a ring network.
typedef TLog#(NUM_STATIONS) MESH_COORD_SZ;

typedef OCN_FLIT MESH_FLIT;
typedef OCN_MSG  MESH_MSG;

typedef 5 NUM_PORTS;
typedef Bit#(TLog#(NUM_PORTS)) PORT_IDX;

PORT_IDX portNorth  = 0;
PORT_IDX portEast   = 1;
PORT_IDX portSouth  = 2;
PORT_IDX portWest   = 3;
PORT_IDX portLocal  = 4;

Integer numPorts = 5;

function String portShow(PORT_IDX p);

    return case (p)
        0: "north";
        1: "east";
        2: "south";
        3: "west";
        4: "local";
        default: "UNKNOWN";
    endcase;

endfunction

typedef Vector#(NUM_PORTS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA))) VC_STATE#(parameter type t_DATA);

typedef enum
{
    INITIALIZING, RUNNING
}
ROUTER_STATE deriving (Eq, Bits);


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

module [HASIM_MODULE] mkInterconnect
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_STATIONS) debugLog <- mkTIMEPDebugFile_Multiplexed("interconnect_mesh.out");

    // ******** Dynamic Parameters ********

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();
    Param#(TLog#(NUM_STATIONS)) paramWidth <- mkDynamicParameter(`PARAMS_HASIM_INTERCONNECT_MESH_WIDTH, paramNode);
    Param#(TLog#(NUM_STATIONS)) paramHeight <- mkDynamicParameter(`PARAMS_HASIM_INTERCONNECT_MESH_HEIGHT, paramNode);
    Param#(TLog#(NUM_STATIONS)) paramMemCtrlLoc <- mkDynamicParameter(`PARAMS_HASIM_INTERCONNECT_MESH_MEM_CTRL_LOC, paramNode);
 
    // ******** Ports *******

    // Queues to/from cores
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

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
    
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, MESH_MSG))       enqTo      = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, MESH_MSG))       enqFrom    = newVector();
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO)) creditTo   = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO)) creditFrom = newVector();

    enqTo[portEast]       <- mkPortSend_Multiplexed("mesh_interconnect_enq_E");
    enqFrom[portWest]     <- mkPortRecv_Multiplexed_ReorderLastToFirstEveryN("mesh_interconnect_enq_E", 1, paramWidth);

    enqTo[portWest]       <- mkPortSend_Multiplexed("mesh_interconnect_enq_W");
    enqFrom[portEast]     <- mkPortRecv_Multiplexed_ReorderFirstToLastEveryN("mesh_interconnect_enq_W", 1, paramWidth);

    enqTo[portNorth]      <- mkPortSend_Multiplexed("mesh_interconnect_enq_N");
    enqFrom[portSouth]    <- mkPortRecv_Multiplexed_ReorderFirstNToLastN("mesh_interconnect_enq_N", 1, paramHeight);

    enqTo[portSouth]      <- mkPortSend_Multiplexed("mesh_interconnect_enq_S");
    enqFrom[portNorth]    <- mkPortRecv_Multiplexed_ReorderLastNToFirstN("mesh_interconnect_enq_S", 1, paramHeight);

    enqTo[portLocal]      <- mkPortSend_Multiplexed_Split(enqToCores, enqToMemCtrl, paramMemCtrlLoc);
    enqFrom[portLocal]    <- mkPortRecv_Multiplexed_Join(enqFromCores, enqFromMemCtrl, paramMemCtrlLoc);
    
    creditTo[portEast]    <- mkPortSend_Multiplexed("mesh_interconnect_credit_E");
    creditFrom[portWest]  <- mkPortRecv_Multiplexed_ReorderLastToFirstEveryN("mesh_interconnect_credit_E", 1, paramWidth);

    creditTo[portWest]    <- mkPortSend_Multiplexed("mesh_interconnect_credit_W");
    creditFrom[portEast]  <- mkPortRecv_Multiplexed_ReorderFirstToLastEveryN("mesh_interconnect_credit_W", 1, paramWidth);

    creditTo[portNorth]   <- mkPortSend_Multiplexed("mesh_interconnect_credit_N");
    creditFrom[portSouth] <- mkPortRecv_Multiplexed_ReorderFirstNToLastN("mesh_interconnect_credit_N", 1, paramHeight);

    creditTo[portSouth]   <- mkPortSend_Multiplexed("mesh_interconnect_credit_S");
    creditFrom[portNorth] <- mkPortRecv_Multiplexed_ReorderLastNToFirstN("mesh_interconnect_credit_S", 1, paramHeight);

    creditTo[portLocal]   <- mkPortSend_Multiplexed_Split(creditToCores, creditToMemCtrl, paramMemCtrlLoc);
    creditFrom[portLocal] <- mkPortRecv_Multiplexed_Join(creditFromCores, creditFromMemCtrl, paramMemCtrlLoc);

    // NOTE: The module does not use a local controller, as it has two sets of ports,
    // one set is NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    inports[0] = enqFromCores.ctrl;
    inports[1] = creditFromCores.ctrl;

    Vector#(8, INSTANCE_CONTROL_IN#(NUM_STATIONS)) inportsR = newVector();
    inportsR[0] = enqFrom[portNorth].ctrl;
    inportsR[1] = enqFrom[portSouth].ctrl;
    inportsR[2] = enqFrom[portEast].ctrl;
    inportsR[3] = enqFrom[portWest].ctrl;
    // Note: Local is handled above.
    inportsR[4] = creditFrom[portNorth].ctrl;
    inportsR[5] = creditFrom[portSouth].ctrl;
    inportsR[6] = creditFrom[portEast].ctrl;
    inportsR[7] = creditFrom[portWest].ctrl;
    // Note: Local is handled above.

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <- mkLocalControllerPlusN(inports, inportsR);
    
    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    // The actual virtual channels. Currently actual FIFOs, but could be implemented
    // in BlockRAM or whatever.
    MULTIPLEXED#(NUM_STATIONS, VC_STATE#(FIFOF#(MESH_FLIT)))    virtualChannelsPool    <- mkMultiplexed(replicateM(replicateM(replicateM(mkUGSizedFIFOF(4)))));
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
    Reg#(Vector#(NUM_STATIONS, MESH_COORD)) routerRowPosition <- mkRegU();
    Reg#(Vector#(NUM_STATIONS, MESH_COORD)) routerColPosition <- mkRegU();
    
    COUNTER#(MESH_COORD_SZ) curInitID  <- mkLCounter(0);
    COUNTER#(MESH_COORD_SZ) curInitRow <- mkLCounter(0);
    COUNTER#(MESH_COORD_SZ) curInitCol <- mkLCounter(0);
    
    Reg#(ROUTER_STATE) state <- mkReg(tagged INITIALIZING);
    
    function PORT_IDX route(STATION_ID my_id, STATION_ID dst);
        
        MESH_COORD dst_row = routerRowPosition[dst];
        MESH_COORD dst_col = routerColPosition[dst];
        MESH_COORD my_row  = routerRowPosition[my_id];
        MESH_COORD my_col  = routerColPosition[my_id];

        STATION_IID cur_pos = 0;

        
        if (dst_col < my_col)
            return portWest;
        else if (dst_col > my_col)
            return portEast;
        else
        begin
            if (dst_row < my_row)
                return portNorth;
            else if (dst_row > my_row)
                return portSouth;
            else
                return portLocal;
        end
        
    endfunction


    // ******* Rules *******

    (* conservative_implicit_conditions *)
    rule initializeRouterPos (state == INITIALIZING);
    
        routerRowPosition[curInitID.value()] <= curInitRow.value();
        routerColPosition[curInitID.value()] <= curInitCol.value();
        
        curInitID.up();
        
        if (curInitCol.value() == (paramWidth - 1))
        begin

            curInitCol.setC(0);
            
            if (curInitRow.value() == (paramHeight - 1))
            begin
                state <= RUNNING;
            end
            else
            begin
                curInitRow.up();
            end

        end
        else
        begin
            curInitCol.up();
        end

    endrule

    rule stage1_updateCreditsIn (state == RUNNING);
    
        // Get the next IID to simulate.
        let iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(iid);
        
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
        
        debugLog.record_next_cycle(iid, $format("1: Update input credits"));
        
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
        VC_STATE#(FIFOF#(MESH_FLIT)) virtualChannels = virtualChannelsPool[iid];
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
        VC_STATE#(FIFOF#(MESH_FLIT))      virtualChannels = virtualChannelsPool[iid];
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes          = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs       = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             usedVCs         = usedVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputCredits   = outputCreditsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputNotFulls  = outputNotFullsPool.getReg(iid);

        // This is the vector of output messages that the virtual channels contend for.
        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msg_to = replicate(tagged Invalid);
        
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
    VC_STATE#(FIFOF#(MESH_FLIT)) stage4_virtualChannels = virtualChannelsPool[stage4IID];
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
        VC_STATE#(FIFOF#(MESH_FLIT))      virtualChannels = virtualChannelsPool[iid];

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
                    if (iid != paramMemCtrlLoc)
                    begin
                        // For now we assume all core traffic goes to the mem controller.
                        new_info.dst = paramMemCtrlLoc;
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

        VC_STATE#(FIFOF#(MESH_FLIT)) virtualChannels = virtualChannelsPool[iid];
        
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
        
    endrule

endmodule

module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderSideBuffer
    #(
        String portname, 
        Integer latency, 
        INSTANCE_ID#(t_NUM_INSTANCES) period,
        function Bool enqToSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid),
        function Bool resetEnq(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid),
        function Bool deqFromSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid),
        function Bool resetDeq(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid)
    )
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));

    Connection_Receive#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))) con <- mkConnection_Receive(portname);
    
    Reg#(INSTANCE_ID#(t_NUM_INSTANCES)) maxInstance <- mkReg(fromInteger(valueof(t_NUM_INSTANCES) - 1));

    Integer rMax = (latency * valueof(t_NUM_INSTANCES)) + 1;

    if (rMax > 64)
        error("Total Port buffering cannot currently exceed 64. Port: " + portname);

    function Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG)) initfunc(Bit#(6) idx);
        INSTANCE_ID#(t_NUM_INSTANCES) iid = truncate(idx);
        return tuple2(iid, tagged Invalid);
    endfunction

    LUTRAM#(Bit#(6), Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))) rs <- mkLUTRAMWith(initfunc);
    LUTRAM#(Bit#(6), Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))) sideBuffer <- mkLUTRAMWith(initfunc);

    COUNTER#(6) head <- mkLCounter(0);
    COUNTER#(6) tail <- mkLCounter((fromInteger(latency * (valueof(t_NUM_INSTANCES) - 1))));
    COUNTER#(6) sideHead <- mkLCounter(0);
    COUNTER#(6) sideTail <- mkLCounter((fromInteger(latency)));
    Reg#(INSTANCE_ID#(t_NUM_INSTANCES)) curEnq <- mkReg(0);
    Reg#(INSTANCE_ID#(t_NUM_INSTANCES)) curDeq <- mkReg(0);

    Bool fullQ  = tail.value() + 1 == head.value();
    Bool emptyQ = head.value() == tail.value();
    Bool sideFull  = sideTail.value() + 1 == sideHead.value();
    Bool sideEmpty = sideHead.value() == sideTail.value();
    Bool canDeq = deqFromSide(curDeq, maxInstance) ? !sideEmpty : !emptyQ;
    Bool canEnq = enqToSide(curEnq, maxInstance)   ? !sideFull  : !fullQ;

    Reg#(Bool) initialized <- mkReg(False);

    rule shift (initialized && canEnq && con.notEmpty());

        match {.iid, .msg} = con.receive();
        con.deq();

        if (enqToSide(curEnq, maxInstance))
        begin
            
            sideBuffer.upd(sideTail.value(), tuple2(iid, msg));
            sideTail.up();
        
        end
        else
        begin
        
            rs.upd(tail.value(), tuple2(iid, msg));
            tail.up();
        
        end
        
        if (resetEnq(curEnq, maxInstance))
        begin
        
            curEnq <= 0;
        
        end
        else
        begin

            curEnq <= curEnq + 1;

        end

    endrule
    
    interface INSTANCE_CONTROL_IN ctrl;


        method Bool empty() = !canDeq;
        method Bool balanced() = True;
        method Bool light() = False;
        
        method Maybe#(INSTANCE_ID#(t_NUM_INSTANCES)) nextReadyInstance();
        
            match {.iid, .m} = deqFromSide(curDeq, maxInstance) ? sideBuffer.sub(sideHead.value()) : rs.sub(head.value());
            return (!canDeq || !initialized) ? tagged Invalid : tagged Valid iid;
        endmethod
        
        method Action setMaxRunningInstance(INSTANCE_ID#(t_NUM_INSTANCES) iid);
        
            Bit#(6) l = fromInteger(latency);
            Bit#(6) k = zeroExtendNP(iid) + 1;
            Bit#(6) n = zeroExtendNP(period);
            tail.setC((k-n) * l);
            sideTail.setC(n * l);
            maxInstance <= iid;
            initialized <= True;
        
        endmethod
        
    endinterface

    method ActionValue#(Maybe#(t_MSG)) receive(INSTANCE_ID#(t_NUM_INSTANCES) dummy) if (canDeq);

        Maybe#(t_MSG) res = tagged Invalid;

        if (deqFromSide(curDeq, maxInstance))
        begin
        
            // Return the side buffer.
            match {.iid, .m} = sideBuffer.sub(sideHead.value());
            res = m;
            sideHead.up();
        
        end
        else
        begin
        
            // Return the main buffer.
            match {.iid, .m} = rs.sub(head.value());
            res = m;
            head.up();
        
        end

        if (resetDeq(curDeq, maxInstance))
        begin
            curDeq <= 0;
        end
        else
        begin
            curDeq <= curDeq + 1;
        end

        return res;

    endmethod

endmodule

module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderFirstToLastEveryN#(String portname, Integer latency, INSTANCE_ID#(t_NUM_INSTANCES) period)
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));
         
    function Bool enqToSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == 0;
    endfunction

    function Bool deqFromSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == period - 1;
    endfunction
    
    function Bool resetEnq(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == (period - 1);
    endfunction
    
    function Bool resetDeq(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == (period - 1);
    endfunction
    
    let p <- mkPortRecv_Multiplexed_ReorderSideBuffer(portname, latency, period, enqToSide, resetEnq, deqFromSide, resetDeq);
    return p;

endmodule


module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderLastToFirstEveryN#(String portname, Integer latency, INSTANCE_ID#(t_NUM_INSTANCES) period)
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));
         
    function Bool enqToSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == period - 1;
    endfunction

    function Bool deqFromSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == 0;
    endfunction
    
    function Bool resetEnq(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == (period - 1);
    endfunction
    
    function Bool resetDeq(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == (period - 1);
    endfunction
    
    let p <- mkPortRecv_Multiplexed_ReorderSideBuffer(portname, latency, period, enqToSide, resetEnq, deqFromSide, resetDeq);
    return p;

endmodule


module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderFirstNToLastN#(String portname, Integer latency, INSTANCE_ID#(t_NUM_INSTANCES) period)
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));
         
    function Bool enqToSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq < period;
    endfunction

    function Bool deqFromSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq > (max_iid - period);
    endfunction
    
    function Bool resetEnq(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == max_iid;
    endfunction
    
    function Bool resetDeq(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == max_iid;
    endfunction
    
    let p <- mkPortRecv_Multiplexed_ReorderSideBuffer(portname, latency, period, enqToSide, resetEnq, deqFromSide, resetDeq);
    return p;

endmodule


module [HASIM_MODULE] mkPortRecv_Multiplexed_ReorderLastNToFirstN#(String portname, Integer latency, INSTANCE_ID#(t_NUM_INSTANCES) period)
    // interface:
        (PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, 6),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));

         
    function Bool enqToSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq > (max_iid - period);
    endfunction

    function Bool deqFromSide(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq < period;
    endfunction
    
    function Bool resetEnq(INSTANCE_ID#(t_NUM_INSTANCES) cur_enq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_enq == max_iid;
    endfunction
    
    function Bool resetDeq(INSTANCE_ID#(t_NUM_INSTANCES) cur_deq, INSTANCE_ID#(t_NUM_INSTANCES) max_iid);
        return cur_deq == max_iid;
    endfunction
    
    let p <- mkPortRecv_Multiplexed_ReorderSideBuffer(portname, latency, period, enqToSide, resetEnq, deqFromSide, resetDeq);
    return p;

endmodule


module [HASIM_MODULE] mkPortRecv_Multiplexed_Join#(PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG) p1, PORT_RECV#(t_MSG) p2, Bit#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) insertion_point)
    // interface:
        (PORT_RECV_MULTIPLEXED#(TAdd#(t_NUM_INSTANCES, 1), t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, TLog#(TAdd#(t_NUM_INSTANCES, 1))),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));


    COUNTER#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) cur <- mkLCounter(0);
    
    Bool canDeq = (cur.value() == insertion_point) ? !p2.ctrl.empty() : !p1.ctrl.empty();
    
    interface INSTANCE_CONTROL_IN ctrl;


        method Bool empty() = !canDeq;
        method Bool balanced() = True;
        method Bool light() = False;
        
        method Maybe#(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1))) nextReadyInstance();
        
            return canDeq ? tagged Valid cur.value() : tagged Invalid;
        
        endmethod
        
        method Action setMaxRunningInstance(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) iid);
        
            noAction;
            // NOTE: We assume that this will be called directly on the child ports.
            // This could be a bad assumption.
        endmethod
        
    endinterface

    method ActionValue#(Maybe#(t_MSG)) receive(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) dummy) if (canDeq);
        
        if (cur.value() == fromInteger(valueof(t_NUM_INSTANCES))) // Note: purposefully leave off -1, to take into account the extra port.
        begin
            cur.setC(0);
        end
        else
        begin
            cur.up();
        end
        
        if (cur.value() == insertion_point)
        begin
            let msg <- p2.receive();
            return msg;
        end
        else
        begin
            let msg <- p1.receive(truncate(dummy));
            return msg;
        end
        
    endmethod

endmodule

module [HASIM_MODULE] mkPortSend_Multiplexed_Split#(PORT_SEND_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG) p1, PORT_SEND#(t_MSG) p2, Bit#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) split_point)
    // interface:
        (PORT_SEND_MULTIPLEXED#(TAdd#(t_NUM_INSTANCES, 1), t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, TLog#(TAdd#(t_NUM_INSTANCES, 1))),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));


    COUNTER#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) cur <- mkLCounter(0);
    
    Bool canEnq = (cur.value() == split_point) ? !p2.ctrl.full() : !p1.ctrl.full();
    
    interface INSTANCE_CONTROL_OUT ctrl;

        method Bool full() = !canEnq;
        method Bool balanced() = True;
        method Bool heavy() = False;

    endinterface

    method Action send(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) dummy, Maybe#(t_MSG) msg) if (canEnq);
        
        if (cur.value() == fromInteger(valueof(t_NUM_INSTANCES))) // Note: purposefully leave off -1, to take into account the extra port.
        begin
            cur.setC(0);
        end
        else
        begin
            cur.up();
        end
        
        if (cur.value() == split_point)
        begin
            p2.send(msg);
        end
        else
        begin
            p1.send(truncate(dummy), msg);
        end
        
    endmethod

endmodule


typedef enum
{
    LCN_Idle,               // Waiting for a command
    LCN_Running,            // Running, allowing slip
    LCN_Synchronizing,      // Running, attempting to synchronize
    LCN_Stepping            // Run one modelCC
}
LCN_STATE
    deriving (Eq, Bits);

module [HASIM_MODULE] mkLocalControllerPlusN

    // parameters:
    #(
    Vector#(t_NUM_INPORTS,  INSTANCE_CONTROL_IN#(t_NUM_INSTANCES))  inctrls, 
    Vector#(t_NUM_INPORTS_N,  INSTANCE_CONTROL_IN#(t_NUM_INSTANCES_PLUS_N))  inctrlsN
    )
    // interface:
        (LOCAL_CONTROLLER#(t_NUM_INSTANCES_PLUS_N))
    provisos
        (Add#(t_N, t_NUM_INSTANCES, t_NUM_INSTANCES_PLUS_N));

    Reg#(LCN_STATE) state <- mkReg(LCN_Idle);
  
    // Counter of active instances. 
    // We start at -1, so we assume at least one instance is active.
    COUNTER#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)) maxActiveInstance <- mkLCounter(~0);
    // Vector of running instances
    Reg#(Vector#(t_NUM_INSTANCES_PLUS_N, Bool)) instanceRunning <- mkReg(replicate(False));
    // Track stepping state.
    Reg#(Vector#(t_NUM_INSTANCES_PLUS_N, Bool)) instanceStepped <- mkReg(replicate(False));

    // Signalled DONE to the software?
    Reg#(Bool) signalDone <- mkReg(False);

    Vector#(t_NUM_INSTANCES_PLUS_N, PulseWire)    startCycleW <- replicateM(mkPulseWire());
    Vector#(t_NUM_INSTANCES_PLUS_N, PulseWire)      endCycleW <- replicateM(mkPulseWire());
    Vector#(t_NUM_INSTANCES_PLUS_N, Wire#(Bit#(8))) pathDoneW <- replicateM(mkWire());
    
    
    // For now this local controller just goes round-robin over the instances.
    // This is guaranteed to be correct accross multiple modules.
    // The performance of this could be improved, but the interaction with time-multiplexed
    // ports needs to be worked out.
    
    COUNTER#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)) nextInstance <- mkLCounter(0);
    
    Connection_Chain#(CONTROLLER_MSG) link_controllers <- mkConnection_Chain(`RINGID_CONTROLLER_MESSAGES);

    function Bool allTrue(Vector#(k, Bool) v);
        return foldr(\&& , True, v);
    endfunction

    // Can this module read from this Port? Purposely ignore the non plus-N ports
    function Bool canReadFromN(INSTANCE_CONTROL_IN#(t_NUM_INSTANCES_PLUS_N) ctrl_in);
        return case (state)
                   LCN_Running:        return !ctrl_in.empty();
                   LCN_Stepping:       return !ctrl_in.empty();
                   LCN_Synchronizing:  return !ctrl_in.light();
                   default:            return False;
               endcase;
    endfunction

    // This function will determine the next instance in a non-round-robin manner when we're ready
    // to go that route. Currently this is unused.

    function Bool instanceReady(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid);
        
        Bool canRead  = True;

        // Can we read/write all of the plus N ports? Disregard normal ports for this.
        for (Integer x = 0; x < valueOf(t_NUM_INPORTS_N); x = x + 1)
            canRead = canRead && canReadFromN(inctrlsN[x]);

        // An instance is ready to go only if it's been enabled.
        return !instanceRunning[iid] && canRead;

    endfunction

    function INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) nextReadyInstance();
        
        INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) res = 0;

        for (Integer x = 0; x < valueof(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            res = instanceReady(fromInteger(x)) ? fromInteger(x) : res;
        end
        
        return res;
    
    endfunction

    function Bool someInstanceReady();
        
        Bool res = False;

        for (Integer x = 0; x < valueof(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            res = instanceReady(fromInteger(x)) || res;
        end
        
        return res;
    
    endfunction



    function Bool balanced();
        Bool res = True;
        
        // Are the plus N ports all balanced? Disregard normal ports for this.
        for (Integer x = 0; x < valueOf(t_NUM_INPORTS_N); x = x + 1)
        begin
            res = res && inctrlsN[x].balanced();
        end

        return res;

    endfunction



    // ====================================================================
    //
    // Process controller commands and send responses.
    //
    // ====================================================================

    FIFO#(Bool) checkBalanceQ <- mkFIFO();
    FIFO#(CONTROLLER_MSG) newCtrlMsgQ <- mkFIFO1();
    
    rule checkBalance (True);
        checkBalanceQ.deq();
        link_controllers.sendToNext(tagged COM_SyncQuery balanced());
    endrule

    rule newControlMsg (True);
        let cmd = newCtrlMsgQ.first();
        newCtrlMsgQ.deq();
        
        link_controllers.sendToNext(cmd);
    endrule

    (* descending_urgency = "checkBalance, newControlMsg, nextCommand" *)
    rule nextCommand (state != LCN_Stepping);
        let newcmd <- link_controllers.recvFromPrev();
        Maybe#(CONTROLLER_MSG) outcmd = tagged Valid newcmd;

        case (newcmd) matches
            tagged COM_RunProgram:
            begin
    
                for (Integer x = 0; x < valueof(t_NUM_INPORTS); x = x + 1)
                begin
                
                    // We know this truncation is safe since the button has only been pushed k times, not k+N.
                    inctrls[x].setMaxRunningInstance(truncateNP(maxActiveInstance.value()));

                end

                for (Integer x = 0; x < valueof(t_NUM_INPORTS_N); x = x + 1)
                begin
                
                    inctrlsN[x].setMaxRunningInstance(maxActiveInstance.value() + fromInteger(valueof(t_N)));

                end
                
                maxActiveInstance.setC(maxActiveInstance.value() + fromInteger(valueof(t_N)));

                state <= LCN_Running;

            end

            tagged COM_Synchronize:
            begin
                state <= LCN_Synchronizing;
            end

            tagged COM_SyncQuery .all_balanced:
            begin
                // The COM_SyncQuery state will remain True if all controllers
                // are balanced.  If a previous controller is unbalanced then
                // just forward the unbalanced state.  If we need to check
                // the state of this controller then queue the request.
                //
                // The Bluespec scheduler throws an error about being unable
                // to break a cycle if we attempt to check balance here
                // because it can't break the connection between setting
                // state, startModelCycle and changes to balance() while
                // the model runs.
                if (all_balanced)
                begin
                    outcmd = tagged Invalid;
                    checkBalanceQ.enq(?);
                end
            end

            tagged COM_Step:
            begin
                state <= LCN_Stepping;
                Vector#(t_NUM_INSTANCES_PLUS_N, Bool) instance_stepped = newVector();
                for (Integer x = 0; x < valueOf(t_NUM_INSTANCES_PLUS_N); x = x + 1)
                begin
                   instance_stepped[x] = False;
                end
                instanceStepped <= instance_stepped;
            end

            tagged COM_Pause .send_response:
            begin
                state <= LCN_Idle;
            end

            // TODO: should this be COM_EnableInstance??
            tagged COM_EnableContext .iid:
            begin
                maxActiveInstance.up();
            end

            // TODO: should this be COM_DisableInstance??
            tagged COM_DisableContext .iid:
            begin
                maxActiveInstance.down();
            end
        endcase

        // Forward command around the ring
        if (outcmd matches tagged Valid .cmd)
        begin
            link_controllers.sendToNext(cmd);
        end
    endrule

    rule updateRunning (True);
    
        Vector#(t_NUM_INSTANCES_PLUS_N, Bool) new_running = instanceRunning;

        for (Integer x = 0; x < valueOf(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            if (instanceRunning[x])
                new_running[x] =  !endCycleW[x];
            else if (startCycleW[x])
                new_running[x] = !endCycleW[x];
            else
                noAction;
        end
        
        instanceRunning <= new_running;
    
    endrule

    //
    // updateStateForStepping --
    //     State update associated with startModelCycle, encoded in a rule
    //     and controlled by a wire in order to set scheduling priority.
    //
    Wire#(Maybe#(Bit#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)))) newModelCycleStarted <- mkDWire(tagged Invalid);

    (* descending_urgency = "updateStateForStepping, nextCommand" *)
    rule updateStateForStepping (state == LCN_Stepping &&&
                                 newModelCycleStarted matches tagged Valid .iid);
        instanceStepped[iid] <= True;
        if (iid == maxActiveInstance.value())
            state <= LCN_Idle;
    endrule

    //
    // ******** Methods *******


    method ActionValue#(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N)) startModelCycle() if ((state != LCN_Idle) && instanceReady(nextInstance.value()));

        let next_iid = nextInstance.value();

        if (state == LCN_Stepping)
        begin
            newModelCycleStarted <= tagged Valid next_iid;
        end
        
        // checkInstanceSanity();
        
        startCycleW[next_iid].send();
        
        if (next_iid >= maxActiveInstance.value())
        begin
            nextInstance.setC(0);
        end
        else
        begin
            nextInstance.up();
        end

        return next_iid;

    endmethod

    method Action endModelCycle(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid, Bit#(8) path);
    
        endCycleW[iid].send();
        pathDoneW[iid] <= path; // Put the path into the waveform.
    
    endmethod

    method Action instanceDone(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid, Bool pf);
        // XXX this should be per-instance.  For now only allowed to fire once.
        if (! signalDone)
        begin
            newCtrlMsgQ.enq(tagged LC_DoneRunning pf);
            signalDone <= True;
        end
    endmethod
    
endmodule
