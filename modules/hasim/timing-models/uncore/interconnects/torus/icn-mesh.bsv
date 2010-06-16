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

    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    // The actual virtual channels. Stored as logical FIFOs in a single Distributed RAM FIFO.
    MULTIPLEXED_STATE_POOL#(NUM_STATIONS, VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4))) virtualChannelsPool <- mkMultiplexedStatePool(replicate(replicate(replicate(funcFIFO_Init))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(PORT_IDX))) routesPool             <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(VC_IDX)))   outputVCsPool          <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             usedVCsPool            <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputCreditsPool      <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputNotFullsPool     <- mkMultiplexedReg(replicate(replicate(replicate(False))));

    // NOTE: The module uses a special local controller, as it has two sets of ports,
    // one set is NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    inports[0] = enqFromCores.ctrl;
    inports[1] = creditFromCores.ctrl;

    Vector#(11, INSTANCE_CONTROL_IN#(NUM_STATIONS)) inportsR = newVector();
    inportsR[0] = enqFrom[portNorth].ctrl;
    inportsR[1] = enqFrom[portSouth].ctrl;
    inportsR[2] = enqFrom[portEast].ctrl;
    inportsR[3] = enqFrom[portWest].ctrl;
    inportsR[4] = enqFrom[portLocal].ctrl;
    inportsR[5] = creditFrom[portNorth].ctrl;
    inportsR[6] = creditFrom[portSouth].ctrl;
    inportsR[7] = creditFrom[portEast].ctrl;
    inportsR[8] = creditFrom[portWest].ctrl;
    inportsR[9] = creditFrom[portLocal].ctrl;
    inportsR[10] = virtualChannelsPool.ctrl;

    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_STATIONS)) outportsR = newVector();
    outportsR[0] = enqTo[portLocal].ctrl;
    outportsR[1] = creditTo[portLocal].ctrl;

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <- mkLocalControllerPlusN(inports, inportsR, outportsR);
    
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER#(NUM_STATIONS, Tuple2#(Vector#(NUM_PORTS, Maybe#(WINNER_INFO)), VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4)))) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4))) stage4Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4))) stage5Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4))) stage6Ctrl <- mkStageController();

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
        VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4)) virtualChannels <- virtualChannelsPool.extractState(iid);
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

                    if (funcFIFO_notEmpty(virtualChannels[p][ln][vc]))
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
                                                                message: funcFIFO_UGfirst(virtualChannels[p][ln][vc])
                                                            };
                            end
                        end
                    end
                end
            end
        end

        stage3Ctrl.ready(iid, tuple2(vc_winners, virtualChannels));

    endrule

    (* conservative_implicit_conditions *)
    rule stage3_crossbar (True);
        
        // Get the info from the previous stage.
        match {.iid, {.vc_winners, .virtualChannels}} <- stage3Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes          = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs       = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             usedVCs         = usedVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputCredits   = outputCreditsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputNotFulls  = outputNotFullsPool.getReg(iid);

        // This is the vector of output messages that the virtual channels contend for.
        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msg_to = replicate(tagged Invalid);
        
        // Vectors to update our registers with.
        VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4)) new_vcs = virtualChannels;
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
                    new_vcs[p][info.lane][info.inputVC] = funcFIFO_UGdeq(virtualChannels[p][info.lane][info.inputVC]);

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

        stage4Ctrl.ready(iid, new_vcs);
    
    endrule

    Wire#(STATION_IID) stage4IID <- mkWire();
    Wire#(VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4))) stage4_virtualChannels <- mkWire();
    PulseWire stage4Running <- mkPulseWire();
    VC_STATE#(RWire#(PORT_IDX)) newRoutesW <- replicateM(replicateM(replicateM(mkRWire())));
    RWire#(Tuple5#(PORT_IDX, LANE_IDX, VC_IDX, PORT_IDX, VC_IDX))   newOutputVCW <- mkRWire();
    
    rule beginStage4 (True);

        // Get the info from the previous stage.
        match {.iid, .virtualChannels} <- stage4Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("4: Begin."));
        stage4IID <= iid;
        stage4_virtualChannels <= virtualChannels;
        stage4Running.send();
        stage5Ctrl.ready(iid, virtualChannels);
        
    endrule

    // Read our local state from the pools.
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
                rule stage4_routeVCs (funcFIFO_UGfirst(stage4_virtualChannels[p][ln][vc]) matches tagged FLIT_HEAD .info &&&
                                      funcFIFO_notEmpty(stage4_virtualChannels[p][ln][vc]) &&&
                                      !isValid(stage4_routes[p][ln][vc]) &&& stage4Running);

                    // Need to obtain a route.
                    let rt = route(stage4IID, info.dst);
                    debugLog.record(stage4IID, $format("4: RC: Got route for %s input port, lane %0d, virtual channel %0d, src %0d, dst %0d: %s output port.", portShow(fromInteger(p)), ln, vc, info.src, info.dst, portShow(rt)));
                    newRoutesW[p][ln][vc].wset(rt);

                endrule
            
                for (Integer vcx = 0; vcx < valueof(VCS_PER_LANE); vcx = vcx + 1)
                begin
    
                    (* conservative_implicit_conditions *)
                    rule stage4_getOuputVCs (funcFIFO_UGfirst(stage4_virtualChannels[p][ln][vc]) matches tagged FLIT_HEAD .info &&&
                                             funcFIFO_notEmpty(stage4_virtualChannels[p][ln][vc]) &&&
                                             stage4_routes[p][ln][vc] matches tagged Valid .rt &&&
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
        match {.iid, .virtualChannels} <- stage5Ctrl.nextReadyInstance();
       
        VC_STATE#(FUNC_FIFO#(MESH_FLIT, 4)) new_vcs = virtualChannels;

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
                new_vcs[p][ln][vc] = funcFIFO_UGenq(virtualChannels[p][ln][vc], new_flit);

            end
            else
            begin

                debugLog.record(iid, $format("5: BW: No enqueue for %s input port.", portShow(fromInteger(p))));

            end

        end

        stage6Ctrl.ready(iid, new_vcs);

    endrule

    (* conservative_implicit_conditions, descending_urgency="endStage4, beginStage4, stage6_creditsOut, stage5_enqs, stage3_crossbar, stage2_multiplexVCs, stage1_updateCreditsIn" *)
    rule stage6_creditsOut (True);
    
        match {.iid, .virtualChannels} <- stage6Ctrl.nextReadyInstance();

        debugLog.record(iid, $format("6: Calculating output credits."));

        
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin

            VC_CREDIT_INFO creds = newVector();
            
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
            
                creds[ln] = newVector();

                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin

                    let have_credit = !funcFIFO_notEmpty(virtualChannels[p][ln][vc]); // XXX capacity - occupancy > round-trip latency.
                    let not_full = !funcFIFO_notEmpty(virtualChannels[p][ln][vc]); // virtualChannels[p][ln][vc].notFull();
                    creds[ln][vc] = tuple2(have_credit, not_full);

                end
            
            end
        
            creditTo[p].send(iid, tagged Valid creds);
        
        end
        
        // End of model cycle (path 1)
        localCtrl.endModelCycle(iid, 0);
        virtualChannelsPool.insertState(iid, virtualChannels);
        
    endrule

endmodule
