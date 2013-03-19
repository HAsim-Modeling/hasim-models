//
// Copyright (C) 2011 Intel Corporation
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

// TODO: Evaluate, possibly switch to BlockRAM.

import Vector::*;
import FIFO::*;
import FIFOF::*;
import FShow::*;
import GetPut::*;
import Connectable::*;


// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/common_services.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/hasim_memory_controller.bsh"
`include "awb/provides/hasim_chip_topology.bsh"


// ****** Generated files ******

`include "awb/dict/EVENTS_MESH.bsh"
`include "awb/dict/TOPOLOGY.bsh"
`include "awb/dict/PARAMS_HASIM_INTERCONNECT.bsh"


typedef STATION_IID MESH_COORD; // Since coordinates can vary dynamically, we need to be able to hold the worst case in each direction, which is a ring network.
typedef TLog#(NUM_STATIONS) MESH_COORD_SZ;

typedef OCN_FLIT MESH_FLIT;
typedef OCN_MSG  MESH_MSG;

typedef 5 NUM_PORTS;
typedef Bit#(TLog#(NUM_PORTS)) PORT_IDX;

typedef 4 NUM_VC_FIFO_ENTRIES;

//
// The order of port numbering is critical!  North, east, south and west must be
// the first 4 entries in order to fit in the stationRoutingTable.
//
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

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) LANE_STATE#(parameter type t_DATA);
typedef Vector#(NUM_PORTS, LANE_STATE#(t_DATA)) VC_STATE#(parameter type t_DATA);

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


//
// Mesh interconnection network timing.
//
module [HASIM_MODULE] mkInterconnect
    // interface:
    ()
    provisos (Alias#(FUNC_FIFO#(MESH_FLIT, NUM_VC_FIFO_ENTRIES), t_VC_FIFO),
              Alias#(Tuple5#(PORT_IDX, LANE_IDX, VC_IDX, PORT_IDX, VC_IDX), t_OUT_VC_REQ),
              Alias#(Vector#(NUM_STATIONS, Bit#(2)), t_ROUTING_TABLE));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_STATIONS) debugLog <- mkTIMEPDebugFile_Multiplexed("interconnect_mesh.out");

    // ******** Dynamic Parameters ********

    PARAMETER_NODE paramNode <- mkDynamicParameterNode();
    Param#(TLog#(NUM_STATIONS)) paramMemCtrlLoc <- mkDynamicParameter(`PARAMS_HASIM_INTERCONNECT_MESH_MEM_CTRL_LOC, paramNode);


    // ******** Initialization from software ********

    ReadOnly#(STATION_IID) meshWidth <- mkTopologyParamReg(`TOPOLOGY_NET_MESH_WIDTH);
    ReadOnly#(STATION_IID) meshHeight <- mkTopologyParamReg(`TOPOLOGY_NET_MESH_HEIGHT);


    // ******** Ports *******

    // Queues to/from cores
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

    // Queues to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_SEND_MULTIPLEXED#(1, OCN_MSG)        enqToMemCtrl      <- mkPortSend_Multiplexed("ocn_to_memctrl_enq");
    PORT_RECV_MULTIPLEXED#(1, OCN_MSG)        enqFromMemCtrl    <- mkPortRecv_Multiplexed("memctrl_to_ocn_enq", 1);
    PORT_SEND_MULTIPLEXED#(1, VC_CREDIT_INFO) creditToMemCtrl   <- mkPortSend_Multiplexed("ocn_to_memctrl_credit");
    PORT_RECV_MULTIPLEXED#(1, VC_CREDIT_INFO) creditFromMemCtrl <- mkPortRecv_Multiplexed("memctrl_to_ocn_credit", 1);

    // Local "NULL" connections when no CPU or memory controller is attached
    // to a local port.  Not knowing in advance how many ports may be NULL,
    // allocate the maximum number.  The only cost is an index bit.
    PORT_SEND_MULTIPLEXED#(NUM_STATIONS, OCN_MSG)        enqToNull      <- mkPortSend_Multiplexed_NULL();
    PORT_RECV_MULTIPLEXED#(NUM_STATIONS, OCN_MSG)        enqFromNull    <- mkPortRecv_Multiplexed_NULL();
    PORT_SEND_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO) creditToNull   <- mkPortSend_Multiplexed_NULL();
    PORT_RECV_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_INFO) creditFromNull <- mkPortRecv_Multiplexed_NULL();

    //
    // Local ports are a dynamic combination of CPUs, memory controllers, and
    // NULL connections.
    //
    // localPortMap indicates, for each multiplexed port instance ID, the type
    // of local port attached (CPU, memory controller, NULL).
    //
    let localPortInit <- mkTopologyParamStream(`TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP);
    LUTRAM#(Bit#(TLog#(TAdd#(TAdd#(MAX_NUM_CPUS, 1),
                             NUM_STATIONS))),
            Bit#(2)) localPortMap <- mkLUTRAMWithGet(localPortInit);

    let enqToLocal      <- mkPortSend_Multiplexed_Split3(enqToCores, enqToMemCtrl, enqToNull, localPortMap);
    let enqFromLocal    <- mkPortRecv_Multiplexed_Join3(enqFromCores, enqFromMemCtrl, enqFromNull, localPortMap);
    let creditToLocal   <- mkPortSend_Multiplexed_Split3(creditToCores, creditToMemCtrl, creditToNull, localPortMap);
    let creditFromLocal <- mkPortRecv_Multiplexed_Join3(creditFromCores, creditFromMemCtrl, creditFromNull, localPortMap);


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
    enqFrom[portWest]     <- mkPortRecv_Multiplexed_ReorderLastToFirstEveryN("mesh_interconnect_enq_E", 1, meshWidth, meshHeight);

    enqTo[portWest]       <- mkPortSend_Multiplexed("mesh_interconnect_enq_W");
    enqFrom[portEast]     <- mkPortRecv_Multiplexed_ReorderFirstToLastEveryN("mesh_interconnect_enq_W", 1, meshWidth, meshHeight);

    enqTo[portNorth]      <- mkPortSend_Multiplexed("mesh_interconnect_enq_N");
    enqFrom[portSouth]    <- mkPortRecv_Multiplexed_ReorderFirstNToLastN("mesh_interconnect_enq_N", 1, meshWidth);

    enqTo[portSouth]      <- mkPortSend_Multiplexed("mesh_interconnect_enq_S");
    enqFrom[portNorth]    <- mkPortRecv_Multiplexed_ReorderLastNToFirstN("mesh_interconnect_enq_S", 1, meshWidth);

    enqTo[portLocal]      <- mkPortSend_Multiplexed_Substr(enqToLocal);
    enqFrom[portLocal]    <- mkPortRecv_Multiplexed_Substr(enqFromLocal);
    
    creditTo[portEast]    <- mkPortSend_Multiplexed("mesh_interconnect_credit_E");
    creditFrom[portWest]  <- mkPortRecv_Multiplexed_ReorderLastToFirstEveryN("mesh_interconnect_credit_E", 1, meshWidth, meshHeight);

    creditTo[portWest]    <- mkPortSend_Multiplexed("mesh_interconnect_credit_W");
    creditFrom[portEast]  <- mkPortRecv_Multiplexed_ReorderFirstToLastEveryN("mesh_interconnect_credit_W", 1, meshWidth, meshHeight);

    creditTo[portNorth]   <- mkPortSend_Multiplexed("mesh_interconnect_credit_N");
    creditFrom[portSouth] <- mkPortRecv_Multiplexed_ReorderFirstNToLastN("mesh_interconnect_credit_N", 1, meshWidth);

    creditTo[portSouth]   <- mkPortSend_Multiplexed("mesh_interconnect_credit_S");
    creditFrom[portNorth] <- mkPortRecv_Multiplexed_ReorderLastNToFirstN("mesh_interconnect_credit_S", 1, meshWidth);

    creditTo[portLocal]   <- mkPortSend_Multiplexed_Substr(creditToLocal);
    creditFrom[portLocal] <- mkPortRecv_Multiplexed_Substr(creditFromLocal);

    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    // The actual virtual channels. Stored as logical FIFOs in a single Distributed RAM FIFO.
    MULTIPLEXED_STATE_POOL#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) virtualChannelsPool    <- mkMultiplexedStatePool(replicate(replicate(replicate(funcFIFO_Init))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(PORT_IDX))) routesPool             <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(VC_IDX)))   outputVCsPool          <- mkMultiplexedReg(replicate(replicate(replicate(tagged Invalid))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             usedVCsPool            <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputCreditsPool      <- mkMultiplexedReg(replicate(replicate(replicate(False))));
    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool))             outputNotFullsPool     <- mkMultiplexedReg(replicate(replicate(replicate(False))));

    // NOTE: The module uses a special local controller, as it has two sets of ports,
    // one set is MAX_NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

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

    Vector#(0, INSTANCE_CONTROL_IN#(NUM_STATIONS)) depports = newVector();

    Vector#(10, INSTANCE_CONTROL_OUT#(NUM_STATIONS)) outportsR = newVector();
    outportsR[0] = enqTo[portLocal].ctrl;
    outportsR[1] = creditTo[portLocal].ctrl;
    //
    // enqTo and creditTo for the router to router ports (not portLocal) are
    // not strictly necessary.  We will be doing a deq from each port, so there
    // will always be space available to write.
    //
    outportsR[2] <- mkConvertControllerAlwaysReady_OUT(enqTo[portNorth].ctrl);
    outportsR[3] <- mkConvertControllerAlwaysReady_OUT(enqTo[portSouth].ctrl);
    outportsR[4] <- mkConvertControllerAlwaysReady_OUT(enqTo[portEast].ctrl);
    outportsR[5] <- mkConvertControllerAlwaysReady_OUT(enqTo[portWest].ctrl);
    outportsR[6] <- mkConvertControllerAlwaysReady_OUT(creditTo[portNorth].ctrl);
    outportsR[7] <- mkConvertControllerAlwaysReady_OUT(creditTo[portSouth].ctrl);
    outportsR[8] <- mkConvertControllerAlwaysReady_OUT(creditTo[portEast].ctrl);
    outportsR[9] <- mkConvertControllerAlwaysReady_OUT(creditTo[portWest].ctrl);

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <-
        mkNamedLocalControllerWithActive("Mesh Network",
                                         0,
                                         False,
                                         inportsR, depports, outportsR);
    
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER#(NUM_STATIONS, Tuple2#(Vector#(NUM_PORTS, Maybe#(WINNER_INFO)), VC_STATE#(t_VC_FIFO))) stage3aCtrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, Tuple4#(Vector#(NUM_PORTS, Maybe#(WINNER_INFO)),
                                            VC_STATE#(t_VC_FIFO),
                                            Vector#(NUM_PORTS, Bool),
                                            Vector#(NUM_PORTS, Maybe#(MESH_MSG)))) stage3bCtrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple4#(VC_STATE#(t_VC_FIFO),
                                            VC_STATE#(Maybe#(PORT_IDX)), // routes
                                            VC_STATE#(Maybe#(VC_IDX)),   // outputVCs
                                            VC_STATE#(Bool)))            // usedVCs
        stage4Ctrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple4#(VC_STATE#(t_VC_FIFO),
                                            VC_STATE#(Maybe#(VC_IDX)),   // outputVCs
                                            VC_STATE#(Bool),             // usedVCs
                                            VC_STATE#(Maybe#(t_OUT_VC_REQ))))
        stage5Ctrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage6Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage7Ctrl <- mkStageController();

    Reg#(ROUTER_STATE) state <- mkReg(tagged INITIALIZING);


    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(NUM_STATIONS) eventGrant <- mkEventRecorder_Multiplexed(`EVENTS_MESH_GRANT_VC);
    EVENT_RECORDER_MULTIPLEXED#(NUM_STATIONS) eventGrantArb <- mkEventRecorder_Multiplexed(`EVENTS_MESH_GRANT_VC_ARB);


    // ******** Router Functions *********
    
    // The station routing table holds, for each station, the direction of
    // the next hop to all other stations.  The table stores one of north,
    // east, south and west.
    MULTIPLEXED_REG#(NUM_STATIONS, t_ROUTING_TABLE)
        stationRoutingTable <- mkMultiplexedRegU();

    if ((portNorth > 3) || (portEast > 3) || (portSouth > 3) || (portWest > 3))
    begin
        errorM("North, east, south and west port IDs don't fit in the routing table.");
    end
    
    // Pick a link for a packet currently in src heading for dst.
    function PORT_IDX route(STATION_ID src, STATION_ID dst);
        
        Reg#(t_ROUTING_TABLE) r_table = stationRoutingTable.getReg(src);

        if (src == dst)
        begin
            return portLocal;
        end
        else
        begin
            return zeroExtend(r_table[dst]);
        end

    endfunction


    //
    // Set the number of active nodes, given the topology.
    //
    Get#(Tuple2#(STATION_IID, Bool)) numActiveNodes <-
        mkTopologyParamStream(`TOPOLOGY_NET_MAX_NODE_IID);

    rule initNumActiveNodes (True);
        let n <- numActiveNodes.get();
        localCtrl.setMaxActiveInstance(tpl_1(n));
    endrule


    // ******* Rules *******

    Reg#(STATION_ID) initRTStationID <- mkReg(0);
    Get#(Tuple2#(t_ROUTING_TABLE, Bool)) routingTableInitStream <-
        mkTopologyParamStream(`TOPOLOGY_NET_ROUTING_TABLE);

    rule initRouterTable (True);
        match {.t, .last} <- routingTableInitStream.get();
        $display("ICN_MESH: %0d  0x%h", initRTStationID, t);

        Reg#(t_ROUTING_TABLE) r_table = stationRoutingTable.getReg(initRTStationID);
        r_table <= t;
        
        if (last)
        begin
            state <= RUNNING;
            $display("ICN_MESH: Go!");
        end
        initRTStationID <= initRTStationID + 1;
    endrule


    rule stage1_updateCreditsIn (state == RUNNING);
    
        // Get the next IID to simulate.
        let iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(iid);
        
        // Get our state from the pools.
        Reg#(VC_STATE#(Bool)) outputCredits  = outputCreditsPool.getReg(iid);
        Reg#(VC_STATE#(Bool)) outputNotFulls = outputNotFullsPool.getReg(iid);
        
        // Update our notions of our neighbor's credits.
        VC_STATE#(Bool) new_credits = newVector();
        VC_STATE#(Bool) new_not_fulls = newVector();
        
        for (Integer p = 0 ; p < numPorts; p = p + 1)
        begin

            // Get the credits for this neighbor.
            let m_credits <- creditFrom[p].receive(iid);

            if (m_credits matches tagged Valid .vcinfo)
            begin
                // New credit info has arrived.
                for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
                begin
                    match {.credits, .not_fulls} = unzip(vcinfo[ln]);
                    new_credits[p][ln] = credits;
                    new_not_fulls[p][ln] = not_fulls;
                end
            end
            else
            begin
                new_credits[p] = outputCredits[p];
                new_not_fulls[p] = outputNotFulls[p];
            end
        end
        
        debugLog.record_next_cycle(iid, $format("1: Update input credits"));
        
        // Do the actual update.
        outputCredits <= new_credits;
        outputNotFulls <= new_not_fulls;
        
        // Move on to the next stage.
        stage2Ctrl.ready(iid);
    
    endrule
    

    MULTIPLEXED_REG#(NUM_STATIONS,
                     Vector#(NUM_PORTS,
                             LOCAL_ARBITER_OPAQUE#(TMul#(NUM_LANES, VCS_PER_LANE))))
        stage2ArbiterStates <- mkMultiplexedReg(replicate(unpack(0)));

    (* conservative_implicit_conditions *)
    rule stage2_multiplexVCs (True);
        
        // Get the info from the previous stage.
        let iid <- stage2Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(t_VC_FIFO) virtualChannels <- virtualChannelsPool.extractState(iid);
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes         = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs      = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputNotFulls = outputNotFullsPool.getReg(iid);

        // Arbiter states for the current instance
        Reg#(Vector#(NUM_PORTS, LOCAL_ARBITER_OPAQUE#(TMul#(NUM_LANES, VCS_PER_LANE)))) arbiters = stage2ArbiterStates.getReg(iid);

        // This simulates the fact that the router only has one VC allocator.
        Bool vc_alloc_in_use = False;
        
        // This simulates the fact that only one VC from each port gets to even ATTEMPT
        // to send a message on the crossbar.
        Vector#(NUM_PORTS, Maybe#(WINNER_INFO)) vc_winners = replicate(tagged Invalid);


        //
        // isReadyVC --
        //   Is the input channel ready to send a flit to an output port?
        //   A ready incoming VC must:
        //     - Have incoming data in the VC
        //     - Have a valid outbound route (port)
        //     - Have a valid outbound virtual channel
        //     - Have space in the outbound virtual channel
        //
        function isReadyVC(Integer in_p, Integer ln, Integer vc);
            if (funcFIFO_notEmpty(virtualChannels[in_p][ln][vc]) &&&
                routes[in_p][ln][vc] matches tagged Valid .out_p &&&
                outputVCs[in_p][ln][vc] matches tagged Valid .out_vc &&&
                outputNotFulls[out_p][ln][out_vc])
            begin
                return True;
            end
            else
            begin
                return False;
            end
        endfunction


        debugLog.record(iid, $format("2: VCA Begin."));

        LANE_STATE#(Tuple2#(Integer, Integer)) identity_map = newVector();
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            identity_map[ln] = genWith(tuple2(ln));
        end

        //
        // Pick a set of winning incoming messages.  At most one winner per input
        // port will be chosen.  In this loop, "winners" may share a conflicting
        // output port.  Only one will proceed this simulated cycle, chosen in
        // the next stage.
        //

        Vector#(NUM_PORTS, LOCAL_ARBITER_OPAQUE#(TMul#(NUM_LANES, VCS_PER_LANE))) arbiters_out = ?;

        for (Integer in_p = 0; in_p < numPorts; in_p = in_p + 1)
        begin
            // Generate a linear bit vector across all lanes and channels within
            // a single port.  The vector indicates whether a message is ready
            // on each incoming channel.
            //
            // - concat(identity_map) linearizes the identity vectors.
            // - map operates over the linearized index of lanes and channels.
            // - uncurry() converts each lane/channel ID tuple to separate
            //   arguments, passed to isReadyVC.
            let ready_vcs = map(uncurry(isReadyVC(in_p)), concat(identity_map));

            // Pick a winner
            match {.grant_idx, .new_state} = localArbiterFunc(ready_vcs, False, arbiters[in_p]);
            arbiters_out[in_p] = new_state;

            if (grant_idx matches tagged Valid .idx)
            begin
                // Reverse map the winning index to a lane and channel
                match {.ln, .in_vc} = concat(identity_map)[idx];

                // Look up the output port and channel.  The lane is the same.
                let out_p = validValue(routes[in_p][ln][in_vc]);
                let out_vc = validValue(outputVCs[in_p][ln][in_vc]);

                let msg = funcFIFO_UGfirst(virtualChannels[in_p][ln][in_vc]);

                debugLog.record(iid, $format("2: VCA: PICK in port %s ln %0d vc %0d, out port %s: ",
                                             portShow(fromInteger(in_p)), ln, in_vc, portShow(out_p)) +
                                     fshow(msg));

                vc_winners[in_p] = tagged Valid WINNER_INFO 
                                   {
                                     lane: fromInteger(ln),
                                     inputVC: fromInteger(in_vc),
                                     outputPort: out_p, 
                                     outputVC: out_vc,
                                     message: msg
                                   };
            end
        end

        // Record the updated internal arbiter state.
        arbiters <= arbiters_out;

        stage3aCtrl.ready(iid, tuple2(vc_winners, virtualChannels));

    endrule


    MULTIPLEXED_REG#(NUM_STATIONS,
                     Vector#(NUM_PORTS,
                             LOCAL_ARBITER_OPAQUE#(NUM_PORTS)))
        stage3ArbiterStates <- mkMultiplexedReg(replicate(unpack(0)));

    (* conservative_implicit_conditions *)
    rule stage3a_crossbarArb (True);
        
        // Get the info from the previous stage.
        match {.iid, {.vc_winners, .virtualChannels}} <- stage3aCtrl.nextReadyInstance();
        
        // This is the vector of output messages that the virtual channels contend for.
        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msg_to = replicate(tagged Invalid);
        
        // Arbiters for the current instance
        Reg#(Vector#(NUM_PORTS, LOCAL_ARBITER_OPAQUE#(NUM_PORTS))) arbiters = stage3ArbiterStates.getReg(iid);

        debugLog.record(iid, $format("3: SA Begin."));


        //
        // Generate a request vector for each output port.  The outer index
        // is the output port.  Each output port has a request vector, indexed
        // by input port, of flits requesting routing from the input port to
        // the output port.
        //
        // Input port arbitration has already completed by this stage, so each
        // input port has at most one request.
        //

        Vector#(NUM_PORTS, Vector#(NUM_PORTS, Bool)) out_port_requests = newVector();

        // Returns true if input port is requesting output port
        function reqVecFromInPort(Integer out_p, Integer in_p);
            return vc_winners[in_p] matches tagged Valid .info &&&
                   info.outputPort == fromInteger(out_p) ? True : False;
        endfunction

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            out_port_requests[out_p] = genWith(reqVecFromInPort(out_p));
        end


        //
        // Find unique winners.  (At most one consumer of a given output port.)
        //

        Vector#(NUM_PORTS, Bool) in_port_used = replicate(False);
        Vector#(NUM_PORTS, LOCAL_ARBITER_OPAQUE#(NUM_PORTS)) arbiters_out = ?;

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            match {.grant_idx, .new_state} = localArbiterFunc(out_port_requests[out_p], False, arbiters[out_p]);
            arbiters_out[out_p] = new_state;

            if (grant_idx matches tagged Valid .in_p)
            begin
                let info = validValue(vc_winners[in_p]);

                in_port_used[in_p] = True;

                // Built a structure of outbound messages, indexed by the
                // output port.
                debugLog.record(iid, $format("3: SA: ARB %s req 0x%x grant %0d", portShow(fromInteger(out_p)), pack(out_port_requests[out_p]), in_p));
                debugLog.record(iid, $format("3: SA: FWD in port %s ln %0d vc %0d to out port %s ln %0d vc %0d: ",
                                             portShow(pack(in_p)), info.lane, info.inputVC,
                                             portShow(fromInteger(out_p)), info.lane, info.outputVC) +
                                     fshow(info.message));
                msg_to[out_p] = tagged Valid tuple3(info.lane, info.outputVC, info.message);
            end
        end

        // Record the updated internal arbiter state.
        arbiters <= arbiters_out;

        stage3bCtrl.ready(iid, tuple4(vc_winners, virtualChannels, in_port_used, msg_to));

    endrule


    (* conservative_implicit_conditions *)
    rule stage3b_crossbarSend (True);
        
        // Get the info from the previous stage.
        match {.iid, {.vc_winners, .virtualChannels, .in_port_used, .msg_to}} <- stage3bCtrl.nextReadyInstance();

        // Read our local state from the pools.
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes          = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs       = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             usedVCs         = usedVCsPool.getReg(iid);

        // Vectors to update our registers with.
        VC_STATE#(t_VC_FIFO) new_vcs = virtualChannels;
        VC_STATE#(Maybe#(PORT_IDX))   new_routes = routes;
        VC_STATE#(Bool)             new_used_vcs = usedVCs;
        VC_STATE#(Maybe#(VC_IDX)) new_output_vcs = outputVCs;

        //
        // Act on the routing crossbar decisions.  There are two loops here
        // for simpler hardware: one indexed by the input port and one by the
        // output port for.
        //

        for (Integer in_p = 0; in_p < numPorts; in_p = in_p + 1)
        begin
            if (in_port_used[in_p])
            begin
                let info = validValue(vc_winners[in_p]);

                // Deq incoming flit
                new_vcs[in_p][info.lane][info.inputVC] = funcFIFO_UGdeq(virtualChannels[in_p][info.lane][info.inputVC]);

                // End of packet?
                if (info.message matches tagged FLIT_BODY .body_info &&& body_info.isTail)
                begin
                    // Yes: tear down the route
                    new_routes[in_p][info.lane][info.inputVC] = tagged Invalid;
                    // Release virtual channel
                    new_output_vcs[in_p][info.lane][info.inputVC] = tagged Invalid;
                    debugLog.record(iid, $format("3: SA: TAIL on in port %s", portShow(fromInteger(in_p))));
                end
            end
        end

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            // End of packet?
            if (msg_to[out_p] matches tagged Valid {.lane, .output_vc, .message} &&&
                message matches tagged FLIT_BODY .body_info &&& body_info.isTail)
            begin
                // Release outbound virtual channel
                new_used_vcs[out_p][lane][output_vc] = False;
            end
        end

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            // Send out our output enqueues in each direction.
            enqTo[out_p].send(iid, msg_to[out_p]);
        end

        stage4Ctrl.ready(iid, tuple4(new_vcs, new_routes, new_output_vcs, new_used_vcs));

    endrule


    (* conservative_implicit_conditions *)
    rule stage4_route (True);
        // Get the info from the previous stage.
        match {.iid, .virtual_channels, .routes, .output_vcs, .used_vcs} <- stage4Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("4: Begin."));
        
        // Read our local state from the pools.
        Reg#(VC_STATE#(Bool))  outputCredits  = outputCreditsPool.getReg(iid);

        //
        // Request routes for head flits that are not yet assigned output ports.
        //

        VC_STATE#(Maybe#(PORT_IDX)) new_routes = newVector();

        function Maybe#(PORT_IDX) reqRoute(Tuple2#(t_VC_FIFO, Maybe#(PORT_IDX)) req);
            match {.vc_fifo, .cur_route} = req;

            // Is this a new head flit with no assigned output port?
            if (funcFIFO_UGfirst(vc_fifo) matches tagged FLIT_HEAD .msg &&&
                funcFIFO_notEmpty(vc_fifo) &&&
                ! isValid(cur_route))
            begin
                // Yes: Compute a new route
                return tagged Valid route(iid, msg.dst);
            end
            else
            begin
                // No: Keep current state
                return cur_route;
            end
        endfunction

        for (Integer in_p = 0; in_p < numPorts; in_p = in_p + 1)
        begin
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                new_routes[in_p][ln] = map(reqRoute,
                                           zip(virtual_channels[in_p][ln], routes[in_p][ln]));

                // This loop is only for printing debug messages
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                    // Detect new routes and print a message
                    if (! isValid(routes[in_p][ln][vc]) &&&
                        new_routes[in_p][ln][vc] matches tagged Valid .out_p)
                    begin
                        debugLog.record(iid, $format("4: RC: ROUTE in port %s ln %0d vc %0d, out port %s: ",
                                                     portShow(fromInteger(in_p)), ln, vc, portShow(out_p)) +
                                             fshow(funcFIFO_UGfirst(virtual_channels[in_p][ln][vc])));
                    end
                end
            end
        end


        //
        // Pick an outbound channel for a head flit that has an output port
        // assigned but no channel assignment.  Due to implementation cost,
        // only one outbound channel is chosen among all a node's output ports
        // per cycle.
        //
        // This code is completely independent of the output port picker loop
        // above.
        //

        //
        // outputVCAvail --
        //   Given the state of a single output virtual channel return whether
        //   the channel is available for allocation to a new packet.
        //
        function Bool outputVCAvail(Tuple2#(Bool, Bool) out_vc_info);
            match {.out_vc_in_use, .out_vc_credits} = out_vc_info;
            return ! out_vc_in_use && out_vc_credits;
        endfunction

        //
        // reqVC --
        //   Given the state of an incoming message on a single input channel
        //   determine whether an output channel should and could be allocated.
        //
        function Maybe#(t_OUT_VC_REQ) reqVC(Integer in_p,
                                            Integer ln,
                                            Tuple4#(Integer,
                                                    t_VC_FIFO,
                                                    Maybe#(PORT_IDX),
                                                    Maybe#(VC_IDX)) req);
            match {.in_vc, .vc_fifo, .out_p, .cur_out_vc} = req;

            if (funcFIFO_notEmpty(vc_fifo) &&&
                //  - The oldest entry in the FIFO is a flit head
                funcFIFO_UGfirst(vc_fifo) matches tagged FLIT_HEAD .info &&&
                //  - The incoming channel has an output port assigned
                out_p matches tagged Valid .rt &&&
                //  - The incoming channel has no output channel assigned
                ! isValid(cur_out_vc) &&&
                //  - An output channel is available
                findIndex(outputVCAvail, zip(used_vcs[rt][ln], outputCredits[rt][ln])) matches tagged Valid .out_vc)
            begin
                // Yes: Request an output channel.
                return tagged Valid tuple5(fromInteger(in_p),
                                           fromInteger(ln),
                                           fromInteger(in_vc),
                                           rt,
                                           zeroExtend(unpack(pack(out_vc))));
            end
            else
            begin
                // No request
                return tagged Invalid;
            end
        endfunction

        // Generate the request vector across all input channels
        VC_STATE#(Maybe#(t_OUT_VC_REQ)) new_out_vc_req = newVector();

        for (Integer in_p = 0; in_p < numPorts; in_p = in_p + 1)
        begin
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                new_out_vc_req[in_p][ln] = map(reqVC(in_p, ln),
                                               zip4(genVector(),
                                                    virtual_channels[in_p][ln],
                                                    routes[in_p][ln],
                                                    output_vcs[in_p][ln]));

                // This loop is only for printing debug messages
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                    if (new_out_vc_req[in_p][ln][vc] matches tagged Valid .req)
                    begin
                        debugLog.record(iid, $format("4: RC: REQ OUT VC in port %s ln %0d vc %0d, out port %s vc %0d: ",
                                                     portShow(fromInteger(in_p)), ln, vc, portShow(tpl_4(req)), tpl_5(req)) +
                                             fshow(funcFIFO_UGfirst(virtual_channels[in_p][ln][vc])));
                    end
                end
            end
        end

        //
        // Update global state.
        //
        routesPool.getReg(iid) <= new_routes;

        stage5Ctrl.ready(iid, tuple4(virtual_channels,
                                     output_vcs,
                                     used_vcs,
                                     new_out_vc_req));
    endrule


    //
    // stage5 completes the routing decisions requested in stage 4.  It
    // consumes requests for output channel mapping from stage 4, arbitrates
    // among them, and picks a single winner.
    //
    // With arbitration, it was too much to fit into a single stage.
    // 

    MULTIPLEXED_REG#(NUM_STATIONS,
                     LOCAL_ARBITER_OPAQUE#(TMul#(NUM_PORTS,
                                                 TMul#(NUM_LANES, VCS_PER_LANE))))
        stage5arbiter <- mkMultiplexedReg(unpack(0));

    (* conservative_implicit_conditions *)
    rule stage5_arbOutChannel (True);
        // Get the info from the previous stage.
        match {.iid, .virtual_channels, .output_vcs, .used_vcs, .new_out_vc_req} <- stage5Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("5: Begin."));

        // Arbiter for the current instance
        Reg#(LOCAL_ARBITER_OPAQUE#(TMul#(NUM_PORTS,
                                         TMul#(NUM_LANES,
                                               VCS_PER_LANE)))) arbiter = stage5arbiter.getReg(iid);

        //
        // Now we have a set of requests in new_out_vc_req.  Pick a winner.
        //
        let new_output_vcs = output_vcs;
        let new_used_vcs = used_vcs;

        // Map the multi-level vector to a single level
        let linear_vc_req = concat(concat(new_out_vc_req));
        // Map the vector to a boolean request vector
        let linear_vc_req_vec = map(isValid, linear_vc_req);

        //
        // Pick a winner among the valid entries (entries with requests).
        // Arbitration here is messy.  In general, we use round robin arbitration.
        // Because of the traffic patterns, round robin can be unfair.  The
        // typical configuration has only one memory controller, so all traffic
        // matches one of two possibilities.  Either it is fanning in through
        // multiple ports, heading out a single port to the memory controller or
        // it is coming in on a single port from the memory controller and fanning
        // out.  The router often gets in a state where it alternates between
        // arbitration of these two classes.  This alternation can cause the
        // round robin pointer always to be set immediately after the port
        // coming from memory.  To alleviate this, we don't change the round
        // robin pointer if only a single request is active.
        //

        let fixed = (countElem(True, linear_vc_req_vec) <= 1);
        match {.grant_idx, .new_state} = localArbiterFunc(linear_vc_req_vec, fixed, arbiter);
        arbiter <= new_state;

        if (grant_idx matches tagged Valid .idx)
        begin
            match {.in_p, .ln, .in_vc, .out_p, .out_vc} = validValue(linear_vc_req[idx]);
            new_output_vcs[in_p][ln][in_vc] = tagged Valid out_vc;
            new_used_vcs[out_p][ln][out_vc] = True;

            // Pack the event data into more readable chunks.
            Bit#(3) evt_in_p = zeroExtend(in_p);
            Bit#(3) evt_ln = zeroExtend(ln);
            Bit#(3) evt_out_p = zeroExtend(out_p);
            Bit#(2) evt_in_vc = zeroExtend(in_vc);
            Bit#(2) evt_out_vc = zeroExtend(out_vc);

            // flit_src/dst are large enough for 128 node simulations.  If we
            // grow larger some bits can be taken from lanes and channels above.
            Bit#(8) flit_src = 0;
            Bit#(8) flit_dst = 0;
            Bit#(1) flit_isHead = 0;
            Bit#(1) flit_isTail = 0;
            Bit#(1) flit_isStore = 0;
            let flit = funcFIFO_UGfirst(virtual_channels[in_p][ln][in_vc]);
            if (flit matches tagged FLIT_HEAD .f)
            begin
                flit_isHead = 1;
                flit_isStore = pack(f.isStore);
                flit_src = zeroExtend(f.src);
                flit_dst = zeroExtend(f.dst);
            end
            else if (flit matches tagged FLIT_BODY .f)
            begin
                flit_isTail = pack(f.isTail);
            end

            let evt_data = { flit_isStore, flit_isTail, flit_isHead, flit_src, flit_dst,
                             evt_in_p, evt_ln, evt_in_vc, evt_out_p, evt_out_vc };
            eventGrant.recordEvent(iid, tagged Valid zeroExtend(evt_data));

            Bit#(8) evt_idx = zeroExtend(pack(idx));
            Bit#(24) evt_req_vec = zeroExtend(pack(linear_vc_req_vec));
            let evt_arb = { evt_idx, evt_req_vec };
            eventGrantArb.recordEvent(iid, tagged Valid zeroExtend(evt_arb));

            debugLog.record(iid, $format("5: RC: ARB 0x%x grant %0d", pack(linear_vc_req_vec), idx));
            debugLog.record(iid, $format("5: RC: GRANT OUT VC in port %s ln %0d vc %0d, out port %s vc %0d: ",
                                         portShow(in_p), ln, in_vc, portShow(out_p), out_vc) +
                                 fshow(funcFIFO_UGfirst(virtual_channels[in_p][ln][in_vc])));
        end
        else
        begin
            eventGrant.recordEvent(iid, tagged Invalid);
            eventGrantArb.recordEvent(iid, tagged Invalid);
        end


        //
        // Update global state.
        //
        outputVCsPool.getReg(iid) <= new_output_vcs;
        usedVCsPool.getReg(iid) <= new_used_vcs;

        stage6Ctrl.ready(iid, virtual_channels);
    endrule


    rule stage6_enqs (True);

        // Get the current IID from the previous stage.    
        match {.iid, .virtualChannels} <- stage6Ctrl.nextReadyInstance();
       
        VC_STATE#(t_VC_FIFO) new_vcs = virtualChannels;

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

                    new_flit = tagged FLIT_HEAD new_info;

                    debugLog.record(iid, $format("6: BW: ENTER in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(new_flit));
                end
                else
                begin
                    debugLog.record(iid, $format("6: BW: ENQ in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(new_flit));
                end
                new_vcs[p][ln][vc] = funcFIFO_UGenq(virtualChannels[p][ln][vc], new_flit);
            end
        end

        stage7Ctrl.ready(iid, new_vcs);

    endrule

    (* conservative_implicit_conditions, descending_urgency="stage7_creditsOut, stage6_enqs, stage5_arbOutChannel, stage4_route, stage3b_crossbarSend, stage3a_crossbarArb, stage2_multiplexVCs, stage1_updateCreditsIn" *)
    rule stage7_creditsOut (True);
    
        match {.iid, .virtualChannels} <- stage7Ctrl.nextReadyInstance();

        debugLog.record(iid, $format("7: Calculating output credits."));

        //
        // Compute credits and not full flags for the next cycle and write them
        // to A-Ports.
        //

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            VC_CREDIT_INFO creds = newVector();
            
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                //
                // To be given credit to start a packet, the receiver must have
                // space to buffer the entire message so as not to block other
                // channels sharing the port.  For now, we simulate this by
                // requiring that the FIFO be empty.
                //
                function Bool fifoHasCredit(t_VC_FIFO fifo) = !funcFIFO_notEmpty(fifo);
                let have_credit = map(fifoHasCredit, virtualChannels[p][ln]);

                //
                // To be "not full" a FIFO must have at least two free slots.
                // One slot for a new message and one slot in case a flit was
                // sent this cycle.
                //
                function Bool fifoHasFreeSlots(t_VC_FIFO fifo) =
                    (funcFIFO_numBusySlots(fifo) < fromInteger((valueOf(NUM_VC_FIFO_ENTRIES) - 1)));
                let not_full = map(fifoHasFreeSlots, virtualChannels[p][ln]);
            
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                //
                // For reasons not yet understood, performance of the
                // target network for networks larger than 3x3 is poor
                // when credits require that a FIFO by empty.  For now
                // we allow a new packet to start when a FIFO is not
                // full.
                //
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                //The correct code should be:
                // creds[ln] = zip(have_credit, not_full);
                creds[ln] = zip(not_full, not_full);
            end
        
            creditTo[p].send(iid, tagged Valid creds);
        end
        
        // End of model cycle (path 1)
        localCtrl.endModelCycle(iid, 0);
        virtualChannelsPool.insertState(iid, virtualChannels);
        
    endrule

endmodule
