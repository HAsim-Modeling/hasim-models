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


import Vector::*;
import FIFO::*;
import FIFOF::*;
import FShow::*;
import GetPut::*;
import Connectable::*;
import DefaultValue::*;


// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/mem_services.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/hasim_memory_controller.bsh"
`include "awb/provides/hasim_chip_topology.bsh"
`include "awb/provides/hasim_interconnect_common.bsh"


// ****** Generated files ******

`include "awb/dict/EVENTS_MESH.bsh"
`include "awb/dict/TOPOLOGY.bsh"
`include "awb/dict/PARAMS_HASIM_INTERCONNECT.bsh"
`include "awb/dict/VDEV_SCRATCH.bsh"


typedef STATION_IID MESH_COORD; // Since coordinates can vary dynamically, we need to be able to hold the worst case in each direction, which is a ring network.
typedef TLog#(NUM_STATIONS) MESH_COORD_SZ;

typedef OCN_FLIT MESH_FLIT;
typedef OCN_MSG  MESH_MSG;

typedef 5 NUM_PORTS;
typedef Bit#(TLog#(NUM_PORTS)) PORT_IDX;

// Packets per channel buffer
typedef 2 NUM_VC_FIFO_PACKETS;
// Buffer slots per channel buffer
typedef TMul#(MAX_FLITS_PER_PACKET, NUM_VC_FIFO_PACKETS) NUM_VC_FIFO_ENTRIES;

//
// Routing request.  For now the request is trivial: just an outbound port.
// The reqeust is stored as a struct to make it easy to change later.
//
typedef struct
{
    PORT_IDX outPort;
}
MESH_ROUTE_REQ
    deriving (Eq, Bits);

//
// Each virtual channel has a FIFO buffer.  This data structure caches the
// minimal state to make routing decisions.  The actual FIFO state is stored
// in a separate buffer and accessed only when needed.
//
typedef TLog#(NUM_VC_FIFO_ENTRIES) VC_FIFO_ENTRY_IDX_SZ;
typedef Bit#(VC_FIFO_ENTRY_IDX_SZ) VC_FIFO_ENTRY_IDX;
typedef struct
{
    Bool notEmpty;
    Maybe#(MESH_ROUTE_REQ) routeReq;
}
VC_FIFO_STATE
    deriving (Eq, Bits);

instance DefaultValue#(VC_FIFO_STATE);
    defaultValue = VC_FIFO_STATE { notEmpty: False,
                                   routeReq: tagged Invalid };
endinstance


//
// The order of port numbering is critical!  North, east, south and west must be
// the first 4 entries in order to fit in the stationRoutingTable.
//
PORT_IDX portNorth  = 0;
PORT_IDX portEast   = 1;
PORT_IDX portSouth  = 2;
PORT_IDX portWest   = 3;
PORT_IDX portLocal  = 4;

Integer numPorts = valueOf(NUM_PORTS);

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

instance FShow#(MESH_ROUTE_REQ);
    function Fmt fshow(MESH_ROUTE_REQ req);
        return $format("out port %s", portShow(req.outPort));
    endfunction
endinstance


//
// Vectors of lanes and channels.
//
typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) LANE_STATE#(parameter type t_DATA);
typedef Vector#(NUM_PORTS, LANE_STATE#(t_DATA)) VC_STATE#(parameter type t_DATA);

// Helper function to initialize a vector of VCs.
function VC_STATE#(t_DATA) initVCState(t_DATA v) = replicate(replicate(replicate(v)));

// Flattened representation of a VC_STATE.
typedef Vector#(TMul#(NUM_PORTS, TMul#(NUM_LANES, VCS_PER_LANE)),
                t_DATA) VC_STATE_FLAT#(parameter type t_DATA);

function VC_STATE_FLAT#(t_DATA) flattenVCState(VC_STATE#(t_DATA) v) =
    concat(concat(v));


// Lane, virtual channel and some Data.  This is often used associated with
// a port.
typedef struct
{
    LANE_IDX lane;
    VC_IDX vc;
    t_DATA val;
}
LANE_VC_DATA#(type t_DATA);


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
}
WINNER_INFO 
    deriving (Eq, Bits);


//
// Mesh interconnection network timing.
//
module [HASIM_MODULE] mkInterconnect
    // interface:
    ()
    provisos (Alias#(VC_FIFO_STATE, t_VC_FIFO),
              Alias#(Tuple5#(PORT_IDX, LANE_IDX, VC_IDX, PORT_IDX, VC_IDX), t_OUT_VC_REQ),
              Alias#(Vector#(NUM_STATIONS, Bit#(2)), t_ROUTING_TABLE));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_STATIONS) debugLog <- mkTIMEPDebugFile_Multiplexed("interconnect_mesh.out");

    // Initialization from software
    ReadOnly#(STATION_IID) meshWidth <- mkTopologyParamReg(`TOPOLOGY_NET_MESH_WIDTH);
    ReadOnly#(STATION_IID) meshHeight <- mkTopologyParamReg(`TOPOLOGY_NET_MESH_HEIGHT);


    // ******** Ports *******

    // Queues to/from cores
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)       enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)       enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_MSG) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_MSG) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

    // Queues to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_SEND_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, OCN_MSG)       enqToMemCtrl      <- mkPortSend_Multiplexed("ocn_to_memctrl_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, OCN_MSG)       enqFromMemCtrl    <- mkPortRecv_Multiplexed("memctrl_to_ocn_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, VC_CREDIT_MSG) creditToMemCtrl   <- mkPortSend_Multiplexed("ocn_to_memctrl_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, VC_CREDIT_MSG) creditFromMemCtrl <- mkPortRecv_Multiplexed("memctrl_to_ocn_credit", 1);

    // Local "NULL" connections when no CPU or memory controller is attached
    // to a local port.  Not knowing in advance how many ports may be NULL,
    // allocate the maximum number.  The only cost is an index bit.
    PORT_SEND_MULTIPLEXED#(NUM_STATIONS, OCN_MSG)       enqToNull      <- mkPortSend_Multiplexed_NULL();
    PORT_RECV_MULTIPLEXED#(NUM_STATIONS, OCN_MSG)       enqFromNull    <- mkPortRecv_Multiplexed_NULL();
    PORT_SEND_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_MSG) creditToNull   <- mkPortSend_Multiplexed_NULL();
    PORT_RECV_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_MSG) creditFromNull <- mkPortRecv_Multiplexed_NULL();

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
    
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, MESH_MSG))      enqTo      = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, MESH_MSG))      enqFrom    = newVector();
    Vector#(NUM_PORTS, PORT_SEND_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_MSG)) creditTo   = newVector();
    Vector#(NUM_PORTS, PORT_RECV_MULTIPLEXED#(NUM_STATIONS, VC_CREDIT_MSG)) creditFrom = newVector();

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

    // Virtual channel buffer management.  Meta-data for each channel FIFO
    // is stored here.
    MULTIPLEXED_STATE_POOL#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) virtualChannelsPool <-
        mkMultiplexedStatePool(initVCState(defaultValue));

    // Values stored in virtual channel buffers.
    VC_FIFOS#(NUM_STATIONS) vcBufferEntries <- mkVCBufferStorage(debugLog);

    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(MESH_ROUTE_REQ))) routesPool <-
        mkMultiplexedReg(initVCState(tagged Invalid));

    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Maybe#(VC_IDX))) outputVCsPool <-
        mkMultiplexedReg(initVCState(tagged Invalid));

    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(Bool)) usedVCsPool <-
        mkMultiplexedReg(initVCState(False));

    MULTIPLEXED_REG#(NUM_STATIONS, VC_STATE#(VC_CREDIT_CNT)) outputCreditsPool <-
        mkMultiplexedReg(initVCState(fromInteger(valueOf(0))));

    MULTIPLEXED_REG#(NUM_STATIONS, Bool) creditInitializedPool <-
        mkMultiplexedReg(False);

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
    
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(VC_CREDIT_CNT))
        stage2Ctrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple3#(VC_STATE#(VC_CREDIT_CNT),
                                            Vector#(NUM_PORTS, Maybe#(WINNER_INFO)),
                                            VC_STATE#(t_VC_FIFO)))
        stage3aCtrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple5#(VC_STATE#(VC_CREDIT_CNT),
                                            Vector#(NUM_PORTS, Maybe#(WINNER_INFO)),
                                            VC_STATE#(t_VC_FIFO),
                                            Vector#(NUM_PORTS, Maybe#(Tuple2#(PORT_IDX, VC_IDX))),
                                            Bool))
        stage3bCtrl <- mkBufferedStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple6#(VC_STATE#(VC_CREDIT_CNT),
                                            VC_STATE#(t_VC_FIFO),
                                            Vector#(NUM_PORTS, Maybe#(MESH_MSG)),
                                            VC_STATE#(Maybe#(MESH_ROUTE_REQ)), // routes
                                            VC_STATE#(Maybe#(VC_IDX)),   // outputVCs
                                            VC_STATE#(Bool)))            // usedVCs
        stage3cCtrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple5#(VC_STATE#(VC_CREDIT_CNT),
                                            VC_STATE#(t_VC_FIFO),
                                            VC_STATE#(Maybe#(MESH_ROUTE_REQ)), // routes
                                            VC_STATE#(Maybe#(VC_IDX)),   // outputVCs
                                            VC_STATE#(Bool)))            // usedVCs
        stage4Ctrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Tuple5#(VC_STATE#(VC_CREDIT_CNT),
                                            VC_STATE#(t_VC_FIFO),
                                            VC_STATE#(Maybe#(VC_IDX)),   // outputVCs
                                            VC_STATE#(Bool),             // usedVCs
                                            VC_STATE#(Maybe#(t_OUT_VC_REQ))))
        stage5Ctrl <- mkStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage6Ctrl <- mkBufferedStageController();

    STAGE_CONTROLLER#(NUM_STATIONS, Bool) stage7Ctrl <- mkBufferedStageController();

    Reg#(ROUTER_STATE) state <- mkReg(tagged INITIALIZING);


    //
    // identityMap is a map from the the vector representation of each virtual
    // channel to the index of the lane and channel pair.  The identity
    // map can be fed into vector mapping functions.
    //
    LANE_STATE#(Tuple2#(Integer, Integer)) identityMap = newVector();
    for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
    begin
        identityMap[ln] = genWith(tuple2(ln));
    end


    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(NUM_STATIONS) eventGrant <- mkEventRecorder_Multiplexed(`EVENTS_MESH_GRANT_VC);
    EVENT_RECORDER_MULTIPLEXED#(NUM_STATIONS) eventGrantArb <- mkEventRecorder_Multiplexed(`EVENTS_MESH_GRANT_VC_ARB);


    // ******** Router Functions *********

    // The station routing table holds, for each station, the direction of
    // the next hop to all other stations.  The table stores one of north,
    // east, south and west.
    MEMORY_MULTI_READ_IFC#(2, STATION_ID, t_ROUTING_TABLE)
        stationRoutingTable <- mkBRAMPseudoMultiRead();

    if ((portNorth > 3) || (portEast > 3) || (portSouth > 3) || (portWest > 3))
    begin
        errorM("North, east, south and west port IDs don't fit in the routing table.");
    end
    
    // Pick a link for a packet currently in src heading for dst.
    function MESH_ROUTE_REQ route(STATION_ID src, STATION_ID dst,
                                  t_ROUTING_TABLE tbl);
        MESH_ROUTE_REQ r;

        if (src == dst)
        begin
            r.outPort = portLocal;
        end
        else
        begin
            r.outPort = zeroExtend(tbl[dst]);
        end

        return r;
    endfunction


    //
    // Set the number of active nodes, given the topology.
    //
    Get#(Maybe#(STATION_IID)) numActiveNodes <-
        mkTopologyParamStream(`TOPOLOGY_NET_MAX_NODE_IID);

    rule initNumActiveNodes (True);
        let m_n_nodes <- numActiveNodes.get();
        if (m_n_nodes matches tagged Valid .n)
        begin
            localCtrl.setMaxActiveInstance(n);
        end
    endrule


    // ******* Rules *******

    Reg#(STATION_ID) initRTStationID <- mkReg(0);
    Get#(Maybe#(t_ROUTING_TABLE)) routingTableInitStream <-
        mkTopologyParamStream(`TOPOLOGY_NET_ROUTING_TABLE);

    rule initRouterTable (True);
        let m_entry <- routingTableInitStream.get();
        if (m_entry matches tagged Valid .t)
        begin
            $display("ICN_MESH: %0d  0x%h", initRTStationID, t);

            stationRoutingTable.write(initRTStationID, t);
            initRTStationID <= initRTStationID + 1;
        end
        else
        begin
            state <= RUNNING;
            $display("ICN_MESH: Go!");
        end
    endrule


    rule stage1_updateCreditsIn (state == RUNNING);
        // Get the next IID to simulate.
        let iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(iid);
        
        debugLog.record(iid, $format("1: Begin."));

        // Get our state from the pools.
        Reg#(VC_STATE#(VC_CREDIT_CNT)) outputCredits = outputCreditsPool.getReg(iid);
        
        VC_STATE#(VC_CREDIT_CNT) upd_credits = outputCredits;
        
        for (Integer p = 0 ; p < numPorts; p = p + 1)
        begin
            // Check for new credits from this neighbor.
            let m_credits <- creditFrom[p].receive(iid);

            if (m_credits matches tagged Valid .new_credits)
            begin
                // New credit info has arrived.
                for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
                begin
                    // Add credits to the output credit counters
                    upd_credits[p][ln] = map(uncurry(boundedPlus),
                                             zip(upd_credits[p][ln], new_credits[ln]));

                    for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                    begin
                        if (new_credits[ln][vc] != 0)
                        begin
                            debugLog.record(iid, $format("1: %0d new credit port %s ln %0d vc %0d, now %0d",
                                                         new_credits[ln][vc], portShow(fromInteger(p)), ln, vc, upd_credits[p][ln]));

                        end
                    end
                end
            end
        end
        
        // Move on to the next stage.
        stage2Ctrl.ready(iid, upd_credits);
    endrule
    

    MULTIPLEXED_REG#(NUM_STATIONS,
                     Vector#(NUM_PORTS,
                             LOCAL_ARBITER_OPAQUE#(TMul#(NUM_LANES, VCS_PER_LANE))))
        stage2ArbiterStates <- mkMultiplexedReg(replicate(unpack(0)));

    (* conservative_implicit_conditions *)
    rule stage2_multiplexVCs (True);
        
        // Get the info from the previous stage.
        match {.iid, .output_credits} <- stage2Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(t_VC_FIFO) virtualChannels <- virtualChannelsPool.extractState(iid);
        Reg#(VC_STATE#(Maybe#(MESH_ROUTE_REQ))) routes = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX))) outputVCs = outputVCsPool.getReg(iid);

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
        //
        //   Note that being ready does not include a check for credits or
        //   space in the outbound buffer.  The existence of a valid
        //   outbound route and VC is proof that a credit for the packet
        //   existed when the outbound route was built.
        //
        function isReadyVC(Integer in_p, Integer ln, Integer vc);
            if (virtualChannels[in_p][ln][vc].notEmpty &&&
                routes[in_p][ln][vc] matches tagged Valid .out_p &&&
                outputVCs[in_p][ln][vc] matches tagged Valid .out_vc)
            begin
                return True;
            end
            else
            begin
                return False;
            end
        endfunction


        debugLog.record(iid, $format("2: VCA Begin."));

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
            // - concat(identityMap) linearizes the identity vectors.
            // - map operates over the linearized index of lanes and channels.
            // - uncurry() converts each lane/channel ID tuple to separate
            //   arguments, passed to isReadyVC.
            let ready_vcs = map(uncurry(isReadyVC(in_p)), concat(identityMap));

            // Pick a winner
            match {.grant_idx, .new_state} = localArbiterFunc(ready_vcs, False, arbiters[in_p]);
            arbiters_out[in_p] = new_state;

            if (grant_idx matches tagged Valid .idx)
            begin
                // Reverse map the winning index to a lane and channel
                match {.ln, .in_vc} = concat(identityMap)[idx];

                // Look up the output port and channel.  The lane is the same.
                let rt = validValue(routes[in_p][ln][in_vc]);
                let out_vc = validValue(outputVCs[in_p][ln][in_vc]);

                debugLog.record(iid, $format("2: VCA: PICK in port %s ln %0d vc %0d, ",
                                             portShow(fromInteger(in_p)), ln, in_vc) + fshow(rt));

                vc_winners[in_p] = tagged Valid WINNER_INFO 
                                   {
                                     lane: fromInteger(ln),
                                     inputVC: fromInteger(in_vc),
                                     outputPort: rt.outPort,
                                     outputVC: out_vc
                                   };
            end
        end

        // Record the updated internal arbiter state.
        arbiters <= arbiters_out;

        stage3aCtrl.ready(iid, tuple3(output_credits, vc_winners, virtualChannels));

    endrule


    MULTIPLEXED_REG#(NUM_STATIONS,
                     Vector#(NUM_PORTS,
                             LOCAL_ARBITER_OPAQUE#(NUM_PORTS)))
        stage3ArbiterStates <- mkMultiplexedReg(replicate(unpack(0)));

    (* conservative_implicit_conditions *)
    rule stage3a_crossbarArb (True);
        
        // Get the info from the previous stage.
        match {.iid, {.output_credits, .vc_winners, .virtualChannels}} <- stage3aCtrl.nextReadyInstance();
        
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

        Vector#(NUM_PORTS, LOCAL_ARBITER_OPAQUE#(NUM_PORTS)) arbiters_out = ?;

        // By the end of the next loop, vc_arb_winners will have only the
        // vc_winners that also won port arbitration.
        Vector#(NUM_PORTS, Maybe#(WINNER_INFO)) vc_arb_winners = replicate(tagged Invalid);
        Vector#(NUM_PORTS, Maybe#(Tuple2#(LANE_IDX, VC_IDX))) in_fwds =
            replicate(tagged Invalid);
        Vector#(NUM_PORTS, Maybe#(Tuple2#(PORT_IDX, VC_IDX))) out_fwds =
            replicate(tagged Invalid);

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            match {.grant_idx, .new_state} = localArbiterFunc(out_port_requests[out_p], False, arbiters[out_p]);
            arbiters_out[out_p] = new_state;

            if (grant_idx matches tagged Valid .in_p)
            begin
                vc_arb_winners[in_p] = vc_winners[in_p];
                let info = validValue(vc_winners[in_p]);

                // Prepare to forward incoming flit.  Generate a vector of
                // requests to the virtual channel buffer manager.
                in_fwds[in_p] = tagged Valid tuple2(info.lane, info.inputVC);
                out_fwds[out_p] = tagged Valid tuple2(pack(in_p), info.outputVC);

                debugLog.record(iid, $format("3: SA: ARB %s req 0x%x grant %0d", portShow(fromInteger(out_p)), pack(out_port_requests[out_p]), in_p));
                debugLog.record(iid, $format("3: SA: FWD in port %s ln %0d vc %0d to out port %s ln %0d vc %0d: ",
                                             portShow(pack(in_p)), info.lane, info.inputVC,
                                             portShow(fromInteger(out_p)), info.lane, info.outputVC));
            end
        end

        // Record the updated internal arbiter state.
        arbiters <= arbiters_out;

        // Request update of virtual channel buffers.
        vcBufferEntries.deqAndReadReq(iid, in_fwds);

        // Load the routing table if it will be needed in the next stage.
        Bool did_deq = any(isValid, in_fwds);
        if (did_deq)
        begin
            stationRoutingTable.readPorts[0].readReq(iid);
        end

        stage3bCtrl.ready(iid, tuple5(output_credits, vc_arb_winners, virtualChannels, out_fwds, did_deq));
    endrule


    rule stage3b_crossbarSend0 (True);
        
        // Get the info from the previous stage.
        match {.iid, {.output_credits, .vc_arb_winners, .virtualChannels, .out_fwds, .did_deq}} <- stage3bCtrl.nextReadyInstance();

        // Read our local state from the pools.
        Reg#(VC_STATE#(Maybe#(MESH_ROUTE_REQ))) routes = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX))) outputVCs = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))           usedVCs   = usedVCsPool.getReg(iid);
        Reg#(Bool) creditInitialized = creditInitializedPool.getReg(iid);

        // Vectors to update our registers with.
        VC_STATE#(Maybe#(MESH_ROUTE_REQ)) new_routes = routes;
        VC_STATE#(Bool)           new_used_vcs = usedVCs;
        VC_STATE#(Maybe#(VC_IDX)) new_output_vcs = outputVCs;

        // This is the vector of output messages that will be sent this cycle.
        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msg_to = replicate(tagged Invalid);

        // Container for updated VC state
        VC_STATE#(t_VC_FIFO) new_vcs = virtualChannels;

        // Receive update from the channel buffer manager.
        let m_deq_rsp <- vcBufferEntries.deqAndReadRsp(iid);
        t_ROUTING_TABLE rt_tbl = ?;
        if (did_deq)
        begin
            rt_tbl <- stationRoutingTable.readPorts[0].readRsp();
        end

        //
        // Update input port state following forwarding of first entry in
        // one or more channel buffers.
        //
        for (Integer in_p = 0 ; in_p < numPorts; in_p = in_p + 1)
        begin
            // New sender credit tracking
            VC_CREDIT_MSG sender_credits = replicate(replicate(0));

            // Was a forward and deq requested from in_p?
            if (m_deq_rsp[in_p] matches tagged Valid {{.ln, .vc, .first_flit}, .m_next})
            begin
                debugLog.record(iid, $format("3b: DEQ in port %s ln %0d vc %0d, ", portShow(fromInteger(in_p)), ln, vc) + fshow(first_flit));

                // End of packet?
                if (first_flit matches tagged FLIT_BODY .body_info &&&
                    body_info.isTail)
                begin
                    // Yes: tear down the route
                    new_routes[in_p][ln][vc] = tagged Invalid;
                    // Release virtual channel
                    new_output_vcs[in_p][ln][vc] = tagged Invalid;

                    // Give sender credit for another packet
                    sender_credits[ln][vc] = 1;

                    debugLog.record(iid, $format("3b: Send credit (tail) on in port %s ln %0d vc %0d", portShow(fromInteger(in_p)), ln, vc));
                end

                // Does the channel buffer have any more flits?
                new_vcs[in_p][ln][vc].notEmpty = isValid(m_next);

                if (m_next matches tagged Valid .next_flit)
                begin
                    // If the next flit is a header then update the routing
                    // request.
                    if (next_flit matches tagged FLIT_HEAD .msg)
                    begin
                        let rt = route(iid, msg.dst, rt_tbl);
                        new_vcs[in_p][ln][vc].routeReq = tagged Valid rt;
                        debugLog.record(iid, $format("3b: ") + fshow(next_flit) + $format(" now first, in port %s ln %0d vc %0d, req ", portShow(fromInteger(in_p)), ln, vc) + fshow(rt));
                    end
                    else
                    begin
                        new_vcs[in_p][ln][vc].routeReq = tagged Invalid;
                        debugLog.record(iid, $format("3b: ") + fshow(next_flit) + $format(" now first, in port %s ln %0d vc %0d", portShow(fromInteger(in_p)), ln, vc));
                    end
                end
                else
                begin
                    new_vcs[in_p][ln][vc].routeReq = tagged Invalid;
                    debugLog.record(iid, $format("3b: in port %s ln %0d vc %0d now empty", portShow(fromInteger(in_p)), ln, vc));
                end
            end

            //
            // Finish sender credit computation.  The first pass only must send
            // initial credits everywhere.
            if (! creditInitialized)
            begin
                sender_credits = replicate(replicate(fromInteger(valueOf(NUM_VC_FIFO_PACKETS))));
            end

            creditTo[in_p].send(iid, tagged Valid sender_credits);
        end

        creditInitializedPool.getReg(iid) <= True;

        //
        // Generate the output flits that are forwarded from input ports.
        //
        for (Integer out_p = 0 ; out_p < numPorts; out_p = out_p + 1)
        begin
            if (out_fwds[out_p] matches tagged Valid {.in_p, .out_vc})
            begin
                // The out_fwds vector claims to be forwarding a value from
                // input port in_p.  If no forwarding of in_p is present
                // in m_deq_rsp there is a bug in the code above.
                if (! isValid(m_deq_rsp[in_p]))
                begin
                    $display("icn-mesh.bsv: m_deq_rsp[in_p] is not valid!");
                    $finish(1);
                end

                match {{.in_ln, .in_vc, .flit}, .m_next} = validValue(m_deq_rsp[in_p]);
                msg_to[out_p] = tagged Valid tuple3(in_ln,
                                                    out_vc,
                                                    flit);

                if (flit matches tagged FLIT_BODY .body_info &&& body_info.isTail)
                begin
                    // Release outbound virtual channel
                    new_used_vcs[out_p][in_ln][out_vc] = False;
                end
            end
        end

        stage3cCtrl.ready(iid, tuple6(output_credits, new_vcs, msg_to, new_routes, new_output_vcs, new_used_vcs));
    endrule


    //
    // stage3c_crossbarSend1 --
    //   Act on the routing crossbar decisions.
    //
    rule stage3c_crossbarSend1 (True);
        // Get the info from the previous stage.
        match {.iid, {.output_credits, .virtual_channels, .msg_to, .routes, .output_vcs, .used_vcs}} <- stage3cCtrl.nextReadyInstance();
        debugLog.record(iid, $format("3c: Begin."));

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            // Send out our output enqueues in each direction.
            enqTo[out_p].send(iid, msg_to[out_p]);
        end

        stage4Ctrl.ready(iid, tuple5(output_credits, virtual_channels, routes, output_vcs, used_vcs));
    endrule


    (* conservative_implicit_conditions *)
    rule stage4_route (True);
        // Get the info from the previous stage.
        match {.iid, {.output_credits, .virtual_channels, .routes, .output_vcs, .used_vcs}} <- stage4Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("4: Begin."));
        
        //
        // Request routes for head flits that are not yet assigned output ports.
        //

        VC_STATE#(Maybe#(MESH_ROUTE_REQ)) new_routes = newVector();

        function Maybe#(MESH_ROUTE_REQ) reqRoute(Tuple2#(t_VC_FIFO, Maybe#(MESH_ROUTE_REQ)) req);
            match {.vc_fifo, .cur_route} = req;

            // Is this a new head flit with no assigned output port?  Head
            // flits store their requested route in the VC metadata.
            if (isValid(vc_fifo.routeReq) && ! isValid(cur_route))
            begin
                // Yes: Request the new route
                return vc_fifo.routeReq;
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
                        debugLog.record(iid, $format("4: RC: ROUTE in port %s ln %0d vc %0d, ",
                                                     portShow(fromInteger(in_p)), ln, vc) + fshow(out_p));
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
        function Bool outputVCAvail(Tuple2#(Bool, VC_CREDIT_CNT) out_vc_info);
            match {.out_vc_in_use, .out_vc_credits} = out_vc_info;
            return ! out_vc_in_use && (out_vc_credits != 0);
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
                                                    Maybe#(MESH_ROUTE_REQ),
                                                    Maybe#(VC_IDX)) req);
            match {.in_vc, .vc_fifo, .out_route, .cur_out_vc} = req;

            if (//  - The first entry in the channel is requesting a route
                isValid(vc_fifo.routeReq) &&&
                //  - The incoming channel has an output port assigned
                out_route matches tagged Valid .rt &&&
                //  - The incoming channel has no output channel assigned
                ! isValid(cur_out_vc) &&&
                //  - An output channel is available
                findIndex(outputVCAvail, zip(used_vcs[rt.outPort][ln],
                                             output_credits[rt.outPort][ln])) matches tagged Valid .out_vc)
            begin
                // Yes: Request an output channel.
                return tagged Valid tuple5(fromInteger(in_p),
                                           fromInteger(ln),
                                           fromInteger(in_vc),
                                           rt.outPort,
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
                                                     portShow(fromInteger(in_p)), ln, vc, portShow(tpl_4(req)), tpl_5(req)));
                    end
                    else if (virtual_channels[in_p][ln][vc].notEmpty)
                    begin
                        if (routes[in_p][ln][vc] matches tagged Valid .rt)
                        begin
                            debugLog.record(iid, $format("4: RC: Blocked in port %s ln %0d vc %0d, ",
                                                         portShow(fromInteger(in_p)), ln, vc) + fshow(rt));
                        end
                        else
                        begin
                            debugLog.record(iid, $format("4: RC: Blocked in port %s ln %0d vc %0d",
                                                         portShow(fromInteger(in_p)), ln, vc));
                        end
                    end
                end
            end
        end

        //
        // Update global state.
        //
        routesPool.getReg(iid) <= new_routes;

        stage5Ctrl.ready(iid, tuple5(output_credits,
                                     virtual_channels,
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
        match {.iid, {.output_credits, .virtual_channels, .output_vcs, .used_vcs, .new_out_vc_req}} <- stage5Ctrl.nextReadyInstance();
        debugLog.record(iid, $format("5: Begin."));

        VC_STATE#(VC_CREDIT_CNT) upd_credits = output_credits;

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

            debugLog.record(iid, $format("5: RC: ARB 0x%x grant %0d", pack(linear_vc_req_vec), idx));
            debugLog.record(iid, $format("5: RC: GRANT OUT VC in port %s ln %0d vc %0d, out port %s vc %0d, credit %0d: ",
                                         portShow(in_p), ln, in_vc, portShow(out_p), out_vc, upd_credits[out_p][ln][out_vc]));

            // Update credits
            upd_credits[out_p][ln][out_vc] = upd_credits[out_p][ln][out_vc] - 1;

`ifdef MESH_EVENTS_BROKEN
            //
            // Need to rethink events for two reasons:
            //  - Limited to 128 node simulations
            //  - Depends on first entry in an incoming channel FIFO being
            //    available here.  That used to be true but isn't now.
            //

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
            let flit = virtual_channels[in_p][ln][in_vc].first;
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
`else
            eventGrant.recordEvent(iid, tagged Invalid);
            eventGrantArb.recordEvent(iid, tagged Invalid);
`endif
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
        outputCreditsPool.getReg(iid) <= upd_credits;

        //
        // Load the routing table for the next stage.  Ideally, this would be
        // loaded only when needed since it shares a read port with stage 3.
        // That would require a lot of logic and stage 3 only requests a read
        // when needed, so it probably isn't worth the effort or hardware.
        // stage6Ctrl is buffered, which should hide many sins.
        //
        stationRoutingTable.readPorts[1].readReq(iid);

        stage6Ctrl.ready(iid, virtual_channels);
    endrule


    (* conservative_implicit_conditions *)
    (* descending_urgency="stage6_enqs, stage5_arbOutChannel, stage4_route, stage3b_crossbarSend0, stage3a_crossbarArb, stage2_multiplexVCs, stage1_updateCreditsIn" *)
    rule stage6_enqs (True);
        // Get the current IID from the previous stage.    
        match {.iid, .virtualChannels} <- stage6Ctrl.nextReadyInstance();
       
        let rt_tbl <- stationRoutingTable.readPorts[1].readRsp();

        VC_STATE#(t_VC_FIFO) new_vcs = virtualChannels;

        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) enqs = replicate(tagged Invalid);
    
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            // Deal with input enqueues from each direction.
            let m_enq <- enqFrom[p].receive(iid);
            if (m_enq matches tagged Valid {.ln, .vc, .flit})
            begin
                let new_flit = flit;
                if (flit matches tagged FLIT_HEAD .info &&& fromInteger(p) == portLocal)
                begin
                    //
                    // Coming in from the local port.  Instead of forcing all
                    // connected objects to know their name, simply force the
                    // source to the proper node ID here.
                    //
                    let new_info = info;
                    new_info.src = iid;
                    new_flit = tagged FLIT_HEAD new_info;

                    debugLog.record(iid, $format("6: BW: ENTER in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(new_flit));
                end
                else
                begin
                    debugLog.record(iid, $format("6: BW: ENQ in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(new_flit));
                end

                let new_enq = tuple3(ln, vc, new_flit);
                enqs[p] = tagged Valid new_enq;

                new_vcs[p][ln][vc].notEmpty = True;

                if (! virtualChannels[p][ln][vc].notEmpty)
                begin
                    // FIFO was empty.  Update the virtual channel state now
                    // that it has an entry.
                    if (new_flit matches tagged FLIT_HEAD .msg)
                    begin
                        let rt = route(iid, msg.dst, rt_tbl);
                        new_vcs[p][ln][vc].routeReq = tagged Valid rt;
                        debugLog.record(iid, $format("6: BW: ") + fshow(new_flit) + $format(" is first, req ") + fshow(rt));
                    end
                    else
                    begin
                        new_vcs[p][ln][vc].routeReq = tagged Invalid;
                        debugLog.record(iid, $format("6: BW: ") + fshow(new_flit) + $format(" is first"));
                    end
                end
                else
                begin
                    // FIFO was not empty.  New flit is written only to channel FIFO.
                    debugLog.record(iid, $format("6: BW: ") + fshow(new_flit));
                end
            end
        end

        virtualChannelsPool.insertState(iid, new_vcs);

        // Any flits to write to the FIFO buffer pool?
        Bool did_new_enqs = any(isValid, enqs);
        if (did_new_enqs)
        begin
            vcBufferEntries.enq(iid, enqs);
        end

        stage7Ctrl.ready(iid, did_new_enqs);
    endrule


    rule stage7_enqAck (True);
        // Get the current IID from the previous stage.    
        match {.iid, .did_new_enqs} <- stage7Ctrl.nextReadyInstance();
       
        debugLog.record(iid, $format("7: Begin."));

        if (did_new_enqs)
        begin
            vcBufferEntries.enqAck(iid);
        end

        // End of model cycle
        localCtrl.endModelCycle(iid, 0);
    endrule
endmodule


//
// VC_FIFOS --
//   Manage access to virtual channel buffers.  The methods allow for either
//   an implementation that updates all ports in parallel or one that serializes
//   them.
//
interface VC_FIFOS#(type n_STATIONS);
    // Enqueue a new flit to one or more ports.  The enq() operation may
    // require multiple FPGA cycles.  enqAck() below confirms that the
    // request has been completed and written to memory.
    method Action enq(INSTANCE_ID#(n_STATIONS) iid,
                      Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msgs);

    // Confirm that enq() is complete.
    method Action enqAck(INSTANCE_ID#(n_STATIONS) iid);


    // Read the first entry, dequeue it, and return both the first entry and
    // the next entry (the new first entry after dequeue).  Like enq(), the
    // method accepts one request per network port.
    method Action deqAndReadReq(INSTANCE_ID#(n_STATIONS) iid,
                                Vector#(NUM_PORTS,
                                        Maybe#(Tuple2#(LANE_IDX, VC_IDX))) deqPort);

    // For each port return the first entry that has just been dequeued and,
    // if present, the next entry (the new first entry).  The first MESH_MSG
    // in the tuple is the oldest entry.
    method ActionValue#(Vector#(NUM_PORTS,
                                Maybe#(Tuple2#(MESH_MSG,
                                               Maybe#(MESH_FLIT))))) deqAndReadRsp(INSTANCE_ID#(n_STATIONS) iid);
endinterface

module [HASIM_MODULE] mkVCBufferStorage#(TIMEP_DEBUG_FILE_MULTIPLEXED#(n_STATIONS) debugLog)
    // Interface:
    (VC_FIFOS#(n_STATIONS))
    provisos (Bits#(PORT_IDX, t_PORT_IDX_SZ),
              Bits#(LANE_IDX, t_LANE_IDX_SZ),
              Bits#(VC_IDX, t_VC_IDX_SZ),

              // Unified address space for all channel buffers within a port
              Alias#(t_GLOB_VC_IDX, Tuple2#(LANE_IDX, VC_IDX)));

    let checkBufNotFull <- mkAssertionStrPvtChecker("icn-mesh.bsv: flit received but buffer is full!",
                                                    ASSERT_ERROR);

    // Metadata storage for all virtual channel FIFOs
    Vector#(NUM_PORTS,
            MEMORY_MULTI_READ_IFC_MULTIPLEXED#(n_STATIONS,
                                               2,
                                               t_GLOB_VC_IDX,
                                               FUNC_FIFO_IDX#(NUM_VC_FIFO_ENTRIES)))
        vcFIFOs = newVector();

    for (Integer p = 0; p < numPorts; p = p + 1)
    begin
        let buf_mem <- mkBRAMBufferedPseudoMultiRead(False);
        let init_buf_mem = mkMultiMemInitialized(buf_mem, funcFIFO_IDX_Init);
        vcFIFOs[p] <- mkMemoryMultiRead_Multiplexed(init_buf_mem);
    end

    // Flit storage for virtual channel buffers, indexed by channel and vcFIFOs.
    // There is an entry here for every vcFIFOs index for every virtual channel.
    Vector#(NUM_PORTS,
            MEMORY_MULTI_READ_IFC_MULTIPLEXED#(n_STATIONS,
                                               2,
                                               Tuple2#(t_GLOB_VC_IDX,
                                                       Bit#(TLog#(NUM_VC_FIFO_ENTRIES))),
                                               MESH_FLIT)) vcFlits;
    if (valueOf(NUM_PORTS) < 128)
    begin
        // Smaller configurations use BRAM for virtual channel FIFO storage.
        vcFlits <- replicateM(mkMemoryMultiRead_Multiplexed(mkBRAMBufferedPseudoMultiRead(False)));
    end
    else
    begin
        // Larger configurations use a scratchpad for virtual channel FIFO storage.
        vcFlits <- mkScratchpadVCBuffers();
    end

    // Flits to be written to virtual channel FIFOs
    Vector#(NUM_PORTS, FIFO#(Tuple2#(INSTANCE_ID#(n_STATIONS),
                                     MESH_MSG))) enqReqQ <- replicateM(mkFIFO());

    // Ports that were updated by an enq() request.
    FIFO#(Vector#(NUM_PORTS, Bool)) enqAckMetaQ <- mkSizedFIFO(4);
    Vector#(NUM_PORTS, FIFO#(Bool)) enqAckQ <- replicateM(mkFIFO());


    // Dequeue requests
    Vector#(NUM_PORTS, FIFO#(Tuple2#(INSTANCE_ID#(n_STATIONS),
                                     Tuple2#(LANE_IDX, VC_IDX)))) deqReqQ <-
        replicateM(mkFIFO());

    // Ports that were updated by a deq() request.
    FIFO#(Vector#(NUM_PORTS, Bool)) deqRspMetaQ <- mkSizedFIFO(4);
    Vector#(NUM_PORTS, FIFO#(Tuple3#(LANE_IDX, VC_IDX, Bool))) readRspQ <-
        replicateM(mkFIFO());


    //
    // Assert there were no enqueues to full buffers.
    //
    PulseWire errorFullFIFO <- mkPulseWireOR();

    rule assertNotFull (True);
        checkBufNotFull(! errorFullFIFO);
    endrule


    Rules enq_deq_rules = emptyRules();
    for (Integer p = 0; p < numPorts; p = p + 1)
    begin
        //
        // doEnq --
        //   For each port, consume the current state of a virtual channel FIFO
        //   read by the enq() method and write the new flit to the FIFO.
        //
        let r_enq =
        (rules
            rule doEnq (True);
                // Flit to enqueue
                match {.iid, {.ln, .vc, .flit}} = enqReqQ[p].first();
                enqReqQ[p].deq();

                // FIFO in which to store it
                let fifo <- vcFIFOs[p].readPorts[0].readRsp(iid);

                // Is the FIFO already full?  That's an error!
                if (! funcFIFO_IDX_notFull(fifo))
                begin
                    errorFullFIFO.send();
                    debugLog.record(iid, $format("FIFO: ERROR enq to full in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(flit));
                end

                // Write the new flit to the FIFO
                match {.upd_fifo, .idx} = funcFIFO_IDX_UGenq(fifo);
                let slot = tuple2(ln, vc);
                vcFIFOs[p].write(iid, slot, upd_fifo);
                vcFlits[p].write(iid, tuple2(slot, idx), flit);
                debugLog.record(iid, $format("FIFO: enq to in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(flit));

                // Tell the client the write is done
                enqAckQ[p].enq(?);
            endrule
        endrules);

        //
        // doDeq --
        //   For each port, consume the current state of a virtual channel FIFO
        //   read by the deqAndReadReq() and deq the FIFO.
        //
        let r_deq =
        (rules
            rule doDeq (True);
                // Virtual channel to dequeue
                match {.iid, {.ln, .vc}} = deqReqQ[p].first();
                deqReqQ[p].deq();

                // FIFO current state
                let fifo <- vcFIFOs[p].readPorts[1].readRsp(iid);

                let slot = tuple2(ln, vc);
                vcFlits[p].readPorts[0].readReq(iid,
                                                tuple2(slot,
                                                       funcFIFO_IDX_UGfirst(fifo)));

                let upd_fifo = funcFIFO_IDX_UGdeq(fifo);

                Bool not_empty = funcFIFO_IDX_notEmpty(upd_fifo);
                if (not_empty)
                begin
                    vcFlits[p].readPorts[1].readReq(iid,
                                                    tuple2(slot,
                                                           funcFIFO_IDX_UGfirst(upd_fifo)));
                end
                else
                begin
                    // The FIFO is empty.  Reset it to the initial state.  This
                    // slightly improves hit rates when virtual channel buffers
                    // are stored in scratchpads instead of BRAM.
                    upd_fifo = funcFIFO_IDX_Init();
                end

                // Write the updated state
                vcFIFOs[p].write(iid, tuple2(ln, vc), upd_fifo);

                readRspQ[p].enq(tuple3(ln, vc, not_empty));
            endrule
        endrules);

        enq_deq_rules = rJoinDescendingUrgency(enq_deq_rules,
                                               rJoinDescendingUrgency(r_enq, r_deq));
    end
    addRules(enq_deq_rules);

    method Action enq(INSTANCE_ID#(n_STATIONS) iid,
                      Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msgs);
        //
        // Read the current FIFO states
        //
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            if (msgs[p] matches tagged Valid {.ln, .vc, .flit})
            begin
                vcFIFOs[p].readPorts[0].readReq(iid, tuple2(ln, vc));
                enqReqQ[p].enq(tuple2(iid, validValue(msgs[p])));
            end
        end

        // This FIFO will tell the enqAck() method which ports were updated.
        enqAckMetaQ.enq(map(isValid, msgs));
    endmethod


    method Action enqAck(INSTANCE_ID#(n_STATIONS) iid);
        let active_ports = enqAckMetaQ.first();
        enqAckMetaQ.deq();

        // Consume the "done" message from all updated ports.
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            if (active_ports[p])
            begin
                enqAckQ[p].deq();
            end
        end
    endmethod


    method Action deqAndReadReq(INSTANCE_ID#(n_STATIONS) iid,
                                Vector#(NUM_PORTS,
                                        Maybe#(Tuple2#(LANE_IDX, VC_IDX))) deqPort);
        //
        // Read the current FIFO states
        //
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            if (deqPort[p] matches tagged Valid {.ln, .vc})
            begin
                vcFIFOs[p].readPorts[1].readReq(iid, tuple2(ln, vc));
                deqReqQ[p].enq(tuple2(iid, validValue(deqPort[p])));
            end
        end

        // This FIFO will tell the deqAndReadRsp() method which ports were read.
        deqRspMetaQ.enq(map(isValid, deqPort));
    endmethod


    method ActionValue#(Vector#(NUM_PORTS,
                                Maybe#(Tuple2#(MESH_MSG,
                                               Maybe#(MESH_FLIT))))) deqAndReadRsp(INSTANCE_ID#(n_STATIONS) iid);
        let active_ports = deqRspMetaQ.first();
        deqRspMetaQ.deq();

        Vector#(NUM_PORTS, Maybe#(Tuple2#(MESH_MSG, Maybe#(MESH_FLIT)))) rsp = newVector();

        // Consolidate individual port responses into a single response.
        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            if (active_ports[p])
            begin
                match {.ln, .vc, .not_empty} = readRspQ[p].first();
                readRspQ[p].deq();

                // Consume the first flit, requested by doDeq.
                let first <- vcFlits[p].readPorts[0].readRsp(iid);

                // If the FIFO isn't now empty, also consume the next flit.
                Maybe#(MESH_FLIT) m_next = tagged Invalid;
                if (not_empty)
                begin
                    let next_flit <- vcFlits[p].readPorts[1].readRsp(iid);
                    m_next = tagged Valid next_flit;
                    debugLog.record(iid, $format("FIFO: deq from in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(first) + $format(", next ") + fshow(validValue(m_next)));
                end
                else
                begin
                    debugLog.record(iid, $format("FIFO: deq from in port %s ln %0d vc %0d: ", portShow(fromInteger(p)), ln, vc) + fshow(first));
                end

                rsp[p] = tagged Valid tuple2(tuple3(ln, vc, first), m_next);
            end
            else
            begin
                rsp[p] = tagged Invalid;
            end
        end

        return rsp;
    endmethod
endmodule


//
// mkScratchpadVCBuffers --
//   Virtual channel flit buffers, stored in a single scratchpad.  As required
//   by mkVCBufferStorage, this code maintains the illusion that each router
//   port gets its own scratchpad.  In reality, all the ports are mapped to
//   a single scratchpad.
//
module [HASIM_MODULE] mkScratchpadVCBuffers
    // Interface:
    (Vector#(NUM_PORTS,
             MEMORY_MULTI_READ_IFC_MULTIPLEXED#(n_STATIONS,
                                                n_MEM_READ_PORTS,
                                                t_ADDR,
                                                t_DATA)))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ));

    // Scratchpad holding the flits.  Note that the Tuple2#() address is the
    // combination of the router port and the address within the port.
    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(n_STATIONS,
                                       TMul#(NUM_PORTS, n_MEM_READ_PORTS),
                                       Tuple2#(PORT_IDX, t_ADDR),
                                       t_DATA) data <-
        mkMemoryMultiRead_Multiplexed(mkMultiReadScratchpad(`VDEV_SCRATCH_HASIM_MESH_FIFO_DATA,
                                                            defaultValue));

    Vector#(NUM_PORTS, MEMORY_MULTI_READ_IFC_MULTIPLEXED#(n_STATIONS,
                                                          n_MEM_READ_PORTS,
                                                          t_ADDR,
                                                          t_DATA)) v = newVector();

    for (Integer p = 0; p < valueOf(NUM_PORTS); p = p + 1)
    begin
        PORT_IDX p_idx = fromInteger(p);

        v[p] =
            interface MEMORY_MULTI_READ_IFC_MULTIPLEXED
                Vector#(n_MEM_READ_PORTS,
                        MEMORY_READER_IFC_MULTIPLEXED#(n_STATIONS,
                                                       t_ADDR,
                                                       t_DATA)) local_ports = newVector();

                for (Integer rp = 0; rp < valueOf(n_MEM_READ_PORTS); rp = rp + 1)
                begin
                    // The caller has two memory port spaces.  In some
                    // implementations this outer level would map to separate
                    // memories.  Here, all ports share the same scratchpad.
                    // Map the outer port dimension to chunks of linearized ports.
                    Integer base_p = (p * valueOf(n_MEM_READ_PORTS));

                    local_ports[rp] =
                        interface MEMORY_READER_IFC_MULTIPLEXED
                            method Action readReq(INSTANCE_ID#(n_STATIONS) iid, t_ADDR addr);
                                data.readPorts[base_p + rp].readReq(iid, tuple2(p_idx, addr));
                            endmethod

                            method ActionValue#(t_DATA) readRsp(INSTANCE_ID#(n_STATIONS) iid);
                                let val <- data.readPorts[base_p + rp].readRsp(iid);
                                return val;
                            endmethod
                        endinterface;
                end

                interface readPorts = local_ports;

                method Action write(INSTANCE_ID#(n_STATIONS) iid,
                                    t_ADDR addr,
                                    t_DATA val);
                    // All writer ports conflict
                    data.write(iid, tuple2(p_idx, addr), val);
                endmethod
            endinterface;
    end

    return v;
endmodule

