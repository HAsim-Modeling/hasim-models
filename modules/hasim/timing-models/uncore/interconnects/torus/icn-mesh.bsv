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

typedef 4 NUM_VC_FIFO_ENTRIES;

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

module [HASIM_MODULE] mkInterconnect
    // interface:
    ()
    provisos (Alias#(FUNC_FIFO#(MESH_FLIT, NUM_VC_FIFO_ENTRIES), t_VC_FIFO));

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
    MULTIPLEXED_STATE_POOL#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) virtualChannelsPool    <- mkMultiplexedStatePool(replicate(replicate(replicate(funcFIFO_Init))));
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
    STAGE_CONTROLLER#(NUM_STATIONS, Tuple2#(Vector#(NUM_PORTS, Maybe#(WINNER_INFO)), VC_STATE#(t_VC_FIFO))) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage4Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage5Ctrl <- mkStageController();
    STAGE_CONTROLLER#(NUM_STATIONS, VC_STATE#(t_VC_FIFO)) stage6Ctrl <- mkStageController();

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
    

    MULTIPLEXED#(NUM_STATIONS,
                 Vector#(NUM_PORTS,
                         LOCAL_ARBITER#(TMul#(NUM_LANES, VCS_PER_LANE)))) stage2Arbiters
        <- mkMultiplexed(replicateM(mkLocalArbiter(False)));

    (* conservative_implicit_conditions *)
    rule stage2_multiplexVCs (True);
        
        // Get the info from the previous stage.
        let iid <- stage2Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(t_VC_FIFO) virtualChannels <- virtualChannelsPool.extractState(iid);
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes         = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs      = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputNotFulls = outputNotFullsPool.getReg(iid);

        // Arbiters for the current instance
        Vector#(NUM_PORTS, LOCAL_ARBITER#(TMul#(NUM_LANES, VCS_PER_LANE))) arbiters = stage2Arbiters[iid];

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
            let grant_idx <- arbiters[in_p].arbitrate(ready_vcs);
            if (grant_idx matches tagged Valid .idx)
            begin
                // Reverse map the winning index to a lane and channel
                match {.ln, .in_vc} = concat(identity_map)[idx];

                // Look up the output port and channel.  The lane is the same.
                let out_p = validValue(routes[in_p][ln][in_vc]);
                let out_vc = validValue(outputVCs[in_p][ln][in_vc]);


                debugLog.record(iid, $format("2: VCA: Multiplexor for port %s chose lane %0d, virtual channel %0d, destination %s.", portShow(fromInteger(in_p)), ln, in_vc, portShow(out_p)));

                vc_winners[in_p] = tagged Valid WINNER_INFO 
                                   {
                                     lane: fromInteger(ln),
                                     inputVC: fromInteger(in_vc),
                                     outputPort: out_p, 
                                     outputVC: out_vc,
                                     message: funcFIFO_UGfirst(virtualChannels[in_p][ln][in_vc])
                                   };
            end
        end

        stage3Ctrl.ready(iid, tuple2(vc_winners, virtualChannels));

    endrule


    MULTIPLEXED#(NUM_STATIONS,
                 Vector#(NUM_PORTS,
                         LOCAL_ARBITER#(NUM_PORTS))) stage3Arbiters
        <- mkMultiplexed(replicateM(mkLocalArbiter(False)));

    (* conservative_implicit_conditions *)
    rule stage3_crossbar (True);
        
        // Get the info from the previous stage.
        match {.iid, {.vc_winners, .virtualChannels}} <- stage3Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        Reg#(VC_STATE#(Maybe#(PORT_IDX))) routes          = routesPool.getReg(iid);
        Reg#(VC_STATE#(Maybe#(VC_IDX)))   outputVCs       = outputVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             usedVCs         = usedVCsPool.getReg(iid);
        Reg#(VC_STATE#(Bool))             outputCredits   = outputCreditsPool.getReg(iid);

        // This is the vector of output messages that the virtual channels contend for.
        Vector#(NUM_PORTS, Maybe#(MESH_MSG)) msg_to = replicate(tagged Invalid);
        
        // Vectors to update our registers with.
        VC_STATE#(t_VC_FIFO) new_vcs = virtualChannels;
        VC_STATE#(Maybe#(PORT_IDX))   new_routes = routes;
        VC_STATE#(Bool)             new_used_vcs = usedVCs;
        VC_STATE#(Maybe#(VC_IDX)) new_output_vcs = outputVCs;

        // Arbiters for the current instance
        Vector#(NUM_PORTS, LOCAL_ARBITER#(NUM_PORTS)) arbiters = stage3Arbiters[iid];

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

        for (Integer out_p = 0; out_p < numPorts; out_p = out_p + 1)
        begin
            let grant_idx <- arbiters[out_p].arbitrate(out_port_requests[out_p]);
            if (grant_idx matches tagged Valid .in_p)
            begin
                let info = validValue(vc_winners[in_p]);

                in_port_used[in_p] = True;

                // Built a structure of outbound messages, indexed by the
                // output port.
                debugLog.record(iid, $format("3: SA: Arb %s req 0x%0x grant %0d", portShow(fromInteger(out_p)), pack(out_port_requests[out_p]), in_p));
                debugLog.record(iid, $format("3: SA: Gave crossbar %s output port to %s input port, lane %0d, virtual channel %0d", portShow(fromInteger(out_p)), portShow(pack(in_p)), info.lane, info.inputVC));
                msg_to[out_p] = tagged Valid tuple3(info.lane, info.outputVC, info.message);
            end
        end


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
                    debugLog.record(iid, $format("3: SA: Detected tail flit on incoming port %s. Tearing down routing info.", portShow(fromInteger(in_p))));
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
    Wire#(VC_STATE#(t_VC_FIFO)) stage4_virtualChannels <- mkWire();
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

        //
        // For now we do not have a true credit scheme.  Instead, available
        // space in each virtual channel FIFO governs whether sending is
        // permitted.  We reserve two slots: one for a send credit and one
        // because the sending router may not have been updated for the
        // current cycle and may still be preparing to use a current credit.
        //

        for (Integer p = 0; p < numPorts; p = p + 1)
        begin
            VC_CREDIT_INFO creds = newVector();
            
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                // A FIFO with 2 free slots will be given a sender credit.
                // One slot is for the data, one slot is for writes that may
                // not yet have been processed for the current cycle due to
                // the timing of the multiplexed model.
                function Bool fifoHasFreeSlots(t_VC_FIFO fifo) =
                    (funcFIFO_numBusySlots(fifo) < fromInteger((valueOf(NUM_VC_FIFO_ENTRIES) - 1)));

                // For now, have_credit and not_full are the same thing...
                let not_full = map(fifoHasFreeSlots, virtualChannels[p][ln]);
                let have_credit = not_full;
            
                creds[ln] = zip(have_credit, not_full);
            end
        
            creditTo[p].send(iid, tagged Valid creds);
        end
        
        // End of model cycle (path 1)
        localCtrl.endModelCycle(iid, 0);
        virtualChannelsPool.insertState(iid, virtualChannels);
        
    endrule

endmodule
