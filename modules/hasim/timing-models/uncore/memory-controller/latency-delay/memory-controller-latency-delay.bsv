//
// Copyright (C) 2013 Intel Corporation
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

import GetPut::*;
import Vector::*;
import FIFOF::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "awb/provides/hasim_chip_topology.bsh"


// ****** Generated files ******

`include "awb/dict/TOPOLOGY.bsh"


typedef struct
{
    LINE_ADDRESS physicalAddress;
    Bool isStore;
    MEM_OPAQUE opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_REQ deriving (Eq, Bits);

typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_RSP deriving (Eq, Bits);


function CORE_MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);

    return CORE_MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };

endfunction

function MEM_CTRL_RSP initMemCtrlRsp(MEM_CTRL_REQ req);

    return MEM_CTRL_RSP
    {
        physicalAddress: req.physicalAddress,
        opaque: req.opaque,
        destination: req.destination
    };

endfunction

`define LANE_MEM_REQ 0
`define LANE_MEM_RSP 1

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) LANE_STATE#(parameter type t_DATA);


module [HASIM_MODULE] mkMemoryController()
    provisos (Alias#(t_VC_FIFO, FUNC_FIFO#(OCN_FLIT, 4)));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_MEM_CTRLS) debugLog <-
        mkTIMEPDebugFile_Multiplexed("interconnect_memory_controller.out");

    // Interfaces to/from interconnect network.
    // Note: in the future these may be multiplexed if we
    // are simulating multiple memory controllers.

    PORT_SEND_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, OCN_MSG)        enqToOCN      <- mkPortSend_Multiplexed("memctrl_to_ocn_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, OCN_MSG)        enqFromOCN    <- mkPortRecv_Multiplexed("ocn_to_memctrl_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, VC_CREDIT_INFO) creditToOCN   <- mkPortSend_Multiplexed("memctrl_to_ocn_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_MEM_CTRLS, VC_CREDIT_INFO) creditFromOCN <- mkPortRecv_Multiplexed("ocn_to_memctrl_credit", 1);

    MULTIPLEXED_STATE_POOL#(MAX_NUM_MEM_CTRLS, LANE_STATE#(t_VC_FIFO)) virtualChannelsPool <-
        mkMultiplexedStatePool(replicate(replicate(funcFIFO_Init)));

    MULTIPLEXED_REG#(MAX_NUM_MEM_CTRLS, LANE_STATE#(Bool)) needLoadRspPool <-
        mkMultiplexedReg(replicate(replicate(False)));

    MULTIPLEXED_REG#(MAX_NUM_MEM_CTRLS, LANE_STATE#(Bool)) notFullsPool <-
        mkMultiplexedReg(replicate(replicate(False)));
    
    Vector#(3, INSTANCE_CONTROL_IN#(MAX_NUM_MEM_CTRLS))  inports  = newVector();
    inports[0] = enqFromOCN.ctrl;
    inports[1] = creditFromOCN.ctrl;
    inports[2] = virtualChannelsPool.ctrl;

    Vector#(2, INSTANCE_CONTROL_OUT#(MAX_NUM_MEM_CTRLS)) outports = newVector();
    outports[0] = enqToOCN.ctrl;
    outports[1] = creditToOCN.ctrl;

    // Coordinate between the pipeline stages.
    LOCAL_CONTROLLER#(MAX_NUM_MEM_CTRLS) localCtrl <-
        mkNamedLocalControllerWithActive("Memory Controller",
                                   0, False,
                                   inports,
                                   Vector::nil,
                                   outports);

    STAGE_CONTROLLER#(MAX_NUM_MEM_CTRLS, LANE_STATE#(Bool)) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_MEM_CTRLS, LANE_STATE#(t_VC_FIFO)) stage3Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_MEM_CTRLS, LANE_STATE#(t_VC_FIFO)) stage4Ctrl <- mkStageController();

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

    //
    // Set the number of active nodes, given the topology.
    //
    let numActiveCtrlrs <- mkTopologyParamStream(`TOPOLOGY_NUM_MEM_CONTROLLERS);

    rule initNumActiveCtrlrs (True);
        let n <- numActiveCtrlrs.get();
        localCtrl.setMaxActiveInstance(tpl_1(n) - 1);
    endrule


    rule stage1_updateCredit (True);
        let iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(iid);

        // Read our local state from the pools.
        Reg#(LANE_STATE#(Bool)) notFulls = notFullsPool.getReg(iid);

        LANE_STATE#(Bool) new_not_fulls = notFulls;
        let m_credits <- creditFromOCN.receive(iid);

        // Update our notion of credits.
        if (m_credits matches tagged Valid .creds)
        begin
            debugLog.record_next_cycle(iid, $format("1: Update credits."));
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin
                    match {.cred, .out_not_full} = creds[ln][vc];
                    new_not_fulls[ln][vc] = out_not_full;
                end
            end
        end
        else
        begin
            debugLog.record_next_cycle(iid, $format("1: No credits in."));
        end

        notFulls <= new_not_fulls;
        stage2Ctrl.ready(iid, new_not_fulls);
    endrule


    (* conservative_implicit_conditions *)
    rule stage2_enqsOut (True);
        match {.iid, .not_fulls} <- stage2Ctrl.nextReadyInstance();

        // Read our local state from the pools.
        LANE_STATE#(t_VC_FIFO) virtualChannels <- virtualChannelsPool.extractState(iid);

        // Calculate if we're sending load data back from our internal queues.
        Maybe#(OCN_MSG) enq_to_send = tagged Invalid;
        
        //
        // isReadyVC --
        //   Is a request available combined with an available output channel?
        //   Always use the response lane to avoid deadlocks.
        //
        function isReadyVC(Integer ln, Integer vc);
            return (funcFIFO_notEmpty(virtualChannels[ln][vc]) &&
                    not_fulls[`LANE_MEM_RSP][vc]);
        endfunction

        let ready_vcs = map(uncurry(isReadyVC), concat(identityMap));

        // Static arbitration
        if (findElem(True, ready_vcs) matches tagged Valid .idx)
        begin
            // Reverse map the winning index to a lane and channel
            match {.ln, .vc} = concat(identityMap)[idx];

            let msg = funcFIFO_UGfirst(virtualChannels[ln][vc]);
            enq_to_send = tagged Valid tuple3(`LANE_MEM_RSP, fromInteger(vc), msg);
                                              
            virtualChannels[ln][vc] = funcFIFO_UGdeq(virtualChannels[ln][vc]);

            case (msg) matches 
                tagged FLIT_HEAD .info:
                begin
                    debugLog.record(iid, $format("2: Initiating response to destination: %0d", info.dst));
                end
                tagged FLIT_BODY .info:
                begin
                    debugLog.record(iid, $format("2: Sending body flit."));
                end
            endcase
        end
        else
        begin
            debugLog.record(iid, $format("2: No send."));
        end

        // Send out the result.
        enqToOCN.send(iid, enq_to_send);

        // Continue in the next stage.
        stage3Ctrl.ready(iid, virtualChannels);
    endrule
    

    rule stage3_enqsIn (True);
        match {.iid, .virtualChannels} <- stage3Ctrl.nextReadyInstance();
        
        // Read incoming ports.
        let m_enq     <- enqFromOCN.receive(iid);

        // Read our local state from the pools.
        Reg#(LANE_STATE#(Bool)) needLoadRsp = needLoadRspPool.getReg(iid);

        // Vectors to update our state.
        LANE_STATE#(Bool) new_need_load_rsp = needLoadRsp;

        // See if there are any incoming enqueues.
        if (m_enq matches tagged Valid {.lane, .vc, .req})
        begin
            case (req) matches 
                tagged FLIT_HEAD .info:
                begin
                    if (info.isStore)
                    begin
                        // No response to stores currently.
                        new_need_load_rsp[lane][vc] = False;
                        debugLog.record(iid, $format("3: Received store from Station %0d", info.src));
                    end
                    else
                    begin
                        virtualChannels[lane][vc] =
                            funcFIFO_UGenq(virtualChannels[lane][vc],
                                           tagged FLIT_HEAD OCN_FLIT_HEAD {src: info.dst,
                                                                           dst: info.src,
                                                                           isStore: False});
                        debugLog.record(iid, $format("3: Received load from Station %0d", info.src));
                        new_need_load_rsp[lane][vc] = True;
                    end
                end

                tagged FLIT_BODY .info:
                begin
                    if (needLoadRsp[lane][vc])
                    begin
                        // Body flits just get sent back for loads, currently.
                        debugLog.record(iid, $format("3: Finishing load rsp."));
                        virtualChannels[lane][vc] =
                            funcFIFO_UGenq(virtualChannels[lane][vc],
                                           tagged FLIT_BODY info);
                    end
                    else
                    begin
                        debugLog.record(iid, $format("3: Dropping store body."));
                    end
                end
            endcase
        end
        else
        begin
            debugLog.record(iid, $format("3: No req."));
        end
        
        needLoadRsp <= new_need_load_rsp;

        // Go on to stage 4.
        stage4Ctrl.ready(iid, virtualChannels);
    endrule

    (* conservative_implicit_conditions *)
    rule stage4_creditsOut (True);
        match {.iid, .virtualChannels} <- stage4Ctrl.nextReadyInstance();

        // Calculate our credits for the OCN.
        VC_CREDIT_INFO creds = newVector();

        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            creds[ln] = newVector();

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin
                let have_credit = ! funcFIFO_notEmpty(virtualChannels[ln][vc]); // XXX capacity - occupancy > round-trip latency.
                let not_full = ! funcFIFO_notEmpty(virtualChannels[ln][vc]); //virtualChannels[ln][vc].notFull();
                creds[ln][vc] = tuple2(have_credit, not_full);
            end
        end

        debugLog.record(iid, $format("4: Send output credits."));
        
        creditToOCN.send(iid, tagged Valid creds);
        
        virtualChannelsPool.insertState(iid, virtualChannels);
        localCtrl.endModelCycle(iid, 0);
    endrule

endmodule
