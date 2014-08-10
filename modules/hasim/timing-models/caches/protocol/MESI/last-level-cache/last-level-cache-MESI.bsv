//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

import Vector::*;
import FShow::*;
import DefaultValue::*;


// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/fpga_components.bsh"


// ******* HAsim Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/hasim_modellib.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_chip_topology.bsh"
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_last_level_cache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"
`include "awb/provides/hasim_interconnect_common.bsh"


// ******* Generated File Imports *******

`include "awb/dict/EVENTS_LLC.bsh"
`include "awb/dict/TOPOLOGY.bsh"
`include "awb/dict/OCN_LANES.bsh"


// ****** Local Definitions *******

//
// A routed LLC message is pair of a cache protocol message and a remote node.
// Depending on the type of the cache protocol message the node is either the
// source of a request or destination of a response.
//
// The node ID is wrapped in a Maybe type in order to provide a simple encoding
// of messages that are associated with the local core.
//
typedef struct
{
    Maybe#(STATION_ID) nodeID;
    CACHE_PROTOCOL_MSG msg;
}
ROUTED_LLC_MSG
    deriving (Eq, Bits);

instance FShow#(ROUTED_LLC_MSG);
    function Fmt fshow(ROUTED_LLC_MSG rmsg);
        let node_id =
            case (rmsg.nodeID) matches
                tagged Valid .nodeID: return $format(", node=%0d", nodeID);
                tagged Invalid: return $format(", local");
            endcase;
        return fshow(rmsg.msg) + node_id;
    endfunction
endinstance


//
// OCN messages are passed in an opaque type and passed partially through
// through the OCN and partially passed as a pointer to a message in a side
// buffer.  The OCN_SEND_TYPE typeclass manages the conversion.
//
instance OCN_SEND_TYPE#(CACHE_PROTOCOL_MSG);
    function OCN_MSG_HEAD_AND_PAYLOAD cvtToOCNFlits(STATION_ID tgt,
                                                    CACHE_PROTOCOL_MSG msg);
        OCN_MSG_HEAD_AND_PAYLOAD info;

        info.headFlit = OCN_FLIT_HEAD { src: ?,
                                        dst: tgt,
                                        isStore: False };

        info.payload = zeroExtend(pack(msg));

        return info;
    endfunction

    function Tuple2#(STATION_ID, CACHE_PROTOCOL_MSG) cvtFromOCNFlits(
        OCN_MSG_HEAD_AND_PAYLOAD ocnMsg
        );

        let head = ocnMsg.headFlit;
        CACHE_PROTOCOL_MSG msg = unpack(truncate(ocnMsg.payload));

        return tuple2(head.src, msg);
    endfunction
endinstance



//
// mkLastLevelCache --
//   The primary routing module.  This module takes in requests from the
//   core and routes them either to the local copy of the distributed
//   cache or across the OCN.  Requests from the local portion of the
//   cache to memory controllers are also routed through this module,
//   as are corresponding responses.
//
//   Very little work is done here.  It is just a stage for passing
//   messages to the right places.
//
module [HASIM_MODULE] mkLastLevelCache
    // Interface:
    ()
    provisos (NumAlias#(n_CHAN_FROM_CORE, 2),
              NumAlias#(n_CHAN_TO_CORE, 1));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc.out");

    // Instantiate other LLC components
    let llc_distrib <- mkDistributedLastLevelCache();

    //
    // Requests from the local core arrive here.  This router will forward
    // the request to the correct distributed LLC segment.
    //
    Vector#(n_CHAN_FROM_CORE,
            PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                         CACHE_PROTOCOL_MSG)) reqFromCore = newVector();
    for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
    begin
        reqFromCore[i] <- mkPortStallRecv_Multiplexed("CorePvtCache_to_UncoreQ_" + integerToString(i));
    end

    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) rspToCore <-
        mkPortStallSend_Multiplexed("Uncore_to_CorePvtCacheQ");
    
    //
    // Queues to/from local LLC segment.  These may either come from the local
    // core or a remote core.
    //
    Vector#(n_CHAN_FROM_CORE,
            PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                         ROUTED_LLC_MSG)) reqToLocalLLC = newVector();
    for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
    begin
        reqToLocalLLC[i] <- mkPortStallSend_Multiplexed("LLCHub_to_LLC_req_" + integerToString(i));
    end

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_MSG) rspFromLocalLLC <-
        mkPortStallRecv_Multiplexed("LLC_to_LLCHub_rsp");

    //
    // Ports from the LLC to the OCN.  The names of the ports are picked
    // indirectly through dictionary entries since the interface to the
    // OCN uses model-independent port/lane name bindings.
    //

    Vector#(n_CHAN_FROM_CORE,
            PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                         OCN_MSG_HEAD_AND_PAYLOAD)) reqToRemoteLLC = newVector();
    if (valueOf(n_CHAN_FROM_CORE) != 2)
    begin
        // OCN lane numbering isn't a vector.  Need a better solution here.
        error("Expected n_CHAN_FROM_CORE to be 2!");
    end
    reqToRemoteLLC[0] <- mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_LLC_REQ_0));
    reqToRemoteLLC[1] <- mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_LLC_REQ_1));

    // LLC responses use the memory response lane since the lane is not
    // otherwise used by core nodes.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) rspToRemoteLLC <-
        mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_SHARED_RSP_LLC_RSP));

    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) dummyMemRspToOCN <-
        mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_SHARED_RSP_MEM_RSP));

    //
    // Ports from the OCN to the LLC.
    //

    Vector#(n_CHAN_FROM_CORE,
            PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                         OCN_MSG_HEAD_AND_PAYLOAD)) reqInFromOCN = newVector();
    reqInFromOCN[0] <- mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_LLC_REQ_0));
    reqInFromOCN[1] <- mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_LLC_REQ_1));

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) rspInFromLLC <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_SHARED_RSP_LLC_RSP));

    // No memory request messages will arrive here, but the named port allocated
    // in the core's OCN interface has to be tied off.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) dummyReqInToMem <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_MEM_REQ));


    Vector#(TAdd#(6, TMul#(4, n_CHAN_FROM_CORE)),
            INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls = newVector();
    inctrls[0] = rspToCore.ctrl.in;
    inctrls[1] = rspFromLocalLLC.ctrl.in;
    inctrls[2] = rspInFromLLC.ctrl.in;
    inctrls[3] = rspToRemoteLLC.ctrl.in;
    inctrls[4] = dummyMemRspToOCN.ctrl.in;
    inctrls[5] = dummyReqInToMem.ctrl.in;
    for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
    begin
        inctrls[6 + i * 4] = reqFromCore[i].ctrl.in;
        inctrls[7 + i * 4] = reqToLocalLLC[i].ctrl.in;
        inctrls[8 + i * 4] = reqInFromOCN[i].ctrl.in;
        inctrls[9 + i * 4] = reqToRemoteLLC[i].ctrl.in;
    end

    Vector#(TAdd#(6, TMul#(4, n_CHAN_FROM_CORE)),
            INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = rspToCore.ctrl.out;
    outctrls[1] = rspFromLocalLLC.ctrl.out;
    outctrls[2] = rspInFromLLC.ctrl.out;
    outctrls[3] = rspToRemoteLLC.ctrl.out;
    outctrls[4] = dummyMemRspToOCN.ctrl.out;
    outctrls[5] = dummyReqInToMem.ctrl.out;
    for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
    begin
        outctrls[6 + i * 4] = reqFromCore[i].ctrl.out;
        outctrls[7 + i * 4] = reqToLocalLLC[i].ctrl.out;
        outctrls[8 + i * 4] = reqInFromOCN[i].ctrl.out;
        outctrls[9 + i * 4] = reqToRemoteLLC[i].ctrl.out;
    end


    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("LLC Hub", inctrls, outctrls);
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(LANE_IDX)) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple2#(LANE_IDX, OCN_FLIT))) stage3Ctrl <- mkStageController();

    //
    // Map core ID to network station ID.  This table is computed by the
    // software-side topology manager.
    //
    let coreToStationMapInit <-
        mkTopologyParamStream(`TOPOLOGY_NET_CORE_STATION_ID_MAP);
    LUTRAM#(CPU_INSTANCE_ID, STATION_ID) coreToStationMap <-
        mkLUTRAMWithGet(coreToStationMapInit);

    //
    // Local ports are a dynamic combination of CPUs, memory controllers, and
    // NULL connections.
    //
    // localPortMap indicates, for each multiplexed port instance ID, the type
    // of local port attached (CPU=0, memory controller=1, NULL=2).
    //
    let localPortInit <- mkTopologyParamStream(`TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP);
    LUTRAM#(STATION_ID, Bit#(2)) localPortMap <- mkLUTRAMWithGet(localPortInit);


    //
    // The LLC is distributed across all cores, with each address having a
    // single entry in a particular LLC.  We use a mapping table, similar
    // to the memory controller table above.  The table is initialized
    // by software.
    //
    
    let llcAddrMapInit <- mkTopologyParamStream(`TOPOLOGY_NET_LLC_ADDR_MAP);
    // The table holds 8 entries for every cache instance.
    LUTRAM#(Bit#(TLog#(TMul#(8, MAX_NUM_CPUS))), STATION_ID) llcDstForAddr <-
        mkLUTRAMWithGet(llcAddrMapInit);

    function STATION_ID getLLCDstForAddr(LINE_ADDRESS addr);
        return llcDstForAddr.sub(resize(addr));
    endfunction


    (* conservative_implicit_conditions *)
    rule stage1_routing (True);
        let cpu_iid <- localCtrl.startModelCycle();

        let station_id = coreToStationMap.sub(cpu_iid);

        //
        // Collect incoming messages.  The "receive" operation here is not a
        // commitment to process a message.  For that, doDeq() must be called.
        //
        let m_rspFromLocalLLC <- rspFromLocalLLC.receive(cpu_iid);
        let m_rspInFromLLC <- rspInFromLLC.receive(cpu_iid);

        Vector#(n_CHAN_FROM_CORE, Maybe#(CACHE_PROTOCOL_MSG)) m_reqFromCore = newVector();
        Vector#(n_CHAN_FROM_CORE,
                Maybe#(OCN_MSG_HEAD_AND_PAYLOAD)) m_reqInFromOCN = newVector();
        for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
        begin
            m_reqFromCore[i] <- reqFromCore[i].receive(cpu_iid);
            m_reqInFromOCN[i] <- reqInFromOCN[i].receive(cpu_iid);
        end

        // Tie off the dummy memory request port.
        let dummy_send_mem_rsp <- dummyMemRspToOCN.canEnq(cpu_iid);
        dummyMemRspToOCN.noEnq(cpu_iid);
        let dummy_recv_mem_req <- dummyReqInToMem.receive(cpu_iid);
        dummyReqInToMem.noDeq(cpu_iid);

        // Check credits for sending to output ports.
        let can_enq_rspToCore <- rspToCore.canEnq(cpu_iid);
        Vector#(n_CHAN_FROM_CORE, Bool) can_enq_reqToLocalLLC = newVector();
        Vector#(n_CHAN_FROM_CORE, Bool) can_enq_reqToRemoteLLC = newVector();
        for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
        begin
            can_enq_reqToLocalLLC[i] <- reqToLocalLLC[i].canEnq(cpu_iid);
            can_enq_reqToRemoteLLC[i] <- reqToRemoteLLC[i].canEnq(cpu_iid);
        end
        let can_enq_rspToRemoteLLC <- rspToRemoteLLC.canEnq(cpu_iid);

        //
        // Make routing choices.  The priority of competing ports is static.
        //

        Maybe#(CACHE_PROTOCOL_MSG) m_new_rspToCore = tagged Invalid;
        Vector#(n_CHAN_FROM_CORE,
                Maybe#(ROUTED_LLC_MSG)) m_new_reqToLocalLLC = replicate(tagged Invalid);
        Vector#(n_CHAN_FROM_CORE,
                Maybe#(Tuple2#(STATION_ID, CACHE_PROTOCOL_MSG))) m_new_reqToRemoteLLC =
            replicate(tagged Invalid);
        Maybe#(Tuple2#(STATION_ID, CACHE_PROTOCOL_MSG)) m_new_rspToRemoteLLC = tagged Invalid;

        Bool did_deq_rspFromLocalLLC = False;

        //
        // Local LLC response to either the local core or a remote core.
        //
        if (m_rspFromLocalLLC matches tagged Valid .rsp &&& can_enq_rspToCore)
        begin
            if (rsp.nodeID matches tagged Valid .dst)
            begin
                // Response is to a remote core.  Is the OCN port available?
                if (can_enq_rspToRemoteLLC && ! isValid(m_new_rspToRemoteLLC))
                begin
                    m_new_rspToRemoteLLC = tagged Valid tuple2(dst, rsp.msg);
                    did_deq_rspFromLocalLLC = True;
                    debugLog.record(cpu_iid, $format("1: LLC to remote Core, ") + fshow(rsp));
                end
            end
            // Reponse is to the local core.
            else if (can_enq_rspToCore &&& ! isValid(m_new_rspToCore))
            begin
                m_new_rspToCore = tagged Valid rsp.msg;
                did_deq_rspFromLocalLLC = True;
                debugLog.record(cpu_iid, $format("1: LLC to local Core, ") + fshow(rsp.msg));
            end
        end

        if (did_deq_rspFromLocalLLC)
        begin
            rspFromLocalLLC.doDeq(cpu_iid);
        end
        else
        begin
            rspFromLocalLLC.noDeq(cpu_iid);
        end

        //
        // Distributed LLC response from remote LLC.
        //
        if (m_rspInFromLLC matches tagged Valid .ocn_msg &&&
            can_enq_rspToCore &&&
            ! isValid(m_new_rspToCore))
        begin
            Tuple2#(STATION_ID, CACHE_PROTOCOL_MSG) ocn_rsp = cvtFromOCNFlits(ocn_msg);
            match {.src, .rsp} = ocn_rsp;

            m_new_rspToCore = tagged Valid rsp;
            rspInFromLLC.doDeq(cpu_iid);

            debugLog.record(cpu_iid, $format("1: Remote LLC %0d to local Core, ", src) + fshow(rsp));
        end
        else
        begin
            rspInFromLLC.noDeq(cpu_iid);
        end

        //
        // New requests from remote cores to local LLC.
        //
        for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
        begin
            if (m_reqInFromOCN[i] matches tagged Valid .ocn_msg &&&
                can_enq_reqToLocalLLC[i] &&&
                ! isValid(m_new_reqToLocalLLC[i]))
            begin
                Tuple2#(STATION_ID, CACHE_PROTOCOL_MSG) ocn_req = cvtFromOCNFlits(ocn_msg);
                match {.src, .lreq} = ocn_req;

                let req = ROUTED_LLC_MSG { nodeID: tagged Valid src, msg: lreq };
                m_new_reqToLocalLLC[i] = tagged Valid req;
                reqInFromOCN[i].doDeq(cpu_iid);
                debugLog.record(cpu_iid, $format("1: Remote REQ[%0d] to local LLC, ", i) + fshow(req));
            end
            else
            begin
                reqInFromOCN[i].noDeq(cpu_iid);
            end
        end
        
        //
        // New requests from the local core to the LLC.
        //
        for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
        begin
            Bool did_deq_reqFromCore = False;

            if (m_reqFromCore[i] matches tagged Valid .req)
            begin
                // Which instance of the distributed cache is responsible?
                let dst = getLLCDstForAddr(req.linePAddr);

                if (dst == station_id)
                begin
                    // Local cache handles the address.
                    if (can_enq_reqToLocalLLC[i] &&&
                        ! isValid(m_new_reqToLocalLLC[i]))
                    begin
                        // Port to LLC is available.  Send the local request.
                        did_deq_reqFromCore = True;
                        m_new_reqToLocalLLC[i] = tagged Valid ROUTED_LLC_MSG { nodeID: tagged Invalid,
                                                                               msg: req };
                        debugLog.record(cpu_iid, $format("1: Core REQ[%0d] to local LLC, ", i) + fshow(req));
                    end
                end
                else if (can_enq_reqToRemoteLLC[i] &&
                         ! isValid(m_new_reqToRemoteLLC[i]))
                begin
                    // Remote cache instance handles the address and the OCN request
                    // port is available.
                    //
                    // These requests share the OCN request port since only one
                    // type of request goes to a given remote station.  Memory
                    // stations get memory requests above.  LLC stations get
                    // core requests here.
                    did_deq_reqFromCore = True;
                    m_new_reqToRemoteLLC[i] = tagged Valid tuple2(dst, req);
                    debugLog.record(cpu_iid, $format("1: Core REQ[%0d] to LLC %0d, ", i, dst) + fshow(req));
                end
            end

            if (did_deq_reqFromCore)
            begin
                reqFromCore[i].doDeq(cpu_iid);
            end
            else
            begin
                reqFromCore[i].noDeq(cpu_iid);
            end
        end


        //
        // Transmit routing decisions...
        //

        if (m_new_rspToCore matches tagged Valid .rsp)
        begin
            rspToCore.doEnq(cpu_iid, rsp);
        end
        else
        begin
            rspToCore.noEnq(cpu_iid);
        end

        for (Integer i = 0; i < valueOf(n_CHAN_FROM_CORE); i = i + 1)
        begin
            if (m_new_reqToLocalLLC[i] matches tagged Valid .req)
            begin
                reqToLocalLLC[i].doEnq(cpu_iid, req);
            end
            else
            begin
                reqToLocalLLC[i].noEnq(cpu_iid);
            end

            if (m_new_reqToRemoteLLC[i] matches tagged Valid {.tgt, .req})
            begin
                reqToRemoteLLC[i].doEnq(cpu_iid, cvtToOCNFlits(tgt, req));
            end
            else
            begin
                reqToRemoteLLC[i].noEnq(cpu_iid);
            end
        end

        if (m_new_rspToRemoteLLC matches tagged Valid {.tgt, .rsp})
        begin
            rspToRemoteLLC.doEnq(cpu_iid, cvtToOCNFlits(tgt, rsp));
        end
        else
        begin
            rspToRemoteLLC.noEnq(cpu_iid);
        end

        localCtrl.endModelCycle(cpu_iid, 0);
        debugLog.nextModelCycle(cpu_iid);
    endrule
endmodule


// ========================================================================
//
//  Distributed LLC implementation.
//
// ========================================================================

//
// States associated with entries.  Invalid isn't listed because it is provided
// as part of the cache algorithm.
//
typedef enum
{
    // Shared here and possibly in L1 or L2.
    LLC_STATE_SS,
    // Modified here and possibly in shared state in L1 or L2.
    LLC_STATE_MS,
    // Present here and possibly modified in some L1 or L2.
    LLC_STATE_SM,
    // Modified here and possibly modified with newer value in some L1 or L2.
    LLC_STATE_MM,

    // Invalidating shared copies of line in all L1 and L2 caches.
    LLC_STATE_S_INVAL,
    // Invalidating line in L1 and L2.  Writeback required at completion.
    LLC_STATE_M_INVAL,
    // Fill to shared in progress.  No data yet.
    LLC_STATE_FILL_S,
    // Fill to exclusive in progress.  No data yet.
    LLC_STATE_FILL_E
}
LLC_ENTRY_STATE
    deriving (Eq, Bits);

instance FShow#(LLC_ENTRY_STATE);
    function Fmt fshow(LLC_ENTRY_STATE state);
        let str =
            case (state) matches
                LLC_STATE_SS: return "SS";
                LLC_STATE_MS: return "MS";
                LLC_STATE_SM: return "SM";
                LLC_STATE_MM: return "MM";
                LLC_STATE_S_INVAL: return "S_INVAL";
                LLC_STATE_M_INVAL: return "M_INVAL";
                LLC_STATE_FILL_S: return "FILL_S";
                LLC_STATE_FILL_E: return "FILL_E";
                default: return "UNDEFINED";
            endcase;

        return $format(str);
    endfunction
endinstance

instance FShow#(Tuple2#(LLC_ENTRY_STATE, LLC_ENTRY_STATE));
    function Fmt fshow(Tuple2#(LLC_ENTRY_STATE, LLC_ENTRY_STATE) states);
        match {.s_from, .s_to} = states;
        return fshow(s_from) + $format("->") + fshow(s_to);
    endfunction
endinstance


//
// Miss tracking state.
//
typedef `LLC_MISS_ID_SIZE LLC_MISS_ID_SIZE;
typedef Bit#(LLC_MISS_ID_SIZE) LLC_MISS_ID;

instance FShow#(Maybe#(LLC_MISS_ID));
    function Fmt fshow(Maybe#(LLC_MISS_ID) m_id);
        if (m_id matches tagged Valid .id)
            return $format("%0d", id);
        else
            return $format("NIL");
    endfunction
endinstance


typedef struct
{
    Maybe#(STATION_ID) reqNode;
    MEM_OPAQUE opaque;
}
LLC_MISS_TOKEN
    deriving (Eq, Bits);


//
// The meta-data stored in each cache entry is the line's state and a pointer
// to a set of miss tokens.  In real hardware the miss tracker would be
// a CAM indexed by address, which would require much less memory than the
// extra state we store along with each line. CAMs are expensive on FPGAs,
// so we simulate the CAM by adding a pre-sorted list from each line's
// meta-data.  The timing is the same.
//
typedef struct
{
    LLC_ENTRY_STATE state;

    // The miss token list pointers have meaning only when the entry is in
    // a fill state.
    LLC_MISS_ID missTokenListHead;
    LLC_MISS_ID missTokenListTail;
}
LLC_ENTRY_META
    deriving (Eq, Bits);

function LLC_ENTRY_META initLLCEntryMeta(LLC_ENTRY_STATE state);
    return LLC_ENTRY_META { state: state,
                            missTokenListHead: ?,
                            missTokenListTail: ? };
endfunction

instance FShow#(LLC_ENTRY_META);
    function Fmt fshow(LLC_ENTRY_META entryMeta);
        Fmt msg = $format("[") + fshow(entryMeta.state);

        if ((entryMeta.state == LLC_STATE_FILL_S) ||
            (entryMeta.state == LLC_STATE_FILL_E))
        begin
            msg = msg + $format(", missIDs (%0d, %0d)", entryMeta.missTokenListHead, entryMeta.missTokenListTail);
        end

        return msg + $format("]");
    endfunction
endinstance


typedef Maybe#(ROUTED_LLC_MSG) LLC_OPER;
function LLC_OPER llcInval() = tagged Invalid;

//
// LLC_LOCAL_STATE
//
// Local State to pass between pipeline stages.
//
typedef struct
{
    // Miss token free list head.  Cached so the state is only read in
    // one place.
    Maybe#(LLC_MISS_ID) tokenFreeList;

    // Once initialized with a cache lookup, cacheMeta is the current entry's
    // state.  cacheMeta is updated in the pipeline and will be written back
    // to the entry if cacheUpdUsed is set.
    LLC_ENTRY_META cacheMeta;

    Bool cacheUpdUsed;
    LLC_CACHE_IDX cacheUpdIdx;
    LINE_ADDRESS cacheUpdPAddr;

    Bool toMemQNotFull;
    Maybe#(MEMORY_REQ) toMemQ;
    
    Bool toCoreQNotFull;
    Maybe#(ROUTED_LLC_MSG) toCoreQ;
}
LLC_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(LLC_LOCAL_STATE);
    defaultValue = LLC_LOCAL_STATE { 
        tokenFreeList: tagged Invalid,
        cacheMeta: initLLCEntryMeta(?),
        cacheUpdUsed: False,
        cacheUpdIdx: ?,
        cacheUpdPAddr: ?,
        toMemQNotFull: False,
        toMemQ: tagged Invalid,
        toCoreQNotFull: False,
        toCoreQ: tagged Invalid
        };
endinstance


//
// mkDistributedLastLevelCache --
//   Cache management.  Each core has an associated portion of the distributed
//   LLC.
//
module [HASIM_MODULE] mkDistributedLastLevelCache();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc_distrib.out");

    // ****** Submodels ******

    //
    // The cache algorithm which determines hits, misses, and evictions.
    // The mayEvict function is used inside the algorithm when returning
    // a candidate for replacement.  mayEvict indicates when an entry
    // may not be victimized.
    //
    function Bool mayEvict(LLC_ENTRY_META entryMeta) =
        (pack(entryMeta.state) <= pack(LLC_STATE_MM));

    LLC_CACHE_ALG#(MAX_NUM_CPUS, LLC_ENTRY_META) llcAlg <-
        mkLastLevelCacheAlg(mayEvict);


    // ****** Miss tracking state ******

    // Track the next Miss ID to give out.  Each instance has its own list.
    // The starting value is entry 0 in each instance's group.  The
    // remainder of the list is initialized with missTrackerPool.
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Maybe#(LLC_MISS_ID)) missTokenFreeListPool <-
        mkMultiplexedReg(tagged Valid 0);

    // Miss tracker storage.  Each instance has log2(LLC_MISS_ID_SIZE) entries.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS,
                            LLC_MISS_ID,
                            LLC_MISS_TOKEN) missTrackerDataPool <-
        mkMemory_Multiplexed(mkBRAM);

    // Each entry in the missTrackerDataPool has a next pointer.  The pointer
    // is stored in a separate memory to manage read/write ports.
    function Maybe#(LLC_MISS_ID) freeListInitFunc(Bit#(n_BITS) idx);
        LLC_MISS_ID cur_id = truncateNP(idx);

        // Next id is the following entry or Invalid if cur_id is the last one.
        return
            (cur_id == maxBound ? tagged Invalid : tagged Valid (cur_id + 1));
    endfunction

    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS,
                            LLC_MISS_ID,
                            Maybe#(LLC_MISS_ID)) missTrackerNextPool <-
        mkMemory_Multiplexed(mkBRAMInitializedWith(freeListInitFunc));



    // ****** Ports ******

    // Queues to/from Cache hierarchy.
    Vector#(2, PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                            ROUTED_LLC_MSG)) reqFromCore = newVector();
    reqFromCore[0] <- mkPortStallRecv_Multiplexed("LLCHub_to_LLC_req_0");
    reqFromCore[1] <- mkPortStallRecv_Multiplexed("LLCHub_to_LLC_req_1");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_MSG) rspToCore <-
        mkPortStallSend_Multiplexed("LLC_to_LLCHub_rsp");
    
    // Requests to memory from the LLC instance responsible for an address.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) reqToMem <-
        mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_MEM_REQ));
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) rspFromMem <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_SHARED_RSP_MEM_RSP));
    
    Vector#(5, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = reqFromCore[0].ctrl.in;
    inctrls[1] = reqFromCore[1].ctrl.in;
    inctrls[2] = rspToCore.ctrl.in;
    inctrls[3] = reqToMem.ctrl.in;
    inctrls[4] = rspFromMem.ctrl.in;

    Vector#(5, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = reqFromCore[0].ctrl.out;
    outctrls[1] = reqFromCore[1].ctrl.out;
    outctrls[2] = rspToCore.ctrl.out;
    outctrls[3] = reqToMem.ctrl.out;
    outctrls[4] = rspFromMem.ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("LLC", inctrls, outctrls);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(LLC_OPER,
                                            LLC_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(LLC_OPER,
                                            LLC_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(LLC_OPER,
                                            LLC_LOCAL_STATE)) stage4Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(LLC_OPER,
                                            LLC_LOCAL_STATE)) stage5Ctrl <-
        mkStageController();

    //
    // Physical addresses are assigned to memory controllers during setup
    // by software.  The map table is larger than the number of controllers
    // in order to enable relatively even mapping even when the number of
    // controllers isn't a power of two.  A large map also makes it
    // unnecessary to hash the addresses.
    //

    let ctrlAddrMapInit <- mkTopologyParamStream(`TOPOLOGY_NET_MEM_CTRL_MAP);
    // The table holds 8 entries for every memory controller.
    LUTRAM#(Bit#(TLog#(TMul#(8, MAX_NUM_MEM_CTRLS))),
            STATION_ID) memCtrlDstForAddr <-
        mkLUTRAMWithGet(ctrlAddrMapInit);

    function STATION_ID getMemCtrlDstForAddr(LINE_ADDRESS addr);
        return memCtrlDstForAddr.sub(resize(addr));
    endfunction


    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statReadHit <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_READ_HIT",
                                           "LLC Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_READ_MISS",
                                           "LLC Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statReadRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_READ_RETRY",
                                           "LLC Read Retries"));

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_LLC_HIT);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_LLC_MISS);

    // ****** Assertions ******

    Vector#(2, ASSERTION_STR_CLIENT) assertNode <- mkAssertionStrClientVec();

    let assertExpectedMessage <-
        mkAssertionStrCheckerWithMsg("last-level-cache-MESI.bsv: Unexpected message type!",
                                     ASSERT_ERROR,
                                     assertNode[0]);
    let assertValidState <-
        mkAssertionStrCheckerWithMsg("last-level-cache-MESI.bsv: Cache in invalid state!",
                                     ASSERT_ERROR,
                                     assertNode[1]);

    // ****** Functions ******

    //
    // getOper --
    //   Pick out the LLC_OPER argument from a stage controller payload.
    //   Assumes the operation is always the first element in the payload
    //   tuple.
    //
    function LLC_OPER getOper(STAGE_CONTROLLER#(MAX_NUM_CPUS, t_ARGS) ctrl)
        provisos (Has_tpl_1#(t_ARGS, LLC_OPER));

        match {.cpu_iid, .payload} = ctrl.peekReadyInstance();
        return tpl_1(payload);
    endfunction


    // ****** Rules ******

    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = defaultValue;

        // Check whether the request ports have room for any new requests.
        local_state.toMemQNotFull <- reqToMem.canEnq(cpu_iid);
        local_state.toCoreQNotFull <- rspToCore.canEnq(cpu_iid);

        // Consume incoming messages
        let m_mem_rsp <- rspFromMem.receive(cpu_iid);
        Vector#(2, Maybe#(ROUTED_LLC_MSG)) m_core_req = newVector();
        for (Integer i = 0; i < 2; i = i + 1)
        begin
            m_core_req[i] <- reqFromCore[i].receive(cpu_iid);
        end

        //
        // Pick an action for the model cycle.
        //

        LLC_OPER m_oper = llcInval();

        if (m_mem_rsp matches tagged Valid .ocn_msg)
        begin
            //
            // FILL received from memory.
            //
            Tuple2#(STATION_ID, MEMORY_RSP) ocn_rsp = cvtFromOCNFlits(ocn_msg);
            match {.dst, .rsp} = ocn_rsp;

            if (local_state.toCoreQNotFull)
            begin
                //
                // Turn the MEMORY_RSP into a CACHE_PROTOCOL_MSG.  The new
                // message will be a RSP_LOAD.  This is the only source of
                // incoming RSP_LOAD messages, so is easily distinguished
                // from other messages.
                //
                // The destination won't be known until the miss entry is read.
                //
                ROUTED_LLC_MSG rrsp = ROUTED_LLC_MSG { nodeID: tagged Invalid,
                                                       msg: cacheMsgFromMemRsp(rsp) };
                m_oper = tagged Valid rrsp;

                Fmt dbg_msg = $format("1: FILL RSP: ") + fshow(rrsp);
                debugLog.record(cpu_iid, dbg_msg);
                assertExpectedMessage(cacheMsg_IsRspLoad(rrsp.msg),
                                      dbg_msg + $format(", cpu %0d", cpu_iid));
            end
            else
            begin
                debugLog.record(cpu_iid, $format("1: RETRY FROM MEM: ") + fshow(rsp));
            end
        end
        else if (m_core_req[0] matches tagged Valid .req)
        begin
            //
            // New request from core received.
            //
            Fmt dbg_msg;

            if (local_state.toCoreQNotFull && local_state.toMemQNotFull)
            begin
                m_oper = tagged Valid req;
                dbg_msg = $format("1: FROM CORE[0]: ") + fshow(req);
            end
            else
            begin
                dbg_msg = $format("1: RETRY FROM CORE[0]: ") + fshow(req);
            end

            debugLog.record(cpu_iid, dbg_msg);

            // Only load requests come in on this channel.
            assertExpectedMessage(cacheMsg_IsReqLoad(req.msg),
                                  dbg_msg + $format(", cpu %0d", cpu_iid));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        stage2Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage2 (True);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        //
        // Look up the request's address in the cache.
        //
        if (m_oper matches tagged Valid .oper)
        begin
            Bool upd_replacement = False;
            Bool is_load = False;

            if (oper.msg.kind matches tagged REQ_LOAD .fill_meta)
            begin
                upd_replacement = True;
                is_load = ! fill_meta.exclusive;
            end

            llcAlg.lookupByAddrReq(cpu_iid, oper.msg.linePAddr,
                                   upd_replacement, is_load);

            debugLog.record(cpu_iid, $format("2: LOOKUP: upd %0d, isLoad %0d, ", upd_replacement, is_load) + fshow(oper));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("2: Bubble"));
        end

        // Read the context's free list head pointer here.  Reading it multiple
        // times would replicate the storage, which we avoid by storing it in
        // local_state.
        Reg#(Maybe#(LLC_MISS_ID)) missTokenFreeList =
            missTokenFreeListPool.getReg(cpu_iid);
        local_state.tokenFreeList = missTokenFreeList;

        stage3Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule



    // ====================================================================
    //
    //  Stage 3:  Exactly one stage3 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage3_REQ_LOAD (getOper(stage3Ctrl) matches tagged Valid .req &&&
                          req.msg.kind matches tagged REQ_LOAD .load_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let src = req.nodeID;
        let msg = req.msg;

        let entry <- llcAlg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = msg.linePAddr;

        // Hit/miss events will be recorded in this rule.
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;

        if (entry.state matches tagged Valid .state)
        begin
            LLC_ENTRY_STATE line_state = state.opaque.state;

            // Line is already in the cache
            if (pack(line_state) <= pack(LLC_STATE_MM))
            begin
                // FIXME.  In case respond always as hit.
                CACHE_PROTOCOL_RSP_LOAD rsp_meta = defaultValue;
                rsp_meta.exclusive = load_meta.exclusive;

                let rsp = ROUTED_LLC_MSG { nodeID: src,
                                           msg: cacheMsg_RspLoad(msg.linePAddr,
                                                                 msg.opaque,
                                                                 rsp_meta) };
                local_state.toCoreQ = tagged Valid rsp;

                statReadHit.incr(cpu_iid);
                evt_hit = tagged Valid resize({ msg.linePAddr, 1'b0 });

                debugLog.record(cpu_iid, $format("3: REQ_LOAD HIT (") + fshow(line_state) + $format("): ") + fshow(req) + $format(", ") + fshow(entry.idx));
            end
            else
            begin
                // Line is already in transition due to an earlier request
                // from another core.  Retry later.
                m_oper = llcInval();
                local_state = defaultValue;
                statReadRetry.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: REQ_LOAD RETRY (") + fshow(line_state) + $format("): ") + fshow(req) + $format(", ") + fshow(entry.idx));
            end
        end
        else if (entry.state matches tagged Blocked)
        begin
            //
            // Line not present and no victim is available due to outstanding
            // activity.  Retry later.
            //
            m_oper = llcInval();
            local_state = defaultValue;
            statReadRetry.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("3: REQ_LOAD BLOCKED (retry later): ") + fshow(req));
        end
        else if (local_state.tokenFreeList matches tagged Valid .miss_tok_idx)
        begin
            // Fill the line.  (FIXME:  Need to evict first!)
            let mem_req = initMemLoad(msg.linePAddr);
            mem_req.opaque = msg.opaque;
            local_state.toMemQ = tagged Valid mem_req;

            // Set the new line's state.
            local_state.cacheUpdUsed = True;
            let new_state = load_meta.exclusive ? LLC_STATE_FILL_E :
                                                  LLC_STATE_FILL_S;
            local_state.cacheMeta = initLLCEntryMeta(new_state);
            local_state.cacheMeta.missTokenListHead = miss_tok_idx;
            local_state.cacheMeta.missTokenListTail = miss_tok_idx;

            // Record the load request details in the miss tracker.
            missTrackerDataPool.write(cpu_iid, miss_tok_idx,
                                      LLC_MISS_TOKEN { reqNode: src,
                                                       opaque: msg.opaque });

            // Read the miss token's next entry in order to update the free
            // list in the next stage.
            missTrackerNextPool.readReq(cpu_iid, miss_tok_idx);

            statReadMiss.incr(cpu_iid);
            evt_miss = tagged Valid resize({ msg.linePAddr, 1'b0 });

            debugLog.record(cpu_iid, $format("3: REQ_LOAD FILL (") + fshow(local_state.cacheMeta) + $format("): ") + fshow(req) + $format(", ") + fshow(entry.idx));
        end
        else
        begin
            //
            // Line not present and miss token is available.  Retry later.
            //
            m_oper = llcInval();
            local_state = defaultValue;
            statReadRetry.incr(cpu_iid);
            debugLog.record(cpu_iid, $format("3: REQ_LOAD NO FREE MISS TOKEN (retry later): ") + fshow(req));
        end

        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    rule stage3_RSP_LOAD (getOper(stage3Ctrl) matches tagged Valid .rsp &&&
                          rsp.msg.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let msg = rsp.msg;

        let entry <- llcAlg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = msg.linePAddr;

        //
        // The entry must be in a FILL state.
        //
        if (entry.state matches tagged Valid .state)
        begin
            local_state.cacheMeta = state.opaque;

            LLC_ENTRY_STATE line_state = state.opaque.state;
            case (line_state)
                LLC_STATE_FILL_S:
                begin
                    local_state.cacheUpdUsed = True;
                    local_state.cacheMeta.state = LLC_STATE_SS;

                    debugLog.record(cpu_iid, $format("3: FILL_S->") + fshow(local_state.cacheMeta) + $format(": ") + fshow(rsp) + $format(", ") + fshow(entry.idx));
                end

                LLC_STATE_FILL_E:
                begin
                    local_state.cacheUpdUsed = True;
                    local_state.cacheMeta.state = LLC_STATE_SM;

                    debugLog.record(cpu_iid, $format("3: FILL_E->") + fshow(local_state.cacheMeta) + $format(": ") + fshow(rsp) + $format(", ") + fshow(entry.idx));
                end

                default:
                begin
                    // Error.  Expected FILL_S or FILL_E.
                    Fmt dbg_msg = $format("3: FILL: Must be in FILL_S or FILL_E but in ") + fshow(line_state) + $format(", ") + fshow(rsp);
                    debugLog.record(cpu_iid, dbg_msg);
                    assertValidState(False, dbg_msg + $format(", cpu %0d", cpu_iid));

                    m_oper = llcInval();
                    local_state = defaultValue;
                end
            endcase
        end
        else
        begin
            // Error.  Expected FILL_S or FILL_E.
            Fmt dbg_msg = $format("3: FILL: Must be in FILL_S or FILL_E but is INVALID, ") + fshow(rsp);
            debugLog.record(cpu_iid, dbg_msg);
            assertValidState(False, dbg_msg + $format(", cpu %0d", cpu_iid));

            m_oper = llcInval();
            local_state = defaultValue;
        end

        //
        // Cache was in acceptable state and fill response will be sent.
        // Get the target of the response from the miss tracker.
        //
        if (isValid(m_oper))
        begin
            missTrackerDataPool.readReq(cpu_iid,
                                        local_state.cacheMeta.missTokenListHead);

            // This token entry will be pushed as the head of the free list.
            missTrackerNextPool.write(cpu_iid,
                                      local_state.cacheMeta.missTokenListHead,
                                      local_state.tokenFreeList);

            debugLog.record(cpu_iid, $format("3: RSP_LOAD missID %0d next is ", local_state.cacheMeta.missTokenListHead) + fshow(local_state.tokenFreeList));

            local_state.tokenFreeList = tagged Valid local_state.cacheMeta.missTokenListHead;
        end

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage3_WB_INVAL (getOper(stage3Ctrl) matches tagged Valid .wb &&&
                          wb.msg.kind matches tagged WB_INVAL .wb_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let src = wb.nodeID;
        let msg = wb.msg;

        let entry <- llcAlg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = msg.linePAddr;

        //
        // Given the eviction protocol the entry must be present.  No entry may
        // be in a private cache without also being in the LLC.
        //
        if (entry.state matches tagged Valid .state)
        begin
            LLC_ENTRY_STATE line_state = state.opaque.state;
            let new_state = line_state;

            debugLog.record(cpu_iid, $format("3: WB_INVAL (") + fshow(tuple2(line_state, new_state)) + $format("): ") + fshow(wb) + $format(", ") + fshow(entry.idx));

            case (line_state)
                LLC_STATE_SS,
                LLC_STATE_MS:
                begin
                    // LLC believes private caches are in S state.  They better
                    // not believe the private line was exclusive or dirty.
                    assertValidState(! wb_meta.exclusive && ! wb_meta.dirty,
                                     fshow(line_state) + $format(", cpu %0d", cpu_iid));
                end

                LLC_STATE_SM:
                begin
                    if (wb_meta.dirty) new_state = LLC_STATE_MM;
                end

                LLC_STATE_MM,
                LLC_STATE_M_INVAL:
                begin
                    noAction;
                end

                LLC_STATE_S_INVAL:
                begin
                    if (wb_meta.dirty) new_state = LLC_STATE_M_INVAL;
                end

                default:
                begin
                    assertValidState(False,
                                     fshow(line_state) + $format(", cpu %0d", cpu_iid));
                end
            endcase

            local_state.cacheUpdUsed = True;
            local_state.cacheMeta.state = new_state;
        end
        else
        begin
            // Error: not present!
            Fmt dbg_msg = $format("3: WB_INVAL: Line not present, ") + fshow(wb);
            debugLog.record(cpu_iid, dbg_msg);
            assertValidState(False, dbg_msg + $format(", cpu %0d", cpu_iid));

            m_oper = llcInval();
            local_state = defaultValue;
        end

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule


    rule stage3_FORCE_INVAL (getOper(stage3Ctrl) matches tagged Valid .req &&&
                             req.msg.kind matches tagged FORCE_INVAL .meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        let dst = req.nodeID;
        let msg = req.msg;

        let entry <- llcAlg.lookupByAddrRsp(cpu_iid);
        local_state.cacheUpdIdx = entry.idx;
        local_state.cacheUpdPAddr = msg.linePAddr;

        // Forward to core private caches.
        local_state.toCoreQ = tagged Valid req;

        debugLog.record(cpu_iid, $format("3: FWD TO Core[%0d]: ", dst) + fshow(msg));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage3_Invalid (getOper(stage3Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("3: Bubble"));

        eventHit.recordEvent(cpu_iid, tagged Invalid);
        eventMiss.recordEvent(cpu_iid, tagged Invalid);

        stage4Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule



    // ====================================================================
    //
    //  Stage 4:  Exactly one stage4 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage4_REQ_LOAD (getOper(stage4Ctrl) matches tagged Valid .req &&&
                          req.msg.kind matches tagged REQ_LOAD .load_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        //
        // Update the miss token free list if a token was allocated.
        //
        if (isValid(local_state.toMemQ))
        begin
            Reg#(Maybe#(LLC_MISS_ID)) missTokenFreeList =
                missTokenFreeListPool.getReg(cpu_iid);

            let new_head <- missTrackerNextPool.readRsp(cpu_iid);
            missTokenFreeList <= new_head;

            debugLog.record(cpu_iid, $format("4: REQ_LOAD tok update free head missID ") + fshow(new_head));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("4: REQ_LOAD Bubble"));
        end

        stage5Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage4_RSP_LOAD (getOper(stage4Ctrl) matches tagged Valid .rsp &&&
                          rsp.msg.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        //
        // Update the response with correct destination and state.
        //
        let miss_info <- missTrackerDataPool.readRsp(cpu_iid);
        ROUTED_LLC_MSG upd_rsp = rsp;
        upd_rsp.nodeID = miss_info.reqNode;
        upd_rsp.msg.opaque = miss_info.opaque;

        //
        // Is the response exclusive access?  Update the RSP_LOAD if necessary.
        // The .kind definitely be RSP_LOAD if this point is reached.  The
        // test is required by Bluespec.
        //
        if (local_state.cacheMeta.state == LLC_STATE_SM &&&
            upd_rsp.msg.kind matches tagged RSP_LOAD .info)
        begin
            let upd_info = info;
            upd_info.exclusive = True;
            upd_rsp.msg.kind = tagged RSP_LOAD upd_info;
        end

        // Forward response to the core
        local_state.toCoreQ = tagged Valid upd_rsp;

        //
        // Put the miss entry back on the free list.
        //
        Reg#(Maybe#(LLC_MISS_ID)) missTokenFreeList =
            missTokenFreeListPool.getReg(cpu_iid);
        missTokenFreeList <= local_state.tokenFreeList;

        debugLog.record(cpu_iid, $format("4: RSP_LOAD free missID ") + fshow(local_state.tokenFreeList));
        debugLog.record(cpu_iid, $format("4: ") + fshow(upd_rsp));

        stage5Ctrl.ready(cpu_iid, tuple2(tagged Valid upd_rsp, local_state));
    endrule

    rule stage4_WB_INVAL (getOper(stage4Ctrl) matches tagged Valid .wb &&&
                          wb.msg.kind matches tagged WB_INVAL .wb_meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("4: WB_INVAL Bubble"));

        stage5Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage4_FORCE_INVAL (getOper(stage4Ctrl) matches tagged Valid .req &&&
                             req.msg.kind matches tagged FORCE_INVAL .meta);

        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("4: FORCE_INVAL Bubble"));

        stage5Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule

    rule stage4_Invalid (getOper(stage4Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("4: Bubble"));

        stage5Ctrl.ready(cpu_iid, tuple2(m_oper, local_state));
    endrule



    // ====================================================================
    //
    //   Flow merges back to a single rule for the final message routing.
    //
    // ====================================================================

    rule stage5_end (True);
        match {.cpu_iid, {.m_oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("5: Done"));

        //
        // Indicate whether requests were consumed.
        //

        if (m_oper matches tagged Valid .oper &&&
            cacheMsg_IsRspLoad(oper.msg))
        begin
            rspFromMem.doDeq(cpu_iid);
        end
        else
        begin
            rspFromMem.noDeq(cpu_iid);
        end


        //
        // Messages from the core are constrained to come in a specific channels
        // based on message type.
        //

        if (m_oper matches tagged Valid .oper &&&
            cacheMsg_IsReqLoad(oper.msg))
        begin
            reqFromCore[0].doDeq(cpu_iid);
        end
        else
        begin
            reqFromCore[0].noDeq(cpu_iid);
        end

        if (m_oper matches tagged Valid .oper &&&
            cacheMsg_IsWBInval(oper.msg))
        begin
            reqFromCore[1].doDeq(cpu_iid);
        end
        else
        begin
            reqFromCore[1].noDeq(cpu_iid);
        end


        //
        // Send requests/responses.
        //

        // Take care of the memory queue.
        if (local_state.toMemQ matches tagged Valid .mem_req)
        begin
            // Which memory controller handles the request's address?
            let dst = getMemCtrlDstForAddr(mem_req.linePAddr);

            reqToMem.doEnq(cpu_iid, cvtToOCNFlits(dst, mem_req));
            debugLog.record(cpu_iid, $format("5: REQ TO MEM %0d: ", dst) + fshow(mem_req));
        end
        else
        begin
            reqToMem.noEnq(cpu_iid);
        end
        
        // Take care of CPU rsp
        if (local_state.toCoreQ matches tagged Valid .rsp)
        begin
            rspToCore.doEnq(cpu_iid, rsp);
        end
        else
        begin
            rspToCore.noEnq(cpu_iid);
        end


        // Take care of the cache update.
        if (local_state.cacheUpdUsed)
        begin
            LLC_ENTRY_STATE new_state = local_state.cacheMeta.state;
            Bool is_dirty = ((new_state == LLC_STATE_MS) ||
                             (new_state == LLC_STATE_MM) ||
                             (new_state == LLC_STATE_M_INVAL));

            CACHE_ENTRY_STATE#(LLC_ENTRY_META) state =
                initCacheEntryState(local_state.cacheUpdPAddr,
                                    is_dirty,
                                    local_state.cacheMeta);

            if (m_oper matches tagged Valid .oper &&&
                oper.msg.kind matches tagged REQ_LOAD .rsp)
            begin
                // Fill allocates a new line.
                llcAlg.allocate(cpu_iid,
                                local_state.cacheUpdIdx,
                                state,
                                CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("5: ALLOC: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                // Update an existing entry.
                llcAlg.update(cpu_iid,
                              local_state.cacheUpdIdx,
                              state);

                debugLog.record(cpu_iid, $format("5: UPDATE: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
        end
        
        // End of model cycle.
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
