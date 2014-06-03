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


// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/librl_bsv_base.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_chip_topology.bsh"
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
// For the distributed LLC with no coherence protocol, requests are just
// memory requests.
//
typedef MEMORY_REQ LLC_REQ;
typedef MEMORY_RSP LLC_RSP;


//
// An LLC memory request is a memory request coming either from the local core
// (src is Invalid) or from a remote network station.  Routed requests contain
// the base request plus routing information.
//
typedef struct
{
    Maybe#(STATION_ID) src;
    LLC_REQ lreq;
}
ROUTED_LLC_REQ
    deriving (Eq, Bits);

typedef struct
{
    Maybe#(STATION_ID) dst;
    LLC_RSP lrsp;
}
ROUTED_LLC_RSP
    deriving (Eq, Bits);

function ROUTED_LLC_RSP initLLCMemRspFromReq(ROUTED_LLC_REQ req);
    return ROUTED_LLC_RSP { dst: req.src, lrsp: initMemRspFromReq(req.lreq) };
endfunction

instance FShow#(ROUTED_LLC_REQ);
    function Fmt fshow(ROUTED_LLC_REQ req);
        let src_msg =
            case (req.src) matches
                tagged Valid .src: return $format(", src=%0d", src);
                tagged Invalid: return $format(", local");
            endcase;
        return fshow(req.lreq) + src_msg;
    endfunction
endinstance

instance FShow#(ROUTED_LLC_RSP);
    function Fmt fshow(ROUTED_LLC_RSP req);
        let dst_msg =
            case (req.dst) matches
                tagged Valid .dst: return $format(", dst=%0d", dst);
                tagged Invalid: return $format(", local");
            endcase;
        return fshow(req.lrsp) + dst_msg;
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
module [HASIM_MODULE] mkLastLevelCache();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc.out");

    // Instantiate other LLC components
    let llc_distrib <- mkDistributedLastLevelCache();

    //
    // Requests from the local core arrive here.  This router will forward
    // the request to the correct distributed LLC segment.
    //
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, LLC_REQ) reqFromCore <-
        mkPortStallRecv_Multiplexed("CorePvtCache_to_UncoreQ");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, LLC_RSP) rspToCore <-
        mkPortStallSend_Multiplexed("Uncore_to_CorePvtCacheQ");
    
    //
    // Queues to/from local LLC segment.  These may either come from the local
    // core or a remote core.
    //
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_REQ) reqToLocalLLC <-
        mkPortStallSend_Multiplexed("LLCHub_to_LLC_req");
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_RSP) rspFromLocalLLC <-
        mkPortStallRecv_Multiplexed("LLC_to_LLCHub_rsp");

    //
    // Ports from the LLC to the OCN.  The names of the ports are picked
    // indirectly through dictionary entries since the interface to the
    // OCN uses model-independent port/lane name bindings.
    //

    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) reqToRemoteLLC <-
        mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_LLC_REQ));

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

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) reqInFromOCN <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_LLC_REQ));

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) rspInFromLLC <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_SHARED_RSP_LLC_RSP));

    // No memory request messages will arrive here, but the named port allocated
    // in the core's OCN interface has to be tied off.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) dummyReqInToMem <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_MEM_REQ));


    Vector#(10, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = reqFromCore.ctrl.in;
    inctrls[1] = rspToCore.ctrl.in;
    inctrls[2] = reqToLocalLLC.ctrl.in;
    inctrls[3] = rspFromLocalLLC.ctrl.in;
    inctrls[4] = reqInFromOCN.ctrl.in;
    inctrls[5] = rspInFromLLC.ctrl.in;
    inctrls[6] = reqToRemoteLLC.ctrl.in;
    inctrls[7] = rspToRemoteLLC.ctrl.in;
    inctrls[8] = dummyMemRspToOCN.ctrl.in;
    inctrls[9] = dummyReqInToMem.ctrl.in;

    Vector#(10, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqToLocalLLC.ctrl.out;
    outctrls[3] = rspFromLocalLLC.ctrl.out;
    outctrls[4] = reqInFromOCN.ctrl.out;
    outctrls[5] = rspInFromLLC.ctrl.out;
    outctrls[6] = reqToRemoteLLC.ctrl.out;
    outctrls[7] = rspToRemoteLLC.ctrl.out;
    outctrls[8] = dummyMemRspToOCN.ctrl.out;
    outctrls[9] = dummyReqInToMem.ctrl.out;


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
        debugLog.nextModelCycle(cpu_iid);

        let station_id = coreToStationMap.sub(cpu_iid);

        //
        // Collect incoming messages.  The "receive" operation here is not a
        // commitment to process a message.  For that, doDeq() must be called.
        //
        let m_reqFromCore <- reqFromCore.receive(cpu_iid);
        let m_rspFromLocalLLC <- rspFromLocalLLC.receive(cpu_iid);
        let m_reqInFromOCN <- reqInFromOCN.receive(cpu_iid);
        let m_rspInFromLLC <- rspInFromLLC.receive(cpu_iid);

        // Tie off the dummy memory request port.
        let dummy_send_mem_rsp <- dummyMemRspToOCN.canEnq(cpu_iid);
        dummyMemRspToOCN.noEnq(cpu_iid);
        let dummy_recv_mem_req <- dummyReqInToMem.receive(cpu_iid);
        dummyReqInToMem.noDeq(cpu_iid);

        // Check credits for sending to output ports.
        let can_enq_rspToCore <- rspToCore.canEnq(cpu_iid);
        let can_enq_reqToLocalLLC <- reqToLocalLLC.canEnq(cpu_iid);
        let can_enq_reqToRemoteLLC <- reqToRemoteLLC.canEnq(cpu_iid);
        let can_enq_rspToRemoteLLC <- rspToRemoteLLC.canEnq(cpu_iid);

        //
        // Make routing choices.  The priority of competing ports is static.
        //

        Maybe#(LLC_RSP) m_new_rspToCore = tagged Invalid;
        Maybe#(ROUTED_LLC_REQ) m_new_reqToLocalLLC = tagged Invalid;
        Maybe#(Tuple2#(STATION_ID, LLC_REQ)) m_new_reqToRemoteLLC = tagged Invalid;
        Maybe#(Tuple2#(STATION_ID, LLC_RSP)) m_new_rspToRemoteLLC = tagged Invalid;

        Bool did_deq_rspFromLocalLLC = False;

        //
        // Local LLC response to either the local core or a remote core.
        //
        if (m_rspFromLocalLLC matches tagged Valid .rsp &&& can_enq_rspToCore)
        begin
            if (rsp.dst matches tagged Valid .dst)
            begin
                // Response is to a remote core.  Is the OCN port available?
                if (can_enq_rspToRemoteLLC && ! isValid(m_new_rspToRemoteLLC))
                begin
                    m_new_rspToRemoteLLC = tagged Valid tuple2(dst, rsp.lrsp);
                    did_deq_rspFromLocalLLC = True;
                    debugLog.record(cpu_iid, $format("1: LLC to remote Core, ") + fshow(rsp));
                end
            end
            // Reponse is to the local core.
            else if (can_enq_rspToCore &&& ! isValid(m_new_rspToCore))
            begin
                m_new_rspToCore = tagged Valid rsp.lrsp;
                did_deq_rspFromLocalLLC = True;
                debugLog.record(cpu_iid, $format("1: LLC to local Core, ") + fshow(rsp.lrsp));
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
            Tuple2#(STATION_ID, LLC_RSP) ocn_rsp = cvtFromOCNFlits(ocn_msg);
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
        if (m_reqInFromOCN matches tagged Valid .ocn_msg &&&
            can_enq_reqToLocalLLC &&&
            ! isValid(m_new_reqToLocalLLC))
        begin
            Tuple2#(STATION_ID, LLC_REQ) ocn_req = cvtFromOCNFlits(ocn_msg);
            match {.src, .lreq} = ocn_req;

            let req = ROUTED_LLC_REQ { src: tagged Valid src, lreq: lreq };
            m_new_reqToLocalLLC = tagged Valid req;
            reqInFromOCN.doDeq(cpu_iid);
            debugLog.record(cpu_iid, $format("1: Remote REQ to local LLC, ") + fshow(req));
        end
        else
        begin
            reqInFromOCN.noDeq(cpu_iid);
        end
        
        //
        // New requests from the local core to the LLC.
        //
        Bool did_deq_reqFromCore = False;

        if (m_reqFromCore matches tagged Valid .req)
        begin
            // Which instance of the distributed cache is responsible?
            let dst = getLLCDstForAddr(req.linePAddr);

            if (dst == station_id)
            begin
                // Local cache handles the address.
                if (can_enq_reqToLocalLLC &&& ! isValid(m_new_reqToLocalLLC))
                begin
                    // Port to LLC is available.  Send the local request.
                    did_deq_reqFromCore = True;
                    m_new_reqToLocalLLC = tagged Valid ROUTED_LLC_REQ { src: tagged Invalid,
                                                                        lreq: req };
                    debugLog.record(cpu_iid, $format("1: Core REQ to local LLC, ") + fshow(req));
                end
            end
            else if (can_enq_reqToRemoteLLC && ! isValid(m_new_reqToRemoteLLC))
            begin
                // Remote cache instance handles the address and the OCN request
                // port is available.
                //
                // These requests share the OCN request port since only one
                // type of request goes to a given remote station.  Memory
                // stations get memory requests above.  LLC stations get
                // core requests here.
                did_deq_reqFromCore = True;
                m_new_reqToRemoteLLC = tagged Valid tuple2(dst, req);
                debugLog.record(cpu_iid, $format("1: Core REQ to LLC %0d, ", dst) + fshow(req));
            end
        end

        if (did_deq_reqFromCore)
        begin
            reqFromCore.doDeq(cpu_iid);
        end
        else
        begin
            reqFromCore.noDeq(cpu_iid);
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

        if (m_new_reqToLocalLLC matches tagged Valid .req)
        begin
            reqToLocalLLC.doEnq(cpu_iid, req);
        end
        else
        begin
            reqToLocalLLC.noEnq(cpu_iid);
        end

        if (m_new_reqToRemoteLLC matches tagged Valid {.tgt, .req})
        begin
            reqToRemoteLLC.doEnq(cpu_iid, cvtToOCNFlits(tgt, req));
        end
        else
        begin
            reqToRemoteLLC.noEnq(cpu_iid);
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
    endrule
endmodule


// ========================================================================
//
//  Distributed LLC implementation.
//
// ========================================================================

typedef union tagged
{
    Tuple2#(STATION_ID, LLC_RSP) FILL_RSP;
    ROUTED_LLC_REQ               LOAD_REQ;
    ROUTED_LLC_REQ               STORE_REQ;
    void Invalid;
}
LLC_OPER
    deriving (Eq, Bits);

// newLLCOper --
//   The compiler sometimes fails to infer the type when initialized.  This
//   function makes types clear.
function LLC_OPER newLLCOper(LLC_OPER oper) = oper;


//
// LLC_LOCAL_STATE
//
// Local State to pass between pipeline stages.
//
typedef struct
{
    Maybe#(LLC_MISS_TOKEN) missTokToFree;

    Bool memQNotFull;
    Bool memQUsed;
    LLC_REQ memQData;
    
    Bool writePortUsed;
    LLC_CACHE_IDX writePortIdx;
    Bool writeDataDirty;
    LINE_ADDRESS writePortData;
    
    Bool coreQNotFull;
    Bool coreQUsed;
    ROUTED_LLC_RSP coreQData;
}
LLC_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(LLC_LOCAL_STATE);
    defaultValue = LLC_LOCAL_STATE { 
        missTokToFree: tagged Invalid,
        memQNotFull: False,
        memQUsed: False,
        memQData: ?,
        coreQNotFull: False,
        coreQUsed: False,
        coreQData: ?,
        writePortUsed: False,
        writePortIdx: ?,
        writeDataDirty: False,
        writePortData: ?
        };
endinstance


typedef `LLC_MISS_ID_SIZE LLC_MISS_ID_SIZE;
typedef CACHE_MISS_INDEX#(LLC_MISS_ID_SIZE) LLC_MISS_ID;
typedef CACHE_MISS_TOKEN#(LLC_MISS_ID_SIZE) LLC_MISS_TOKEN;
typedef TExp#(LLC_MISS_ID_SIZE) NUM_LLC_MISS_IDS;


//
// mkDistributedLastLevelCache --
//   Cache management.  Each core has an associated portion of the distributed
//   LLC.
//
module [HASIM_MODULE] mkDistributedLastLevelCache();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_llc_distrib.out");

    // ****** Submodels ******

    // The cache algorithm which determines hits, misses, and evictions.
    function Bool alwaysTrue(t_DUMMY d) = True;
    LLC_CACHE_ALG#(MAX_NUM_CPUS, void) llcAlg <- mkLastLevelCacheAlg(alwaysTrue);

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, LLC_MISS_ID_SIZE) outstandingMisses <- mkCacheMissTracker();

    // A RAM To map our miss IDs into the original opaques, that we return to higher levels.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS,
                            LLC_MISS_ID,
                            Tuple2#(Maybe#(STATION_ID), LLC_MISS_TOKEN)) opaquesPool <-
        mkMemory_Multiplexed(mkBRAM);

    // ****** Ports ******

    // Queues to/from Cache hierarchy.
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_REQ) reqFromCore <-
        mkPortStallRecv_Multiplexed("LLCHub_to_LLC_req");
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ROUTED_LLC_RSP) rspToCore <-
        mkPortStallSend_Multiplexed("LLC_to_LLCHub_rsp");
    
    // Requests to memory from the LLC instance responsible for an address.
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) reqToMem <-
        mkPortStallSend_Multiplexed(laneNameToOCN("Core", `OCN_LANES_MEM_REQ));
    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS,
                                 OCN_MSG_HEAD_AND_PAYLOAD) rspFromMem <-
        mkPortStallRecv_Multiplexed(laneNameFromOCN("Core", `OCN_LANES_SHARED_RSP_MEM_RSP));
    
    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inctrls = newVector();
    inctrls[0] = reqFromCore.ctrl.in;
    inctrls[1] = rspToCore.ctrl.in;
    inctrls[2] = reqToMem.ctrl.in;
    inctrls[3] = rspFromMem.ctrl.in;

    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    outctrls[0] = reqFromCore.ctrl.out;
    outctrls[1] = rspToCore.ctrl.out;
    outctrls[2] = reqToMem.ctrl.out;
    outctrls[3] = rspFromMem.ctrl.out;

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
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteHit <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_WRITE_HIT",
                                           "LLC Write Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_WRITE_MISS",
                                           "LLC Write Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statWriteRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_WRITE_RETRY",
                                           "LLC Write Retries"));
    STAT_VECTOR#(MAX_NUM_CPUS) statFillRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_LLC_FILL_RETRY",
                                           "LLC Fill Retries"));

    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventHit  <- mkEventRecorder_Multiplexed(`EVENTS_LLC_HIT);
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventMiss <- mkEventRecorder_Multiplexed(`EVENTS_LLC_MISS);


    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);

        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Make a conglomeration of local information to pass from stage to stage.
        let local_state = defaultValue;

        // Check whether the request ports have room for any new requests.
        let can_enq_mem_req <- reqToMem.canEnq(cpu_iid);
        let can_enq_core_rsp <- rspToCore.canEnq(cpu_iid);
        local_state.memQNotFull = can_enq_mem_req;
        local_state.coreQNotFull = can_enq_core_rsp;

        // Consume incoming messages
        let m_mem_rsp <- rspFromMem.receive(cpu_iid);
        let m_core_req <- reqFromCore.receive(cpu_iid);

        LLC_OPER oper = tagged Invalid;

        if (m_mem_rsp matches tagged Valid .ocn_msg &&&
            can_enq_core_rsp)
        begin
            //
            // FILL received and the core response port is available.
            //

            // Convert OCN message to a LLC_RSP
            Tuple2#(STATION_ID, LLC_RSP) ocn_rsp = cvtFromOCNFlits(ocn_msg);
            oper = tagged FILL_RSP ocn_rsp;

            match {.src, .rsp} = ocn_rsp;
            LLC_MISS_TOKEN miss_tok = fromMemOpaque(rsp.opaque);
            debugLog.record(cpu_iid, $format("1: FILL RSP: %0d, ", miss_tok) + fshow(rsp));
        end
        else if (m_core_req matches tagged Valid .req)
        begin
            //
            // New request from core received.
            //
            if (req.lreq.isStore)
            begin
                oper = tagged STORE_REQ req;
                debugLog.record(cpu_iid, $format("1: STORE REQ: ") + fshow(req));
            end
            else
            begin
                oper = tagged LOAD_REQ req;
                debugLog.record(cpu_iid, $format("1: LOAD REQ: ") + fshow(req));
            end
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        stage2Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule

    rule stage2 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        case (oper) matches
            tagged FILL_RSP {.src, .rsp}:
            begin
                let fill = rsp;

                // Fill will write to the cache.  The write may still be turned
                // off later in this pipeline and retried in another model cycle
                // if a needed eviction is blocked.
                local_state.writePortUsed = True;
                local_state.writePortData = fill.linePAddr;
                local_state.writeDataDirty = False;

                // Get the Miss ID.
                LLC_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

                // Free the token in the next stage, in case we had to retry.
                local_state.missTokToFree = tagged Valid miss_tok;

                // Only respond to loads.
                if (missTokIsLoad(miss_tok))
                begin
                    local_state.coreQUsed = True;
                    local_state.coreQData = ROUTED_LLC_RSP { dst: ?,
                                                             lrsp: fill };

                    // The destination and opaque are stored in opaquesPool.
                    // Retrieve them and overwrite dst/opaque in coreQData
                    // in stage 2.
                    opaquesPool.readReq(cpu_iid, missTokIndex(miss_tok));

                    debugLog.record(cpu_iid, $format("2: FILL LOAD RSP: %0d, ", miss_tok) + fshow(fill));
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("2: FILL RSP: %0d, ", miss_tok) + fshow(fill));
                end

                // Pick the victim
                llcAlg.lookupByAddrReq(cpu_iid,
                                       fill.linePAddr,
                                       False,
                                       missTokIsLoad(miss_tok));
            end

            tagged LOAD_REQ .req:
            begin
                // Look up the address in the cache.
                llcAlg.lookupByAddrReq(cpu_iid, req.lreq.linePAddr, True, True);
                debugLog.record(cpu_iid, $format("2: LOAD REQ: ") + fshow(req));
            end

            tagged STORE_REQ .req:
            begin
                // Look up the address in the cache.
                llcAlg.lookupByAddrReq(cpu_iid, req.lreq.linePAddr, True, False);
                debugLog.record(cpu_iid, $format("2: STORE REQ: ") + fshow(req));
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("2: Bubble"));
            end
        endcase

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule

    rule stage3 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();

        // Hit/miss events will be recorded in this rule.
        Maybe#(EVENT_PARAM) evt_hit = tagged Invalid;
        Maybe#(EVENT_PARAM) evt_miss = tagged Invalid;

        case (oper) matches
            tagged FILL_RSP {.src, .rsp}:
            begin
                if (local_state.coreQUsed)
                begin
                    //
                    // Restore the opaque to the context that sent the request
                    // to this cache.  This cache modified only the bits required
                    // for a local miss token.  Merge the unmodified bits (still
                    // in the opaque) and the preserved, overwritten bits (stored
                    // in opaquesPool).
                    //
                    match {.prev_src, .prev_opaque} <- opaquesPool.readRsp(cpu_iid);
                    local_state.coreQData.dst = prev_src;
                    local_state.coreQData.lrsp.opaque =
                        updateMemOpaque(local_state.coreQData.lrsp.opaque,
                                        prev_opaque);
                end

                let evict <- llcAlg.lookupByAddrRsp(cpu_iid);

                local_state.writePortIdx = evict.idx;

                if (evict.state matches tagged Blocked)
                begin
                    // No available victim due to temporary state of the old
                    // line.  Wait for it to settle.
                    oper = newLLCOper(tagged Invalid);
                    local_state = defaultValue;

                    statFillRetry.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: BLOCKED EVICTION: new line 0x%h", rsp.linePAddr));
                end
                else if (evict.state matches tagged MustEvict .state &&&
                         state.dirty)
                begin
                    // Is there any room in the memQ?
                    if (local_state.memQNotFull)
                    begin
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION: new line 0x%h, old line 0x%h, ", rsp.linePAddr, state.linePAddr) + fshow(evict.idx));

                        // Record that we're using the memQ.
                        local_state.memQUsed = True;
                        local_state.memQData = initMemStore(state.linePAddr);
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: DIRTY EVICTION WB BLOCKED: new line 0x%h, old line 0x%h, ", rsp.linePAddr, state.linePAddr) + fshow(evict.idx));

                        // Retry on a later model cycle.
                        statFillRetry.incr(cpu_iid);
                        oper = newLLCOper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
                else
                begin
                    // Clean eviction.  No writeback needed.
                    if (evict.state matches tagged Valid .state)
                    begin
                        debugLog.record(cpu_iid, $format("3: ALREADY PRESENT: line 0x%h, ", rsp.linePAddr) + fshow(evict.idx));
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, $format("3: CLEAN EVICTION: line 0x%h", rsp.linePAddr) + fshow(evict.idx));
                    end
                end
            end

            tagged LOAD_REQ .req:
            begin
                let entry <- llcAlg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // Hit!
                    if (local_state.coreQNotFull)
                    begin
                        // Load response indicates data present.
                        local_state.coreQUsed = True;
                        local_state.coreQData = initLLCMemRspFromReq(req);

                        statReadHit.incr(cpu_iid);
                        evt_hit = tagged Valid resize({ req.lreq.linePAddr, 1'b0 });

                        debugLog.record(cpu_iid, $format("3: LOAD HIT: ") + fshow(req) + $format(", ") + fshow(entry.idx));
                    end
                    else
                    begin
                        // A load hit, but the response port is busy.
                        statReadRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD HIT RETRY: ") + fshow(req) + $format(", ") + fshow(entry.idx));

                        oper = newLLCOper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
                else
                begin
                    // Miss.  Fill the line from memory.
                    if (outstandingMisses.canAllocateLoad(cpu_iid) &&
                        local_state.memQNotFull)
                    begin
                        // Allocate the next miss ID.
                        local_state.memQUsed = True;
                        outstandingMisses.allocateLoadReq(cpu_iid,
                                                          req.lreq.linePAddr);

                        statReadMiss.incr(cpu_iid);
                        evt_miss = tagged Valid resize({ req.lreq.linePAddr, 1'b0 });

                        debugLog.record(cpu_iid, $format("3: LOAD MISS: ") + fshow(req));
                    end
                    else
                    begin
                        // The request must stall.
                        statReadRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: LOAD MISS RETRY"));

                        oper = newLLCOper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
            end

            tagged STORE_REQ .req:
            begin
                let entry <- llcAlg.lookupByAddrRsp(cpu_iid);

                if (entry.state matches tagged Valid .state)
                begin
                    // Hit!  Store sets the dirty bit.
                    local_state.writePortUsed = True;
                    local_state.writePortData = req.lreq.linePAddr;
                    local_state.writePortIdx = entry.idx;
                    local_state.writeDataDirty = True;

                    statWriteHit.incr(cpu_iid);
                    evt_hit = tagged Valid resize({ req.lreq.linePAddr, 1'b1 });

                    debugLog.record(cpu_iid, $format("3: STORE HIT: ") + fshow(req) + $format(", ") + fshow(entry.idx));
                end
                else
                begin
                    // Miss.  Fill the line from memory.
                    if (outstandingMisses.canAllocateStore(cpu_iid) &&
                        local_state.memQNotFull)
                    begin
                        // Allocate the next miss ID
                        local_state.memQUsed = True;
                        outstandingMisses.allocateStoreReq(cpu_iid);

                        statWriteMiss.incr(cpu_iid);
                        evt_miss = tagged Valid resize({ req.lreq.linePAddr, 1'b1 });

                        debugLog.record(cpu_iid, $format("3: STORE MISS: ") + fshow(req));
                    end
                    else
                    begin
                        // Memory request queue if busy.  The request must stall.
                        statWriteRetry.incr(cpu_iid);
                        debugLog.record(cpu_iid, $format("3: STORE MISS RETRY: ") + fshow(req));

                        oper = newLLCOper(tagged Invalid);
                        local_state = defaultValue;
                    end
                end
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("3: Bubble"));
            end
        endcase

        eventHit.recordEvent(cpu_iid, evt_hit);
        eventMiss.recordEvent(cpu_iid, evt_miss);

        stage4Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule
    

    rule stage4 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged FILL_RSP {.src, .rsp}:
            begin
                debugLog.record(cpu_iid, $format("4: FILL RSP Bubble"));
            end

            tagged LOAD_REQ .req:
            begin
                if (local_state.memQUsed)
                begin
                    // A fill request is being sent to memory.  Complete the
                    // request details.
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);

                    // Record the source core of the request and the original
                    // opaque.
                    opaquesPool.write(cpu_iid, missTokIndex(miss_tok),
                                      tuple2(req.src, fromMemOpaque(req.lreq.opaque)));

                    // Use the opaque bits to store the miss token.
                    let mem_req = initMemLoad(req.lreq.linePAddr);
                    mem_req.opaque = updateMemOpaque(req.lreq.opaque, miss_tok);
                    local_state.memQData = mem_req;

                    debugLog.record(cpu_iid, $format("4: LOAD MISS: %0d, ", miss_tok.index) + fshow(req));
                end
            end

            tagged STORE_REQ .req:
            begin
                if (local_state.memQUsed)
                begin
                    // A fill request is being sent to memory.  Complete the
                    // request details.
                    let miss_tok <- outstandingMisses.allocateStoreRsp(cpu_iid);

                    // Use the opaque bits to store the miss token.
                    // We use a load to simulate getting exclusive access.
                    let mem_req = initMemLoad(req.lreq.linePAddr);
                    mem_req.opaque = updateMemOpaque(req.lreq.opaque, miss_tok);
                    local_state.memQData = mem_req;

                    debugLog.record(cpu_iid, $format("4: STORE MISS: %0d, ", miss_tok.index) + fshow(req));
                end
            end

            default:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble"));
            end
        endcase

        stage5Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule
    
    rule stage5_end (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("5: Done"));

        //
        // Indicate whether requests were consumed.
        //

        if (oper matches tagged FILL_RSP {.src, .rsp})
        begin
            rspFromMem.doDeq(cpu_iid);
        end
        else
        begin
            rspFromMem.noDeq(cpu_iid);
        end


        if (oper matches tagged LOAD_REQ .req)
        begin
            reqFromCore.doDeq(cpu_iid);
        end
        else if (oper matches tagged STORE_REQ .req)
        begin
            reqFromCore.doDeq(cpu_iid);
        end
        else
        begin
            reqFromCore.noDeq(cpu_iid);
        end


        //
        // Send requests/responses.
        //

        // Take care of the memory queue.
        if (local_state.memQUsed)
        begin
            let req = local_state.memQData;

            // Which memory controller handles the request's address?
            let dst = getMemCtrlDstForAddr(req.linePAddr);

            reqToMem.doEnq(cpu_iid, cvtToOCNFlits(dst, req));
            debugLog.record(cpu_iid, $format("5: REQ TO MEM %0d: ", dst) + fshow(req));
        end
        else
        begin
            reqToMem.noEnq(cpu_iid);
        end
        
        // Take care of the cache update.
        if (local_state.writePortUsed)
        begin
            CACHE_ENTRY_STATE#(void) state =
                initCacheEntryState(local_state.writePortData,
                                    local_state.writeDataDirty,
                                    ?);

            if (oper matches tagged FILL_RSP .rsp)
            begin
                // Fill allocates a new line.
                llcAlg.allocate(cpu_iid,
                                local_state.writePortIdx,
                                state,
                                CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("5: ALLOC: ") + fshow(local_state.writePortIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                // Store is the only other path.  It updates the state of an
                // existing entry.
                llcAlg.update(cpu_iid,
                              local_state.writePortIdx,
                              state);

                debugLog.record(cpu_iid, $format("5: UPDATE: ") + fshow(local_state.writePortIdx) + $format(", ") + fshow(state));
            end
        end
        
        // Take care of CPU rsp
        if (local_state.coreQUsed)
        begin
            rspToCore.doEnq(cpu_iid, local_state.coreQData); 
        end
        else
        begin
            rspToCore.noEnq(cpu_iid);
        end


        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        // End of model cycle.
        localCtrl.endModelCycle(cpu_iid, 1); 
    endrule

endmodule
