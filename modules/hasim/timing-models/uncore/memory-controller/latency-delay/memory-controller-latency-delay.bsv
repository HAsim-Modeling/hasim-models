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

import GetPut::*;
import Vector::*;
import FIFOF::*;

// ******* Project Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_chip_topology.bsh"
`include "awb/provides/hasim_interconnect_common.bsh"


// ****** Generated files ******

`include "awb/dict/TOPOLOGY.bsh"
`include "awb/dict/PARAMS_HASIM_MEMORY_CONTROLLER.bsh"
`include "awb/dict/OCN_LANES.bsh"


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


function MEM_CTRL_RSP initMemCtrlRsp(MEM_CTRL_REQ req);
    return MEM_CTRL_RSP
    {
        physicalAddress: req.physicalAddress,
        opaque: req.opaque,
        destination: req.destination
    };
endfunction



module [HASIM_MODULE] mkMemoryController()
    provisos (Alias#(t_VC_FIFO, FUNC_FIFO#(OCN_FLIT, 4)),
              NumAlias#(n_MAX_IN_FLIGHT, 256));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_MEM_CTRLS) debugLog <-
        mkTIMEPDebugFile_Multiplexed("interconnect_memory_controller.out");

    //
    // Wrapped interfaces to/from interconnect network.  The wrappers simplify
    // the protocol for credit management.
    //
    PORT_OCN_LOCAL_SEND_MULTIPLEXED#(MAX_NUM_MEM_CTRLS) ocnSend <-
        mkLocalNetworkPortSend("memctrl_to_ocn",
                               "ocn_to_memctrl",
                               debugLog);

    PORT_OCN_LOCAL_RECV_MULTIPLEXED#(MAX_NUM_MEM_CTRLS,
                                     MAX_FLITS_PER_PACKET) ocnRecv <-
        mkLocalNetworkPortRecv("memctrl_to_ocn",
                               "ocn_to_memctrl",
                               debugLog);

    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_MEM_CTRLS))  inports  = newVector();
    inports[0] = ocnSend.ctrl.in;
    inports[1] = ocnRecv.ctrl.in;

    Vector#(2, INSTANCE_CONTROL_OUT#(MAX_NUM_MEM_CTRLS)) outports = newVector();
    outports[0] = ocnSend.ctrl.out;
    outports[1] = ocnRecv.ctrl.out;

    //
    // Statistics
    //

    // Compute average latency from Little's Law:
    //     statPacketsInFlight / statPackets
    STAT_VECTOR#(MAX_NUM_MEM_CTRLS) statPackets <-
        mkStatCounter_Multiplexed(statName("MODEL_MEM_CTRL_LOAD_REQ",
                                           "Number of load requests"));
    STAT_VECTOR#(MAX_NUM_MEM_CTRLS) statPacketsInFlight <-
        mkStatCounter_Multiplexed(statName("MODEL_MEM_CTRL_LOAD_REQ_IN_FLIGHT",
                                           "Sum of packets in flight each cycle (used for Little's Law)"));

    // Coordinate between the pipeline stages.
    LOCAL_CONTROLLER#(MAX_NUM_MEM_CTRLS) localCtrl <-
        mkNamedLocalControllerWithActive("Memory Controller",
                                         0, False,
                                         inports,
                                         Vector::nil,
                                         outports);

    MULTIPLEXED_REG#(MAX_NUM_MEM_CTRLS, Bool) processingLoadPool <-
        mkMultiplexedReg(False);

    MULTIPLEXED_REG#(MAX_NUM_MEM_CTRLS, UInt#(TLog#(n_MAX_IN_FLIGHT))) nInFlightPool <-
        mkMultiplexedReg(0);

    //
    // Side memories hold the actual contents of a packet instead of forcing all
    // datapaths in the simulated OCN to be wide enough to pass a full packet.
    // The memory is actually allocated in the LLC controller.  Here we just
    // connect to a port.
    //
    OCN_PACKET_PAYLOAD_CLIENT payloadStorage <- mkNetworkPacketPayloadClient(0);


    STAGE_CONTROLLER#(MAX_NUM_MEM_CTRLS, Bool) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_MEM_CTRLS, Tuple2#(Bool, Bool)) stage3Ctrl <-
        mkStageController();

    //
    // Queue that simulates the latency of DRAM with run-time variable
    // latency.
    //
    PARAMETER_NODE paramNode  <- mkDynamicParameterNode();
    Param#(8) memLatency <- mkDynamicParameter(`PARAMS_HASIM_MEMORY_CONTROLLER_MEM_CTRL_LATENCY, paramNode);

    PORT_SPARSE_DELAY_MULTIPLEXED#(MAX_NUM_MEM_CTRLS,
                                   OCN_FLIT,
                                   `MEM_CTRL_CAPACITY,
                                   n_MAX_IN_FLIGHT) memRespQ <-
        mkPortSparseDelay_Multiplexed(memLatency);

    //
    // Set the number of active nodes, given the topology.
    //
    let numActiveCtrlrs <- mkTopologyParamStream(`TOPOLOGY_NUM_MEM_CONTROLLERS);

    rule initNumActiveCtrlrs (True);
        let m_n_ctrl <- numActiveCtrlrs.get();
        if (m_n_ctrl matches tagged Valid .n)
        begin
            localCtrl.setMaxActiveInstance(n - 1);
        end
    endrule


    (* conservative_implicit_conditions *)
    rule stage1_sendToOCN (True);
        let iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(iid);
        
        // Check credits for sending to the network
        let can_enq <- ocnSend.canEnq(iid);

        Bool did_enq = False;
        Bool finished_req = False;

        // Have credit to send?
        if (can_enq[`OCN_LANES_MEM_RSP])
        begin
            // Have a message to send?
            let m_flit <- memRespQ.receive(iid);
            if (m_flit matches tagged Valid .flit)
            begin
                // Send it
                ocnSend.doEnq(iid, `OCN_LANES_MEM_RSP, flit);
                memRespQ.doDeq(iid);
                did_enq = True;

                if (flit matches tagged FLIT_BODY .body &&& body.isTail)
                begin
                    finished_req = True;
                end
            end
        end

        if (! did_enq)
        begin
            ocnSend.noEnq(iid);
            memRespQ.noDeq(iid);
        end

        stage2Ctrl.ready(iid, finished_req);
    endrule


    (* conservative_implicit_conditions *)
    rule stage2_recvFromOCN (True);
        match {.iid, .finished_req} <- stage2Ctrl.nextReadyInstance();

        // Pick a channel from which to receive
        if (ocnRecv.pickChannel(iid, replicate(memRespQ.canEnq(iid))) matches
                tagged Valid {.ln, .vc})
        begin
            // Request the winning flit
            ocnRecv.receiveReq(iid, ln, vc);
            stage3Ctrl.ready(iid, tuple2(finished_req, True));
        end
        else
        begin
            // Either nothing to receive or the local buffer is full.
            ocnRecv.noDeq(iid);
            stage3Ctrl.ready(iid, tuple2(finished_req, False));
        end
    endrule


    rule stage3_recvFromOCN (True);
        match {.iid, .finished_req, .get_flit} <- stage3Ctrl.nextReadyInstance();

        // Read our local state from the pools.
        Reg#(Bool) processingLoad = processingLoadPool.getReg(iid);
        Reg#(UInt#(TLog#(n_MAX_IN_FLIGHT))) nInFlight = nInFlightPool.getReg(iid);

        // Track the change in the number of requests in flight.
        Int#(2) in_flight_delta = (finished_req ? -1 : 0);

        Bool did_enq = False;

        if (get_flit)
        begin
            let flit <- ocnRecv.receiveRsp(iid);

            case (flit) matches 
                tagged FLIT_HEAD .info:
                begin
                    if (! info.isStore)
                    begin
                        // The simulated response flit only needs to do enough
                        // to model time.  For this, we simply send the same
                        // payload back.
                        memRespQ.doEnq(iid,
                                       tagged FLIT_HEAD OCN_FLIT_HEAD {src: info.dst,
                                                                       dst: info.src,
                                                                       isStore: False});

                        statPackets.incr(iid);
                        in_flight_delta = in_flight_delta + 1;

                        did_enq = True;
                        debugLog.record(iid, $format("3: Received load from Station %0d", info.src));
                        processingLoad <= True;
                    end
                    else
                    begin
                        // No response to stores.  Drop.
                        debugLog.record(iid, $format("3: Dropping store from Station %0d", info.src));
                        processingLoad <= False;
                    end
                end

                tagged FLIT_BODY .body:
                begin
                    if (processingLoad)
                    begin
                        // Body flits just get sent back for loads, currently.
                        // This would normally be the time when we remove the
                        // packet's payload metadata from payloadStorage.
                        // Since the packet is going right back to the network
                        // we don't, instead reusing the metadata for the
                        // response packet.
                        memRespQ.doEnq(iid, tagged FLIT_BODY body);
                        did_enq = True;
                        debugLog.record(iid, $format("3: Load recv body"));
                    end
                    else
                    begin
                        // Packet complete.  Release the buffer entry.
                        let h = body.opaque;
                        payloadStorage.freeHandle(h);
                        debugLog.record(iid, $format("3: Dropping store body, 0x%0x", h));
                    end
                end
            endcase
        end

        // Track the total number of request packets in flight on each cycle.
        let n_in_flight = nInFlight + unpack(pack(signExtend(in_flight_delta)));
        nInFlight <= n_in_flight;
        statPacketsInFlight.incrBy(iid, pack(zeroExtend(n_in_flight)));

        if (! did_enq)
        begin
            memRespQ.noEnq(iid);
            debugLog.record(iid, $format("3: No ENQ"));
        end

        localCtrl.endModelCycle(iid, 0);
    endrule

endmodule
