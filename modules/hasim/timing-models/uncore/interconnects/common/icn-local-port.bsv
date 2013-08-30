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


//
// Modules here manage the port protocol between a local connection (e.g.
// a CPU or memory controller) and the on-chip network.  The port-based
// communication is mapped to method-based interfaces.  The complexity of
// credit and virtual channel management is hidden by the interface.
//


import FIFO::*;
import SpecialFIFOs::*;
import Vector::*;
import List::*;


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
`include "awb/provides/hasim_chip_topology.bsh"


//
// PORT_OCN_LOCAL_SEND_MULTIPLEXED --
//   The send interface to the on-chip-network (OCN) is essentially the same
//   interface as the basic PORT_STALL_SEND_MULTIPLEXED.  The only significant
//   differences are:
//     - canEnq returns a vector indicating which lanes may receive messages.
//     - doEnq specifies the target lane.
//
interface PORT_OCN_LOCAL_SEND_MULTIPLEXED#(type ni);
    // Vector of lanes willing to accept a message this cycle.
    method ActionValue#(Vector#(NUM_LANES, Bool)) canEnq(INSTANCE_ID#(ni) iid);

    // One of doEnq/noEnq must be called exactly once each simulated cycle
    // for each iid.  Only lanes for which canEnq() is True may receive
    // messages.
    method Action doEnq(INSTANCE_ID#(ni) iid, LANE_IDX lane, OCN_FLIT flit);
    method Action noEnq(INSTANCE_ID#(ni) iid);

    interface INSTANCE_CONTROL_IN_OUT#(ni) ctrl;
endinterface

//
// PORT_OCN_LOCAL_RECV_MULTIPLEXED --
//   Unfortunately, the receive interface does not look like the stall
//   port receive interface.  This is mainly a consequence of receive needing
//   a request/response interface in order to implement channel buffers as
//   BRAM.
//   
interface PORT_OCN_LOCAL_RECV_MULTIPLEXED#(type ni);
    // Returns a vector indicating which lanes/virtual channels have
    // incoming messages.
    method Vector#(NUM_LANES,
                   Vector#(VCS_PER_LANE, Bool)) notEmpty(INSTANCE_ID#(ni) iid);

    // Multi-cycle receive request/response.  The requested lane/vc must have
    // a message, as indicated by notEmpty() above.
    method Action receiveReq(INSTANCE_ID#(ni) iid, LANE_IDX lane, VC_IDX vc);
    method ActionValue#(OCN_FLIT) receiveRsp(INSTANCE_ID#(ni) iid);

    // receiveReq or noDeq must be called exactly once for each simulated
    // cycle for each iid.  If no receiveReq is issued, call noDeq.
    method Action noDeq(INSTANCE_ID#(ni) iid);

    interface INSTANCE_CONTROL_IN_OUT#(ni) ctrl;
endinterface


//
// mkLocalNetworkPortSend --
//   Wrap the ports for sending from a local controller to the OCN.  Credit
//   management is hidden inside the module.
//
module [HASIM_MODULE] mkLocalNetworkPortSend#(
    String portNameSend,
    String portNameRecv,
    TIMEP_DEBUG_FILE_MULTIPLEXED#(ni) debugLog)
    // Interface:
    (PORT_OCN_LOCAL_SEND_MULTIPLEXED#(ni))
    provisos (Alias#(t_IID, INSTANCE_ID#(ni)));

    PORT_SEND_MULTIPLEXED#(ni, OCN_MSG) enqToOCN <-
        mkPortSend_Multiplexed(portNameSend + "_enq");
    PORT_RECV_MULTIPLEXED#(ni, VC_CREDIT_INFO) creditFromOCN <-
        mkPortRecv_Multiplexed(portNameRecv + "_credit", 1);

    MULTIPLEXED_REG#(ni, Vector#(NUM_LANES, Bool)) notFullsPool <-
        mkMultiplexedReg(replicate(False));

    method Action doEnq(t_IID iid, LANE_IDX lane, OCN_FLIT flit);
        // Sender always uses virtual channel 0.  The network may switch to
        // a different VC during routing.
        OCN_MSG msg = tuple3(lane, 0, flit);
        enqToOCN.send(iid, tagged Valid msg);

        debugLog.record(iid, $format("lpSend: ENQ (ln %0d)", lane));
    endmethod

    method Action noEnq(t_IID iid);
        enqToOCN.send(iid, tagged Invalid);

        debugLog.record(iid, $format("lpSend: No ENQ"));
    endmethod

    method ActionValue#(Vector#(NUM_LANES, Bool)) canEnq(t_IID iid);
        // Read our local state from the pools.
        Reg#(Vector#(NUM_LANES, Bool)) notFulls = notFullsPool.getReg(iid);

        Vector#(NUM_LANES, Bool) new_not_fulls = notFulls;
        let m_credits <- creditFromOCN.receive(iid);

        // Update our notion of credits.
        if (m_credits matches tagged Valid .creds)
        begin
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                // Only interested in VC 0.
                match {.cred, .out_not_full} = creds[ln][0];
                new_not_fulls[ln] = out_not_full;
            end
            debugLog.record(iid, $format("lpSend canEnq: Update credits (%b)", pack(new_not_fulls)));
        end
        else
        begin
            debugLog.record(iid, $format("lpSend canEnq: No credits"));
        end

        notFulls <= new_not_fulls;
        return new_not_fulls;
    endmethod


    interface INSTANCE_CONTROL_IN_OUT ctrl;
        interface INSTANCE_CONTROL_IN in;
            method Bool empty() = creditFromOCN.ctrl.empty;
            method Bool balanced() = creditFromOCN.ctrl.balanced;
            method Bool light() = creditFromOCN.ctrl.light;
            
            method Maybe#(t_IID) nextReadyInstance() = creditFromOCN.ctrl.nextReadyInstance;
            method Action setMaxRunningInstance(t_IID iid);
                creditFromOCN.ctrl.setMaxRunningInstance(iid);
            endmethod
        
            method List#(PORT_INFO) portInfo() = creditFromOCN.ctrl.portInfo;
        endinterface
    
        interface INSTANCE_CONTROL_OUT out;
            method Bool full() = enqToOCN.ctrl.full;
            method Bool balanced() = enqToOCN.ctrl.balanced;
            method Bool heavy() = enqToOCN.ctrl.heavy;
            method Action setMaxRunningInstance(t_IID iid);
                enqToOCN.ctrl.setMaxRunningInstance(iid);
            endmethod
        
            method List#(String) portName() = enqToOCN.ctrl.portName;
        endinterface
    endinterface
endmodule



//
// mkLocalNetworkPortRecv --
//   Wrap the ports for sending from the OCN to a local controller.  Credit
//   and local VC buffer management is hidden inside the module.
//
module [HASIM_MODULE] mkLocalNetworkPortRecv#(
    String portNameSend,
    String portNameRecv,
    TIMEP_DEBUG_FILE_MULTIPLEXED#(ni) debugLog)
    // Interface:
    (PORT_OCN_LOCAL_RECV_MULTIPLEXED#(ni))
    provisos (Alias#(t_IID, INSTANCE_ID#(ni)),
              // Incoming buffering for each virtual channel
              NumAlias#(n_VC_BUF_ENTRIES, 4),
              Alias#(t_VC_FIFO, FUNC_FIFO_IDX#(n_VC_BUF_ENTRIES)),
              // All FIFOs managing channel buffers for an instance
              Alias#(t_BUFFER_FIFOS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_VC_FIFO))));

    PORT_RECV_MULTIPLEXED#(ni, OCN_MSG) enqFromOCN <-
        mkPortRecv_Multiplexed(portNameRecv + "_enq", 1);
    PORT_SEND_MULTIPLEXED#(ni, VC_CREDIT_INFO) creditToOCN <-
        mkPortSend_Multiplexed(portNameSend + "_credit");

    // Each virtual channel has an associated incoming buffer (FIFO)
    MULTIPLEXED_REG#(ni, t_BUFFER_FIFOS) vcFIFOsPool <-
        mkMultiplexedReg(replicate(replicate(funcFIFO_IDX_Init)));

    // Storage for each VC's FIFO buffer.  The FIFOs above are merely
    // pointers to this storage.
    MEMORY_IFC#(Tuple4#(INSTANCE_ID#(ni),
                        LANE_IDX, VC_IDX,
                        Bit#(TLog#(n_VC_BUF_ENTRIES))),
                OCN_FLIT) vcBufEntries <- mkBRAM();

    FIFO#(Tuple2#(t_IID, t_BUFFER_FIFOS)) recv1Q <- mkFIFO();
    FIFO#(Tuple2#(t_IID, t_BUFFER_FIFOS)) recv2Q <- mkFIFO();

    //
    // recv1 --
    //   Receive a new message from the network port and store it in the local
    //   buffer.  This rule could logically be part of the same cycle as
    //   the first stage of the receive pipeline, but the hardware is simpler
    //   given the complexity of managing multiple lanes and virtual channels.
    //
    rule recv1 (True);
        match {.iid, .upd_vc_buf} = recv1Q.first();
        recv1Q.deq();

        //
        // Check the network port for a new incoming message.
        //
        let m_enq <- enqFromOCN.receive(iid);
        if (m_enq matches tagged Valid {.lane, .vc, .flit})
        begin
            match {.upd_fifo, .idx} = funcFIFO_IDX_UGenq(upd_vc_buf[lane][vc]);
            upd_vc_buf[lane][vc] = upd_fifo;
            vcBufEntries.write(tuple4(iid, lane, vc, idx), flit);

            debugLog.record(iid, $format("lpRecv recv1: New msg ln %0d, vc %0d", lane, vc));
        end
        else
        begin
            debugLog.record(iid, $format("lpRecv recv1: No message"));
        end

        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        vcBuf <= upd_vc_buf;

        recv2Q.enq(tuple2(iid, upd_vc_buf));
    endrule

    rule recv2 (True);
        match {.iid, .upd_vc_buf} = recv2Q.first();
        recv2Q.deq();

        //
        // Send updated credits to the network.
        //
        VC_CREDIT_INFO creds = newVector();

        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            creds[ln] = newVector();

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin
                let have_credit = ! funcFIFO_IDX_notEmpty(upd_vc_buf[ln][vc]); // XXX capacity - occupancy > round-trip latency.
                let not_full = ! funcFIFO_IDX_notEmpty(upd_vc_buf[ln][vc]);
                creds[ln][vc] = tuple2(have_credit, not_full);
            end
        end
        
        creditToOCN.send(iid, tagged Valid creds);
        debugLog.record(iid, $format("lpRecv creditToOCN: %b", pack(creds)));
    endrule


    method Vector#(NUM_LANES,
                   Vector#(VCS_PER_LANE, Bool)) notEmpty(t_IID iid);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);

        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) not_empty = newVector();
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin
                not_empty[ln][vc] = funcFIFO_IDX_notEmpty(vcBuf[ln][vc]);
            end
        end

        return not_empty;
    endmethod

    method Action receiveReq(t_IID iid, LANE_IDX lane, VC_IDX vc);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        t_BUFFER_FIFOS upd_vc_buf = vcBuf;

        let idx = funcFIFO_IDX_UGfirst(upd_vc_buf[lane][vc]);
        vcBufEntries.readReq(tuple4(iid, lane, vc, idx));

        upd_vc_buf[lane][vc] = funcFIFO_IDX_UGdeq(upd_vc_buf[lane][vc]);

        // Side effect of receiveReq: check for new messages
        recv1Q.enq(tuple2(iid, upd_vc_buf));
        debugLog.record(iid, $format("lpRecv req: ln %0d, vc %0d", lane, vc));
    endmethod

    method ActionValue#(OCN_FLIT) receiveRsp(t_IID iid);
        let flit <- vcBufEntries.readRsp();
        return flit;
    endmethod

    method Action noDeq(t_IID iid);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        t_BUFFER_FIFOS upd_vc_buf = vcBuf;

        // Side effect of noDeq: check for new messages
        recv1Q.enq(tuple2(iid, upd_vc_buf));
        debugLog.record(iid, $format("lpRecv no req"));
    endmethod


    interface INSTANCE_CONTROL_IN_OUT ctrl;
        interface INSTANCE_CONTROL_IN in;
            method Bool empty() = enqFromOCN.ctrl.empty;
            method Bool balanced() = enqFromOCN.ctrl.balanced;
            method Bool light() = enqFromOCN.ctrl.light;
            
            method Maybe#(t_IID) nextReadyInstance() = enqFromOCN.ctrl.nextReadyInstance;
            method Action setMaxRunningInstance(t_IID iid);
                enqFromOCN.ctrl.setMaxRunningInstance(iid);
            endmethod
        
            method List#(PORT_INFO) portInfo() = enqFromOCN.ctrl.portInfo;
        endinterface
    
        interface INSTANCE_CONTROL_OUT out;
            method Bool full() = creditToOCN.ctrl.full;
            method Bool balanced() = creditToOCN.ctrl.balanced;
            method Bool heavy() = creditToOCN.ctrl.heavy;
            method Action setMaxRunningInstance(t_IID iid);
                creditToOCN.ctrl.setMaxRunningInstance(iid);
            endmethod
        
            method List#(String) portName() = creditToOCN.ctrl.portName;
        endinterface
    endinterface
endmodule