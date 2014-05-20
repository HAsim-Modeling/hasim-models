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

`include "awb/dict/OCN_LANES.bsh"


//
// laneNameToOCN and laneNameFromOCN --
//   OCN lanes may be enumerated in a model as dictionary entries.  This
//   convenience function maps a lane number to a soft connection name.
//
function String laneNameToOCN(String prefix, Integer lane);
    return "OCN_LANE_SEND_" + prefix + "_" + integerToString(lane);
endfunction

function String laneNameFromOCN(String prefix, Integer lane);
    return "OCN_LANE_RECV_" + prefix + "_" + integerToString(lane);
endfunction


typedef Vector#(`OCN_LANES_DICT_ENTRIES, Tuple2#(LANE_IDX, OCN_PACKET_TAG))
    OCN_MAP_PORTS_TO_LANES;

typedef Vector#(NUM_LANES, Tuple2#(Integer, Bool)) OCN_MAP_LANES_TO_PORTS;

//
// ocnMapPortsToLanes --
//   Build a vector that maps all lane indices in the OCN_LANES dictionary
//   space to a OCN lanes.
//
//   Names that have a hierarchy (i.e. names not at top level in the OCN_LANES
//   space) form groups that share a lane.  These sharing groups
//   use a tag to differentiate channels.  The tag is returned as the second
//   integer in the tuples.
//  
module ocnMapPortsToLanes
    // Interface (just a returned type for this module):
    (OCN_MAP_PORTS_TO_LANES);

    Integer lane = 0;
    Integer idx = 0;
    OCN_MAP_PORTS_TO_LANES result = newVector();

    // Walk all groups of names in the OCN_LANES space.  Group 0 is special
    // because it is the top-level group.  Each top-level entry gets its own
    // lane with tag 0.  Subsequent groups share a lane among the group and
    // are disambiguated with the second part of the tuple -- the tag.
    for (Integer g = 0; g < nGroupsOCN_LANES_DICT; g = g + 1)
    begin
        // Iterate through all entries within the group.
        for (Integer i = 0; i < groupsOCN_LANES_DICT[g][1]; i = i + 1)
        begin
            Integer tag = (g == 0 ? 0 : i);
            result[idx] = tuple2(fromInteger(lane), fromInteger(tag));

            idx = idx + 1;
            if (g == 0) lane = lane + 1;
        end

        if (g != 0) lane = lane + 1;
    end

    // This hack puts the proper offset for MEM_RSP in the MEM_REQ tag.
    // It allows for a simple memory controller that doesn't need a write
    // port to the payload storage buffers.  The tag needed for the response
    // will be set as a side-effect of making the request.
    //
    // For outbound port mapping, setting the tag is harmless since tags are
    // ignored on non-shared ports.  MEM_REQ is non-shared.
    result[`OCN_LANES_MEM_REQ] = tuple2(tpl_1(result[`OCN_LANES_MEM_REQ]),
                                        tpl_2(result[`OCN_LANES_SHARED_RSP_MEM_RSP]));

    return result;
endmodule


//
// ocnMapLanesToPorts --
//   The reverse mapping of ocnMapPortsToLanes.  Given an OCN lane, return the
//   index of the corresponding A-port to the model.  The 2nd, bool part of the
//   tuple indicates whether the lane maps to a group of ports all sharing
//   the lane.
//
module ocnMapLanesToPorts
    // Interface (just a returned type for this module):
    (OCN_MAP_LANES_TO_PORTS);

    Integer idx = 0;
    OCN_MAP_LANES_TO_PORTS result = newVector();

    // The first set of lanes is not shared
    for (Integer p = 0; p < groupsOCN_LANES_DICT[0][1]; p = p + 1)
    begin
        result[idx] = tuple2(p, False);
        idx = idx + 1;
    end

    // Remaining groups are shared.
    for (Integer g = 1; g < nGroupsOCN_LANES_DICT; g = g + 1)
    begin
        result[idx] = tuple2(groupsOCN_LANES_DICT[g][0], True);
        idx = idx + 1;
    end

    return result;
endmodule



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
//   n_MAX_FLITS_PER_PACKET is needed to guarantee proper internal buffering
//   and credit management so that credits are passed to the OCN only when
//   buffer space is available for an entire packet.
//
interface PORT_OCN_LOCAL_RECV_MULTIPLEXED#(type ni,
                                           numeric type n_MAX_FLITS_PER_PACKET);
    // Returns a vector indicating which lanes/virtual channels have
    // incoming messages.
    method Vector#(NUM_LANES,
                   Vector#(VCS_PER_LANE, Bool)) notEmpty(INSTANCE_ID#(ni) iid);

    // Return a single lane/virtual channel from which a message
    // may be received.  Clients may either use notEmpty() above
    // and pick a channel using the notEmpty bits or may use the
    // pickChannel method.  pickChannel() gets the notEmpty bits
    // on its own and then picks a single winner.
    //
    // pickChannel() will only select a lane that has the corresponding
    // bit set in the "request" parameter.
    method Maybe#(Tuple2#(LANE_IDX,
                          VC_IDX)) pickChannel(INSTANCE_ID#(ni) iid,
                                               Vector#(NUM_LANES, Bool) request);

    // Multi-cycle receive request/response.  The requested lane/vc must have
    // a message, as indicated by notEmpty() or pickChannel() above.
    method Action receiveReq(INSTANCE_ID#(ni) iid, LANE_IDX lane, VC_IDX vc);
    method ActionValue#(OCN_FLIT) receiveRsp(INSTANCE_ID#(ni) iid);

    // Either receiveReq/Rsp or noDeq must be called for each cyle for each iid.
    // There is no doDeq() because receiveReq/Rsp already have side effects
    // and calling them is a commitment to deq a flit.  Adding doDeq would
    // make the interface uniform but is otherwise of no value.  Supporting
    // doDeq either requires more tracking state to prevent bugs or opens
    // the possibility of a bug in which receiveReq and noDeq are called
    // together.
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
    PORT_RECV_MULTIPLEXED#(ni, VC_CREDIT_MSG) creditFromOCN <-
        mkPortRecv_Multiplexed(portNameRecv + "_credit", 1);

    MULTIPLEXED_REG#(ni, Vector#(NUM_LANES, VC_CREDIT_CNT)) senderCreditsPool <-
        mkMultiplexedReg(replicate(0));


    //
    // updateCredits --
    //   Used by both doEnq() and noEnq() to update credits, both for outbound
    //   messages and for credits received from remote receivers.
    //
    function Action updateCredits(t_IID iid, Maybe#(LANE_IDX) m_lane);
    action
        Reg#(Vector#(NUM_LANES, VC_CREDIT_CNT)) senderCredits =
            senderCreditsPool.getReg(iid);

        Vector#(NUM_LANES, VC_CREDIT_CNT) new_sender_credits = senderCredits;

        //
        // Decrement credits for a message sent?
        //
        if (m_lane matches tagged Valid .ln)
        begin
            new_sender_credits[ln] = new_sender_credits[ln] - 1;
            debugLog.record(iid, $format("lpSend lane %0d: Used 1 credit, now %0d", ln, new_sender_credits[ln]));
        end

        //
        // Collect credits from remote receiver.
        //
        let m_credits <- creditFromOCN.receive(iid);
        if (m_credits matches tagged Valid .creds)
        begin
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin
                // Only interested in VC 0.
                new_sender_credits[ln] = boundedPlus(new_sender_credits[ln],
                                                     creds[ln][0]);

                if (creds[ln][0] != 0)
                begin
                    debugLog.record(iid, $format("lpSend lane %0d: %0d new credits, now %0d", ln, creds[ln][0], new_sender_credits[ln]));
                end
            end
        end

        senderCredits <= new_sender_credits;
    endaction
    endfunction


    method Action doEnq(t_IID iid, LANE_IDX lane, OCN_FLIT flit);
        // Sender always uses virtual channel 0.  The network may switch to
        // a different VC during routing.
        OCN_MSG msg = tuple3(lane, 0, flit);
        enqToOCN.send(iid, tagged Valid msg);

        // Update credits.  A credit is consumed at the TAIL of a packet.
        // This may seem counterintuitive.  A credit guarantees space for
        // a full packet -- head to tail.  If the credit were decremented
        // at head we would need a more complex scheme for canEnq() below to
        // know that credit is available.
        Maybe#(LANE_IDX) m_used_credit = tagged Invalid;
        if (flit matches tagged FLIT_BODY .body &&& body.isTail)
        begin
            m_used_credit = tagged Valid lane;
        end
        updateCredits(iid, m_used_credit);

        debugLog.record(iid, $format("lpSend: ENQ ") + fshow(msg));
    endmethod

    method Action noEnq(t_IID iid);
        enqToOCN.send(iid, tagged Invalid);
        updateCredits(iid, tagged Invalid);

        debugLog.record(iid, $format("lpSend: No ENQ"));
    endmethod

    method ActionValue#(Vector#(NUM_LANES, Bool)) canEnq(t_IID iid);
        // Read our local state from the pools.
        Reg#(Vector#(NUM_LANES, VC_CREDIT_CNT)) senderCredits =
            senderCreditsPool.getReg(iid);

        //
        // Reduce sender credits to a bit vector and return the vector.
        //
        function Bool notZero(val) = (val != 0);
        let can_enq = map(notZero, senderCredits);
        debugLog.record(iid, $format("lpSend: Can ENQ %b", pack(can_enq)));

        return can_enq;
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
//   A key attribute of this receiver:  once the header of a packet is
//   accepted, the only channel available on subsequent cycles will be
//   the flit's channel until the entire packet is received.  Receivers
//   should not start a packet unless they can fully buffer it.  The
//   advantage of this is that receivers tracking state for multi-flit
//   packets need only track one packet at a time and do not need to
//   track each channel separately.
//
module [HASIM_MODULE] mkLocalNetworkPortRecv#(
    String portNameSend,
    String portNameRecv,
    TIMEP_DEBUG_FILE_MULTIPLEXED#(ni) debugLog)
    // Interface:
    (PORT_OCN_LOCAL_RECV_MULTIPLEXED#(ni, n_MAX_FLITS_PER_PACKET))
    provisos (Alias#(t_IID, INSTANCE_ID#(ni)),
              // Incoming buffering for each virtual channel
              NumAlias#(n_VC_BUF_PACKETS, 2),
              NumAlias#(n_VC_BUF_FLITS, TMul#(n_VC_BUF_PACKETS, n_MAX_FLITS_PER_PACKET)),
              Alias#(t_VC_FIFO, FUNC_FIFO_IDX#(n_VC_BUF_FLITS)),
              // All FIFOs managing channel buffers for an instance
              Alias#(t_BUFFER_FIFOS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_VC_FIFO))));

    PORT_RECV_MULTIPLEXED#(ni, OCN_MSG) enqFromOCN <-
        mkPortRecv_Multiplexed(portNameRecv + "_enq", 1);
    PORT_SEND_MULTIPLEXED#(ni, VC_CREDIT_MSG) creditToOCN <-
        mkPortSend_Multiplexed(portNameSend + "_credit");

    // Assertion objects
    let checkBufNotFull <- mkAssertionStrPvtChecker("icn-local-port.bsv: " + portNameRecv + " flit received but buffer is full!",
                                                    ASSERT_ERROR);

    // Each virtual channel has an associated incoming buffer (FIFO)
    MULTIPLEXED_REG#(ni, t_BUFFER_FIFOS) vcFIFOsPool <-
        mkMultiplexedReg(replicate(replicate(funcFIFO_IDX_Init)));

    // Storage for each VC's FIFO buffer.  The FIFOs above are merely
    // pointers to this storage.
    MEMORY_IFC#(Tuple4#(INSTANCE_ID#(ni),
                        LANE_IDX, VC_IDX,
                        Bit#(TLog#(n_VC_BUF_FLITS))),
                OCN_FLIT) vcBufEntries <- mkBRAM();
    // Same indexed storage, but just the "isTail" flag
    LUTRAM#(Tuple4#(INSTANCE_ID#(ni),
                    LANE_IDX, VC_IDX,
                    Bit#(TLog#(n_VC_BUF_FLITS))),
            Bool) vcBufIsTail <- mkLUTRAMU();

    // Once a packet begins, the entire packet is received before another
    // may begin.
    MULTIPLEXED_REG#(ni, Maybe#(Tuple2#(LANE_IDX, VC_IDX))) activeVCPool <-
        mkMultiplexedReg(tagged Invalid);

    MULTIPLEXED_REG#(ni, Bool) creditInitializedPool <- mkMultiplexedReg(False);

    FIFO#(Tuple2#(LANE_IDX, VC_IDX)) rspMetaQ <- mkSizedFIFO(4);


    //
    // identityMap is a map from the the vector representation of each virtual
    // channel to the index of the lane and channel pair.  The identity
    // map can be fed into vector mapping functions.
    //
    Vector#(NUM_LANES,
            Vector#(VCS_PER_LANE,
                    Tuple2#(Integer, Integer))) identityMap = newVector();
    for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
    begin
        identityMap[ln] = genWith(tuple2(ln));
    end


    //
    // updCredit --
    //   Forward credits to remote senders as packets are consumed.
    //
    function Action updCredit(t_IID iid,
                              Maybe#(Tuple2#(LANE_IDX, VC_IDX)) m_vc_credit);
    action
        Reg#(Bool) creditInitialized = creditInitializedPool.getReg(iid);

        //
        // Send updated credits to the network.
        //
        if (! creditInitialized)
        begin
            // Send full buffer credits at startup.
            creditInitializedPool.getReg(iid) <= True;

            VC_CREDIT_MSG creds = replicate(replicate(fromInteger(valueOf(n_VC_BUF_PACKETS))));
            creditToOCN.send(iid, tagged Valid creds);
            debugLog.record(iid, $format("lpRecv creditToOCN: init all with %0d", valueOf(n_VC_BUF_PACKETS)));
        end
        else if (m_vc_credit matches tagged Valid {.ln, .vc})
        begin
            VC_CREDIT_MSG creds = replicate(replicate(0));
            creds[ln][vc] = 1;
            creditToOCN.send(iid, tagged Valid creds);
            debugLog.record(iid, $format("lpRecv creditToOCN: ln %0d, vc %0d", ln, vc));
        end
        else
        begin
            creditToOCN.send(iid, tagged Invalid);
        end
    endaction
    endfunction


    //
    // recvFlits --
    //   Receive a new message from the network port and store it in the local
    //   buffer.
    //
    function Action recvFlits(t_IID iid, t_BUFFER_FIFOS upd_vc_buf);
    action
        //
        // Check the network port for a new incoming message.
        //
        let m_enq <- enqFromOCN.receive(iid);
        if (m_enq matches tagged Valid {.lane, .vc, .flit})
        begin
            checkBufNotFull(funcFIFO_IDX_notFull(upd_vc_buf[lane][vc]));

            match {.upd_fifo, .idx} = funcFIFO_IDX_UGenq(upd_vc_buf[lane][vc]);
            upd_vc_buf[lane][vc] = upd_fifo;
            let entry_idx = tuple4(iid, lane, vc, idx);
            vcBufEntries.write(entry_idx, flit);

            Bool is_tail = False;
            if (flit matches tagged FLIT_BODY .body &&& body.isTail)
            begin
                is_tail = True;
            end
            vcBufIsTail.upd(entry_idx, is_tail);

            debugLog.record(iid, $format("lpRecv in: ") + fshow(validValue(m_enq)));
        end
        else
        begin
            debugLog.record(iid, $format("lpRecv in: No message"));
        end

        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        vcBuf <= upd_vc_buf;
    endaction
    endfunction


    function notEmptyVCs(t_IID iid);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);

        Reg#(Maybe#(Tuple2#(LANE_IDX, VC_IDX))) activeVC = activeVCPool.getReg(iid);
        Bool locked = isValid(activeVC);
        match {.locked_ln, .locked_vc} = validValue(activeVC);

        // VC is not empty if it has data and either no VC is currently
        // active or the VC is the active channel.
        function Bool vcNotEmpty(Integer ln, Integer vc);
            return funcFIFO_IDX_notEmpty(vcBuf[ln][vc]) &&
                   (! locked || (locked_ln == fromInteger(ln) &&
                                 locked_vc == fromInteger(vc)));
        endfunction

        // Build a mask of channels from which data may be received this cycle.
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) not_empty = newVector();
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            not_empty[ln] = genWith(vcNotEmpty(ln));
        end

        return not_empty;
    endfunction


    method Vector#(NUM_LANES,
                   Vector#(VCS_PER_LANE, Bool)) notEmpty(t_IID iid);
        return notEmptyVCs(iid);
    endmethod


    method Maybe#(Tuple2#(LANE_IDX,
                          VC_IDX)) pickChannel(t_IID iid,
                                               Vector#(NUM_LANES, Bool) request);
        // Which incoming virtual channels have messages?
        Vector#(NUM_LANES,
                Vector#(VCS_PER_LANE, Bool)) not_empty = notEmptyVCs(iid);

        //
        // isReadyVC --
        //   Is a request available?
        //
        function Bool isReadyVC(Integer ln, Integer vc) = not_empty[ln][vc] &&
                                                          request[ln];

        let ready_vcs = map(uncurry(isReadyVC), concat(identityMap));

        // Static arbitration.  We might have to fix this for fairness.
        Maybe#(Tuple2#(LANE_IDX, VC_IDX)) winner;
        if (findElem(True, ready_vcs) matches tagged Valid .idx)
        begin
            match {.ln, .vc} = concat(identityMap)[idx];
            winner = tagged Valid tuple2(fromInteger(ln),
                                         fromInteger(vc));
        end
        else
        begin
            // Nothing to receive
            winner = tagged Invalid;
        end

        return winner;
    endmethod


    method Action receiveReq(t_IID iid, LANE_IDX lane, VC_IDX vc);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        t_BUFFER_FIFOS upd_vc_buf = vcBuf;

        let idx = funcFIFO_IDX_UGfirst(upd_vc_buf[lane][vc]);
        let entry_idx = tuple4(iid, lane, vc, idx);
        vcBufEntries.readReq(entry_idx);

        // Calling receiveReq() is an implicit deq of the flit.  There
        // is no doDeq().  See comment on PORT_OCN_LOCAL_RECV_MULTIPLEXED
        // noDeq() method for details.
        upd_vc_buf[lane][vc] = funcFIFO_IDX_UGdeq(upd_vc_buf[lane][vc]);

        // Side effect of deq: check for new messages
        recvFlits(iid, upd_vc_buf);

        // Release the packet's buffer and pass credit to the sender.
        Maybe#(Tuple2#(LANE_IDX, VC_IDX)) vc_credit = tagged Invalid;
        if (vcBufIsTail.sub(entry_idx))
        begin
            vc_credit = tagged Valid tuple2(lane, vc);
        end
        updCredit(iid, vc_credit);

        debugLog.record(iid, $format("lpRecv req and deq: ln %0d, vc %0d", lane, vc));
        rspMetaQ.enq(tuple2(lane, vc));
    endmethod

    method ActionValue#(OCN_FLIT) receiveRsp(t_IID iid);
        match {.lane, .vc} = rspMetaQ.first();
        rspMetaQ.deq();

        let flit <- vcBufEntries.readRsp();

        //
        // Track the active channel.  A channel remains active from first
        // flit in a packet until the tail.  This forces the receiver to
        // receive a packet completely before starting another from a
        // different channel.
        //
        // It might seem too late to track active packets late in the pipeline
        // here.  It is not because the local controller permits only one cycle
        // to be active for an instance.  Thus, receiveRsp() will be invoked
        // for an instance before notEmpty() may be called for the next cycle.
        //
        Reg#(Maybe#(Tuple2#(LANE_IDX, VC_IDX))) activeVC = activeVCPool.getReg(iid);

        case (flit) matches 
            tagged FLIT_HEAD .info:
            begin
                activeVC <= tagged Valid tuple2(lane, vc);
            end

            tagged FLIT_BODY .body:
            begin
                if (body.isTail)
                begin
                    activeVC <= tagged Invalid;
                end
            end
        endcase

        OCN_MSG dbg_msg = tuple3(lane, vc, flit);
        debugLog.record(iid, $format("lpRecv rsp: ") + fshow(dbg_msg));

        return flit;
    endmethod

    method Action noDeq(t_IID iid);
        // Read our local state from the pools.
        Reg#(t_BUFFER_FIFOS) vcBuf = vcFIFOsPool.getReg(iid);
        t_BUFFER_FIFOS upd_vc_buf = vcBuf;

        // Side effect of noDeq: check for new messages
        recvFlits(iid, upd_vc_buf);

        // Update credits
        updCredit(iid, tagged Invalid);

        debugLog.record(iid, $format("lpRecv no deq"));
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


// ========================================================================
//
//   Expose the OCN interface as vectors of stall ports.
//
// ========================================================================


//
// mkPortsToOCNLanes --
//   The lowest level connection between model component (e.g. a core's
//   memory hierarhcy) and the on-chip network.  The module maps a set
//   of soft connections into OCN lanes.
//
module [HASIM_MODULE] mkPortsToOCNLanes#(String name,
                                         NumTypeParam#(n_INSTANCES) p0)
    // Interface:
    ();

    //
    // Port names are simply a function of the lane number.
    //

    OCN_FROM_PORT_VEC_MULTIPLEXED#(n_INSTANCES, `OCN_LANES_DICT_ENTRIES)
        portsToOCN = newVector();

    OCN_TO_PORT_VEC_MULTIPLEXED#(n_INSTANCES, `OCN_LANES_DICT_ENTRIES)
        ocnToPorts = newVector();

    for (Integer p = 0; p < `OCN_LANES_DICT_ENTRIES; p = p + 1)
    begin
        portsToOCN[p] <- mkPortStallRecv_Multiplexed(laneNameToOCN(name, p));
        ocnToPorts[p] <- mkPortStallSend_Multiplexed(laneNameFromOCN(name, p));
    end

    let mapPortsToLanes <- ocnMapPortsToLanes();
    let mapLanesToPorts <- ocnMapLanesToPorts();

    mkOCNConnection(portsToOCN, ocnToPorts,
                    mapPortsToLanes, mapLanesToPorts,
                    name + "_OCN_Connection");
endmodule


//
// Ports connecting to the OCN must be of types that are members of the
// OCN_SEND_TYPE typeclass.
//
typedef struct
{
    OCN_FLIT_HEAD headFlit;
    OCN_PACKET_PAYLOAD payload;
}
OCN_MSG_HEAD_AND_PAYLOAD
    deriving (Eq, Bits);


typeclass OCN_SEND_TYPE#(type t_MSG);
    function OCN_MSG_HEAD_AND_PAYLOAD cvtToOCNFlits(STATION_ID tgt, t_MSG msg);

    function Tuple2#(STATION_ID, t_MSG) cvtFromOCNFlits(
        OCN_MSG_HEAD_AND_PAYLOAD ocnMsg
        );
endtypeclass


//
// Convert between MEMORY_REQ and OCN_SEND_TYPE.
//
instance OCN_SEND_TYPE#(MEMORY_REQ);
    function OCN_MSG_HEAD_AND_PAYLOAD cvtToOCNFlits(STATION_ID tgt, MEMORY_REQ req);
        OCN_MSG_HEAD_AND_PAYLOAD info;

        info.headFlit = OCN_FLIT_HEAD { src: ?,
                                        dst: tgt,
                                        isStore: req.isStore };

        info.payload = zeroExtend(pack(tuple2(req.opaque,
                                              req.linePAddr)));

        return info;
    endfunction

    function Tuple2#(STATION_ID, MEMORY_REQ) cvtFromOCNFlits(
        OCN_MSG_HEAD_AND_PAYLOAD ocnMsg
        );

        let head = ocnMsg.headFlit;

        MEM_OPAQUE opaque;
        LINE_ADDRESS pa;
        {opaque, pa} = unpack(truncate(ocnMsg.payload));

        MEMORY_REQ mreq = head.isStore ? initMemStore(pa) : initMemLoad(pa);
        mreq.opaque = opaque;

        return tuple2(head.src, mreq);
    endfunction
endinstance

//
// Convert between MEMORY_RSP and OCN_SEND_TYPE.
//
instance OCN_SEND_TYPE#(MEMORY_RSP);
    function OCN_MSG_HEAD_AND_PAYLOAD cvtToOCNFlits(STATION_ID tgt, MEMORY_RSP rsp);
        OCN_MSG_HEAD_AND_PAYLOAD info;

        info.headFlit = OCN_FLIT_HEAD { src: ?,
                                        dst: tgt,
                                        isStore: False };

        info.payload = zeroExtend(pack(tuple2(rsp.opaque,
                                              rsp.linePAddr)));

        return info;
    endfunction

    function Tuple2#(STATION_ID, MEMORY_RSP) cvtFromOCNFlits(
        OCN_MSG_HEAD_AND_PAYLOAD ocnMsg
        );

        let head = ocnMsg.headFlit;

        MEM_OPAQUE opaque;
        LINE_ADDRESS pa;
        {opaque, pa} = unpack(truncate(ocnMsg.payload));

        return tuple2(head.src, initMemRsp(pa, opaque));
    endfunction
endinstance


//
// Ports are passed in as the following vectors.
//

typedef Vector#(n_PORTS,
                PORT_STALL_RECV_MULTIPLEXED#(n_INSTANCES,
                                             OCN_MSG_HEAD_AND_PAYLOAD))
    OCN_FROM_PORT_VEC_MULTIPLEXED#(numeric type n_INSTANCES, numeric type n_PORTS);

typedef Vector#(n_PORTS,
                PORT_STALL_SEND_MULTIPLEXED#(n_INSTANCES,
                                             OCN_MSG_HEAD_AND_PAYLOAD))
    OCN_TO_PORT_VEC_MULTIPLEXED#(numeric type n_INSTANCES, numeric type n_PORTS);


//
// mkOCNConnection --
//   Build a shim, connecting a set of stall ports to a set of OCN virtual
//   channels.  The ports are passed in as two vectors, one for each
//   direction:  portsToOCN and ocnToPorts.  Positions in the vector
//   correspond to virtual channel numbers.
//
module [HASIM_MODULE] mkOCNConnection#(
    OCN_FROM_PORT_VEC_MULTIPLEXED#(n_INSTANCES, n_PORTS) portsToOCN,
    OCN_TO_PORT_VEC_MULTIPLEXED#(n_INSTANCES, n_PORTS) ocnToPorts,
    OCN_MAP_PORTS_TO_LANES mapPortsToLanes,
    OCN_MAP_LANES_TO_PORTS mapLanesToPorts,
    String ifcName)
    // Interface:
    ()
    provisos (Alias#(t_PORT_IDX, Bit#(TLog#(n_PORTS))));

    TIMEP_DEBUG_FILE_MULTIPLEXED#(n_INSTANCES) debugLog <-
        mkTIMEPDebugFile_Multiplexed("ocn_connection_" + ifcName + ".out");

    //
    // Side memories hold the actual contents of a packet instead of forcing all
    // datapaths in the simulated OCN to be wide enough to pass a full packet.
    //
    OCN_PACKET_PAYLOAD_CLIENT payloadStorage <- mkNetworkPacketPayloadClient(1);

    //
    // Wrapped interfaces to/from the OCN.  The wrappers simplify
    // the protocol for credit management.
    //
    PORT_OCN_LOCAL_SEND_MULTIPLEXED#(n_INSTANCES) ocnSend <-
        mkLocalNetworkPortSend(ifcName + "_OutQ",
                               ifcName + "_InQ",
                               debugLog);

    PORT_OCN_LOCAL_RECV_MULTIPLEXED#(n_INSTANCES,
                                     MAX_FLITS_PER_PACKET) ocnRecv <-
        mkLocalNetworkPortRecv(ifcName + "_OutQ",
                               ifcName + "_InQ",
                               debugLog);


    Vector#(TAdd#(2, TMul#(2, n_PORTS)),
            INSTANCE_CONTROL_IN#(n_INSTANCES)) inctrls = newVector();
    inctrls[0] = ocnSend.ctrl.in;
    inctrls[1] = ocnRecv.ctrl.in;

    Vector#(TAdd#(2, TMul#(2, n_PORTS)),
            INSTANCE_CONTROL_OUT#(n_INSTANCES)) outctrls = newVector();
    outctrls[0] = ocnSend.ctrl.out;
    outctrls[1] = ocnRecv.ctrl.out;

    //    
    // Add controls for the ports passed in to the connector.
    //    
    Integer slot = 2;
    for (Integer i = 0; i < valueOf(n_PORTS); i = i + 1)    
    begin
        inctrls[slot] = portsToOCN[i].ctrl.in;
        outctrls[slot] = portsToOCN[i].ctrl.out;
        slot = slot + 1;
    end

    for (Integer i = 0; i < valueOf(n_PORTS); i = i + 1)    
    begin
        inctrls[slot] = ocnToPorts[i].ctrl.in;
        outctrls[slot] = ocnToPorts[i].ctrl.out;
        slot = slot + 1;
    end


    LOCAL_CONTROLLER#(n_INSTANCES) localCtrl <- mkNamedLocalController(ifcName, inctrls, outctrls);
    STAGE_CONTROLLER#(n_INSTANCES, Tuple2#(Bool, Maybe#(LANE_IDX))) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(n_INSTANCES, Tuple3#(Bool,
                                            Maybe#(OCN_PACKET_HANDLE),
                                            Maybe#(Tuple2#(LANE_IDX, OCN_FLIT)))) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(n_INSTANCES, Bool) stage4Ctrl <- mkBufferedStageController();

    //
    // Messages are broken into flits in this module when transmitted on the
    // OCN.  For now we always use packets that are two flits.  These
    // registers hold the tail flits after a head is transmitted.
    //
    MULTIPLEXED_REG#(n_INSTANCES,
                     Maybe#(Tuple2#(LANE_IDX, OCN_FLIT_OPAQUE))) packetizingToOCNPool <-
        mkMultiplexedReg(tagged Invalid);

    //
    // Track the source of the packet being received.  The OCN receiver interface
    // guarantees that once a packet starts the only flits seen will be from
    // the same packet.  This makes matching header and body flits trivial.
    //
    MULTIPLEXED_REG#(n_INSTANCES, OCN_FLIT_HEAD) recvPacketHeadPool <-
        mkMultiplexedReg(?);


    (* conservative_implicit_conditions *)
    rule stage1_sendToOCN (True);
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Check credits for sending to the network
        let can_enq <- ocnSend.canEnq(cpu_iid);

        // Remaining packets for all outbound lanes.
        Reg#(Maybe#(Tuple2#(LANE_IDX, OCN_FLIT_OPAQUE))) packetizingToOCN =
            packetizingToOCNPool.getReg(cpu_iid);

        //
        // Collect messages heading toward the OCN.  The "receive" operation
        // here is not a commitment to process a message.  For that, doDeq()
        // must be called.
        //
        Vector#(n_PORTS, Maybe#(OCN_MSG_HEAD_AND_PAYLOAD)) m_outToOCN = newVector();

        for (Integer i = 0; i < valueOf(n_PORTS); i = i + 1)
        begin
            m_outToOCN[i] <- portsToOCN[i].receive(cpu_iid);
        end

        //
        // Pick a winner.
        //

        Vector#(n_PORTS, Bool) did_enq_outToOCN = replicate(False);
        Bool did_payload_storage_write = False;
        Bool did_ocn_enq = False;

        //
        // Send a flit -- either a remaining flit from a previous packet or
        // a new packet.
        //
        if (packetizingToOCN matches tagged Valid {.lane, .op})
        begin
            //
            // Complete an LLC response by sending the remainder of an old packet.
            //
            if (can_enq[lane])
            begin
                let msg = tagged FLIT_BODY OCN_FLIT_BODY {opaque: op, isTail: True};
                ocnSend.doEnq(cpu_iid, pack(lane), msg);
                packetizingToOCN <= tagged Invalid;
                did_ocn_enq = True;

                debugLog.record(cpu_iid, $format("1: Lane %0d: tail", lane));
            end
        end
        else
        begin
            //
            // Start a new packet if one exists for a channel that is ready
            // to receive.
            //
            // For now, arbitration is static with priority going to lower
            // channels.
            //
            for (Integer i = 0; i < valueOf(n_PORTS); i = i + 1)
            begin
                match {.ln, .tag} = mapPortsToLanes[i];

                if (m_outToOCN[i] matches tagged Valid .info &&&
                    can_enq[ln] &&&
                    payloadStorage.allocNotEmpty &&&
                    ! did_payload_storage_write)
                begin
                    // Send the head flit.
                    OCN_FLIT flit = tagged FLIT_HEAD info.headFlit;
                    ocnSend.doEnq(cpu_iid, ln, flit);
                    did_ocn_enq = True;

                    // Note deq from port.
                    portsToOCN[i].doDeq(cpu_iid);
                    did_enq_outToOCN[i] = True;

                    // Save the full payload in the side buffer.  The flits
                    // sent over the OCN in te model are a subset of the message.
                    let h <- payloadStorage.allocHandle();
                    payloadStorage.write(h, OCN_PACKET_BUNDLE { tag: tag,
                                                                payload: info.payload });
                    did_payload_storage_write = True;

                    // The packet is broken into two flits.  The second flit
                    // holds the index of the payload storage record.
                    packetizingToOCN <= tagged Valid tuple2(ln, h);

                    debugLog.record(cpu_iid, $format("1: Port %0d, Lane %0d: FWD to OCN, payload idx 0x%0x, ", i, ln, h) + fshow(flit));
                end
            end
        end

        // Was anything sent to the OCN?
        if (! did_ocn_enq)
        begin
            ocnSend.noEnq(cpu_iid);
        end

        // For each incoming port note lack of deq if no message consumed.
        for (Integer i = 0; i < fromInteger(valueOf(n_PORTS)); i = i + 1)
        begin
            if (! did_enq_outToOCN[i])
            begin
                portsToOCN[i].noDeq(cpu_iid);
            end
        end

        
        //
        // Is a message available incoming from the OCN?  We start here
        // because it is a multi-cycle operation.
        //

        // Only request packets that can be forwarded this cycle.
        Vector#(NUM_LANES, Bool) can_enq_vec = replicate(True);
        for (Integer i = 0; i < fromInteger(valueOf(n_PORTS)); i = i + 1)
        begin
            // Multiple ports may map to the same lane.  Claim ability to
            // enq to a lane only if all ports in a group can accept
            // a message.  All the ports shared a lane (virtual channel)
            // in the OCN already, so this requirement won't induce a deadlock.
            match {.ln, .tag} = mapPortsToLanes[i];
            let can_enq_to_port <- ocnToPorts[i].canEnq(cpu_iid);
            can_enq_vec[ln] = can_enq_vec[ln] && can_enq_to_port;
        end

        Maybe#(LANE_IDX) recv_ln = tagged Invalid;
        if (ocnRecv.pickChannel(cpu_iid, can_enq_vec) matches
                tagged Valid {.ln_in, .vc_in})
        begin
            ocnRecv.receiveReq(cpu_iid, ln_in, vc_in);
            recv_ln = tagged Valid ln_in;
        end
        else
        begin
            ocnRecv.noDeq(cpu_iid);
        end

        stage2Ctrl.ready(cpu_iid, tuple2(did_payload_storage_write, recv_ln));
    endrule


    rule stage2_lookupAddr (True);
        match {.cpu_iid,
               .did_payload_storage_write,
               .m_ln} <- stage2Ctrl.nextReadyInstance();

        Maybe#(OCN_PACKET_HANDLE) m_payload_storage_read = tagged Invalid;

        OCN_FLIT flit = ?;
        if (isValid(m_ln))
        begin
            flit <- ocnRecv.receiveRsp(cpu_iid);
        end

        // Need to read the PA associated with an incoming message?  Stage 3
        // is separate from this step to wait for the BRAM read.
        if (flit matches tagged FLIT_BODY .info &&& info.isTail)
        begin
            payloadStorage.readReq(info.opaque);
            m_payload_storage_read = tagged Valid info.opaque;
        end

        if (m_ln matches tagged Valid .ln)
        begin
            stage3Ctrl.ready(cpu_iid, tuple3(did_payload_storage_write,
                                             m_payload_storage_read,
                                             tagged Valid tuple2(ln, flit)));
        end
        else
        begin
            stage3Ctrl.ready(cpu_iid, tuple3(did_payload_storage_write,
                                             m_payload_storage_read,
                                             tagged Invalid));
        end
    endrule


    rule stage3_recvFromOCN (True);
        match {.cpu_iid,
               .did_payload_storage_write,
               .m_payload_storage_read,
               .m_flit} <- stage3Ctrl.nextReadyInstance();
        
        // Remaining packets for all outbound lanes.
        Reg#(OCN_FLIT_HEAD) recvPacketHead =
            recvPacketHeadPool.getReg(cpu_iid);

        Vector#(n_PORTS, Bool) did_enq_ocnToPorts = replicate(False);

        // Looking up payload contents in the side storage buffer?
        OCN_PACKET_BUNDLE bundle = ?;
        if (m_payload_storage_read matches tagged Valid .h)
        begin
            bundle <- payloadStorage.readRsp();
            
            // Packet complete.  Release the buffer entry.
            payloadStorage.freeHandle(h);
            debugLog.record(cpu_iid, $format("3: Free 0x%0x", h));
        end

        if (m_flit matches tagged Valid {.ln, .flit})
        begin
            if (flit matches tagged FLIT_HEAD .info)
            begin
                // Record the source of the packet
                recvPacketHead <= info;
                debugLog.record(cpu_iid, $format("3: Lane %0d: Recv ", ln) + fshow(flit));
            end
            else if (flit matches tagged FLIT_BODY .info &&& info.isTail)
            begin
                // Message arrived completely.  Forward it to the outgoing port.
                OCN_MSG_HEAD_AND_PAYLOAD msg;
                msg.headFlit = recvPacketHead;
                msg.payload = bundle.payload;

                // Compute the A-Port corresponding to the OCN lane.
                t_PORT_IDX idx = fromInteger(tpl_1(mapLanesToPorts[ln]));
                Bool is_group = tpl_2(mapLanesToPorts[ln]);
                if (is_group)
                begin
                    // Multiple ports connect to the lane.  The tag disambiguates.
                    idx = idx + resize(bundle.tag);
                end

                ocnToPorts[idx].doEnq(cpu_iid, msg);
                did_enq_ocnToPorts[idx] = True;
                debugLog.record(cpu_iid, $format("3: Lane %0d, Port %0d: Recv ", ln, idx) + fshow(flit));
            end
        end

        for (Integer i = 0; i < fromInteger(valueOf(n_PORTS)); i = i + 1)
        begin
            if (! did_enq_ocnToPorts[i])
            begin
                ocnToPorts[i].noEnq(cpu_iid);
            end
        end
        
        stage4Ctrl.ready(cpu_iid, did_payload_storage_write);
    endrule


    //
    // Wait for confirmation that the payload storage buffer has been updated
    // and is visible to the recipient.
    //
    rule stage4_writeAck (True);
        match {.cpu_iid,
               .did_payload_storage_write} <- stage4Ctrl.nextReadyInstance();

        if (did_payload_storage_write)
        begin
            payloadStorage.writeAck();
        end

        localCtrl.endModelCycle(cpu_iid, 0);
    endrule
endmodule
