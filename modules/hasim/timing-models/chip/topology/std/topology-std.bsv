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
// Topology configuration parameters are streamed in from software.
//

import FIFO::*;
import Vector::*;
import GetPut::*;

`include "awb/rrr/server_stub_TOPOLOGY.bsh"
`include "awb/dict/TOPOLOGY.bsh"


// Size of a single topology value message
typedef 16 TOPOLOGY_VALUE_SZ;


// ========================================================================
//
//  Modules for receiving topology configuration for use by user code.
//
// ========================================================================

//
// mkTopologyParamReg --
//   A topology parameter register.  One value for the paramID (in the TOPOLOGY
//   dictionary space) is received and stored here.  The register is enabled
//   only after a value is received.
//
module [CONNECTED_MODULE] mkTopologyParamReg#(Integer paramID)
    // Interface:
    (ReadOnly#(t_VALUE))
    provisos (Bits#(t_VALUE, t_VALUE_SZ));

    // Receive values
    CONNECTION_CHAIN#(TOPOLOGY_PARAM_MSG) chain <- mkConnectionChain("TopologyConfigRing");

    Reg#(Bool) initialized <- mkReg(False);

    //
    // Does the value fit in a single message?
    //
    if (valueOf(t_VALUE_SZ) <= valueOf(TOPOLOGY_VALUE_SZ))
    begin
        //
        // Simple version.  One message initializes the value.
        //

        Reg#(t_VALUE) value <- mkRegU();

        rule recvValues (True);
            // Get the next message
            let msg <- chain.recvFromPrev();
            // Forward it around the chain
            chain.sendToNext(msg);

            // Does message hold the requested parameter?
            if (fromInteger(paramID) == msg.id)
            begin
                initialized <= True;
                value <= unpack(truncateNP(msg.value));
            end
        endrule


        method t_VALUE _read() if (initialized);
            return value;
        endmethod
    end
    else
    begin
        //
        // Marshalled version.  Multiple messages initialize a value.
        //

        // The value, represented as chunks.
        Reg#(Vector#(TDiv#(t_VALUE_SZ, TOPOLOGY_VALUE_SZ),
                     Bit#(TOPOLOGY_VALUE_SZ))) value <- mkRegU();

        rule recvValues (True);
            // Get the next message
            let msg <- chain.recvFromPrev();
            // Forward it around the chain
            chain.sendToNext(msg);

            // Does message hold the requested parameter?
            if (fromInteger(paramID) == msg.id)
            begin
                // Update value, scanning in one new chunk
                value <= shiftInAtN(value, msg.value);
                initialized <= msg.lastForID;
            end
        endrule


        method t_VALUE _read() if (initialized);
            return unpack(truncateNP(pack(value)));
        endmethod
    end
endmodule


//
// mkTopologyParamStream --
//   A topology parameter that is a stream of data (e.g. initialization of
//   a table.)  The output value is marked valid until end of stream.  End of
//   stream is indicated by a single invalid message.  Some streams may have
//   multiple chunks, indicated by multiple data/end of stream sequences.
//
module [CONNECTED_MODULE] mkTopologyParamStream#(Integer paramID)
    // Interface:
    (Get#(Maybe#(t_VALUE)))
    provisos (Bits#(t_VALUE, t_VALUE_SZ));

    // Receive values
    CONNECTION_CHAIN#(TOPOLOGY_PARAM_MSG) chain <- mkConnectionChain("TopologyConfigRing");

    // Use "slow" FIFO1 to save space.  Performance doesn't matter.
    FIFO#(Maybe#(t_VALUE)) valueQ <- mkFIFO1();

    Reg#(Bool) eos <- mkReg(False);

    //
    // Does the value fit in a single message?
    //
    if (valueOf(t_VALUE_SZ) <= valueOf(TOPOLOGY_VALUE_SZ))
    begin
        //
        // Simple version.  One message per stream entry.
        //
        rule recvValues (! eos);
            // Get the next message
            let msg <- chain.recvFromPrev();
            // Forward it around the chain
            chain.sendToNext(msg);

            // Does message hold the requested parameter?
            if (fromInteger(paramID) == msg.id)
            begin
                valueQ.enq(tagged Valid unpack(truncateNP(msg.value)));
                eos <= msg.lastForStream;
            end
        endrule
    end
    else
    begin
        //
        // Marshalled version.  Multiple messages initialize a value.
        //

        // The value, represented as chunks.
        Reg#(Vector#(TDiv#(t_VALUE_SZ, TOPOLOGY_VALUE_SZ),
                     Bit#(TOPOLOGY_VALUE_SZ))) value <- mkRegU();

        rule recvValues (! eos);
            // Get the next message
            let msg <- chain.recvFromPrev();
            // Forward it around the chain
            chain.sendToNext(msg);

            // Does message hold the requested parameter?
            if (fromInteger(paramID) == msg.id)
            begin
                // Update value, scanning in one new chunk
                let n_value = shiftInAtN(value, msg.value);
                value <= n_value;

                if (msg.lastForID)
                begin
                    valueQ.enq(tagged Valid unpack(truncateNP(pack(n_value))));
                    eos <= msg.lastForStream;
                end
            end
        endrule
    end


    rule endOfStream (eos);
        valueQ.enq(tagged Invalid);
        eos <= False;
    endrule


    method ActionValue#(Maybe#(t_VALUE)) get();
        let m = valueQ.first();
        valueQ.deq();

        return m;
    endmethod
endmodule


// ========================================================================
//
//  Internal code.
//
// ========================================================================

typedef struct
{
    Bit#(`TOPOLOGY_DICT_BITS) id;
    Bit#(TOPOLOGY_VALUE_SZ) value;

    // Last message for a marshalled value requiring multiple messages
    Bool lastForID;
 
    // Last value for a stream of values (relevant to mkTopologyParamStream
    // only).
    Bool lastForStream;
}
TOPOLOGY_PARAM_MSG
    deriving (Eq, Bits);

//
// mkTopology --
//   Receive state streamed in from software and pass it to the configuration
//   ring.
//
module [CONNECTED_MODULE] mkTopology
    // Interface:
    ();

    // ****** State Elements ******

    // Communication link to the Params themselves
    CONNECTION_CHAIN#(TOPOLOGY_PARAM_MSG) chain <- mkConnectionChain("TopologyConfigRing");
 
    // Communication to our RRR server
    ServerStub_TOPOLOGY serverStub <- mkServerStub_TOPOLOGY();
    
    // ****** Rules ******

    rule fwdParam (True);
        let r <- serverStub.acceptRequest_setParam();

        let msg = TOPOLOGY_PARAM_MSG { id: truncate(r.id),
                                       value: r.value,
                                       lastForID: unpack(r.last[0]),
                                       lastForStream: unpack(r.last[1]) };
        chain.sendToNext(msg);
    endrule
    
    //
    // Sink messages coming around the ring.
    //
    rule receive (True);
        let msg <- chain.recvFromPrev();
    endrule
endmodule
