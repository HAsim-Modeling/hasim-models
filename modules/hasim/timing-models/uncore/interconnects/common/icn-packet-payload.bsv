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
// To save buffer space in the model, the true payload in a packet is not
// sent across the simulated network.  Instead, a handle is sent in the
// tail flit.  The payload is stored in a shared memory, managed here.
//


import Vector::*;
import Arbiter::*;


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


typedef Bit#(`OCN_PACKET_PAYLOAD_BITS) OCN_PACKET_PAYLOAD;
typedef OCN_FLIT_OPAQUE OCN_PACKET_HANDLE;

//
// Client interface to the pool of storage for holding packet payloads.
// There may be multiple clients.
//
interface OCN_PACKET_PAYLOAD_CLIENT;
    // Allocate and free handles for storing packet payloads.
    method ActionValue#(OCN_PACKET_HANDLE) allocHandle();
    method Bool allocNotEmpty();
    method Action freeHandle(OCN_PACKET_HANDLE handle);

    method Action readReq(OCN_PACKET_HANDLE handle);
    method ActionValue#(OCN_PACKET_PAYLOAD) readRsp();

    method Action write(OCN_PACKET_HANDLE handle, OCN_PACKET_PAYLOAD payload);
    // An ACK is returned for every write when the write is globally visible.
    method Action writeAck();
endinterface


//
// mkNetworkPacketPayloadClient --
//   Instantiate one client interface to the pool of storage for holding
//   packet payloads.  One client must be allocated for each ID in the
//   range 0 .. (n_CLIENTS-1), where n_CLIENTS is the value passed to
//   mkNetworkPacketPayloadStorage() below.
//
//   This implementation uses a single heap, allocated in
//   mkNetworkPacketPayloadStorage.  A different implementation could use
//   the same interface but manage the storage as a set of coherent
//   scratchpads.
//
module [HASIM_MODULE] mkNetworkPacketPayloadClient#(Integer id)
    // Interface:
    (OCN_PACKET_PAYLOAD_CLIENT);

    String suffix = integerToString(id);

    CONNECTION_RECV#(OCN_PACKET_HANDLE) allocQ <-
        mkConnectionRecv("OCN_PACKET_PAYLOAD_ALLOC_" + suffix);
    CONNECTION_SEND#(OCN_PACKET_HANDLE) freeQ <-
        mkConnectionSend("OCN_PACKET_PAYLOAD_FREE_" + suffix);

    CONNECTION_SEND#(OCN_PACKET_HANDLE) readReqQ <-
        mkConnectionSend("OCN_PACKET_PAYLOAD_READREQ_" + suffix);
    CONNECTION_RECV#(OCN_PACKET_PAYLOAD) readRspQ <-
        mkConnectionRecv("OCN_PACKET_PAYLOAD_READRSP_" + suffix);

    CONNECTION_SEND#(Tuple2#(OCN_PACKET_HANDLE, OCN_PACKET_PAYLOAD)) writeQ <-
        mkConnectionSend("OCN_PACKET_PAYLOAD_WRITE_" + suffix);
    CONNECTION_RECV#(Bool) writeAckQ <-
        mkConnectionRecv("OCN_PACKET_PAYLOAD_WRITEACK_" + suffix);

    method ActionValue#(OCN_PACKET_HANDLE) allocHandle();
        let h = allocQ.receive();
        allocQ.deq();

        return h;
    endmethod

    method Bool allocNotEmpty = allocQ.notEmpty;
    method freeHandle = freeQ.send;

    method readReq = readReqQ.send;

    method ActionValue#(OCN_PACKET_PAYLOAD) readRsp();
        let r = readRspQ.receive();
        readRspQ.deq();

        return r;
    endmethod

    method Action write(OCN_PACKET_HANDLE handle, OCN_PACKET_PAYLOAD payload) =
        writeQ.send(tuple2(handle, payload));

    method Action writeAck() = writeAckQ.deq();
endmodule


//
// mkNetworkPacketPayloadStorage --
//   Allocate the server that manages storage for a collection of packet
//   payload clients.  Only one instance of the server should be allocated
//   and n_CLIENTS clients must be allocated in order for the soft connections
//   to match.
//
module [HASIM_MODULE] mkNetworkPacketPayloadStorage#(NumTypeParam#(n_CLIENTS) p)
    // Interface:
    ();

    Vector#(n_CLIENTS, CONNECTION_SEND#(OCN_PACKET_HANDLE)) allocQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_RECV#(OCN_PACKET_HANDLE)) freeQ = newVector();

    Vector#(n_CLIENTS, CONNECTION_RECV#(OCN_PACKET_HANDLE)) readReqQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_SEND#(OCN_PACKET_PAYLOAD)) readRspQ = newVector();

    Vector#(n_CLIENTS, CONNECTION_RECV#(Tuple2#(OCN_PACKET_HANDLE,
                                               OCN_PACKET_PAYLOAD))) writeQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_SEND#(Bool)) writeAckQ = newVector();

    MEMORY_HEAP_MULTI_READ#(n_CLIENTS, OCN_PACKET_HANDLE, OCN_PACKET_PAYLOAD) mem <-
        mkMultiReadMemoryHeapUnionBRAM();

    //
    // Allocate soft connections to all clients.
    //
    for (Integer i = 0; i < valueOf(n_CLIENTS); i = i + 1)
    begin
        String suffix = integerToString(i);
        allocQ[i] <- mkConnectionSend("OCN_PACKET_PAYLOAD_ALLOC_" + suffix);
        freeQ[i] <- mkConnectionRecv("OCN_PACKET_PAYLOAD_FREE_" + suffix);

        readReqQ[i] <- mkConnectionRecv("OCN_PACKET_PAYLOAD_READREQ_" + suffix);
        readRspQ[i] <- mkConnectionSend("OCN_PACKET_PAYLOAD_READRSP_" + suffix);

        writeQ[i] <- mkConnectionRecv("OCN_PACKET_PAYLOAD_WRITE_" + suffix);
        writeAckQ[i] <- mkConnectionSend("OCN_PACKET_PAYLOAD_WRITEACK_" + suffix);
    end

    //
    // Keep the alloc queues full using the heap's free list.
    //
    Arbiter_IFC#(n_CLIENTS) allocArb <- mkArbiter(False);
    Rules allocRuleSet = emptyRules;

    for (Integer i = 0; i < valueOf(n_CLIENTS); i = i + 1)
    begin
        (* fire_when_enabled, no_implicit_conditions *)
        rule allocReq (allocQ[i].notFull);
            allocArb.clients[i].request();
        endrule

        Rules next_rule =
            rules
                rule doAlloc (allocArb.clients[i].grant);
                    let h <- mem.malloc();
                    allocQ[i].send(h);
                endrule
            endrules;

        allocRuleSet = rJoinMutuallyExclusive(allocRuleSet, next_rule);
    end

    addRules(allocRuleSet);


    //
    // Push returned entries back on the free list.
    //
    Arbiter_IFC#(n_CLIENTS) freeArb <- mkArbiter(False);
    Rules freeRuleSet = emptyRules;

    for (Integer i = 0; i < valueOf(n_CLIENTS); i = i + 1)
    begin
        (* fire_when_enabled, no_implicit_conditions *)
        rule freeReq (freeQ[i].notEmpty);
            freeArb.clients[i].request();
        endrule

        Rules next_rule =
            rules
                rule doFree (freeArb.clients[i].grant);
                    mem.free(freeQ[i].receive);
                    freeQ[i].deq();
                endrule
            endrules;

        freeRuleSet = rJoinMutuallyExclusive(freeRuleSet, next_rule);
    end

    addRules(freeRuleSet);

    //
    // Read request/response map 1:1 to mem read ports.
    //
    for (Integer i = 0; i < valueOf(n_CLIENTS); i = i + 1)
    begin
        rule doReadReq (True);
            mem.readPorts[i].readReq(readReqQ[i].receive);
            readReqQ[i].deq();
        endrule

        rule doReadRsp (True);
            let r <- mem.readPorts[i].readRsp();
            readRspQ[i].send(r);
        endrule
    end

    //
    // Handle writes.
    //
    Arbiter_IFC#(n_CLIENTS) writeArb <- mkArbiter(False);
    Rules writeRuleSet = emptyRules;

    for (Integer i = 0; i < valueOf(n_CLIENTS); i = i + 1)
    begin
        (* fire_when_enabled, no_implicit_conditions *)
        rule writeReq (writeQ[i].notEmpty);
            writeArb.clients[i].request();
        endrule

        Rules next_rule =
            rules
                rule doWrite (writeArb.clients[i].grant);
                    match {.h, .p} = writeQ[i].receive();
                    writeQ[i].deq();

                    mem.write(h, p);

                    // Confirm write
                    writeAckQ[i].send(?);
                endrule
            endrules;

        writeRuleSet = rJoinMutuallyExclusive(writeRuleSet, next_rule);
    end

    addRules(writeRuleSet);
endmodule

