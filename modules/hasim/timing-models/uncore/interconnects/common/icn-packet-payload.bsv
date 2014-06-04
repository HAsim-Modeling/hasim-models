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
// To save buffer space in the model, the true payload in a packet is not
// sent across the simulated network.  Instead, a handle is sent in the
// tail flit.  The payload is stored in a shared memory, managed here.
//


import Vector::*;
import Arbiter::*;
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
`include "awb/provides/hasim_chip_topology.bsh"


// ****** Generated files ******

`include "awb/dict/VDEV_SCRATCH.bsh"


typedef OCN_FLIT_OPAQUE OCN_PACKET_HANDLE;

//
// Packets are a combination of the payload and a tag.  The tag is generally
// used to route messages as they leave the OCN when multiple model clients
// share a virtual channel.  E.g., both the LLC and memory controllers may share
// a common response virtual channel.
//
typedef Bit#(`OCN_PACKET_TAG_BITS) OCN_PACKET_TAG;
typedef Bit#(`OCN_PACKET_PAYLOAD_BITS) OCN_PACKET_PAYLOAD;

typedef struct
{
    OCN_PACKET_TAG tag;
    OCN_PACKET_PAYLOAD payload;
}
OCN_PACKET_BUNDLE
    deriving (Eq, Bits);


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
    method ActionValue#(OCN_PACKET_BUNDLE) readRsp();

    method Action write(OCN_PACKET_HANDLE handle, OCN_PACKET_BUNDLE payload);
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
    CONNECTION_RECV#(OCN_PACKET_BUNDLE) readRspQ <-
        mkConnectionRecv("OCN_PACKET_PAYLOAD_READRSP_" + suffix);

    CONNECTION_SEND#(Tuple2#(OCN_PACKET_HANDLE, OCN_PACKET_BUNDLE)) writeQ <-
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

    method ActionValue#(OCN_PACKET_BUNDLE) readRsp();
        let r = readRspQ.receive();
        readRspQ.deq();

        return r;
    endmethod

    method Action write(OCN_PACKET_HANDLE handle, OCN_PACKET_BUNDLE payload) =
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
    ()
    provisos (Bits#(OCN_PACKET_HANDLE, t_OCN_PACKET_HANDLE_SZ));

    Vector#(n_CLIENTS, CONNECTION_SEND#(OCN_PACKET_HANDLE)) allocQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_RECV#(OCN_PACKET_HANDLE)) freeQ = newVector();

    Vector#(n_CLIENTS, CONNECTION_RECV#(OCN_PACKET_HANDLE)) readReqQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_SEND#(OCN_PACKET_BUNDLE)) readRspQ = newVector();

    Vector#(n_CLIENTS, CONNECTION_RECV#(Tuple2#(OCN_PACKET_HANDLE,
                                                OCN_PACKET_BUNDLE))) writeQ = newVector();
    Vector#(n_CLIENTS, CONNECTION_SEND#(Bool)) writeAckQ = newVector();

    MEMORY_HEAP_MULTI_READ#(n_CLIENTS, OCN_PACKET_HANDLE, OCN_PACKET_BUNDLE) mem;
    if (valueOf(t_OCN_PACKET_HANDLE_SZ) <= 14)
    begin
        // Smaller configurations use BRAM for storing messages.
        mem <- mkMultiReadMemoryHeapUnionBRAM();
    end
    else
    begin
        // Larger configurations use a scratchpad.
        mem <- mkMultiReadMemoryHeapUnionMem(
                   mkMultiReadScratchpad(`VDEV_SCRATCH_HASIM_ICN_PACKET_PAYLOAD,
                                         defaultValue));
    end

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
    // Raise an error if the heap is empty.  Check our assumption that the
    // heap is large enough to accommodate all outstanding traffic.  If wrong,
    // the address space will have to grow.
    //
    let checkHeap <- mkAssertionStrPvtChecker("icn-packet-payload.bsv: Heap is empty!",
                                              ASSERT_ERROR);

    (* fire_when_enabled, no_implicit_conditions *)
    rule assertHeapNotEmpty (True);
        checkHeap(mem.heapNotEmpty);
    endrule


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

