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

import FIFO::*;
import FIFOF::*;
import SpecialFIFOs::*;
import Vector::*;

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/hasim_isa.bsh"

`include "awb/provides/model_structures_base_types.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/pipeline_base_types.bsh"


//
// BTB read ports
//
`define PORT_UPD  0
`define PORT_PRED 1


//
// mkBranchTargetPredAlg --
//     Branch target buffer algorithm.  Manage an n-way table of predictions.
//     Since the BTB is just a predictor it is not necessary to store full tags
//     or to support all offsets.  The table can be significantly smaller and
//     still cover most cases using a small tag (causing some incorrect aliases)
//     and an instruction offset instead of a target address.
//
module mkBranchTargetPredAlg
    // interface:
    (BRANCH_TARGET_BUFFER_ALG)
    provisos (NumAlias#(n_WAYS, `BTB_NUM_WAYS),
              NumAlias#(t_SET_IDX_SZ, `BTB_SET_IDX_BITS),
              NumAlias#(t_TAG_SZ, `BTB_TAG_BITS),
              NumAlias#(t_OFFSET_SZ, `BTB_OFFSET_BITS),

              Alias#(t_TAG, Bit#(t_TAG_SZ)),
              Alias#(t_OFFSET, Bit#(t_OFFSET_SZ)),

              Alias#(t_SET_IDX, Bit#(t_SET_IDX_SZ)),
              Bits#(ISA_ADDRESS, t_ISA_ADDRESS_SZ),

              // Address excluding the ignored offset
              Alias#(t_ADDR_IDX, FUNCP_PC_IDX_PART),
              Bits#(t_ADDR_IDX, t_ADDR_IDX_SZ),

              // Pseudo-LRU for ways within a set
              Alias#(t_PLRU, Vector#(n_WAYS, Bool)),

              // Type of one way:  maybe indicates whether a prediction exists.
              // The t_TAG indicates whether the entry corresponds to a given
              // address index.  The t_OFFSET is the offset from entry's address
              // to the target address in t_ADDR_IDX space.
              Alias#(t_PRED_WAY, Maybe#(Tuple2#(t_TAG, t_OFFSET))),

              // Full set
              Alias#(t_PRED, Vector#(n_WAYS, t_PRED_WAY)));

    DEBUG_FILE debugLog <- mkDebugFile("alg_btb_multi_entry.out");

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(MAX_NUM_CPUS, 2, t_SET_IDX, t_PRED)
        btbPool <- mkMemoryMultiRead_Multiplexed(
                       mkBRAMBufferedPseudoMultiReadInitialized(False,
                                                                replicate(tagged Invalid)));

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(MAX_NUM_CPUS, 2, t_SET_IDX, t_PLRU)
        plruPool <- mkMemoryMultiRead_Multiplexed(
                       mkBRAMBufferedPseudoMultiReadInitialized(False,
                                                                replicate(False)));

    // Lock an entire CPU IID during an update.  This isn't a problem, since
    // HAsim multiplexed timing models only allow one operation per IID to
    // be live at a time.
    MULTIPLEXED_REG#(MAX_NUM_CPUS, Bool) lock <- mkMultiplexedReg(False);

    FIFO#(Tuple2#(CPU_INSTANCE_ID, t_ADDR_IDX)) newReqQ <- mkFIFO();
    FIFO#(Tuple4#(CPU_INSTANCE_ID, t_ADDR_IDX, t_SET_IDX, t_TAG)) reqQ <- mkFIFO();
    FIFO#(Maybe#(t_ADDR_IDX)) rspQ <- mkSizedFIFO(valueof(NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS));

    FIFO#(Tuple3#(CPU_INSTANCE_ID,
                  t_ADDR_IDX,
                  Maybe#(t_OFFSET))) newUpdQ <- mkFIFO();
    FIFO#(Tuple4#(CPU_INSTANCE_ID,
                  t_SET_IDX,
                  t_TAG,
                  Maybe#(t_OFFSET))) upd0Q <- mkFIFO();
    FIFO#(Tuple3#(CPU_INSTANCE_ID,
                  t_SET_IDX,
                  t_PRED)) upd1Q <- mkFIFO();

    function Bool isTrue(Bool b) = b;

    //
    // cpuIsLockedForUpdate --
    //     A CPU is locked iff the table has been read but not
    //     yet updated.
    //
    function Bool cpuIsLockedForUpdate(CPU_INSTANCE_ID iid);
        let k = lock.getReg(iid);
        return k;
    endfunction

    function Action lockCPU(CPU_INSTANCE_ID iid);
    action
        lock.getReg(iid) <= True;
    endaction
    endfunction

    function Action unlockCPU(CPU_INSTANCE_ID iid);
    action
        lock.getReg(iid) <= False;
    endaction
    endfunction


    //
    // Split address into set index and tag.
    //
    function Tuple2#(t_SET_IDX, t_TAG) setAndTag(t_ADDR_IDX addr);
        Tuple2#(t_TAG, t_SET_IDX) st = unpack(resize(hashBits(addr)));
        match {.tag, .set} = st;
        return tuple2(set, tag);
    endfunction

    //
    // Match way within a set?
    //
    function Bool matchWay(t_TAG tag, t_PRED_WAY way);
        if (way matches tagged Valid .w &&& tpl_1(w) == tag)
            return True;
        else
            return False;
    endfunction

    //
    // Update pseudo-LRU access vector.  We do the updates during the upd()
    // phase because the entry is being written already.
    //
    function t_PLRU updatePLRU(t_PLRU plru, wIdx);
        if (valueOf(n_WAYS) <= 1)
        begin
            // Only one way.  Always fill into the same way.
            return replicate(False);
        end
        else
        begin
            plru[wIdx] = True;
            // Are all the pseudo LRU bits set?  If so, clear all but the new one.
            if (all(isTrue, plru))
            begin
                plru = replicate(False);
                plru[wIdx] = True;
            end
            return plru;
        end
    endfunction


    //
    // predReq --
    //     New prediction request.  Block if an update is in progress for the
    //     same CPU.
    //
    rule predReq (! cpuIsLockedForUpdate(tpl_1(newReqQ.first())));
        match {.iid, .addr} = newReqQ.first();
        newReqQ.deq();

        match {.set, .tag} = setAndTag(addr);
        debugLog.record($format("<%0d>: Lookup pc=0x%h, set=0x%h, tag=0x%h", iid, pcAddAlignmentBits(addr), set, tag));

        btbPool.readPorts[`PORT_PRED].readReq(iid, set);
        plruPool.readPorts[`PORT_PRED].readReq(iid, set);
        reqQ.enq(tuple4(iid, addr, set, tag));
    endrule

    //
    // predRsp --
    //     Seek matching entry in the BTB and generate a response.
    //
    rule predRsp (True);
        match {.iid, .addr, .set, .tag} = reqQ.first();
        reqQ.deq();

        let btb_entry <- btbPool.readPorts[`PORT_PRED].readRsp(iid);
        let plru <- plruPool.readPorts[`PORT_PRED].readRsp(iid);

        if (findIndex(matchWay(tag), btb_entry) matches tagged Valid .w_idx)
        begin
            // Found a hit in the BTB
            let way = validValue(btb_entry[w_idx]);
            match {.w_tag, .w_offset} = way;

            // Compute target from address and offset
            let w_target = addr + signExtend(w_offset);

            let plru_upd = updatePLRU(plru, w_idx);
            plruPool.write(iid, set, plru_upd);

            debugLog.record($format("<%0d>: Hit way=%0d, addr=0x%h, tgt=0x%h, old_lru=0x%h, new_lru=0x%h", iid, w_idx, pcAddAlignmentBits(addr), pcAddAlignmentBits(w_target), pack(plru), pack(plru_upd)));
            rspQ.enq(tagged Valid w_target);
        end
        else
        begin
            debugLog.record($format("<%0d>: Miss", iid));
            rspQ.enq(tagged Invalid);
        end
    endrule


    //
    // startUpd --
    //     Update prediction table.  Only one update per CPU is permitted.
    //
    rule startUpd (! cpuIsLockedForUpdate(tpl_1(newUpdQ.first())));
        match {.iid, .addr, .m_actual} = newUpdQ.first();
        newUpdQ.deq();

        lockCPU(iid);

        // Read current table value
        match {.set, .tag} = setAndTag(addr);

        if (m_actual matches tagged Valid .actual)
            debugLog.record($format("<%0d>: Update pc=0x%h, set=0x%h, tag=0x%h, offset=0x%h", iid, pcAddAlignmentBits(addr), set, tag, actual));
        else
            debugLog.record($format("<%0d>: Inval pc=0x%h, set=0x%h, tag=0x%h", iid, pcAddAlignmentBits(addr), set, tag));

        btbPool.readPorts[`PORT_UPD].readReq(iid, set);
        plruPool.readPorts[`PORT_UPD].readReq(iid, set);

        upd0Q.enq(tuple4(iid, set, tag, m_actual));
    endrule

    //
    // pickWay --
    //     Pick a victim way in the BTB set and update it.
    //
    rule pickWay (True);
        match {.iid, .set, .tag, .m_actual} = upd0Q.first();
        upd0Q.deq();

        let btb_entry <- btbPool.readPorts[`PORT_UPD].readRsp(iid);
        let plru <- plruPool.readPorts[`PORT_UPD].readRsp(iid);

        let w_idx = ?;
        if (findIndex(matchWay(tag), btb_entry) matches tagged Valid .w)
        begin
            // Entry is already in the BTB.  Update the existing way.
            w_idx = w;
            debugLog.record($format("<%0d>: Update way=%0d, was=0x%h", iid, w_idx, tpl_2(validValue(btb_entry[w_idx]))));
        end
        else
        begin
            // Entry not yet in the BTB.  Pick either an invalid way or the LRU way.
            if (findElem(tagged Invalid, btb_entry) matches tagged Valid .i)
            begin
                w_idx = i;
                debugLog.record($format("<%0d>: Write unused way=%0d", iid, w_idx));
            end
            else
            begin
                w_idx = validValue(findElem(False, plru));
                debugLog.record($format("<%0d>: Write LRU way=%0d, plru=0x%h", iid, w_idx, pack(plru)));
            end
        end

        // Now that a way is picked, update the entry.  Was the instruction
        // actually a branch to some target?
        if (m_actual matches tagged Valid .offset)
        begin
            // Yes.  Update target.
            btb_entry[w_idx] = tagged Valid tuple2(tag, offset);
        end
        else
        begin
            // Not a branch.  Perhaps an alias?  Remove it.
            btb_entry[w_idx] = tagged Invalid;
        end

        upd1Q.enq(tuple3(iid, set, btb_entry));
    endrule

    //
    // finishUpd --
    //     Write the updated BTB entry back to memory.  Separated from previous
    //     rule for FPGA timing.
    //
    rule finishUpd (True);
        match {.iid, .set, .btb_entry} = upd1Q.first();
        upd1Q.deq();

        btbPool.write(iid, set, btb_entry);
        unlockCPU(iid);
    endrule


    method Action getPredReq(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        newReqQ.enq(tuple2(iid, pcAddrWithoutAlignmentBits(addr)));
    endmethod

    method ActionValue#(Maybe#(ISA_ADDRESS)) getPredRsp(CPU_INSTANCE_ID iid);
        rspQ.deq();
        if (rspQ.first() matches tagged Valid .addr)
            return tagged Valid pcAddAlignmentBits(addr);
        else
            return tagged Invalid;
    endmethod

    method Action upd(CPU_INSTANCE_ID iid,
                      ISA_ADDRESS addr,
                      Bool wasCorrect,
                      Maybe#(ISA_ADDRESS) m_actual);

        // Convert the address to the indexed address space (without low
        // alignment bits.)
        let addr_idx = pcAddrWithoutAlignmentBits(addr);

        // Compute the offset from address to the target.
        Maybe#(t_OFFSET) m_offset = tagged Invalid;
        if (m_actual matches tagged Valid .actual)
        begin
            let tgt_addr_idx = pcAddrWithoutAlignmentBits(actual);
            let full_offset = tgt_addr_idx - addr_idx;
            t_OFFSET tbl_offset = truncate(full_offset);

            // Can the offset be represented in the BTB entry size?  If not
            // then treat the request as an invalidate.
            if ((addr_idx + signExtend(tbl_offset)) == tgt_addr_idx)
            begin
                // It fits
                m_offset = tagged Valid tbl_offset;
                debugLog.record($format("<%0d>: Update req addr=0x%h, tgt=0x%h, offset=0x%h", iid, addr, actual, tbl_offset));
            end
            else
            begin
                debugLog.record($format("<%0d>: Update req addr=0x%h, tgt=0x%h, offset too large (0x%h)", iid, addr, actual, full_offset));
            end
        end
   
        newUpdQ.enq(tuple3(iid, addr_idx, m_offset));
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        noAction;
    endmethod

endmodule
