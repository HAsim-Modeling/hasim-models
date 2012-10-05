//
// Copyright (C) 2011 Intel Corporation
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
//     The t_ADDR_OFFSET_SZ low bits of an address are ignored (assumed zero).
//
module mkBranchTargetPredAlg
    // interface:
    (BRANCH_TARGET_BUFFER_ALG)
    provisos (NumAlias#(t_SET_IDX_SZ, `BTB_SET_IDX_SIZE),
              NumAlias#(n_WAYS, `BTB_NUM_WAYS),

              Alias#(t_SET_IDX, Bit#(t_SET_IDX_SZ)),
              Bits#(ISA_ADDRESS, t_ISA_ADDRESS_SZ),

              // Address excluding the ignored offset
              Alias#(t_ADDR_IDX, FUNCP_PC_IDX_PART),
              Bits#(t_ADDR_IDX, t_ADDR_IDX_SZ),

              // Figure out the size of the tag (the rest of an address beyond
              // the offset and the table index.)
              Add#(t_ADDR_TAG_SZ, t_SET_IDX_SZ, t_ADDR_IDX_SZ),
              Alias#(t_ADDR_TAG, Bit#(t_ADDR_TAG_SZ)),

              // Type of one way:  maybe indicates whether a prediction exists.
              // The t_ADDR_TAG indicates whether the entry corresponds to a given
              // address index.  The t_ADDR_IDX is the target address (with leading
              // 0's removed.)
              Alias#(t_PRED_WAY, Maybe#(Tuple2#(t_ADDR_TAG, t_ADDR_IDX))),
       
              // Pseudo-LRU for ways within a set
              Alias#(t_PLRU, Vector#(n_WAYS, Bool)),

              // Full set
              Alias#(t_PRED, Tuple2#(t_PLRU, Vector#(n_WAYS, t_PRED_WAY))));

    MEMORY_MULTI_READ_IFC_MULTIPLEXED#(NUM_CPUS, 2, t_SET_IDX, t_PRED)
        btbPool <- mkMemoryMultiRead_Multiplexed(
                       mkBRAMBufferedPseudoMultiReadInitialized(False,
                                                                tuple2(replicate(False),
                                                                       replicate(tagged Invalid))));

    // Lock an entire CPU IID during an update.  This isn't a problem, since
    // HAsim multiplexed timing models only allow one operation per IID to
    // be live at a time.
    MULTIPLEXED_REG#(NUM_CPUS, Bool) lock <- mkMultiplexedReg(False);

    FIFO#(Tuple2#(CPU_INSTANCE_ID, t_ADDR_IDX)) newReqQ <- mkFIFO();
    FIFO#(Tuple2#(CPU_INSTANCE_ID, t_ADDR_TAG)) reqQ <- mkFIFO();
    FIFO#(Maybe#(t_ADDR_IDX)) rspQ <- mkSizedFIFO(valueof(NEXT_ADDR_PRED_MIN_RSP_BUFFER_SLOTS));

    FIFO#(Tuple3#(CPU_INSTANCE_ID,
                  t_ADDR_IDX,
                  Maybe#(t_ADDR_IDX))) newUpdQ <- mkFIFO();
    FIFO#(Tuple4#(CPU_INSTANCE_ID,
                  t_SET_IDX,
                  t_ADDR_TAG,
                  Maybe#(t_ADDR_IDX))) updQ <- mkFIFO();

    //
    // cpuIsLockedForUpdate --
    //     A CPU is locked if an if the table has been read but not
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
    function Tuple2#(t_SET_IDX, t_ADDR_TAG) setAndTag(t_ADDR_IDX addr);
        Tuple2#(t_ADDR_TAG, t_SET_IDX) st = unpack(hashBits(addr));
        match {.tag, .set} = st;
        return tuple2(set, tag);
    endfunction

    //
    // Match way within a set?
    //
    function Bool matchWay(t_ADDR_TAG tag, t_PRED_WAY way);
        if (way matches tagged Valid .w &&& tpl_1(w) == tag)
            return True;
        else
            return False;
    endfunction

    //
    // Update pseudo-LRU access vector.  We do the updates during the upd()
    // phase because the entry is being written already.
    //
    function Bool isTrue(Bool b) = b;

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
        btbPool.readPorts[`PORT_PRED].readReq(iid, set);
        reqQ.enq(tuple2(iid, tag));
    endrule

    //
    // predRsp --
    //     Seek matching entry in the BTB and generate a response.
    //
    rule predRsp (True);
        match {.iid, .tag} = reqQ.first();
        reqQ.deq();

        match {.plru, .btb_entry} <- btbPool.readPorts[`PORT_PRED].readRsp(iid);
        if (find(matchWay(tag), btb_entry) matches tagged Valid .way)
        begin
            // Found a hit in the BTB
            match {.w_tag, .w_target} = validValue(way);
            rspQ.enq(tagged Valid w_target);
        end
        else
        begin
            rspQ.enq(tagged Invalid);
        end
    endrule


    //
    // startUpd --
    //     Update prediction table.  Only one update per CPU is permitted.
    //
    rule startUpd (! cpuIsLockedForUpdate(tpl_1(newUpdQ.first())));
        match {.iid, .addr, .actual} = newUpdQ.first();
        newUpdQ.deq();

        lockCPU(iid);

        // Read current table value
        match {.set, .tag} = setAndTag(addr);
        btbPool.readPorts[`PORT_UPD].readReq(iid, set);

        updQ.enq(tuple4(iid, set, tag, actual));
    endrule

    //
    // finishUpd --
    //     Pick a victim way in the BTB set and update it.
    //
    rule finishUpd (True);
        match {.iid, .set, .tag, .actual} = updQ.first();
        updQ.deq();

        unlockCPU(iid);

        match {.plru, .btb_entry} <- btbPool.readPorts[`PORT_UPD].readRsp(iid);
        let w_idx = ?;
        if (findIndex(matchWay(tag), btb_entry) matches tagged Valid .w)
        begin
            // Entry is already in the BTB.  Update the existing way.
            w_idx = w;
        end
        else
        begin
            // Entry not yet in the BTB.  Pick a way.
            w_idx = validValue(findElem(False, plru));
        end

        // Now that a way is picked, update the entry.  Was the instruction
        // actually a branch to some target?
        if (actual matches tagged Valid .tgt)
        begin
            // Yes.  Update target.
            btb_entry[w_idx] = tagged Valid tuple2(tag, tgt);
            plru = updatePLRU(plru, w_idx);
        end
        else
        begin
            // Not a branch.  Perhaps an alias?  Remove it.
            btb_entry[w_idx] = tagged Invalid;
            plru[w_idx] = False;
        end

        btbPool.write(iid, set, tuple2(plru, btb_entry));
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
                      Maybe#(ISA_ADDRESS) actual);
        let i_actual = isValid(actual) ?
            tagged Valid pcAddrWithoutAlignmentBits(validValue(actual)) :
            tagged Invalid;

        newUpdQ.enq(tuple3(iid, pcAddrWithoutAlignmentBits(addr), i_actual));
    endmethod

    method Action abort(CPU_INSTANCE_ID iid, ISA_ADDRESS addr);
        noAction;
    endmethod

endmodule
