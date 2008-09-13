//
// Copyright (C) 2008 Intel Corporation
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

typedef enum { SB_STATE_DEALLOC, SB_STATE_SEARCH } SB_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkStoreBuffer ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_storebuffer.out");

    Port_Receive#(Tuple2#(TOKEN,CacheInput)) inQ      <- mkPort_Receive("storebuffer_req", 0);
    Port_Receive#(TOKEN)                     deallocQ <- mkPort_Receive("storebuffer_dealloc", 1);
    Port_Send#(Tuple2#(TOKEN,SB_RESPONSE))   outQ     <- mkPort_Send("storebuffer_resp");

    Reg#(SB_STATE) state <- mkReg(SB_STATE_DEALLOC);

    //Local Controller
    Vector#(2, Port_Control) inports  = newVector();
    Vector#(1, Port_Control) outports = newVector();
    inports[0]  = inQ.ctrl;
    inports[1]  = deallocQ.ctrl;
    outports[0] = outQ.ctrl;
    LocalController local_ctrl <- mkLocalController(inports, outports);

    Vector#(4, Reg#(Maybe#(ISA_ADDRESS))) vec <- replicateM(mkReg(Invalid));
    Reg#(Bit#(2)) head <- mkReg(0);
    Reg#(Bit#(2)) tail <- mkReg(0);

    Bool empty = head == tail;
    Bool full  = head == tail + 1;

    rule dealloc (state == SB_STATE_DEALLOC);
        local_ctrl.startModelCC();
        let m <- deallocQ.receive();
        if (m matches tagged Valid .tok)
        begin
            head <= head + 1;
            vec[head] <= Invalid;
            debugLog.record(fshow("DEALLOC ") + fshow(tok));
        end
        state <= SB_STATE_SEARCH;
    endrule

    rule search (state == SB_STATE_SEARCH);
        let m <- inQ.receive();
        case (m) matches
            tagged Invalid:
                outQ.send(Invalid);
            tagged Valid { .tok, .x }:
            case (x) matches
                tagged Data_read_mem_ref { .pc, .a }:
                begin
                    if (elem(Valid(a), readVReg(vec)))
                    begin
                        outQ.send(Valid(tuple2(tok,SB_HIT)));
                        debugLog.record(fshow("LOAD HIT ") + fshow(tok));
                    end
                    else
                    begin
                        outQ.send(Valid(tuple2(tok,SB_MISS)));
                        debugLog.record(fshow("LOAD MISS ") + fshow(tok));
                    end
                end
                tagged Data_write_mem_ref { .pc, .a }:
                begin
                    if (full)
                    begin
                        outQ.send(Valid(tuple2(tok,SB_STALL)));
                        debugLog.record(fshow("SB STORE RETRY (SB FULL!) ") + fshow(tok));
                    end
                    else
                    begin
                        tail <= tail + 1;
                        vec[tail] <= Valid(a);
                        outQ.send(Invalid);
                        debugLog.record(fshow("SB STORE ALLOC ") + fshow(tok));
                    end
                end
            endcase
        endcase
        state <= SB_STATE_DEALLOC;
    endrule
endmodule
