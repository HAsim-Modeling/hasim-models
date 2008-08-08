
typedef enum { SB_STATE_DEALLOC, SB_STATE_SEARCH } SB_STATE deriving (Bits, Eq);

module [HASIM_MODULE] mkStoreBuffer ();

    ModelDebugFile debug <- mkModelDebugFile("pipe_storebuffer.out");

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
        debug.startModelCC();
        let m <- deallocQ.receive();
        if (m matches tagged Valid .tok)
        begin
            head <= head + 1;
            vec[head] <= Invalid;
            debug <= fshow("DEALLOC ") + fshow(tok);
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
                        debug <= fshow("LOAD HIT ") + fshow(tok);
                    end
                    else
                    begin
                        outQ.send(Valid(tuple2(tok,SB_MISS)));
                        debug <= fshow("LOAD MISS ") + fshow(tok);
                    end
                end
                tagged Data_write_mem_ref { .pc, .a }:
                begin
                    if (full)
                    begin
                        outQ.send(Valid(tuple2(tok,SB_STALL)));
                        debug <= fshow("SB STORE RETRY (SB FULL!) ") + fshow(tok);
                    end
                    else
                    begin
                        tail <= tail + 1;
                        vec[tail] <= Valid(a);
                        outQ.send(Invalid);
                        debug <= fshow("SB STORE ALLOC ") + fshow(tok);
                    end
                end
            endcase
        endcase
        state <= SB_STATE_DEALLOC;
    endrule
endmodule
