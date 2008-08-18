import fpga_components::*;

`include "asim/provides/funcp_base_types.bsh"

typedef 12 BTB_IDX_SZ;

typedef Bit#(2) BTB_OFFSET;
typedef Bit#(TSub#(`FUNCP_ISA_V_ADDR_SIZE,TAdd#(BTB_IDX_SZ,2))) BTB_TAG;
typedef Bit#(BTB_IDX_SZ) BTB_INDEX;

typedef enum { BP_STATE_TRAIN, BP_STATE_BTB, BP_STATE_BP } BP_STATE deriving (Bits, Eq);

function BTB_INDEX getIndex (ISA_ADDRESS a);
    Tuple3#(BTB_TAG,BTB_INDEX,BTB_OFFSET) tup = unpack(a);
    match { .tag, .idx, .off } = tup;
    // assert off = 0b00
    return idx;
endfunction
function BTB_TAG getTag (ISA_ADDRESS a);
    Tuple3#(BTB_TAG,BTB_INDEX,BTB_OFFSET) tup = unpack(a);
    match { .tag, .idx, .off } = tup;
    // assert off = 0b00
    return tag;
endfunction

module [HASIM_MODULE] mkBranchPredictor ();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("pipe_bp.out");

    Port_Receive#(ISA_ADDRESS)     pcQ <- mkPort_Receive("bp_req", 0);
    Port_Send#(ISA_ADDRESS)    nextpcQ <- mkPort_Send("bp_reply_pc");
    Port_Send#(BRANCH_ATTR)      predQ <- mkPort_Send("bp_reply_pred");
    Port_Receive#(Tuple2#(ISA_ADDRESS,BRANCH_ATTR)) trainQ <- mkPort_Receive("bp_train", 1);

    BRAM#(BTB_INDEX, Maybe#(Tuple2#(BTB_TAG,ISA_ADDRESS))) btb <- mkBRAMInitialized(Invalid);

    Reg#(ISA_ADDRESS) pc <- mkReg(0);
    Reg#(BP_STATE) state <- mkReg(BP_STATE_TRAIN);

    //Local Controller
    Vector#(2, Port_Control) inports  = newVector();
    Vector#(2, Port_Control) outports = newVector();
    inports[0]  = pcQ.ctrl;
    inports[1]  = trainQ.ctrl;
    outports[0] = nextpcQ.ctrl;
    outports[1] = predQ.ctrl;

    LocalController local_ctrl <- mkLocalController(inports, outports);

    rule train (state == BP_STATE_TRAIN);
        local_ctrl.startModelCC();
        let x <- trainQ.receive();
        if (x matches tagged Valid { .pc, .brattr })
        begin
            if (brattr matches tagged BranchTaken .tgt) begin
                btb.write(getIndex(pc), Valid(tuple2(getTag(pc),tgt)));
                debugLog.record($format("TRAIN: %h -> %h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
        end
        state <= BP_STATE_BTB;
    endrule
    rule btbreq (state == BP_STATE_BTB);
        let mpc <- pcQ.receive();
        if (mpc matches tagged Valid .a)
        begin
            btb.readReq(getIndex(a));
            pc <= a;
            state <= BP_STATE_BP;
        end
        else
        begin
            nextpcQ.send(Invalid);
            predQ.send(Invalid);
            state <= BP_STATE_TRAIN;
        end
    endrule
    rule bp (state == BP_STATE_BP);
        let x <- btb.readRsp();
        if (x matches tagged Valid { .tag, .tgt } &&& getTag(pc) == tag)
        begin
            if (tgt <= pc)
            begin
                nextpcQ.send(Valid(tgt));
                predQ.send(Valid(BranchTaken(tgt)));
                debugLog.record($format("PRED: %h -> taken; tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
            else
            begin
                nextpcQ.send(Valid(pc + 4));
                predQ.send(Valid(BranchNotTaken(tgt)));
                debugLog.record($format("PRED: %h -> not-taken; taken-tgt=%h", pc, tgt) + $format(" (idx:%h, tag:%h)", getIndex(pc), getTag(pc)));
            end
        end
        else
        begin
            nextpcQ.send(Valid(pc + 4));
            predQ.send(Valid(NotBranch));
            debugLog.record($format("PRED: %h -> not-branch", pc));
        end
        state <= BP_STATE_TRAIN;
    endrule
endmodule
