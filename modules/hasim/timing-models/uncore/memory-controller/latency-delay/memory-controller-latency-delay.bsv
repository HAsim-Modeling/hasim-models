import Vector::*;
import FIFOF::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"

typedef struct
{
    LINE_ADDRESS physicalAddress;
    Bool isStore;
    MEM_OPAQUE opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_REQ deriving (Eq, Bits);

typedef struct
{
    LINE_ADDRESS physicalAddress;
    MEM_OPAQUE   opaque;
    CPU_INSTANCE_ID destination;
}
MEM_CTRL_RSP deriving (Eq, Bits);


function CORE_MEMORY_RSP initMemRsp(LINE_ADDRESS addr, MEM_OPAQUE op);

    return CORE_MEMORY_RSP
    {
        physicalAddress: addr,
        opaque: op
    };

endfunction

function MEM_CTRL_RSP initMemCtrlRsp(MEM_CTRL_REQ req);

    return MEM_CTRL_RSP
    {
        physicalAddress: req.physicalAddress,
        opaque: req.opaque,
        destination: req.destination
    };

endfunction

`define LANE_MEM_REQ 0
`define LANE_MEM_RSP 1

module [HASIM_MODULE] mkMemoryController();

    TIMEP_DEBUG_FILE debugLog <- mkTIMEPDebugFile("interconnect_memory_controller.out");

    // Interfaces to/from interconnect network.
    // Note: in the future these may be multiplexed if we
    // are simulating multiple memory controllers.

    PORT_SEND#(OCN_MSG)        enqToOCN      <- mkPortSend("memctrl_to_ocn_enq");
    PORT_RECV#(OCN_MSG)        enqFromOCN    <- mkPortRecv("ocn_to_memctrl_enq", 1);
    PORT_SEND#(VC_CREDIT_INFO) creditToOCN   <- mkPortSend("memctrl_to_ocn_credit");
    PORT_RECV#(VC_CREDIT_INFO) creditFromOCN <- mkPortRecv("ocn_to_memctrl_credit", 1);

    Vector#(NUM_LANES, Vector#(VCS_PER_LANE, FIFOF#(OCN_FLIT))) qs <- replicateM(replicateM(mkUGSizedFIFOF(16)));
    Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) needLoadRsp <- mkReg(replicate(replicate(False)));
    Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) notFulls <- mkReg(replicate(replicate(False)));
    
    // Coordinate between the pipeline stages.
    MULTIPLEX_CONTROLLER#(1)  runCtrl <- mkMultiplexController(Vector::nil);
    DEPENDENCE_CONTROLLER#(1) stage2Ctrl <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(1) stage3Ctrl <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(1) stage4Ctrl <- mkDependenceController();
    Reg#(Bool) initialized <- mkReg(False);
    
    rule initialize (!initialized && runCtrl.running);
        stage2Ctrl.ctrl.setMaxRunningInstance(1);
        stage3Ctrl.ctrl.setMaxRunningInstance(1);
        stage4Ctrl.ctrl.setMaxRunningInstance(1);
        initialized <= True;
    endrule

    rule stage1_updateCredit (initialized && stage2Ctrl.producerCanStart());
    
        stage2Ctrl.producerStart();
        debugLog.nextModelCycle();

        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_not_fulls = notFulls;
        let m_credits <- creditFromOCN.receive();

        // Update our notion of credits.
        if (m_credits matches tagged Valid .creds)
        begin

            debugLog.record_next_cycle($format("1: Update credits."));
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin

                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin

                    match {.cred, .out_not_full} = creds[ln][vc];
                    new_not_fulls[ln][vc] = out_not_full;

                end

            end

        end
        else
        begin
            debugLog.record_next_cycle($format("1: No credits in."));
        end

        notFulls <= new_not_fulls;
        stage2Ctrl.producerDone();
    
    endrule

    (* conservative_implicit_conditions *)
    rule stage2_enqsOut (stage2Ctrl.consumerCanStart() && stage3Ctrl.producerCanStart());
    
        stage2Ctrl.consumerStart();
        stage3Ctrl.producerStart();


        // Calculate if we're sending load data back from our internal queues.
        Maybe#(OCN_MSG) enq_to_send = tagged Invalid;
        
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                if (qs[ln][vc].notEmpty() && notFulls[`LANE_MEM_RSP][vc])
                begin

                    // If a queue is not empty, and the output queue has room, 
                    // send one of them onwards (static priority).
                    
                    //enq_to_send = tagged Valid tuple3(fromInteger(ln), fromInteger(vc), qs[ln][vc].first()); 
                    enq_to_send = tagged Valid tuple3(`LANE_MEM_RSP, fromInteger(vc), qs[ln][vc].first()); // Always use the response lane to avoid deadlocks.
                    qs[ln][vc].deq();

                end

            end

        end
        
        if (enq_to_send matches tagged Valid {.ln, .vc, .msg})
        begin

            case (msg) matches 
                tagged FLIT_HEAD .info:
                begin
                
                    debugLog.record($format("2: Initiating response to destination: %0d", info.dst));
                end
                tagged FLIT_BODY .info:
                begin
                    debugLog.record($format("2: Sending body flit."));
                end
            endcase
        end
        else
        begin
            debugLog.record($format("2: No send."));
        end

        // Send out the result.
        enqToOCN.send(enq_to_send);

        // Continue in the next stage.
        stage3Ctrl.producerDone();
        
    endrule
    
    rule stage3_enqsIn (stage3Ctrl.consumerCanStart() && stage4Ctrl.producerCanStart());
        
        stage3Ctrl.consumerStart();
        stage4Ctrl.producerStart();
        
        // Read incoming ports.
        let m_enq     <- enqFromOCN.receive();

        // Vectors to update our state.
        Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool)) new_need_load_rsp = needLoadRsp;
        // See if there are any incoming enqueues.
        if (m_enq matches tagged Valid {.lane, .vc, .req})
        begin

            case (req) matches 

                tagged FLIT_HEAD .info:
                begin
                    if (info.isStore)
                    begin
                        // No response to stores currently.
                        new_need_load_rsp[lane][vc] = False;
                        debugLog.record($format("3: Received store from Station %0d", info.src));
                    end
                    else
                    begin
                        qs[lane][vc].enq(tagged FLIT_HEAD {src: ?, dst: info.src, isStore: False});
                        debugLog.record($format("3: Received load from Station %0d", info.src));
                        new_need_load_rsp[lane][vc] = True;
                    end
                end

                tagged FLIT_BODY .info:
                begin

                    if (needLoadRsp[lane][vc])
                    begin

                        // Body flits just get sent back for loads, currently.
                        debugLog.record($format("3: Finishing load rsp."));
                        qs[lane][vc].enq(tagged FLIT_BODY info);

                    end
                    else
                    begin
                        debugLog.record($format("3: Dropping store body."));
                    end

                end

            endcase

        end
        else
        begin
            debugLog.record($format("3: No req."));
        end
        
        needLoadRsp <= new_need_load_rsp;

        // Go on to stage 4.
        stage4Ctrl.producerDone();

    endrule

    (* conservative_implicit_conditions *)
    rule stage4_creditsOut (stage4Ctrl.consumerCanStart());
    
        stage4Ctrl.consumerStart();

        // Calculate our credits for the OCN.
        VC_CREDIT_INFO creds = newVector();

        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
            
            creds[ln] = newVector();

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                let have_credit = !qs[ln][vc].notEmpty(); // XXX capacity - occupancy > round-trip latency.
                let not_full = !qs[ln][vc].notEmpty(); //qs[ln][vc].notFull();
                creds[ln][vc] = tuple2(have_credit, not_full);
            end
        
        end
        debugLog.record($format("4: Send output credits."));
        
        creditToOCN.send(tagged Valid creds);
        
        // Go back to stage 1.
        stage4Ctrl.consumerDone();
        stage3Ctrl.consumerDone();
        stage2Ctrl.consumerDone();
        
    endrule

endmodule
