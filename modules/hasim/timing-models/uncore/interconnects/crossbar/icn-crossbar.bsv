// TODO: Evaluate, possibly switch to BlockRAM.

import Vector::*;
import FIFO::*;
import FIFOF::*;

// ******* Project Imports *******

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/common_services.bsh"


// ******* Timing Model Imports *******

`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"
`include "asim/provides/hasim_memory_controller.bsh"


typedef 3 NUM_PORTS;
typedef Bit#(TLog#(NUM_PORTS)) PORT_IDX;

PORT_IDX portEast   = 0;
PORT_IDX portWest   = 1;
PORT_IDX portLocal  = 2;

Integer numPorts = 3;

function String portShow(PORT_IDX p);

    return case (p)
        0: "east";
        1: "west";
        2: "local";
        default: "UNKNOWN";
    endcase;

endfunction

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) VC_STATE#(parameter type t_DATA);


typedef struct
{
    LANE_IDX lane;
    VC_IDX   virtualChannel;
    STATION_IID inputPort;
    STATION_IID outputPort;
}
WINNER_INFO 
    deriving (Eq, Bits);
    
`define MEM_CTRL_LOCATION 0

module [HASIM_MODULE] mkInterconnect
    // interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(NUM_STATIONS) debugLog <- mkTIMEPDebugFile_Multiplexed("interconnect_bus.out");

    // ******** Ports *******

    // Queues to/from cores
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, OCN_MSG)        enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VC_CREDIT_INFO) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

    // Queues to/from memory controller
    // Note: non-multiplexed as there is only one memory controller.
    PORT_RECV#(OCN_MSG)        enqFromMemCtrl    <- mkPortRecv("memctrl_to_ocn_enq", 1);
    PORT_SEND#(OCN_MSG)        enqToMemCtrl      <- mkPortSend("ocn_to_memctrl_enq");
    PORT_RECV#(VC_CREDIT_INFO) creditFromMemCtrl <- mkPortRecv("memctrl_to_ocn_credit", 1);
    PORT_SEND#(VC_CREDIT_INFO) creditToMemCtrl   <- mkPortSend("ocn_to_memctrl_credit");

    // Links to/from neighboring routers
    // Note: These ports actually connect together (they're the same port).
    // This is the main technique which makes this module work.
    // The token reordering keeps things in the correct order.
    // Note: We need an extra instance here for the memory controller's router.
    // Note: We have to control these ourselves since they have more instances than normal.

    PORT_SEND_MULTIPLEXED#(TAdd#(MAX_NUM_CPUS, 1), OCN_MSG) enqToLocal   <- mkPortSend_Multiplexed_Split(enqToCores, enqToMemCtrl, `MEM_CTRL_LOCATION);
    PORT_RECV_MULTIPLEXED#(TAdd#(MAX_NUM_CPUS, 1), OCN_MSG) enqFromLocal <- mkPortRecv_Multiplexed_Join(enqFromCores, enqFromMemCtrl, `MEM_CTRL_LOCATION);
    
    PORT_SEND_MULTIPLEXED#(TAdd#(MAX_NUM_CPUS, 1), VC_CREDIT_INFO) creditToLocal   <- mkPortSend_Multiplexed_Split(creditToCores, creditToMemCtrl, `MEM_CTRL_LOCATION);
    PORT_RECV_MULTIPLEXED#(TAdd#(MAX_NUM_CPUS, 1), VC_CREDIT_INFO) creditFromLocal <- mkPortRecv_Multiplexed_Join(creditFromCores, creditFromMemCtrl, `MEM_CTRL_LOCATION);

    // NOTE: The module does not use a local controller, as it has two sets of ports,
    // one set is MAX_NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    inports[0] = enqFromCores.ctrl;
    inports[1] = creditFromCores.ctrl;

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_STATIONS)) inportsR = newVector();
    inportsR[0] = enqFromLocal.ctrl;
    inportsR[1] = creditFromLocal.ctrl;

    Vector#(2, INSTANCE_CONTROL_OUT#(NUM_STATIONS)) outportsR = newVector();
    outportsR[0] = enqToLocal.ctrl;
    outportsR[1] = creditToLocal.ctrl;

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <- mkLocalControllerPlusN(inports, inportsR, outportsR);

    // This module simulates by reading/writing it's multiplexed ports once for every CPU,
    // and reading/writing the (non-multiplexed) memory controller port once.

    MULTIPLEXED#(NUM_STATIONS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, FIFOF#(OCN_FLIT))))    virtualChannelsPool    <- mkMultiplexed(replicateM(replicateM(mkUGSizedFIFOF(4))));
    Vector#(NUM_STATIONS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, FIFOF#(OCN_FLIT))))         outputQs               <- replicateM(replicateM(replicateM(mkUGSizedFIFOF(2))));

    MULTIPLEXED_REG#(NUM_STATIONS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCreditsPool  <- mkMultiplexedReg(replicate(replicate(False)));
    MULTIPLEXED_REG#(NUM_STATIONS, Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputNotFullsPool <- mkMultiplexedReg(replicate(replicate(False)));
 
    Reg#(Maybe#(WINNER_INFO))    holdingBus   <- mkReg(tagged Invalid);
    Reg#(Bool               )    busUsed      <- mkReg(False);

    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage2Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage3Ctrl <- mkStageControllerVoid();
    STAGE_CONTROLLER_VOID#(NUM_STATIONS) stage4Ctrl <- mkStageControllerVoid();

    // ******* Rules *******

    rule stage1_creditInEnqOut (True);
    
        // Get the next IID to simulate.
        let iid <- localCtrl.startModelCycle();
        
        if (iid == 0)
        begin
            // Reset this variable at the beginning of a model cycle.
            busUsed <= False;
        end
        
        // Get our state from the pools.
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputCredits  = outputCreditsPool.getReg(iid);
        Reg#(Vector#(NUM_LANES, Vector#(VCS_PER_LANE, Bool))) outputNotFulls = outputNotFullsPool.getReg(iid);
        
        // Update our notions of our this output's credits.
        VC_STATE#(Bool) new_credits = outputCredits;
        VC_STATE#(Bool) new_not_fulls = outputNotFulls;
        

        // Get the credits for this local user.
        let m_credits <- creditFromLocal.receive(iid);

        if (m_credits matches tagged Valid .vcinfo)
        begin

            // New credit info has arrived.
            for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
            begin

                for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
                begin

                    match {.cred, .not_full} = vcinfo[ln][vc];
                    new_credits[ln][vc] = cred;
                    new_not_fulls[ln][vc] = not_full;

                end

            end

        end
  
        Maybe#(OCN_MSG) m_enq = tagged Invalid;
  
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin
                if (outputQs[iid][ln][vc].notEmpty())
                begin

                    m_enq = tagged Valid tuple3(fromInteger(ln), fromInteger(vc), outputQs[iid][ln][vc].first());

                end
            end

        end
        
        enqToLocal.send(iid, m_enq);

        if (m_enq matches tagged Valid {.ln, .vc, .msg})
        begin
            outputQs[iid][ln][vc].deq();
        end
        
        debugLog.record(iid, $format("1: Update input credits"));
        
        // Do the actual update.
        outputCredits <= new_credits;
        outputNotFulls <= new_not_fulls;
        
        // Move on to the next stage.
        stage2Ctrl.ready(iid);
    
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage2_multiplexVCs (True);
        
        // Get the info from the previous stage.
        let iid <- stage2Ctrl.nextReadyInstance();
        
        // Read our local state from the pools.
        VC_STATE#(FIFOF#(OCN_FLIT)) virtualChannels = virtualChannelsPool[iid];

        Bool bus_used = busUsed;
        Maybe#(WINNER_INFO) new_holding_bus = holdingBus;
        
        debugLog.record(iid, $format("3: SA: Begin."));
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                if (virtualChannels[ln][vc].notEmpty())
                begin
                
                    if (virtualChannels[ln][vc].first() matches tagged FLIT_HEAD .info)
                    begin
                        
                        if (outputQs[info.dst][ln][vc].notFull() &&& !isValid(new_holding_bus) &&& !bus_used)
                        begin
                            debugLog.record(iid, $format("3: SA: Bus grant: lane %0d, vc %0d destination: %0d", ln, vc, info.dst));
                            new_holding_bus = tagged Valid WINNER_INFO 
                                    {
                                        inputPort: info.src,
                                        lane: fromInteger(ln),
                                        virtualChannel: fromInteger(vc), 
                                        outputPort: info.dst
                                    };
                            outputQs[info.dst][ln][vc].enq(virtualChannels[ln][vc].first());
                            bus_used = True;
                            virtualChannels[ln][vc].deq();
                        end
                        else
                        begin
                            debugLog.record(iid, $format("3: SA: Head flit stall."));
                        end

                    end
                    else
                    begin
                        // Body flit at head of queue. We'd better have the bus!
                        if (new_holding_bus matches tagged Valid .winner &&& winner.inputPort == iid &&& winner.lane == fromInteger(ln) &&& winner.virtualChannel == fromInteger(vc))
                        begin

                            debugLog.record(iid, $format("3: SA: Bus grant: lane %0d, vc %0d destination: %0d", ln, vc, winner.outputPort));
                            // assert !bus_used
                            bus_used = True;
                            outputQs[winner.outputPort][ln][vc].enq(virtualChannels[ln][vc].first());
                        
                            if (virtualChannels[ln][vc].first() matches tagged FLIT_BODY .body_info &&& body_info.isTail)
                            begin

                                debugLog.record(iid, $format("3: SA: Detected tail flit. Tearing down routing info."));
                                new_holding_bus = tagged Invalid;

                            end

                            virtualChannels[ln][vc].deq();

                        end
                        else
                        begin

                            $display("ERROR: Bus: Body flit at head of virtual channel without holding bus.");
                            $finish(1);

                        end

                    end
                end
            end
        end
        
        busUsed <= bus_used;
        
        holdingBus <= new_holding_bus;

        stage3Ctrl.ready(iid);

    endrule

    rule stage3_enqs (True);

        // Get the current IID from the previous stage.    
        let iid <- stage3Ctrl.nextReadyInstance();
       
        // Read our local state from the pools.
        VC_STATE#(FIFOF#(OCN_FLIT))      virtualChannels = virtualChannelsPool[iid];

        // Deal with input enqueues from each direction.
        let m_enq <- enqFromLocal.receive(iid);
        if (m_enq matches tagged Valid {.ln, .vc, .flit})
        begin

            let new_flit = flit;
            if (flit matches tagged FLIT_HEAD .info)
            begin
                let new_info = info;
                new_info.src = iid;
                if (iid != `MEM_CTRL_LOCATION)
                begin
                    // For now we assume all core traffic goes to the mem controller.
                    new_info.dst = `MEM_CTRL_LOCATION;
                end
                debugLog.record(iid, $format("5: BW: MESSAGE ENTER: src %0d, dst %0d, isStore %0d, lane %0d, virtual channel %0d", new_info.src, new_info.dst, pack(new_info.isStore), ln, vc));
                new_flit = tagged FLIT_HEAD new_info;
            end
            else
            begin
                debugLog.record(iid, $format("5: BW: Enqueuing into lane %0d, virtual channel %0d", ln, vc));
            end
            virtualChannels[ln][vc].enq(new_flit);

        end
        else
        begin

            debugLog.record(iid, $format("5: BW: No enqueue."));

        end

        stage4Ctrl.ready(iid);

    endrule

    (* conservative_implicit_conditions, descending_urgency="stage4_creditsOut, stage3_enqs, stage2_multiplexVCs, stage1_creditInEnqOut" *)
    rule stage4_creditsOut (True);
    
        let iid <- stage4Ctrl.nextReadyInstance();

        debugLog.record(iid, $format("6: Calculating output credits."));

        VC_STATE#(FIFOF#(OCN_FLIT)) virtualChannels = virtualChannelsPool[iid];
        
        VC_CREDIT_INFO creds = newVector();
        
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin
        
            creds[ln] = newVector();

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                let have_credit = !virtualChannels[ln][vc].notEmpty(); // XXX capacity - occupancy > round-trip latency.
                let not_full = !virtualChannels[ln][vc].notEmpty(); // virtualChannels[ln][vc].notFull();
                creds[ln][vc] = tuple2(have_credit, not_full);

            end
        
        end
        
        creditToLocal.send(iid, tagged Valid creds);
        
        
        // End of model cycle (path 1)
        localCtrl.endModelCycle(iid, 0);
        debugLog.nextModelCycle(iid);
        
    endrule

endmodule

