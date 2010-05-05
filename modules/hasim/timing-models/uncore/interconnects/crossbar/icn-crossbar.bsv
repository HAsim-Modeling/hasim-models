// TODO: Evaluate, possibly switch to BlockRAM.

import Vector::*;
import FIFO::*;
import FIFOF::*;

// TEMPORARY:
`include "asim/dict/RINGID.bsh"

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
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqToCores      <- mkPortSend_Multiplexed("CoreMemInQ_enq");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, OCN_MSG)        enqFromCores    <- mkPortRecv_Multiplexed("CoreMemOutQ_enq", 1);
    PORT_SEND_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditToCores   <- mkPortSend_Multiplexed("CoreMemInQ_credit");
    PORT_RECV_MULTIPLEXED#(NUM_CPUS, VC_CREDIT_INFO) creditFromCores <- mkPortRecv_Multiplexed("CoreMemOutQ_credit", 1);

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

    PORT_SEND_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), OCN_MSG) enqToLocal   <- mkPortSend_Multiplexed_Split(enqToCores, enqToMemCtrl, `MEM_CTRL_LOCATION);
    PORT_RECV_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), OCN_MSG) enqFromLocal <- mkPortRecv_Multiplexed_Join(enqFromCores, enqFromMemCtrl, `MEM_CTRL_LOCATION);
    
    PORT_SEND_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), VC_CREDIT_INFO) creditToLocal   <- mkPortSend_Multiplexed_Split(creditToCores, creditToMemCtrl, `MEM_CTRL_LOCATION);
    PORT_RECV_MULTIPLEXED#(TAdd#(NUM_CPUS, 1), VC_CREDIT_INFO) creditFromLocal <- mkPortRecv_Multiplexed_Join(creditFromCores, creditFromMemCtrl, `MEM_CTRL_LOCATION);

    // NOTE: The module does not use a local controller, as it has two sets of ports,
    // one set is NUM_CPUS multiplexed, the other is NUM_STATIONS multiplexed.
    // This local controller variant handles that.

    Vector#(2, INSTANCE_CONTROL_IN#(NUM_CPUS)) inports = newVector();
    inports[0] = enqFromCores.ctrl;
    inports[1] = creditFromCores.ctrl;

    Vector#(0, INSTANCE_CONTROL_IN#(NUM_STATIONS)) inportsR = newVector();

    LOCAL_CONTROLLER#(NUM_STATIONS) localCtrl <- mkLocalControllerPlusN(inports, inportsR);

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
        debugLog.nextModelCycle(iid);
        
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
        
        debugLog.record_next_cycle(iid, $format("1: Update input credits"));
        
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
        
        debugLog.record(iid, $format("2: VCA Begin."));
        for (Integer ln = 0; ln < valueof(NUM_LANES); ln = ln + 1)
        begin

            for (Integer vc = 0; vc < valueof(VCS_PER_LANE); vc = vc + 1)
            begin

                if (virtualChannels[ln][vc].notEmpty())
                begin
                
                    if (virtualChannels[ln][vc].first() matches tagged FLIT_HEAD .info &&& outputQs[info.dst][ln][vc].notFull() &&& !isValid(new_holding_bus) &&& !bus_used)
                    begin

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
                    else if (new_holding_bus matches tagged Valid .winner)
                    begin
                    
                        if (winner.inputPort == iid &&& winner.lane == fromInteger(ln) &&& winner.virtualChannel == fromInteger(vc))
                        begin

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

                    end
                    else
                    begin
                    
                        $display("ERROR: Bus: Body flit at head of virtual channel without holding bus.");
                        $finish(1);

                    end
                end
            end
        end
        
        if (iid == fromInteger(valueof(NUM_STATIONS) - 1))
        begin
            busUsed <= False;
        end
        else
        begin
            busUsed <= bus_used;
        end
        
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
        
    endrule

endmodule


module [HASIM_MODULE] mkPortRecv_Multiplexed_Join#(PORT_RECV_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG) p1, PORT_RECV#(t_MSG) p2, Bit#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) insertion_point)
    // interface:
        (PORT_RECV_MULTIPLEXED#(TAdd#(t_NUM_INSTANCES, 1), t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, TLog#(TAdd#(t_NUM_INSTANCES, 1))),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));


    COUNTER#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) cur <- mkLCounter(0);
    
    Bool canDeq = (cur.value() == insertion_point) ? !p2.ctrl.empty() : !p1.ctrl.empty();
    
    interface INSTANCE_CONTROL_IN ctrl;


        method Bool empty() = !canDeq;
        method Bool balanced() = True;
        method Bool light() = False;
        
        method Maybe#(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1))) nextReadyInstance();
        
            return canDeq ? tagged Valid cur.value() : tagged Invalid;
        
        endmethod
        
        method Action setMaxRunningInstance(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) iid);
        
            noAction;
            // NOTE: We assume that this will be called directly on the child ports.
            // This could be a bad assumption.
        endmethod
        
    endinterface

    method ActionValue#(Maybe#(t_MSG)) receive(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) dummy) if (canDeq);
        
        if (cur.value() == fromInteger(valueof(t_NUM_INSTANCES))) // Note: purposefully leave off -1, to take into account the extra port.
        begin
            cur.setC(0);
        end
        else
        begin
            cur.up();
        end
        
        if (cur.value() == insertion_point)
        begin
            let msg <- p2.receive();
            return msg;
        end
        else
        begin
            let msg <- p1.receive(truncate(dummy));
            return msg;
        end
        
    endmethod

endmodule

module [HASIM_MODULE] mkPortSend_Multiplexed_Split#(PORT_SEND_MULTIPLEXED#(t_NUM_INSTANCES, t_MSG) p1, PORT_SEND#(t_MSG) p2, Bit#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) split_point)
    // interface:
        (PORT_SEND_MULTIPLEXED#(TAdd#(t_NUM_INSTANCES, 1), t_MSG))
    provisos
        (Bits#(t_MSG, t_MSG_SZ),
         Add#(TLog#(t_NUM_INSTANCES), t_TMP, TLog#(TAdd#(t_NUM_INSTANCES, 1))),
         Transmittable#(Tuple2#(INSTANCE_ID#(t_NUM_INSTANCES), Maybe#(t_MSG))));


    COUNTER#(TLog#(TAdd#(t_NUM_INSTANCES, 1))) cur <- mkLCounter(0);
    
    Bool canEnq = (cur.value() == split_point) ? !p2.ctrl.full() : !p1.ctrl.full();
    
    interface INSTANCE_CONTROL_OUT ctrl;

        method Bool full() = !canEnq;
        method Bool balanced() = True;
        method Bool heavy() = False;

    endinterface

    method Action send(INSTANCE_ID#(TAdd#(t_NUM_INSTANCES, 1)) dummy, Maybe#(t_MSG) msg) if (canEnq);
        
        if (cur.value() == fromInteger(valueof(t_NUM_INSTANCES))) // Note: purposefully leave off -1, to take into account the extra port.
        begin
            cur.setC(0);
        end
        else
        begin
            cur.up();
        end
        
        if (cur.value() == split_point)
        begin
            p2.send(msg);
        end
        else
        begin
            p1.send(truncate(dummy), msg);
        end
        
    endmethod

endmodule

typedef enum
{
    LCN_Idle,               // Waiting for a command
    LCN_Running,            // Running, allowing slip
    LCN_Synchronizing,      // Running, attempting to synchronize
    LCN_Stepping            // Run one modelCC
}
LCN_STATE
    deriving (Eq, Bits);

module [HASIM_MODULE] mkLocalControllerPlusN

    // parameters:
    #(
    Vector#(t_NUM_INPORTS,  INSTANCE_CONTROL_IN#(t_NUM_INSTANCES))  inctrls, 
    Vector#(t_NUM_INPORTS_N,  INSTANCE_CONTROL_IN#(t_NUM_INSTANCES_PLUS_N))  inctrlsN
    )
    // interface:
        (LOCAL_CONTROLLER#(t_NUM_INSTANCES_PLUS_N))
    provisos
        (Add#(t_N, t_NUM_INSTANCES, t_NUM_INSTANCES_PLUS_N));

    Reg#(LCN_STATE) state <- mkReg(LCN_Idle);
  
    // Counter of active instances. 
    // We start at -1, so we assume at least one instance is active.
    COUNTER#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)) maxActiveInstance <- mkLCounter(~0);
    // Vector of running instances
    Reg#(Vector#(t_NUM_INSTANCES_PLUS_N, Bool)) instanceRunning <- mkReg(replicate(False));
    // Track stepping state.
    Reg#(Vector#(t_NUM_INSTANCES_PLUS_N, Bool)) instanceStepped <- mkReg(replicate(False));

    // Signalled DONE to the software?
    Reg#(Bool) signalDone <- mkReg(False);

    Vector#(t_NUM_INSTANCES_PLUS_N, PulseWire)    startCycleW <- replicateM(mkPulseWire());
    Vector#(t_NUM_INSTANCES_PLUS_N, PulseWire)      endCycleW <- replicateM(mkPulseWire());
    Vector#(t_NUM_INSTANCES_PLUS_N, Wire#(Bit#(8))) pathDoneW <- replicateM(mkWire());
    
    
    // For now this local controller just goes round-robin over the instances.
    // This is guaranteed to be correct accross multiple modules.
    // The performance of this could be improved, but the interaction with time-multiplexed
    // ports needs to be worked out.
    
    COUNTER#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)) nextInstance <- mkLCounter(0);
    
    Connection_Chain#(CONTROLLER_MSG) link_controllers <- mkConnection_Chain(`RINGID_CONTROLLER_MESSAGES);

    function Bool allTrue(Vector#(k, Bool) v);
        return foldr(\&& , True, v);
    endfunction

    // Can this module read from this Port? Purposely ignore the non plus-N ports
    function Bool canReadFromN(INSTANCE_CONTROL_IN#(t_NUM_INSTANCES_PLUS_N) ctrl_in);
        return case (state)
                   LCN_Running:        return !ctrl_in.empty();
                   LCN_Stepping:       return !ctrl_in.empty();
                   LCN_Synchronizing:  return !ctrl_in.light();
                   default:            return False;
               endcase;
    endfunction

    // This function will determine the next instance in a non-round-robin manner when we're ready
    // to go that route. Currently this is unused.

    function Bool instanceReady(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid);
        
        Bool canRead  = True;

        // Can we read/write all of the plus N ports? Disregard normal ports for this.
        for (Integer x = 0; x < valueOf(t_NUM_INPORTS_N); x = x + 1)
            canRead = canRead && canReadFromN(inctrlsN[x]);

        // An instance is ready to go only if it's been enabled.
        return !instanceRunning[iid] && canRead;

    endfunction

    function INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) nextReadyInstance();
        
        INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) res = 0;

        for (Integer x = 0; x < valueof(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            res = instanceReady(fromInteger(x)) ? fromInteger(x) : res;
        end
        
        return res;
    
    endfunction

    function Bool someInstanceReady();
        
        Bool res = False;

        for (Integer x = 0; x < valueof(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            res = instanceReady(fromInteger(x)) || res;
        end
        
        return res;
    
    endfunction



    function Bool balanced();
        Bool res = True;
        
        // Are the plus N ports all balanced? Disregard normal ports for this.
        for (Integer x = 0; x < valueOf(t_NUM_INPORTS_N); x = x + 1)
        begin
            res = res && inctrlsN[x].balanced();
        end

        return res;

    endfunction



    // ====================================================================
    //
    // Process controller commands and send responses.
    //
    // ====================================================================

    FIFO#(Bool) checkBalanceQ <- mkFIFO();
    FIFO#(CONTROLLER_MSG) newCtrlMsgQ <- mkFIFO1();
    
    rule checkBalance (True);
        checkBalanceQ.deq();
        link_controllers.sendToNext(tagged COM_SyncQuery balanced());
    endrule

    rule newControlMsg (True);
        let cmd = newCtrlMsgQ.first();
        newCtrlMsgQ.deq();
        
        link_controllers.sendToNext(cmd);
    endrule

    (* descending_urgency = "checkBalance, newControlMsg, nextCommand" *)
    rule nextCommand (state != LCN_Stepping);
        let newcmd <- link_controllers.recvFromPrev();
        Maybe#(CONTROLLER_MSG) outcmd = tagged Valid newcmd;

        case (newcmd) matches
            tagged COM_RunProgram:
            begin
    
                for (Integer x = 0; x < valueof(t_NUM_INPORTS); x = x + 1)
                begin
                
                    // We know this truncation is safe since the button has only been pushed k times, not k+N.
                    inctrls[x].setMaxRunningInstance(truncateNP(maxActiveInstance.value()));

                end

                for (Integer x = 0; x < valueof(t_NUM_INPORTS_N); x = x + 1)
                begin
                
                    inctrlsN[x].setMaxRunningInstance(maxActiveInstance.value() + fromInteger(valueof(t_N)));

                end
                
                maxActiveInstance.setC(maxActiveInstance.value() + fromInteger(valueof(t_N)));

                state <= LCN_Running;

            end

            tagged COM_Synchronize:
            begin
                state <= LCN_Synchronizing;
            end

            tagged COM_SyncQuery .all_balanced:
            begin
                // The COM_SyncQuery state will remain True if all controllers
                // are balanced.  If a previous controller is unbalanced then
                // just forward the unbalanced state.  If we need to check
                // the state of this controller then queue the request.
                //
                // The Bluespec scheduler throws an error about being unable
                // to break a cycle if we attempt to check balance here
                // because it can't break the connection between setting
                // state, startModelCycle and changes to balance() while
                // the model runs.
                if (all_balanced)
                begin
                    outcmd = tagged Invalid;
                    checkBalanceQ.enq(?);
                end
            end

            tagged COM_Step:
            begin
                state <= LCN_Stepping;
                Vector#(t_NUM_INSTANCES_PLUS_N, Bool) instance_stepped = newVector();
                for (Integer x = 0; x < valueOf(t_NUM_INSTANCES_PLUS_N); x = x + 1)
                begin
                   instance_stepped[x] = False;
                end
                instanceStepped <= instance_stepped;
            end

            tagged COM_Pause .send_response:
            begin
                state <= LCN_Idle;
            end

            // TODO: should this be COM_EnableInstance??
            tagged COM_EnableContext .iid:
            begin
                maxActiveInstance.up();
            end

            // TODO: should this be COM_DisableInstance??
            tagged COM_DisableContext .iid:
            begin
                maxActiveInstance.down();
            end
        endcase

        // Forward command around the ring
        if (outcmd matches tagged Valid .cmd)
        begin
            link_controllers.sendToNext(cmd);
        end
    endrule

    rule updateRunning (True);
    
        Vector#(t_NUM_INSTANCES_PLUS_N, Bool) new_running = instanceRunning;

        for (Integer x = 0; x < valueOf(t_NUM_INSTANCES_PLUS_N); x = x + 1)
        begin
            if (instanceRunning[x])
                new_running[x] =  !endCycleW[x];
            else if (startCycleW[x])
                new_running[x] = !endCycleW[x];
            else
                noAction;
        end
        
        instanceRunning <= new_running;
    
    endrule

    //
    // updateStateForStepping --
    //     State update associated with startModelCycle, encoded in a rule
    //     and controlled by a wire in order to set scheduling priority.
    //
    Wire#(Maybe#(Bit#(INSTANCE_ID_BITS#(t_NUM_INSTANCES_PLUS_N)))) newModelCycleStarted <- mkDWire(tagged Invalid);

    (* descending_urgency = "updateStateForStepping, nextCommand" *)
    rule updateStateForStepping (state == LCN_Stepping &&&
                                 newModelCycleStarted matches tagged Valid .iid);
        instanceStepped[iid] <= True;
        if (iid == maxActiveInstance.value())
            state <= LCN_Idle;
    endrule

    //
    // ******** Methods *******


    method ActionValue#(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N)) startModelCycle() if ((state != LCN_Idle) && instanceReady(nextInstance.value()));

        let next_iid = nextInstance.value();

        if (state == LCN_Stepping)
        begin
            newModelCycleStarted <= tagged Valid next_iid;
        end
        
        // checkInstanceSanity();
        
        startCycleW[next_iid].send();
        
        if (next_iid >= maxActiveInstance.value())
        begin
            nextInstance.setC(0);
        end
        else
        begin
            nextInstance.up();
        end

        return next_iid;

    endmethod

    method Action endModelCycle(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid, Bit#(8) path);
    
        endCycleW[iid].send();
        pathDoneW[iid] <= path; // Put the path into the waveform.
    
    endmethod

    method Action instanceDone(INSTANCE_ID#(t_NUM_INSTANCES_PLUS_N) iid, Bool pf);
        // XXX this should be per-instance.  For now only allowed to fire once.
        if (! signalDone)
        begin
            newCtrlMsgQ.enq(tagged LC_DoneRunning pf);
            signalDone <= True;
        end
    endmethod
    
endmodule
