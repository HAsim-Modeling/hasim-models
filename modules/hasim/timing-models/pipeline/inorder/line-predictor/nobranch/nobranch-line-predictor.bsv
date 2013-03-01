
import Vector::*;
import FIFO::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/funcp_base_types.bsh"
`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/pipeline_base_types.bsh"

// No Branch Line Predictor
//  Always predicts next pc as pc + 4.

// Possible ways to end a model cycle:
// Path 1: A pc was requested. Send pc + 4 back.
// Path 2: No pc was requested, so emit a bubble.

module [HASIM_MODULE] mkLinePredictor ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_lp.out");


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ISA_ADDRESS)                 pcFromFet <- mkPortRecv_Multiplexed("Fet_to_LP_pc", 0);
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ISA_ADDRESS)                 predToFet <- mkPortSend_Multiplexed("LP_to_Fet_newpc");


    // ****** Local Controller ******

    Vector#(1, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls  = newVector();
    Vector#(1, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    inctrls[0]  = pcFromFet.ctrl;
    outctrls[0] = predToFet.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("Line Predictor", inctrls, outctrls);


    // ****** Rules ******
    
    // stage1 - Predict pc+4 as next pc.

    rule stage1 (True);
    
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        debugLog.nextModelCycle(cpu_iid);

        // Check for a pc 
        let m_pc <- pcFromFet.receive(cpu_iid);

        if (m_pc matches tagged Valid .pc)
        begin

            // Predict pc + 4
            let pred = pc + 4;
            debugLog.record_next_cycle(cpu_iid, $format("LPRED: %h -> %h", pc, pred));
            predToFet.send(cpu_iid, tagged Valid pred);

            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);
        end
        else
        begin

            // No prediction request. Propogate the bubble.
            debugLog.record_next_cycle(cpu_iid, $format("BUBBLE"));
            predToFet.send(cpu_iid, tagged Invalid);
            
            // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);
        end

    endrule

endmodule
