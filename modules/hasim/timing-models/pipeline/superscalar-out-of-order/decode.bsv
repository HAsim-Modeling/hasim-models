import FIFOF::*;
import Vector::*;
import FShow::*;

`include "hasim_common.bsh"
`include "hasim_modellib.bsh"
`include "hasim_isa.bsh"
`include "soft_connections.bsh"
`include "funcp_interface.bsh"

`include "pipeline_base_types.bsh"

// This should not exist, but the ROB seems to be making assumptions about the number of credits.
// Changing the ROB to interact with the fetchBuffer directly was unsuccesful.
// So this will remain until a better workaround can be found.

module [HASIM_MODULE] mkDecode();
    PORT_FIFO_RECEIVE#(FETCH_BUNDLE, `FETCH_NUM, LOG_FETCH_CREDITS)                fetchBuffer <- mkPortFifoReceive("fetch", True, `FETCH_CREDITS);
    PORT_CREDIT_SEND#(FETCH_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)               decodePort <- mkPortCreditSend("decode");

   rule passThrough (True);
        if(fetchBuffer.canReceive() && decodePort.canSend())
        begin
            decodePort.enq(fetchBuffer.first());
            fetchBuffer.deq();
        end
        else
        begin
            fetchBuffer.done;
            decodePort.done;
        end
    endrule

endmodule
