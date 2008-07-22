import hasim_common::*;
import soft_connections::*;

interface PORT_SEND#(type t);
    method Action enq(t data);
endinterface

interface PORT_RECEIVE#(type t);
    method ActionValue#(t) pop;
endinterface

interface PORT_BANDWIDTH_CREDIT_SEND#(type t, numeric type bandwidth, numeric type credit);
    method Bool canSend();
    method Action enq(t data);
    method Action done();
endinterface

interface PORT_BANDWIDTH_CREDIT_RECEIVE#(type t, numeric type bandwidth, numeric type credit);
    method Action deq();
    method t first();
    method ActionValue#(t) pop();
    method Action done(Bit#(TLog#(TAdd#(1,credit))) consumerCredit);
    method Bool canReceive();
endinterface

module [HASIM_MODULE] mkPortSend#(String str, t init)
    (PORT_SEND#(t))
    provisos(Transmittable#(t),
             Bits#(t, tSz));

    Connection_Send#(t) dataConnection <- mkConnection_Send(str);

    Reg#(Bool) initialized <- mkReg(False);

    rule initialize(!initialized);
        initialized <= True;
        dataConnection.send(init);
    endrule

    method Action enq(t data) if(initialized);
        dataConnection.send(data);
    endmethod
endmodule

module [HASIM_MODULE] mkPortReceive#(String str)
    (PORT_RECEIVE#(t))
    provisos(Transmittable#(t),
             Bits#(t, tSz));

    Connection_Receive#(t) dataConnection <- mkConnection_Receive(str);

    method ActionValue#(t) pop();
        let data = dataConnection.receive();
        dataConnection.deq();
        return data;
    endmethod
endmodule

module [HASIM_MODULE] mkPortBandwidthCreditSend#(String str)
    (PORT_BANDWIDTH_CREDIT_SEND#(t, bandwidthT, creditT))
    provisos(Transmittable#(Maybe#(t)),
             Transmittable#(Bit#(logCredit)),
             Add#(bandwidthT, 1, bandwidth1),
             Add#(creditT, 1, credit1),
             Log#(bandwidth1, logBandwidth),
             Log#(credit1, logCredit),
             Bits#(t, tSz));

    Reg#(Bit#(logBandwidth)) producerBandwidth <- mkRegU();
    Reg#(Bit#(logCredit))       consumerCredit <- mkRegU();

    Connection_Receive#(Bit#(logCredit)) consumerCreditPort <- mkConnection_Receive(strConcat(str, ":consumerCredit"));
    Connection_Send#(Maybe#(t))              dataConnection <- mkConnection_Send(strConcat(str, ":dataConnection"));

    Reg#(Bool)  startCycle <- mkReg(False);

    rule startingCycle(!startCycle);
        producerBandwidth <= fromInteger(valueOf(bandwidthT));
        consumerCredit <= consumerCreditPort.receive();
        consumerCreditPort.deq();
        startCycle <= True;
    endrule

    Bool _canSend = consumerCredit != 0 && producerBandwidth != 0;

    method Bool canSend() if(startCycle);
        return _canSend;
    endmethod

    method Action enq(t data) if(startCycle && _canSend);
        dataConnection.send(tagged Valid data);
        producerBandwidth <= producerBandwidth - 1;
        consumerCredit <= consumerCredit - 1;
    endmethod

    method Action done() if(startCycle);
        dataConnection.send(tagged Invalid);
        startCycle <= False;
    endmethod
endmodule

module [HASIM_MODULE] mkPortBandwidthCreditReceive#(String str, Bit#(logCredit) initCredit)
    (PORT_BANDWIDTH_CREDIT_RECEIVE#(t, bandwidthT, creditT))
    provisos(Transmittable#(Maybe#(t)),
             Transmittable#(Bit#(logCredit)),
             Add#(bandwidthT, 1, bandwidth1),
             Add#(creditT, 1, credit1),
             Log#(bandwidth1, logBandwidth),
             Add#(logCredit, 0, TLog#(TAdd#(1, creditT))),
             Log#(credit1, logCredit),
             Bits#(t, tSz));

    Reg#(Bit#(logBandwidth)) consumerBandwidth <- mkRegU();

    Connection_Send#(Bit#(logCredit)) consumerCreditPort <- mkConnection_Send(strConcat(str, ":consumerCredit"));
    Connection_Receive#(Maybe#(t))        dataConnection <- mkConnection_Receive(strConcat(str, ":dataConnection"));

    FIFOF#(Maybe#(t))                             buffer <- mkSizedFIFOF(valueOf(bandwidthT));

    Reg#(Bool) initialized <- mkReg(False);

    rule initialize(!initialized);
        buffer.enq(tagged Invalid);
        initialized <= True;
    endrule

    rule fillBuffer(initialized);
        buffer.enq(dataConnection.receive());
        dataConnection.deq();
    endrule

    Bool _canReceive = consumerBandwidth != 0 && isValid(buffer.first());

    method Action deq() if(initialized && _canReceive);
        consumerBandwidth <= consumerBandwidth - 1;
        buffer.deq();
    endmethod

    method t first() if(initialized && _canReceive);
        return validValue(buffer.first());
    endmethod

    method ActionValue#(t) pop() if(initialized && _canReceive);
        consumerBandwidth <= consumerBandwidth - 1;
        buffer.deq();
        return validValue(buffer.first());
    endmethod

    method Action done(Bit#(logCredit) consumerCredit) if(initialized && !_canReceive);
        consumerCreditPort.send(consumerCredit);
        consumerBandwidth <= fromInteger(valueOf(bandwidthT));
        buffer.deq();
    endmethod

    method Bool canReceive() if(initialized);
        return _canReceive;
    endmethod
endmodule
