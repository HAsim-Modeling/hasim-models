import hasim_common::*;
import soft_connections::*;

interface PORT_SEND#(type t);
    method Action enq(t data);
endinterface

interface PORT_RECEIVE#(type t);
    method ActionValue#(t) pop;
endinterface

interface PORT_CREDIT_SEND#(type t, numeric type bandwidth, numeric type logCredit);
    method Bool canSend();
    method Action reserve();
    method Action enq(t data);
    method Action enqReserve(t data);
    method Action done();
    method Action send(t data);
endinterface

interface PORT_CREDIT_RECEIVE#(type t, numeric type bandwidth, numeric type logCredit);
    method ActionValue#(t) pop();
    method Action done(Bit#(logCredit) consumerCredit);
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

module [HASIM_MODULE] mkPortCreditSend#(String str)
    (PORT_CREDIT_SEND#(dataT, bandwidth, logCredit))
    provisos(Transmittable#(dataT),
             Transmittable#(Bit#(logCredit)),
             Transmittable#(Bit#(logBandwidth)),
             Add#(TLog#(TAdd#(1, bandwidth)), 0, logBandwidth),
             Bits#(dataT, dataSz));

    Connection_Send#(dataT)   dataConnection <- mkConnection_Send(str + ":data");
    Connection_Send#(Bit#(logBandwidth)) dataCountFifo <- mkConnection_Send(str + ":dataConnection");
    Connection_Receive#(Bit#(logCredit)) creditsConnection <- mkConnection_Receive(str + ":credits");

    FIFOF#(dataT)                   dataFifo <- mkSizedFIFOF(valueOf(bandwidth));
    Reg#(Bit#(logCredit))            credits <- mkRegU;
    Reg#(Bit#(logBandwidth))       dataCount <- mkReg(0);
    PulseWire                         doneEn <- mkPulseWire;
    PulseWire                      reserveEn <- mkPulseWire;

    Reg#(Bool)                   initialized <- mkReg(False);

    Reg#(Bool)                         start <- mkReg(False);

    let _canSend = dataCount != fromInteger(valueOf(bandwidth)) && credits != 0;

    rule sendData(initialized);
        dataFifo.deq;
        dataConnection.send(dataFifo.first);
    endrule

    rule receiveCredits(initialized && !start);
        credits <= creditsConnection.receive;
        creditsConnection.deq;
        start <= True;
    endrule

    rule cycle;
        if(doneEn)
        begin
            dataCountFifo.send(reserveEn? dataCount + 1: dataCount);
            dataCount <= 0;
        end
        else if(reserveEn)
            dataCount <= dataCount + 1;
    endrule

    // A hack to allow consumer's done to be called in the beginning of the cycle
    rule initialize(!initialized);
        initialized <= True;
        dataCountFifo.send(0);
    endrule

    method Bool canSend() if(initialized && start);
        return _canSend;
    endmethod

    method Action reserve() if(initialized && start && _canSend);
        credits <= credits - 1;
        reserveEn.send;
    endmethod

    method Action enq(dataT data) if(initialized && start && _canSend);
        credits <= credits - 1;
        reserveEn.send;
        dataFifo.enq(data);
    endmethod

    // Unwritten guard: dataCount < enqCount
    method Action enqReserve(dataT data) if(initialized && start);
        dataFifo.enq(data);
    endmethod

    // Unwritten guard: currDataCount = currEnqCount
    method Action done() if(initialized && start);
        start <= False;
        doneEn.send;
    endmethod

    method Action send(dataT data) if(initialized && start && _canSend);
        credits <= credits - 1;
        reserveEn.send;
        dataFifo.enq(data);
        start <= False;
        doneEn.send;
    endmethod
endmodule

module [HASIM_MODULE] mkPortCreditReceive#(String str)
    (PORT_CREDIT_RECEIVE#(dataT, bandwidth, logCredit))
    provisos(Transmittable#(dataT),
             Transmittable#(Bit#(logCredit)),
             Transmittable#(Bit#(logBandwidth)),
             Add#(TLog#(TAdd#(1, bandwidth)), 0, logBandwidth),
             Bits#(dataT, dataSz));

    Connection_Receive#(dataT) dataFifo <- mkConnection_Receive(str + ":data");
    Connection_Receive#(Bit#(logBandwidth)) dataCountFifo <- mkConnection_Receive(str + ":dataCount");
    Connection_Send#(Bit#(logCredit)) creditsFifo <- mkConnection_Send(str + ":credits");

    Reg#(Bit#(logBandwidth))       dataCount <- mkReg(0);
    PulseWire                          popEn <- mkPulseWire;
    PulseWire                         doneEn <- mkPulseWire;

    let _canReceive = dataCount != dataCountFifo.receive;

    rule cycle;
        if(doneEn)
            dataCount <= 0;
        else if(popEn)
            dataCount <= dataCount + 1;
    endrule

    method Bool canReceive() if(dataFifo.notEmpty || dataCountFifo.notEmpty);
        if(dataCountFifo.notEmpty)
            return dataCount != dataCountFifo.receive;
        else
            return True;
    endmethod

    method ActionValue#(dataT) pop();
        dataFifo.deq;
        popEn.send;
        return dataFifo.receive;
    endmethod

    // Unwritten guard: dataCountFifo.first == currDataCount
    method Action done(Bit#(logCredit) _credits);
        doneEn.send;
        dataCountFifo.deq;
        creditsFifo.send(_credits);
    endmethod
endmodule
