
import FIFOF::*;

// DECODE_PRF_SCOREBOARD

// A submodule which tracks valid bits of physical registers. This module has critical physical properties, 
// so we abstract it as a submodule so that any nastiness is buried in here.

interface DECODE_PRF_SCOREBOARD_WB_PORT;

    method Action ready(FUNCP_PHYSICAL_REG_INDEX prf);

endinterface


interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;

    method Action unready(FUNCP_PHYSICAL_REG_INDEX prf);

endinterface


interface DECODE_PRF_SCOREBOARD;

    interface Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wbExe;
    interface Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wbHit;
    interface Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wbMiss;
    
    interface Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_ISSUE_PORT) issue;

    method Bool isReady(FUNCP_PHYSICAL_REG_INDEX prf);

endinterface


//
// mkPRFScoreboardLUTRAM --
//     A compromise PRF scoreboard between one with multiple write ports that
//     support parallel updates of all writebacks and a simple implementation
//     with only one port.
//
//     This version maintains separate LUTRAMs for each source of updates
//     (execute, cache hit, and cache miss).  A PRF is valid if any one of
//     those LUTRAMs says it is valid.
//
//     While committing buffered updates the scoreboard may not be accessed.
//
//     This has significantly better scaling properties than the multi-write-
//     port version below, which clobbers Bluespec as the number of PRFs
//     grows large.
//

typedef 3 DEC_PRF_PORTS;

module [HASIM_MODULE] mkPRFScoreboardLUTRAM (DECODE_PRF_SCOREBOARD);

    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);
    let maxInitReg = numIsaArchRegisters * valueOf(NUM_CONTEXTS);
    
    function Bool initPRFValid(FUNCP_PHYSICAL_REG_INDEX pr);
        return (pr <= fromInteger(maxInitReg - 1));
    endfunction
    
    // Separate PRF valid arrays for each source of register writes.
    Vector#(DEC_PRF_PORTS,
            LUTRAM#(FUNCP_PHYSICAL_REG_INDEX,
                    Bool)) prfValids <- replicateM(mkLUTRAMWith(initPRFValid));

    // FIFOs holding pending updates
    Vector#(DEC_PRF_PORTS,
            Vector#(TSub#(ISA_MAX_DSTS, 1),
                    FIFOF#(Tuple2#(FUNCP_PHYSICAL_REG_INDEX, Bool)))) stalledOps = replicate(newVector());
    
    // Stall flag indicating pending writes to a specific LUTRAM
    Vector#(DEC_PRF_PORTS, Bool) stalled = replicate(False);

    function Bool fifoNotEmpty(FIFOF#(t) f) = f.notEmpty();

    // All pending writes must eventually write the prfValids flags
    for (Integer port = 0; port < valueOf(DEC_PRF_PORTS); port = port + 1)
    begin
        for (Integer i = 0; i < (valueof(ISA_MAX_DSTS)-1); i = i + 1)
        begin
            stalledOps[port][i] <- mkUGLFIFOF();
    
            rule unstall (stalledOps[port][i].notEmpty()); // Make the implicit condition explicit.
                match {.prf, .new_val} = stalledOps[port][i].first();
            
                prfValids[port].upd(prf, new_val);
                stalledOps[port][i].deq();
            endrule
        end

        // Is some write pending for LUTRAM "port"
        stalled[port] = Vector::any(fifoNotEmpty, stalledOps[port]);
    end

    // Is some write pending for any LUTRAM?
    function Bool anyStalled() = (pack(stalled) != 0);

    //
    // Instantiate separate interfaces for all possible sources of writes.
    //

    Vector#(DEC_PRF_PORTS,
            Vector#(ISA_MAX_DSTS,
                    DECODE_PRF_SCOREBOARD_WB_PORT)) wbPorts = replicate(newVector());

    Vector#(ISA_MAX_DSTS,
            DECODE_PRF_SCOREBOARD_ISSUE_PORT) issue_port = newVector();

    // The first register in every group of ISA_MAX_DSTS gets immediate access
    // to a prfValids write port.
    for (Integer port = 0; port < valueOf(DEC_PRF_PORTS); port = port + 1)
    begin
        wbPorts[port][0] =
            interface DECODE_PRF_SCOREBOARD_WB_PORT;
                method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled[port])  = prfValids[port].upd(prf, True);
            endinterface;
    end

    issue_port[0] =
        interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;
            method Action unready(FUNCP_PHYSICAL_REG_INDEX prf) if (!anyStalled);
                for (Integer port = 0; port < valueOf(DEC_PRF_PORTS); port = port + 1)
                begin
                    prfValids[port].upd(prf, False);
                end
            endmethod
        endinterface;

    // Other registers in ISA_MAX_DSTS groups are buffered in stallOps ports
    // and will be written later.  In order to maintain proper order, no new
    // requests will be accepted until these writes complete.
    for (Integer i = 1; i < valueof(ISA_MAX_DSTS); i = i + 1)
    begin
        for (Integer port = 0; port < valueOf(DEC_PRF_PORTS); port = port + 1)
        begin
            wbPorts[port][i] =
                interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled[port])   = stalledOps[port][i-1].enq(tuple2(prf, True));
                endinterface;
        end
        
        issue_port[i] =
            interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;
                method Action unready(FUNCP_PHYSICAL_REG_INDEX prf) if (!anyStalled);
                    for (Integer port = 0; port < valueOf(DEC_PRF_PORTS); port = port + 1)
                    begin
                        stalledOps[port][i-1].enq(tuple2(prf, False));
                    end
                endmethod
            endinterface;
    end
    
    interface wbExe = wbPorts[0];
    interface wbHit = wbPorts[1];
    interface wbMiss = wbPorts[2];
    
    interface issue = issue_port;

    method Bool isReady(FUNCP_PHYSICAL_REG_INDEX prf) if (!anyStalled);
        function Bool regIsValid(LUTRAM#(FUNCP_PHYSICAL_REG_INDEX,
                                         Bool) v) = v.sub(prf);

        return Vector::any(regIsValid, prfValids);
    endmethod

endmodule


// mkPRFScoreboardMultiWrite

// An expensive PRF scoreboard that uses RWires to implement multiple
// write ports. This may turn out to not scale.

module [HASIM_MODULE] mkPRFScoreboardMultiWrite (DECODE_PRF_SCOREBOARD);


    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prfValidInit = newVector();

    let maxInitReg = numIsaArchRegisters * valueOf(NUM_CONTEXTS);

    for (Integer i = 0; i < maxInitReg; i = i + 1)
        prfValidInit[i] = True;

    for (Integer i = maxInitReg; i < numFuncpPhyRegisters; i = i + 1)
        prfValidInit[i] = False;

    Reg#(Vector#(FUNCP_NUM_PHYSICAL_REGS, Bool))      prfValid <- mkReg(prfValidInit);

    // Wires for updating the prfValid bits.
    Vector#(ISA_MAX_DSTS, RWire#(FUNCP_PHYSICAL_REG_INDEX)) wbExeW = newVector();
    Vector#(ISA_MAX_DSTS, RWire#(FUNCP_PHYSICAL_REG_INDEX)) wbMemHitW = newVector();
    Vector#(ISA_MAX_DSTS, RWire#(FUNCP_PHYSICAL_REG_INDEX)) wbMemMissW = newVector();
    Vector#(ISA_MAX_DSTS, RWire#(FUNCP_PHYSICAL_REG_INDEX)) allocW = newVector();

    for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
    begin
    
        wbExeW[x] <- mkRWire();
        wbMemHitW[x] <- mkRWire();
        wbMemMissW[x] <- mkRWire();
        allocW[x] <- mkRWire();
    
    end
        
    // updatePRFValid
    
    // Update the Physical Register valid bits with all writebacks and allocates.
    // This only works because we know the indices are disjoint. This is hairy
    // logic, and an argument against the current system.
    
    // One problem is that it's possible for an allocated register to get
    // readied and un-readied on the SAME cycle. This is because unalloc'd
    // physical registers go on the head of the freelist rather than the tail,
    // which helps caching effects.
    
    // The logic we implement is this. For each Valid bit v:
    //
    // v_new = v_old ? !unready(v) : (ready(v) & !unready(v))
    //
    // IE if the register was ready, it becomes unready if someone is pushing unready.
    // Conversely, if the register was not ready, if someone was pushing ready and not
    // simultaneously pushing unready, then it becomes ready.
    
    rule updatePRFValid (True);
    
        
        Vector#(FUNCP_NUM_PHYSICAL_REGS, Bool) new_valids = prfValid;
        Vector#(FUNCP_NUM_PHYSICAL_REGS, Bool) pushing_unready = replicate(False);
        Vector#(FUNCP_NUM_PHYSICAL_REGS, Bool) pushing_ready = replicate(False);


        for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
        begin

            if (allocW[x].wget() matches tagged Valid .prf)
            begin
                pushing_unready[prf] = True;
            end

            if (wbExeW[x].wget() matches tagged Valid .prf)
            begin
                pushing_ready[prf] = True;
            end

            if (wbMemHitW[x].wget() matches tagged Valid .prf)
            begin
                pushing_ready[prf] = True;
            end

            if (wbMemMissW[x].wget() matches tagged Valid .prf)
            begin
                pushing_ready[prf] = True;
            end

        end

        for (Integer x = 0; x < valueof(FUNCP_NUM_PHYSICAL_REGS); x = x + 1)
        begin
                    
            new_valids[x] = new_valids[x] ?
                             // It is currently ready, so:
                             !pushing_unready[x] : 
                             // It is currently unready, so:
                             (pushing_ready[x] && (!pushing_unready[x]));
        end
    
        prfValid <= new_valids;
    
    endrule

    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_exe_port = newVector();
    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_hit_port = newVector();
    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_miss_port = newVector();

    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_ISSUE_PORT) issue_port = newVector();

    for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
    begin
    
        wb_exe_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf)   = wbExeW[i].wset(prf);

                 endinterface;
        
        wb_hit_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf)   = wbMemHitW[i].wset(prf);

                 endinterface;

        wb_miss_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf)   = wbMemMissW[i].wset(prf);

                 endinterface;
        
        issue_port[i] = interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;
                    
                     method Action unready(FUNCP_PHYSICAL_REG_INDEX prf)   = allocW[i].wset(prf);

                 endinterface;

    end
    
    interface wbExe = wb_exe_port;
    interface wbHit = wb_hit_port;
    interface wbMiss = wb_miss_port;
    
    interface issue = issue_port;

    method Bool isReady(FUNCP_PHYSICAL_REG_INDEX prf) = prfValid[prf];
    

endmodule
