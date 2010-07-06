
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

// mkPRFScoreboardLUTRAM

// A PRF scoreboard that LUTRAM with a single write port.
// This means all updates to it conflict. Multiple simultaneous
// updates are buffered and then serially committed.

// While committing buffered updates the scoreboard may not
// be accessed.

// This is likely to scale well, but could result in bad performance.

module [HASIM_MODULE] mkPRFScoreboardLUTRAM (DECODE_PRF_SCOREBOARD);

    // The maximum achitectural register.
    ISA_REG_INDEX maxAR = maxBound;

    // Each context gets maxAR physical registers.
    FUNCP_PHYSICAL_REG_INDEX maxInitPR = (1 + zeroExtend(pack(maxAR))) * fromInteger(valueof(NUM_CONTEXTS)) - 1;

    function Bool initPRFValid(FUNCP_PHYSICAL_REG_INDEX pr);
    
        return (pr <= maxInitPR);
    
    endfunction
    
    LUTRAM#(FUNCP_PHYSICAL_REG_INDEX, Bool) prfValids <- mkLUTRAMWith(initPRFValid);
    Vector#(TSub#(ISA_MAX_DSTS, 1), FIFOF#(Tuple2#(FUNCP_PHYSICAL_REG_INDEX, Bool))) stalledOps = newVector();
    
    Bool stalled = False;

    for (Integer i = 0; i < (valueof(ISA_MAX_DSTS)-1); i = i + 1)
    begin
    
        stalledOps[i] <- mkFIFOF();
    
        rule unstall (stalledOps[i].notEmpty()); // Make the implicit condition explicit.
        
            match {.prf, .new_val} = stalledOps[i].first();
            
            prfValids.upd(prf, new_val);
            stalledOps[i].deq();
        
        endrule
        
        stalled = stalled || stalledOps[i].notEmpty();
    end


    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_exe_port = newVector();
    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_hit_port = newVector();
    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_WB_PORT) wb_miss_port = newVector();

    Vector#(ISA_MAX_DSTS, DECODE_PRF_SCOREBOARD_ISSUE_PORT) issue_port = newVector();

    wb_exe_port[0] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)  = prfValids.upd(prf, True);

                 endinterface;

    wb_hit_port[0] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)  = prfValids.upd(prf, True);

                 endinterface;


    wb_miss_port[0] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)  = prfValids.upd(prf, True);

                 endinterface;


    issue_port[0] = interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;
                    
                     method Action unready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled) = prfValids.upd(prf, False);

                 endinterface;

    for (Integer i = 1; i < valueof(ISA_MAX_DSTS); i = i + 1)
    begin
    
        wb_exe_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)   = stalledOps[i-1].enq(tuple2(prf, True));

                 endinterface;
        
        wb_hit_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)   = stalledOps[i-1].enq(tuple2(prf, True));

                 endinterface;

        wb_miss_port[i] = interface DECODE_PRF_SCOREBOARD_WB_PORT;
                    
                     method Action ready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled)   = stalledOps[i-1].enq(tuple2(prf, True));

                 endinterface;
        
        issue_port[i] = interface DECODE_PRF_SCOREBOARD_ISSUE_PORT;
                    
                     method Action unready(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled) = stalledOps[i-1].enq(tuple2(prf, False));

                 endinterface;

    end
    
    interface wbExe = wb_exe_port;
    interface wbHit = wb_hit_port;
    interface wbMiss = wb_miss_port;
    
    interface issue = issue_port;

    method Bool isReady(FUNCP_PHYSICAL_REG_INDEX prf) if (!stalled) = prfValids.sub(prf);
    

endmodule

// mkPRFScoreboardMultiWrite

// An expensive PRF scoreboard that uses RWires to implement multiple
// write ports. This may turn out to not scale.

module [HASIM_MODULE] mkPRFScoreboardMultiWrite (DECODE_PRF_SCOREBOARD);


    Integer numIsaArchRegisters  = valueof(TExp#(SizeOf#(ISA_REG_INDEX)));
    Integer numFuncpPhyRegisters = valueof(FUNCP_NUM_PHYSICAL_REGS);

    Vector#(FUNCP_NUM_PHYSICAL_REGS,Bool) prfValidInit = newVector();

    // The maximum achitectural register.
    ISA_REG_INDEX maxAR = maxBound;

    // Each context gets maxAR physical registers.
    FUNCP_PHYSICAL_REG_INDEX maxInitPR = (1 + zeroExtend(pack(maxAR))) * fromInteger(valueof(NUM_CONTEXTS)) - 1;

    for (FUNCP_PHYSICAL_REG_INDEX i = 0; i < maxInitPR; i = i + 1)
        prfValidInit[i] = True;

    for (FUNCP_PHYSICAL_REG_INDEX i = maxInitPR; i < fromInteger(numFuncpPhyRegisters); i = i + 1)
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
