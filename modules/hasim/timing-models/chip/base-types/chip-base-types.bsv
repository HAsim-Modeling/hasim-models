
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/memory_base_types.bsh"

typedef `NUM_CPUS NUM_CPUS;
typedef INSTANCE_ID#(NUM_CPUS) CPU_INSTANCE_ID;

typedef `NUM_LANES NUM_LANES;
typedef INSTANCE_ID#(NUM_LANES) LANE_IDX;

typedef `VCS_PER_LANE VCS_PER_LANE;
typedef INSTANCE_ID#(VCS_PER_LANE) VC_IDX;

typedef Vector#(NUM_LANES, Vector#(VCS_PER_LANE, t_DATA)) VC_INFO#(parameter type t_DATA);

// First Bool is credit, second Bool is NotFull.
typedef VC_INFO#(Tuple2#(Bool, Bool)) VC_CREDIT_INFO;

// There is one more station because of the memory controller.
// Different topologies may handle this differently in the future.
typedef TAdd#(NUM_CPUS, 1) NUM_STATIONS;
typedef INSTANCE_ID#(NUM_STATIONS) STATION_IID;
typedef STATION_IID STATION_ID;


typedef union tagged
{
    struct {STATION_ID src; STATION_ID dst; Bool isStore;} FLIT_HEAD;
    struct {MEM_OPAQUE opaque; Bool isTail;} FLIT_BODY; // Address?
}
OCN_FLIT deriving (Eq, Bits);

typedef Tuple3#(LANE_IDX, VC_IDX, OCN_FLIT) OCN_MSG;
