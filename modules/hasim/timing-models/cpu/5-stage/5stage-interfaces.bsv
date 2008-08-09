`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"

`include "asim/provides/hasim_isa.bsh"

typedef struct
{
  TOKEN token;
  Bool updatePredictor;        // True when branch predictor tables should be updated
  Bool correctPrediction;      // True when prediction was correct
  ISA_ADDRESS instrPC;         // PC of the current instruction
  ISA_ADDRESS newPC;           // PC of next instruction to fetch
}
  EXE_TO_FET_MSG
    deriving (Eq, Bits);
