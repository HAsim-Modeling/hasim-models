#include "awb/provides/chip_base_types.h"

#ifndef __UNPIPELINED_PIPELINE__
#define __UNPIPELINED_PIPELINE__

typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS
{
  public:
    HASIM_PIPELINE_CLASS() {}
    ~HASIM_PIPELINE_CLASS() {}

    void Init() {}
};

#endif // __UNPIPELINED_PIPELINE__
