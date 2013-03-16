#include "awb/provides/chip_base_types.h"

#ifndef __UNPIPELINED_PIPELINE_WITH_CACHE__
#define __UNPIPELINED_PIPELINE_WITH_CACHE__

typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS
{
  public:
    HASIM_PIPELINE_CLASS() {}
    ~HASIM_PIPELINE_CLASS() {}

    void Init() {}
};

#endif // __UNPIPELINED_PIPELINE_WITH_CACHE__
