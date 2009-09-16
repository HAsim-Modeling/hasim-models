#include "asim/provides/hasim_core.h"

// A single-core chip

typedef class HASIM_CHIP_CLASS* HASIM_CHIP;

class HASIM_CHIP_CLASS
{
    private:
        HASIM_CORE core;
    public:
    
        HASIM_CHIP_CLASS() : core(new HASIM_CORE_CLASS()) {}
        ~HASIM_CHIP_CLASS() { delete core; }

        void Init() { core->Init(); }
        void MapContexts(int num_ctxts) { core->MapContexts(num_ctxts); }
};
