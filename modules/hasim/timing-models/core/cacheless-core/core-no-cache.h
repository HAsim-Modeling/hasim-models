#include "asim/provides/hasim_pipeline.h"

// A cacheless core

typedef class HASIM_CORE_CLASS* HASIM_CORE;

class HASIM_CORE_CLASS
{
    private:
        HASIM_PIPELINE pipe;
    public:
    
        HASIM_CORE_CLASS() : pipe(new HASIM_PIPELINE_CLASS()) {}
        ~HASIM_CORE_CLASS() { delete pipe; }

        void Init() { pipe->Init(); }
        void MapContexts(int num_ctxts) { pipe->MapContexts(num_ctxts); }
};
