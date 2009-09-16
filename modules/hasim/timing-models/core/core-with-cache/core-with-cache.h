#include "asim/provides/hasim_pipeline.h"
#include "asim/provides/hasim_l1_caches.h"

// A core with L1 caches

typedef class HASIM_CORE_CLASS* HASIM_CORE;

class HASIM_CORE_CLASS
{
    private:
        HASIM_PIPELINE pipe;
        HASIM_L1_CACHES l1Caches;
    public:
    
        HASIM_CORE_CLASS() : pipe(new HASIM_PIPELINE_CLASS()), l1Caches(new HASIM_L1_CACHES_CLASS()) {}
        ~HASIM_CORE_CLASS() { delete pipe; delete l1Caches; }

        void Init() { pipe->Init(); l1Caches->Init(); }
        void MapContexts(int num_ctxts) { pipe->MapContexts(num_ctxts); }
};
