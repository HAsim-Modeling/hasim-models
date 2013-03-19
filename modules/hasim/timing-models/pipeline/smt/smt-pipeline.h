#include "asim/provides/chip_base_types.h"
#include "asim/provides/pipeline_base_types.h"
#include "asim/mesg.h"

#include "asim/rrr/client_stub_SMT_PIPELINE.h"

// The threads-per-core switch can set the number of threads per core.
// We must check that threads-per-core x num-cores <= workload_contexts.
// If this is not set the default mapping is to add HW threads round-robin across cores.

class THREADS_SWITCH_CLASS : public COMMAND_SWITCH_INT_CLASS
{
  private:
    UINT32 threadsPerCore;
  public:
    THREADS_SWITCH_CLASS() : threadsPerCore(MAX_NUM_THREADS_PER_CORE), COMMAND_SWITCH_INT_CLASS("threads-per-core") {}
    ~THREADS_SWITCH_CLASS() {}
    UINT32 ThreadsPerCore() { return threadsPerCore; }

    void ProcessSwitchInt(int arg) 
    { 
            
        // Verify that the number is less than the static maximum.
        VERIFY(arg > 0, "Told to run with nonpositive number of threads per core");
        VERIFY(arg <= MAX_NUM_THREADS_PER_CORE, "Told to run more hw thread instances than statically available!");
        threadsPerCore = arg;
    }
    bool ShowSwitch(char* buff)
    {
        strcpy(buff, "[--threads-per-core=<n>]        Number of hw threads per core.");
        return true;
    }
};


typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS : public PLATFORMS_MODULE_CLASS,
                             public HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS
{
  private:
    THREADS_SWITCH_CLASS threadsSwitch;
    SMT_PIPELINE_CLIENT_STUB_CLASS smtPipelineClient;
  public:
    
    HASIM_PIPELINE_CLASS() :
        PLATFORMS_MODULE_CLASS(),
        HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS("smt-pipeline"),
        threadsSwitch(),
        smtPipelineClient(this)
    {}
    ~HASIM_PIPELINE_CLASS() {}

    void Init() {}

    //
    // Topology
    //
    bool MapTopology(HASIM_CHIP_TOPOLOGY topology)
    {
        // Make sure state upon which this module depends is ready.
        if (! topology->ParamIsSet(TOPOLOGY_NUM_CONTEXTS) ||
            ! topology->ParamIsSet(TOPOLOGY_NUM_CORES))
        {
            return false;
        }

        UINT32 num_ctxts = topology->GetParam(TOPOLOGY_NUM_CONTEXTS);
        UINT32 num_cores = topology->GetParam(TOPOLOGY_NUM_CORES);

        UINT32 threads_per_core = threadsSwitch.ThreadsPerCore();
        topology->SetParam(TOPOLOGY_THREADS_PER_CORE, threads_per_core);

        VERIFY(num_cores * threads_per_core <= num_ctxts,
               "Number of threads exceeds number of benchmark contexts");
            
        smtPipelineClient.SetNumThreadsPerCore(threads_per_core);

        return true;
    }
};
