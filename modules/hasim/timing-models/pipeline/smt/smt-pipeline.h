#include "asim/provides/chip_base_types.h"
#include "asim/provides/pipeline_base_types.h"
#include "asim/mesg.h"

// An inorder pipeline.

// Multi-Instance: this pipeline can dynamically duplicate itself once per functional partiton context.
// This is useful for simulating multiple contexts.

// The num-cores switch can set the number of cores to be less than the number of contexts the benchmark expects.
// If this is not set the default mapping is one core per context.

class CORES_SWITCH_CLASS : public COMMAND_SWITCH_INT_CLASS
{
    private:
        UINT32 numCores;
    public:
        CORES_SWITCH_CLASS() : numCores(0), COMMAND_SWITCH_INT_CLASS("num-cores") {}
        ~CORES_SWITCH_CLASS() {}
        UINT32 NumCores() { return numCores; }
        
        void ProcessSwitchInt(int arg) 
        { 
            
            // Verify that the number is less than the static maximum.
            VERIFY(arg <= NUM_CPUS, "Told to run more core instances than statically available!");
            numCores = arg;
        }
        bool ShowSwitch(char* buff)
        {
            strcpy(buff, "[--num-cores=<n>]        Number of cores to simulate.");
            return true;
        }
};

// The threads-per-core switch can set the number of threads per core.
// We must check that threads-per-core x num-cores <= workload_contexts.
// If this is not set the default mapping is to add HW threads round-robin across cores.

class THREADS_SWITCH_CLASS : public COMMAND_SWITCH_INT_CLASS
{
    private:
        UINT32 threadsPerCore;
    public:
        THREADS_SWITCH_CLASS() : threadsPerCore(0), COMMAND_SWITCH_INT_CLASS("threads-per-core") {}
        ~THREADS_SWITCH_CLASS() {}
        UINT32 ThreadsPerCore() { return threadsPerCore; }
        
        void ProcessSwitchInt(int arg) 
        { 
            
            // Verify that the number is less than the static maximum.
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

class HASIM_PIPELINE_CLASS
{
    private:
        CORES_SWITCH_CLASS numCoresSwitch;
        THREADS_SWITCH_CLASS threadsSwitch;
    public:
    
        HASIM_PIPELINE_CLASS() : numCoresSwitch(), threadsSwitch() {}
        ~HASIM_PIPELINE_CLASS() {}

        void Init() {}
        void MapContexts(int num_ctxts) 
        {
            // Verify that the number is less than the static maximum number of hardware threads.
            VERIFY(num_ctxts <= (NUM_CPUS * MAX_NUM_THREADS_PER_CORE), "Told to map more benchmark contexts than available hardware instances!");
            
            // See if the user over-rode the default mapping.
            UINT32 num_cores = numCoresSwitch.NumCores();
            UINT32 threads_per_core = threadsSwitch.ThreadsPerCore();
            
            // TODO: handle threads-per-core here.
            if (threads_per_core != 0)
            {
                cerr << "Warning: Ignored currently unsupported argument --threads-per-core" << endl;
            }

            if (num_cores != 0)
            {
                // See if they are doing it with a sensible number. It must be less than the number of benchmark contexts.
                VERIFY(num_cores <= num_ctxts, "Told to run more core instances than than available benchmark contexts!");
                               
                // Go with the new smaller number.
                COMMANDS_SERVER_CLASS::GetInstance()->SetNumHardwareThreads(num_cores);
            
            }
            else
            {
                // The default mapping is one core instance per benchmark context.
                COMMANDS_SERVER_CLASS::GetInstance()->SetNumHardwareThreads(num_ctxts);
            }
        }
};
