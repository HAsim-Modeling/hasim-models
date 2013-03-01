#include "asim/provides/chip_base_types.h"

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
            VERIFY(arg <= MAX_NUM_CPUS, "Told to run more core instances than statically available!");
            numCores = arg;
        }
        void ShowSwitch(std::ostream& ostr, const string& prefix)
        {
            ostr << prefix << "[--num-cores=<n>]        Number of cores to simulate." << endl;
        }
};

typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS
{
    private:
        CORES_SWITCH_CLASS numCoresSwitch;
    public:
    
        HASIM_PIPELINE_CLASS() : numCoresSwitch() {}
        ~HASIM_PIPELINE_CLASS() {}

        void Init() {}
        void MapContexts(int num_ctxts) 
        {
            // Verify that the number is less than the static maximum number of core instances.
            VERIFY(num_ctxts <= MAX_NUM_CPUS, "Told to map more benchmark contexts than available hardware instances!");
            
            // See if the user over-rode the default mapping.
            UINT32 num_cores = numCoresSwitch.NumCores();
            
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
