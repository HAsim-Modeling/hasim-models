// A 6-stage pipeline suitable for OOO issue.

// Single-Instance: this pipeline is single-threaded and single-instance, so it does not support multiple contexts.

typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS
{
    public:
    
        HASIM_PIPELINE_CLASS() {}
        ~HASIM_PIPELINE_CLASS() {}

        void Init() {}
        void MapContexts(int num_ctxts) 
        {
            VERIFY(num_ctxts == 1, "Error: OOO pipeline currently only supports one functional partition context.");
            // Use the command relay to start 1 HW thread.
            COMMANDS_SERVER_CLASS::GetInstance()->SetNumHardwareThreads(1);
        }
};
