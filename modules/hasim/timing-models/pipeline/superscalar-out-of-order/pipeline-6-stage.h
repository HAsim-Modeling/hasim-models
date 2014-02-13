//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef __PIPELINE_6_STAGE__
#define __PIPELINE_6_STAGE__

#include "asim/syntax.h"
#include "asim/mesg.h"

#include "awb/provides/chip_base_types.h"
#include "awb/provides/hasim_chip_topology.h"


// A 6-stage pipeline suitable for OOO issue.

// Single-Instance: this pipeline is single-threaded and single-instance, so it does not support multiple contexts.

typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS : public HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS
{
  public:
    
    HASIM_PIPELINE_CLASS() :
        HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS("pipeline-6-stage")
    {}
    ~HASIM_PIPELINE_CLASS() {}

    void Init() {}

    //
    // Topology
    //
    bool MapTopology(HASIM_CHIP_TOPOLOGY topology)
    {
        // Make sure state upon which this module depends is ready.
        if (! topology->ParamIsSet(TOPOLOGY_NUM_CORES))
        {
            return false;
        }

        UINT32 num_cores = topology->GetParam(TOPOLOGY_NUM_CORES);

        VERIFY(num_cores == 1,
               "Error: OOO pipeline currently supports only one functional partition context.");

        return true;
    }
};

#endif // __PIPELINE_6_STAGE__
