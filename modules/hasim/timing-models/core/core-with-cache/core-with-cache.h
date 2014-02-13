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

#ifndef __CORE_WITH_CACHE__
#define __CORE_WITH_CACHE__

#include "awb/provides/command_switches.h"
#include "awb/provides/chip_base_types.h"
#include "awb/provides/hasim_chip_topology.h"
#include "awb/provides/hasim_pipeline.h"
#include "awb/provides/hasim_private_caches.h"

// A core with private caches

typedef class HASIM_CORE_CLASS* HASIM_CORE;

class HASIM_CORE_CLASS : public HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS
{
  private:
    HASIM_PIPELINE pipe;
    HASIM_PRIVATE_CACHES privateCaches;

  public:
    HASIM_CORE_CLASS() :
        HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS("core-with-cache"),
        pipe(new HASIM_PIPELINE_CLASS()),
        privateCaches(new HASIM_PRIVATE_CACHES_CLASS())
    {}

    ~HASIM_CORE_CLASS() { delete pipe; delete privateCaches; }

    void Init() { pipe->Init(); privateCaches->Init(); }

    //
    // Topology
    //
    bool MapTopology(HASIM_CHIP_TOPOLOGY topology)
    {
        // Make sure state upon which this module depends is ready.
        if (! topology->ParamIsSet(TOPOLOGY_NUM_CONTEXTS))
        {
            return false;
        }

        UINT32 num_ctxts = topology->GetParam(TOPOLOGY_NUM_CONTEXTS);

        // Verify that the number is less than the static maximum number
        // of core instances.
        VERIFY(num_ctxts <= MAX_NUM_CPUS, "Told to map more benchmark contexts than available hardware instances!");

        // See if the user over-rode the default mapping.
        if (NUM_CORES != 0)
        {
            // See if they are doing it with a sensible number. It must be
            // less than the number of benchmark contexts.
            VERIFY(NUM_CORES <= num_ctxts, "Told to run more core instances than than available benchmark contexts!");
        }
        else
        {
            NUM_CORES = num_ctxts;
        }

        topology->SetParam(TOPOLOGY_NUM_CORES, NUM_CORES);
        COMMANDS_SERVER_CLASS::GetInstance()->SetNumHardwareThreads(NUM_CORES);

        return true;
    }
};

#endif // __CORE_WITH_CACHE__
