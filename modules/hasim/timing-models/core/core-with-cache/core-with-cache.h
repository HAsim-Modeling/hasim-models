//
// Copyright (C) 2013 Intel Corporation
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
