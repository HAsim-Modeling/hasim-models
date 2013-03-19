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
