//
// Copyright (C) 2012 Intel Corporation
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

#ifndef __UNCORE_WITH_INTERCONNECT__
#define __UNCORE_WITH_INTERCONNECT__

#include "awb/provides/hasim_interconnect.h"
#include "awb/provides/hasim_chip_topology.h"

// Uncore with an interconnect

typedef class HASIM_UNCORE_CLASS* HASIM_UNCORE;

class HASIM_UNCORE_CLASS : public HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS
{
  private:
    HASIM_INTERCONNECT inter;

  public:
    HASIM_UNCORE_CLASS() :
        HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS("uncore-with-interconnect"),
        inter(new HASIM_INTERCONNECT_CLASS())
    {}

    ~HASIM_UNCORE_CLASS() { delete inter; }

    void Init() { inter->Init(); }


    //
    // Topology
    //
    bool MapTopology(HASIM_CHIP_TOPOLOGY topology)
    {
        topology->SetParam(TOPOLOGY_NUM_MEM_CONTROLLERS, NUM_MEM_CTRL);
        return true;
    }
};

#endif // __UNCORE_WITH_INTERCONNECT__
