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

#include "awb/provides/hasim_core.h"
#include "awb/provides/hasim_chip_topology.h"

#ifndef __SINGLE_CORE_CHIP__
#define __SINGLE_CORE_CHIP__

// A single-core chip

typedef class HASIM_CHIP_CLASS* HASIM_CHIP;

class HASIM_CHIP_CLASS
{
  private:
    HASIM_CORE core;
    HASIM_CHIP_TOPOLOGY chipTopology;

  public:
    HASIM_CHIP_CLASS() :
        core(new HASIM_CORE_CLASS()),
        chipTopology(new HASIM_CHIP_TOPOLOGY_CLASS())
    {};

    ~HASIM_CHIP_CLASS() { delete core; }

    void Init()
    {
        core->Init();
        chipTopology->Init();
    }

    void MapContexts(UINT32 numCtxts) {
        chipTopology->MapContexts(numCtxts);
    }
};

#endif // __SINGLE_CORE_CHIP__
