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

//
// Standard topology manager
//

#include "asim/syntax.h"
#include "asim/mesg.h"

#include "awb/provides/hasim_chip_topology.h"

list<HASIM_CHIP_TOPOLOGY_MAPPERS> HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS::allMappers;


void
HASIM_CHIP_TOPOLOGY_CLASS::MapContexts(UINT32 numCtxts)
{
    list<HASIM_CHIP_TOPOLOGY_MAPPERS>::const_iterator topo_obj;
    list<HASIM_CHIP_TOPOLOGY_MAPPERS>::const_iterator topo_obj_end =
        HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS::getAllMappers().end();

    // Set NUM_CONTEXTS parameter.  All other mappers will be able to query
    // the value.
    SetParam(TOPOLOGY_NUM_CONTEXTS, numCtxts);

    topo_obj = HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS::getAllMappers().cbegin();
    while (topo_obj != topo_obj_end)
    {
        (*topo_obj)->InitTopology(this);
        topo_obj++;
    }

    topo_obj = HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS::getAllMappers().cbegin();
    while (topo_obj != topo_obj_end)
    {
        (*topo_obj)->MapTopology(this);
        topo_obj++;
    }
}


void
HASIM_CHIP_TOPOLOGY_CLASS::SetParam(TOPOLOGY_DICT_ENUM param, UINT32 value)
{
    VERIFY(topoParams.find(param) == topoParams.end(),
           "Topology parameter " << param << " set twice!");

    topoParams[param] = value;
}

UINT32
HASIM_CHIP_TOPOLOGY_CLASS::GetParam(TOPOLOGY_DICT_ENUM param) const
{
    map<TOPOLOGY_DICT_ENUM, UINT32>::const_iterator p;

    p = topoParams.find(param);
    VERIFY(p != topoParams.end(), "Failed to find topology parameter " << param);

    return p->second;
}

bool
HASIM_CHIP_TOPOLOGY_CLASS::ParamIsSet(TOPOLOGY_DICT_ENUM param) const
{
    return (topoParams.find(param) != topoParams.end());
}
