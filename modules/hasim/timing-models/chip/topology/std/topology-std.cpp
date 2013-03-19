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

    //
    // Call all topology mappers until each of them is complete.  The
    // iteration allows clients that depend on global state set by other
    // clients to be called in arbitrary order.
    //
    UINT32 trips = 0;
    bool done;
    do
    {
        done = true;
        topo_obj = HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS::getAllMappers().cbegin();
        while (topo_obj != topo_obj_end)
        {
            // Map client's topology if it hasn't been mapped already.
            if (! (*topo_obj)->InitDone())
            {
                bool ok = (*topo_obj)->MapTopology(this);
                done = done && ok;
                if (ok)
                {
                    (*topo_obj)->SetInitDone();
                }
                else if (trips == 10)
                {
                    // Must terminate eventually
                    ASIMERROR("MapTopology failed to terminate for client: " <<
                              (*topo_obj)->GetName());
                }
            }

            topo_obj++;
        }

        trips += 1;
    }
    while (! done);

    //
    // Send all the parameters to the hardware.
    //
    map<TOPOLOGY_DICT_ENUM, TOPOLOGY_VALUE>::const_iterator params;
    params = topoParams.cbegin();
    while (params != topoParams.end())
    {
        SendParam(params->first, params->second);
        params++;
    }
}


void
HASIM_CHIP_TOPOLOGY_CLASS::SetParam(TOPOLOGY_DICT_ENUM param, TOPOLOGY_VALUE value)
{
    VERIFY(topoParams.find(param) == topoParams.end(),
           "Topology parameter " << param << " set twice!");

    topoParams[param] = value;
}

TOPOLOGY_VALUE
HASIM_CHIP_TOPOLOGY_CLASS::GetParam(TOPOLOGY_DICT_ENUM param) const
{
    map<TOPOLOGY_DICT_ENUM, TOPOLOGY_VALUE>::const_iterator p;

    p = topoParams.find(param);
    VERIFY(p != topoParams.end(), "Failed to find topology parameter " << param);

    return p->second;
}

bool
HASIM_CHIP_TOPOLOGY_CLASS::ParamIsSet(TOPOLOGY_DICT_ENUM param) const
{
    return (topoParams.find(param) != topoParams.end());
}

void
HASIM_CHIP_TOPOLOGY_CLASS::SendParam(
    TOPOLOGY_DICT_ENUM param,
    TOPOLOGY_VALUE value)
{
    clientStub->setParam(param, value, 3);
}

void
HASIM_CHIP_TOPOLOGY_CLASS::SendParam(
    TOPOLOGY_DICT_ENUM param,
    const void* value,
    int len,
    bool last)
{
    TOPOLOGY_VALUE* topo_vals = (TOPOLOGY_VALUE*) value;

    while (len > 0)
    {
        len -= sizeof(TOPOLOGY_VALUE);

        // Encode the last info (last in stream and last in this marshalled
        // value.)
        UINT8 last_enc = 0;
        if (len <= 0)
        {
            last_enc = 1 + (last ? 2 : 0);
        }

        clientStub->setParam(param, *topo_vals++, last_enc);
    }
}
