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

#include "asim/syntax.h"
#include "awb/provides/hasim_interconnect.h"
#include "awb/provides/chip_base_types.h"
#include "awb/provides/hasim_chip_topology.h"


// constructor
HASIM_INTERCONNECT_CLASS::HASIM_INTERCONNECT_CLASS() :
    HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS("icn-mesh")
{
}

// init
void
HASIM_INTERCONNECT_CLASS::Init()
{
}

bool
HASIM_INTERCONNECT_CLASS::MapTopology(HASIM_CHIP_TOPOLOGY topology)
{
    // Make sure state upon which this module depends is ready.
    if (! topology->ParamIsSet(TOPOLOGY_NUM_CORES) ||
        ! topology->ParamIsSet(TOPOLOGY_NUM_MEM_CONTROLLERS))
    {
        return false;
    }

    UINT32 num_cores = topology->GetParam(TOPOLOGY_NUM_CORES);
    UINT32 num_mem_ctrl = topology->GetParam(TOPOLOGY_NUM_MEM_CONTROLLERS);

    topology->SetParam(TOPOLOGY_NET_MESH_WIDTH, MESH_WIDTH);
    topology->SetParam(TOPOLOGY_NET_MESH_HEIGHT, MESH_HEIGHT);
    topology->SetParam(TOPOLOGY_NET_MAX_NODE_IID, MESH_WIDTH * MESH_HEIGHT - 1);

    //
    // Stream out map of network nodes to CPUs and memory controllers.  Nodes
    // are walked in order (width then height) and each entry indicates the
    // device attached to the node.
    //
    UINT64 cpu_id = 0;
    UINT64 node_id = 0;
    for (int r = 0; r < MESH_WIDTH; r += 1)
    {
        for (int c = 0; c < MESH_HEIGHT; c += 1)
        {
            bool done = (r == MESH_WIDTH-1) && (c == MESH_HEIGHT-1);

            // For now there is only one memory controller and all other
            // nodes are CPUs.
            TOPOLOGY_VALUE t;
            if (node_id == MESH_MEM_CTRL_LOC)
            {
                t = 1;
            }
            else if (cpu_id >= num_cores)
            {
                t = 2;
            }
            else
            {
                t = 0;
                cpu_id += 1;
            }

            topology->SendParam(TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP, &t, 1, done);
            node_id += 1;
        }
    }

    //
    // Stream in routing tables.  Each node has a table that is a vector
    // of next hops to all other nodes.  A route is stored as a 2-bit
    // entry:
    //    0 - North
    //    1 - East
    //    2 - South
    //    3 - West
    //
    // There are MAX_NUM_CPUS + 1 stations in the table.
    //
    int max_nodes = MAX_NUM_CPUS + MAX_NUM_MEM_CTRLS;
    for (int s = 0; s < max_nodes; s++)
    {
        UINT8 buf[MAX_NUM_CPUS + MAX_NUM_MEM_CTRLS];
        int bufIdx = 0;

        UINT8 rt = 0;
        int chunks = 0;

        UINT64 s_col = s % MESH_WIDTH;
        UINT64 s_row = s / MESH_WIDTH;

        for (int d = 0; d < max_nodes; d++)
        {
            UINT64 d_col = d % MESH_WIDTH;
            UINT64 d_row = d / MESH_WIDTH;

            // Pick a route from s to d
            UINT8 s_d_rt = 0;
            if (d_col < s_col)
            {
                s_d_rt = 3;     // West
            }
            else if (d_col > s_col)
            {
                s_d_rt = 1;     // East
            }
            else if (d_row > s_row)
            {
                s_d_rt = 2;     // South
            }
            else
            {
                s_d_rt = 0;     // North
            }

            // Stream s to d route to hardware
            rt = (s_d_rt << 6) | (rt >> 2);

            // Done with a chunk?
            chunks += 1;
            if (chunks == 4)
            {
                buf[bufIdx++] = rt;

                chunks = 0;
                rt = 0;
            }
        }

        // Any remainder for this source node?
        if (chunks != 0)
        {
            while (chunks != 4)
            {
                rt >>= 2;
                chunks += 1;
            }

            buf[bufIdx++] = rt;
        }

        topology->SendParam(TOPOLOGY_NET_ROUTING_TABLE, buf, bufIdx,
                            s == max_nodes - 1);
    }

    return true;
}
