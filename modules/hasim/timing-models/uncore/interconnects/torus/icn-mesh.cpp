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

#include <math.h>

#include "asim/syntax.h"
#include "awb/provides/hasim_interconnect.h"
#include "awb/provides/chip_base_types.h"
#include "awb/provides/hasim_chip_topology.h"


//
// The FPGA uses a table to map local ports to either cores or memory
// controllers.  These are the possible values.
//
typedef enum
{
    ICN_LOCAL_PORT_CORE = 0,
    ICN_LOCAL_PORT_MEMCTRL = 1,
    ICN_LOCAL_PORT_NONE = 2
}
ICN_LOCAL_PORT_TYPE;


//
// numericToBits --
//   Number of bits needed to represent n elements.  Like Bluespec's TLog.
//
static unsigned int numericToBits(unsigned int n)
{
    if (n <= 2)
    {
        return 1;
    }
    else
    {
        return ceil(log2(n));
    }
}


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

    //
    // The memory controllers will be added as extra rows at the top and
    // bottom of the mesh.  If only only one memory controller is requested
    // then it will be at the top.
    //
    UINT32 num_cols = MESH_WIDTH;
    UINT32 num_rows = MESH_HEIGHT + ((num_mem_ctrl != 1) ? 2 : 1);

    VERIFY(num_cores <= MESH_WIDTH * MESH_HEIGHT,
           "Number of cores exceeds the number of positions in the network!");
    VERIFY(MESH_WIDTH >= MESH_HEIGHT,
           "Mesh WIDTH must be >= mesh HEIGHT!");
    VERIFY(num_mem_ctrl != 0,
           "No memory controllers requested!");
    VERIFY(num_mem_ctrl <= MESH_WIDTH * 2,
           "Too many memory controllers for network size!");

    //
    // The user specifies the mesh dimensions for cores.  The FPGA model
    // dimensions include memory controllers.
    //
    topology->SetParam(TOPOLOGY_NET_MESH_WIDTH, num_cols);
    topology->SetParam(TOPOLOGY_NET_MESH_HEIGHT, num_rows);
    topology->SetParam(TOPOLOGY_NET_MAX_NODE_IID, num_cols * num_rows - 1);

    //
    // Stream out map of network nodes to CPUs and memory controllers.
    // Numbers indicate the type of node (0 CPU, 1 memory controller,
    // 2 empty).
    //

    // Array to record the chosen positions of all memory controllers
    TOPOLOGY_VALUE* memctrl_net_pos = new TOPOLOGY_VALUE[num_mem_ctrl];
    int memctrl_idx = 0;

    // Start with the first row of memory controllers, putting them near the
    // center of the row.
    TOPOLOGY_VALUE* memctrl_map = new TOPOLOGY_VALUE[num_cols];

    // Start with all positions empty
    for (int c = 0; c < num_cols; c++)
    {
        memctrl_map[c] = ICN_LOCAL_PORT_NONE;
    }

    // Lay down 1/4th of the controllers in the top left quadrant.  Addition
    // before division by 4 (shift right by 2) handles remainders.
    int pos = (num_cols - 1) >> 1;
    for (int m = (num_mem_ctrl + 3) >> 2; m > 0; m--)
    {
        VERIFYX(pos >= 0);
        memctrl_net_pos[memctrl_idx++] = pos;
        memctrl_map[pos--] = ICN_LOCAL_PORT_MEMCTRL;
    }

    // Top right quadrant.
    pos = (num_cols - 1) >> 1;
    for (int m = (num_mem_ctrl + 1) >> 2; m > 0; m--)
    {
        VERIFYX((pos + 1) < num_cols);
        memctrl_map[++pos] = ICN_LOCAL_PORT_MEMCTRL;
        memctrl_net_pos[memctrl_idx++] = pos;
    }

    for (int c = 0; c < num_cols; c++)
    {
        printf(" %c", memctrl_map[c] == ICN_LOCAL_PORT_MEMCTRL ? 'M' : 'x');
    }
    printf("\n");

    topology->SendParam(TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP,
                        memctrl_map, sizeof(TOPOLOGY_VALUE) * num_cols,
                        false);

    //
    // Put the cores in the center.
    //
    int net_core_slots = num_cols * MESH_HEIGHT;

    // Cores map is like memctrl_map.
    TOPOLOGY_VALUE* cores_map = new TOPOLOGY_VALUE[net_core_slots];

    // Start with all positions empty
    for (int c = 0; c < net_core_slots; c++)
    {
        cores_map[c] = ICN_LOCAL_PORT_NONE;
    }

    // First half of cores
    pos = (net_core_slots - 1) >> 1;
    for (int c = (num_cores + 1) >> 1; c > 0; c--)
    {
        VERIFYX(pos >= 0);
        cores_map[pos--] = ICN_LOCAL_PORT_CORE;
    }

    // Second half of cores
    pos = (net_core_slots - 1) >> 1;
    for (int c = num_cores >> 1; c > 0; c--)
    {
        VERIFYX((pos + 1) < net_core_slots);
        cores_map[++pos] = ICN_LOCAL_PORT_CORE;
    }

    //
    // Compute the network positions of all the assigned cores.
    //
    TOPOLOGY_VALUE* cores_net_pos = new TOPOLOGY_VALUE[num_cores];
    int c_idx = 0;
    for (int c = 0; c < net_core_slots; c++)
    {
        if (cores_map[c] == ICN_LOCAL_PORT_CORE)
        {
            VERIFY(c_idx < num_cores, "Too many cores were mapped!");

            // Add num_cols to compute the network index because there is
            // one row of memory controllers at the top.
            cores_net_pos[c_idx++] = c + num_cols;
        }
    }

    pos = 0;
    for (int r = 0; r < MESH_HEIGHT; r++)
    {
        for (int c = 0; c < num_cols; c++)
        {
            printf(" %c", cores_map[pos++] == ICN_LOCAL_PORT_CORE ? 'C' : 'x');
        }
        printf("\n");
    }

    topology->SendParam(TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP,
                        cores_map, sizeof(TOPOLOGY_VALUE) * net_core_slots,
                        num_mem_ctrl == 1);

    // Is there another row with memory controllers?
    if (num_mem_ctrl > 1)
    {
        for (int c = 0; c < num_cols; c++)
        {
            memctrl_map[c] = ICN_LOCAL_PORT_NONE;
        }

        // Bottom left quadrant
        pos = num_cols >> 1;
        for (int m = num_mem_ctrl >> 2; m > 0; m--)
        {
            VERIFYX(pos > 0);
            memctrl_map[--pos] = ICN_LOCAL_PORT_MEMCTRL;
            memctrl_net_pos[memctrl_idx++] = pos + net_core_slots + num_cols;
        }

        // Bottom right quadrant.
        pos = num_cols >> 1;
        for (int m = (num_mem_ctrl + 2) >> 2; m > 0; m--)
        {
            VERIFYX(pos < num_cols);
            memctrl_net_pos[memctrl_idx++] = pos + net_core_slots + num_cols;
            memctrl_map[pos++] = ICN_LOCAL_PORT_MEMCTRL;
        }

        for (int c = 0; c < num_cols; c++)
        {
            printf(" %c", memctrl_map[c] == ICN_LOCAL_PORT_MEMCTRL ? 'M' : 'x');
        }
        printf("\n");

        topology->SendParam(TOPOLOGY_NET_LOCAL_PORT_TYPE_MAP,
                            memctrl_map, sizeof(TOPOLOGY_VALUE) * num_cols,
                            true);
    }

    VERIFY(num_mem_ctrl == memctrl_idx,
           "Failed to lay down requested number of memory controllers!");


    //
    // Map addresses to memory controllers.  The map has 8 entries for
    // every memory controller, allowing an even spread of addresses to
    // controllers.
    //
    int n_mem_map_entries = 1 << numericToBits(8 * MAX_NUM_MEM_CTRLS);
    for (int addr_idx = 0; addr_idx < n_mem_map_entries; addr_idx++)
    {
        bool is_last = (addr_idx + 1 == n_mem_map_entries);
        topology->SendParam(TOPOLOGY_NET_MEM_CTRL_MAP,
                            &memctrl_net_pos[addr_idx % num_mem_ctrl],
                            sizeof(TOPOLOGY_VALUE),
                            is_last);
    }

    //
    // Build a similar map for distributed LLC entries.
    //
    int n_llc_map_entries = 1 << numericToBits(8 * MAX_NUM_CPUS);
    for (int addr_idx = 0; addr_idx < n_llc_map_entries; addr_idx++)
    {
        bool is_last = (addr_idx + 1 == n_llc_map_entries);
        topology->SendParam(TOPOLOGY_NET_LLC_ADDR_MAP,
                            &cores_net_pos[addr_idx % num_cores],
                            sizeof(TOPOLOGY_VALUE),
                            is_last);
    }

    //
    // Send the map of core ID to network station ID.
    //
    for (int c = 0; c < num_cores; c++)
    {
        topology->SendParam(TOPOLOGY_NET_CORE_STATION_ID_MAP,
                            &cores_net_pos[c],
                            sizeof(TOPOLOGY_VALUE),
                            num_cores - 1 == c);
    }

    delete[] memctrl_map;
    delete[] memctrl_net_pos;
    delete[] cores_map;
    delete[] cores_net_pos;


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

        UINT64 s_col = s % num_cols;
        UINT64 s_row = s / num_cols;

        //
        // Don't allow traversal through memory controllers.  Force a packet
        // first to go to a core router before heading east/west.
        //
        bool allow_ew_flow = (s_row != 0) &&
                             ((s_row != (num_rows - 1)) || (num_mem_ctrl == 1));

        for (int d = 0; d < max_nodes; d++)
        {
            UINT64 d_col = d % num_cols;
            UINT64 d_row = d / num_cols;

            // Pick a route from s to d
            UINT8 s_d_rt = 0;
            if ((d_col < s_col) && allow_ew_flow)
            {
                s_d_rt = 3;     // West
            }
            else if ((d_col > s_col) && allow_ew_flow)
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
