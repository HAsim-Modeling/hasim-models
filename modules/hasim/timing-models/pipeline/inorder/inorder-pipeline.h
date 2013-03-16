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

#ifndef __INORDER_PIPELINE__
#define __INORDER_PIPELINE__

#include "awb/provides/chip_base_types.h"
#include "awb/provides/hasim_chip_topology.h"


// An inorder pipeline.

// Multi-Instance: this pipeline can dynamically duplicate itself once
// per functional partiton context.  This is useful for simulating
// multiple contexts.


typedef class HASIM_PIPELINE_CLASS* HASIM_PIPELINE;

class HASIM_PIPELINE_CLASS
{
  public:
    HASIM_PIPELINE_CLASS() {}
    ~HASIM_PIPELINE_CLASS() {}

    void Init() {};
};

#endif // __INORDER_PIPELINE__
