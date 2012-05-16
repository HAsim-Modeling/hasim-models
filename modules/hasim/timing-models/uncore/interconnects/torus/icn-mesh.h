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

//
// @file icn-mesh.h
// @brief Configure the routing table
//
// @author Michael Adler
//

#ifndef _ICN_MESH_
#define _ICN_MESH_

#include "asim/syntax.h"

#include "platforms-module.h"
#include "awb/provides/rrr.h"
#include "awb/rrr/client_stub_ICN_MESH.h"

typedef class HASIM_INTERCONNECT_CLASS* HASIM_INTERCONNECT;

class HASIM_INTERCONNECT_CLASS: public PLATFORMS_MODULE_CLASS
{
  private:
    // stub
    ICN_MESH_CLIENT_STUB clientStub;

  public:
    HASIM_INTERCONNECT_CLASS();
    ~HASIM_INTERCONNECT_CLASS() {};

    void Init();
};

#endif // _ICN_MESH_
