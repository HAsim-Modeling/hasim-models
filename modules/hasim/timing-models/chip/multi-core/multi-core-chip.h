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

#include "awb/provides/hasim_core.h"
#include "awb/provides/hasim_uncore.h"

// A single-core chip

typedef class HASIM_CHIP_CLASS* HASIM_CHIP;

class HASIM_CHIP_CLASS
{
  private:
    HASIM_CORE core;
    HASIM_UNCORE uncore;

  public:
    HASIM_CHIP_CLASS() :
        core(new HASIM_CORE_CLASS()),
        uncore(new HASIM_UNCORE_CLASS())
    {}

    ~HASIM_CHIP_CLASS() { delete core; delete uncore; }

    void Init() { uncore->Init(); core->Init(); }
    void MapContexts(int num_ctxts) { core->MapContexts(num_ctxts); }
};
