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

#ifndef __TOPOLOGY_STD__
#define __TOPOLOGY_STD__


#include <string>
#include <list>
#include <map>

#include "awb/provides/chip_base_types.h"
#include "awb/dict/TOPOLOGY.h"

using namespace std;

typedef class HASIM_CHIP_TOPOLOGY_CLASS* HASIM_CHIP_TOPOLOGY;


//
// HASIM_CHIP_TOPOLOGY_MAPPERS --
//   Base type for a list of objects that want to be called during the
//   topology mapping phase.  They will be passed a pointer to the
//   HASIM_CHIP_TOPOLOGY instance, enabling the called objects to
//   make further topological choices.
//
typedef class HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS* HASIM_CHIP_TOPOLOGY_MAPPERS;

class HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS
{
  private:
    static list<HASIM_CHIP_TOPOLOGY_MAPPERS> allMappers;
    const string name;

  public:
    HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS(const string myName) :
        name(myName)
    {
        // Add this instance to the global list of mappers
        allMappers.push_back(this);
    }
    
    // Could remove this entry from allMappers, but we currently don't
    ~HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS() {}

    const string& GetName() const { return name; }

    // InitTopology is called as a first pass through all managed topology
    // clients.  Clients may not depend on the order in which they are
    // called.  Clients may use this pass to set values of global parameters
    // (see SetParam in HASIM_CHIP_TOPOLOGY_CLASS).
    virtual void InitTopology(HASIM_CHIP_TOPOLOGY topology) {};

    // Each object provides a method for mapping its locally managed topology.
    virtual void MapTopology(HASIM_CHIP_TOPOLOGY topology) = 0;

    static const list<HASIM_CHIP_TOPOLOGY_MAPPERS>& getAllMappers()
    {
        return allMappers;
    }
};


//
// Top-level framework for managing topology.
//

class HASIM_CHIP_TOPOLOGY_CLASS
{
  private:
    map<TOPOLOGY_DICT_ENUM, UINT32> topoParams;

  public:
    HASIM_CHIP_TOPOLOGY_CLASS() {};
    ~HASIM_CHIP_TOPOLOGY_CLASS() {};

    void Init() {};
    void MapContexts(UINT32 numCtxts);

    //
    // Methods used by topology mappers to maintain a global database of
    // configuration parameters and to communicate parameter values to
    // hardware.
    //
    void SetParam(TOPOLOGY_DICT_ENUM param, UINT32 value);
    UINT32 GetParam(TOPOLOGY_DICT_ENUM param) const;
    bool ParamIsSet(TOPOLOGY_DICT_ENUM param) const;
};

#endif // __TOPOLOGY_STD__
