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
#include "awb/rrr/client_stub_TOPOLOGY.h"
#include "awb/dict/TOPOLOGY.h"

using namespace std;

typedef UINT16 TOPOLOGY_VALUE;

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
    bool initDone;

    bool InitDone() const { return initDone; }
    void SetInitDone() { initDone = true; }

  public:
    HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS(const string myName) :
        name(myName),
        initDone(false)
    {
        // Add this instance to the global list of mappers
        allMappers.push_back(this);
    }
    
    // Could remove this entry from allMappers, but we currently don't
    ~HASIM_CHIP_TOPOLOGY_MAPPERS_CLASS() {}

    const string& GetName() const { return name; }

    //
    // MapTopology is a virtual method that must be provided by all instances
    // of the mapper class.  Clients may not depend on the order in which they
    // are called.  Clients should return "true" for success and "false" if
    // the client should be called again after all other clients have been
    // called.  This allows a client to postpone its action until some other
    // topology mapper has set a global parameter.
    //
    // The top level topology manager will stop calling a client after
    // it returns "true."  The top level manager will abort after too many
    // failed attempts.
    //
    virtual bool MapTopology(HASIM_CHIP_TOPOLOGY topology) = 0;

    static const list<HASIM_CHIP_TOPOLOGY_MAPPERS>& getAllMappers()
    {
        return allMappers;
    }

    friend class HASIM_CHIP_TOPOLOGY_CLASS;
};


//
// Top-level framework for managing topology.
//

class HASIM_CHIP_TOPOLOGY_CLASS : PLATFORMS_MODULE_CLASS
{
  private:
    TOPOLOGY_CLIENT_STUB clientStub;
    map<TOPOLOGY_DICT_ENUM, TOPOLOGY_VALUE> topoParams;

  public:
    HASIM_CHIP_TOPOLOGY_CLASS() :
        clientStub(new TOPOLOGY_CLIENT_STUB_CLASS(this))
    {};

    ~HASIM_CHIP_TOPOLOGY_CLASS() {};

    void Init() {};
    void MapContexts(UINT32 numCtxts);

    //
    // Methods used by topology mappers to maintain a global database of
    // configuration parameters and to communicate parameter values to
    // hardware.
    //
    void SetParam(TOPOLOGY_DICT_ENUM param, TOPOLOGY_VALUE value);
    TOPOLOGY_VALUE GetParam(TOPOLOGY_DICT_ENUM param) const;
    bool ParamIsSet(TOPOLOGY_DICT_ENUM param) const;

    //
    // Direct access to sending topology data to the hardware.  These methods
    // may be used to topology mapping clients to stream data or to set
    // parameters different than the standard TOPOLOGY_VALUE parameters
    // above.
    //

    // Send single value
    void SendParam(TOPOLOGY_DICT_ENUM param, TOPOLOGY_VALUE value);
    
    // Stream
    void SendParam(TOPOLOGY_DICT_ENUM param,
                   const void* value,
                   int len,
                   bool last);
};

#endif // __TOPOLOGY_STD__
