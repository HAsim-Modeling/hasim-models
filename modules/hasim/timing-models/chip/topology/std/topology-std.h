//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
