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

#include "awb/provides/hasim_core.h"
#include "awb/provides/hasim_chip_topology.h"

#ifndef __SINGLE_CORE_CHIP__
#define __SINGLE_CORE_CHIP__

// A single-core chip

typedef class HASIM_CHIP_CLASS* HASIM_CHIP;

class HASIM_CHIP_CLASS
{
  private:
    HASIM_CORE core;
    HASIM_CHIP_TOPOLOGY chipTopology;

  public:
    HASIM_CHIP_CLASS() :
        core(new HASIM_CORE_CLASS()),
        chipTopology(new HASIM_CHIP_TOPOLOGY_CLASS())
    {};

    ~HASIM_CHIP_CLASS() { delete core; }

    void Init()
    {
        core->Init();
        chipTopology->Init();
    }

    void MapContexts(UINT32 numCtxts) {
        chipTopology->MapContexts(numCtxts);
    }
};

#endif // __SINGLE_CORE_CHIP__
