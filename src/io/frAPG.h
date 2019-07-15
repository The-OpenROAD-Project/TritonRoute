/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FR_APG_H_
#define _FR_APG_H_

#include "frBaseTypes.h"
#include "frDesign.h"
#include "db/tech/frConstraint.h"
#include "drc/frDRC.h"

namespace fr {
  class APGWorker {
  public:
    APGWorker() {}
    APGWorker(frDesign* designIn, frInst* instIn): design(designIn), inst(instIn) {

    }

    void init();
    void setup();
    void main();
    void end();



  protected:
    frDesign* design;
    frInst* inst;
    // frVector<frAccessPattern*> aps;
    std::set<frPin*> unConnectedPins;
    frVector<frPin*> pinOrder;
    std::set<FlexAccessPattern*> validAPs;
    frVector<FlexAccessPattern*> bestAPComb;
    std::map<frPin*, std::vector<FlexAccessPattern*> > pin2APs;
    std::map<frLayerNum, bgi::rtree<std::pair<point_t, FlexAccessPattern*>, bgi::quadratic<16> > > layer2APRTree;
    Rectangle apBBoxRect;

    // functions
    void updateAPStatus();
  };
}



#endif