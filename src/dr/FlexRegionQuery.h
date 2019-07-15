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

#ifndef _FLEX_REGIONQUERY_H_
#define _FLEX_REGIONQUERY_H_

//#include <mutex>
//#include <shared_mutex>
#include "frBaseTypes.h"
#include "frDesign.h"
#include "db/obj/frShape.h"
#include "db/obj/frVia.h"
#include "FlexAccessPattern.h"

namespace fr {
  class FlexRegionQuery {
  public:
    //FlexRegionQuery(): tech(std::make_shared<frTechObject>()), design(std::make_shared<frDesign>()) {};
    FlexRegionQuery(const std::shared_ptr<frTechObject> &techIn, const std::shared_ptr<frDesign> &designIn): 
                tech(techIn), design(designIn) {};
    // getters
    std::shared_ptr<frTechObject> getTech() const {
      return tech.lock();
    }
    std::shared_ptr<frDesign> getDesign() const {
      return design.lock();
    }
    // setters
    void add(frListIter<frPathSeg> &pathSeg);
    void add(frListIter<frVia> &via);
    void add(frListIter<FlexAccessPattern> &ap);
    void add(frShape* shape);
    void add(frVia* shape);
    void remove(frListIter<frPathSeg> &pathSeg);
    void remove(frListIter<frVia> &via);
    void remove(frListIter<FlexAccessPattern> &ap);
    void remove(frShape* shape);
    void remove(frVia* shape);
    void query(const frBox &box, frLayerNum layerNum, frVector<frListIter<frPathSeg> > &result);
    void query(const frBox &box, frLayerNum layerNum, frVector<frListIter<frVia> > &result);
    void query(const frBox &box, frLayerNum layerNum, frVector<frListIter<FlexAccessPattern> > &result);
    void query(const frBox &box, frLayerNum layerNum, frVector<frPinFig*> &result);
    void query(const frBox &box, frVector<frListIter<frPathSeg> > &result);
    void query(const frBox &box, frVector<frListIter<frVia> > &result);
    void query(const frBox &box, frVector<frListIter<FlexAccessPattern> > &result);
    void query(const frBox &box, frVector<frPinFig*> &result);
    // others
    void init(frLayerNum numLayers);
    void print();
    
  protected:
    std::weak_ptr<frTechObject>     tech;
    std::weak_ptr<frDesign>         design;
    std::vector< bgi::rtree< rq_iter_value_t<frPathSeg>,          bgi::quadratic<16> > > pathsegs;
    std::vector< bgi::rtree< rq_iter_value_t<frVia>,              bgi::quadratic<16> > > vias;
    std::vector< bgi::rtree< rq_iter_value_t<FlexAccessPattern>,  bgi::quadratic<16> > > aps;
    std::vector< bgi::rtree< rq_rptr_value_t<frPinFig>,           bgi::quadratic<16> > > shapes;
    //mutable std::shared_mutex m;
    //std::vector< mutable std::shared_mutex > mutex_pathsegs;
    //std::vector< mutable std::shared_mutex > mutex_vias;
    //std::vector< mutable std::shared_mutex > mutex_shapes;
  };
}

#endif

