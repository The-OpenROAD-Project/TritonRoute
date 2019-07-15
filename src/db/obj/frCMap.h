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

#ifndef _FR_CMAP_H_
#define _FR_CMAP_H_

#include "db/obj/frBlockObject.h"

namespace fr {
  class frGCellPattern: public frBlockObject {
  public:
    // constructors
    frGCellPattern(): frBlockObject() {}
    // getters
    bool isHorizontal() const {
      return horizontal;
    }
    frCoord getStartCoord() const {
      return startCoord;
    }
    frUInt4 getSpacing() const {
      return spacing;
    }
    frUInt4 getCount() const {
      return count;
    }
    // setters
    void setHorizontal(bool isH) {
      horizontal = isH;
    }
    void setStartCoord(frCoord scIn) {
      startCoord = scIn;
    }
    void setSpacing(frUInt4 sIn) {
      spacing = sIn;
    }
    void setCount(frUInt4 cIn) {
      count = cIn;
    }
    // others
  protected:
    bool    horizontal;
    frCoord startCoord;
    frUInt4 spacing;
    frUInt4 count;
  };

  class frCMap: public frBlockObject {
  public:
    // constructors
    frCMap(): frBlockObject(), numX(0), numY(0) {}
    // getters
    frUInt4 getThroughDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return cmap_through.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getLocalDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return cmap_local.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getEdge1Demand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return cmap_edge1.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getEdge2Demand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return cmap_edge2.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getUpDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return cmap_up.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getDownDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      if (layerNum == 0) { 
        return 0;
      } else {
        return cmap_up.at(xIndex).at(yIndex).at(layerNum/2 - 1);
      }
    }
    frUInt4 getSupply(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum) const {
      return supply.at(xIndex).at(yIndex).at(layerNum/2);
    }
    frUInt4 getNumX() {
      return numX;
    }
    frUInt4 getNumY() {
      return numY;
    }
    // setters
    void setThroughDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      cmap_through.at(xIndex).at(yIndex).at(layerNum/2) = num;
    }
    void setLocalDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      cmap_local.at(xIndex).at(yIndex).at(layerNum/2)   = num;
    }
    void setEdge1Demand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      cmap_edge1.at(xIndex).at(yIndex).at(layerNum/2)   = num;
    }
    void setEdge2Demand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      cmap_edge2.at(xIndex).at(yIndex).at(layerNum/2)   = num;
    }
    void setUpDemand(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      cmap_up.at(xIndex).at(yIndex).at(layerNum/2)      = num;
    }
    void setSupply(frUInt4 xIndex, frUInt4 yIndex, frLayerNum layerNum, frUInt4 num) {
      supply.at(xIndex).at(yIndex).at(layerNum/2)      = num;
    }
    // others
    void init(frUInt4 numX, frUInt4 numY, frLayerNum topLayer) {
      this->numX = numX;
      this->numY = numY;
      frCollection<frUInt4> v3(topLayer/2 + 1, 0);
      frCollection<frCollection<frUInt4> > v2(numY, v3);
      frCollection<frCollection<frCollection<frUInt4> > > v1(numX, v2);
      supply = cmap_through = cmap_local = cmap_edge1=  cmap_edge2 = cmap_up = v1;
    }

  protected:
    frUInt4 numX, numY;
    frCollection<frCollection<frCollection<frUInt4> > > supply;
    frCollection<frCollection<frCollection<frUInt4> > > cmap_through;
    frCollection<frCollection<frCollection<frUInt4> > > cmap_local;
    frCollection<frCollection<frCollection<frUInt4> > > cmap_edge1;
    frCollection<frCollection<frCollection<frUInt4> > > cmap_edge2;
    frCollection<frCollection<frCollection<frUInt4> > > cmap_up;
  };
}

#endif
