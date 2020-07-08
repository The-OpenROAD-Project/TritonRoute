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

#ifndef _FR_BLOCKAGE_H_
#define _FR_BLOCKAGE_H_

#include <memory>
#include "frBaseTypes.h"
#include "db/obj/frPin.h"

namespace fr {
  class frBlockage: public frBlockObject {
  public:
    // constructors
    frBlockage(): frBlockObject(), pin(nullptr)/*, spacing(), designRuleWidth()*/ {}
    // getters
    frPin* getPin() const {
      return pin.get();
    }
    //bool hasSpacing(frLayerNum in) const {
    //  return !(spacing.find(in) == spacing.end());
    //}
    //frCoord getSpacing(frLayerNum in) const {
    //  return spacing.find(in)->second;
    //}
    //bool hasDesignRuleWidth(frLayerNum in) const {
    //  return !(designRuleWidth.find(in) == designRuleWidth.end());
    //}
    //frCoord getDesignRuleWidth(frLayerNum in) const {
    //  return designRuleWidth.find(in)->second;
    //}
    // setters
    void setPin(std::unique_ptr<frPin>& in) {
      pin = std::move(in);
    }
    //void setSpacing(frLayerNum lNum, frCoord in) {
    //  spacing[lNum] = in;
    //}
    //void setDesignRuleWidth(frLayerNum lNum, frCoord in) {
    //  designRuleWidth[lNum] = in;
    //}
    // others
    frBlockObjectEnum typeId() const override {
      return frcBlockage;
    }
  protected:
    std::unique_ptr<frPin> pin;
    //std::map<frLayerNum, frCoord> spacing;
    //std::map<frLayerNum, frCoord> designRuleWidth;
  };
  //class frBlockage: public frFig {
  //public:
  //  // constructors
  //  frBlockage(): frFig(), points(), owner(nullptr) {}
  //  // getters
  //  frBlockObject* getOwner() const {
  //    return owner;
  //  }
  //  const std::vector<frPoint>& getPoints() const {
  //    return points;
  //  }
  //  std::vector<frPoint>& getPoints() {
  //    return points;
  //  }
  //  frUInt4 getNumPoints() const {
  //    return points.size();
  //  }
  //  // setters
  //  void setOwner(frBlockObject* in) {
  //    owner = in;
  //  }
  //  void setPoints(const fr::frCollection<frPoint> &pIn) {
  //    points = pIn;
  //  }
  //  // others
  //  frBlockObjectEnum typeId() const override {
  //    return frcBlockage;
  //  }

  //  void getBBox(frBox &boxIn) const override {
  //    frCoord llx = 0;
  //    frCoord lly = 0;
  //    frCoord urx = 0;
  //    frCoord ury = 0;
  //    if (points.size()) {
  //      llx = points.begin()->x();
  //      urx = points.begin()->x();
  //      lly = points.begin()->y();
  //      ury = points.begin()->y();
  //    }
  //    for (auto &point: points) {
  //      llx = (llx < point.x()) ? llx : point.x();
  //      lly = (lly < point.y()) ? lly : point.y();
  //      urx = (urx > point.x()) ? urx : point.x();
  //      ury = (ury > point.y()) ? ury : point.y();
  //    }
  //    boxIn.set(llx, lly, urx, ury);
  //  }
  //  void move(const frTransform &xform) override {
  //    for (auto &point: points) {
  //      point.transform(xform);
  //    }
  //  }
  //  bool overlaps(const frBox &box) const override {
  //    return false;
  //  }
  //protected:
  //  std::vector<frPoint> points;
  //  frBlockObject*       owner; // optional, set later
  //};

  //class frLayerBlockage: public frBlockage {
  //public:
  //  // constructors
  //  frLayerBlockage(): frBlockage() {}
  //  // getters
  //  frLayerNum getLayerNum() const {
  //    return layerNum;
  //  }
  //  // setters
  //  void setLayerNum(frLayerNum numIn) {
  //    layerNum = numIn;
  //  }

  //  // others
  //  frBlockObjectEnum typeId() const override {
  //    return frcLayerBlockage;
  //  }
  //protected:
  //  frLayerNum layerNum;
  //};
}

#endif
