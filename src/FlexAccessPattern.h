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

#ifndef _FLEX_ACCESS_PATTERN_H
#define _FLEX_ACCESS_PATTERN_H

#include "frBaseTypes.h"
#include "db/obj/frShape.h"
#include "db/obj/frVia.h"
#include <memory>

namespace fr {
  class FlexAccessPattern: public frBlockObject {
  public:
    FlexAccessPattern(): beginPoint(),beginLayerNum(0), pin(nullptr), insts(), 
                         validAccess(std::vector<bool>(6, false)), accessViaDefs(std::vector<std::vector<frViaDef*> >(2)), 
                         apCost(std::numeric_limits<int>::max()), nearestAPDist(0), conflict(true), preferred(false) {}
    FlexAccessPattern(const FlexAccessPattern &in): beginPoint(in.beginPoint), beginLayerNum(in.beginLayerNum),
                                                    pin(in.pin), net(in.net), insts(in.insts), validAccess(in.validAccess),
                                                    accessViaDefs(in.accessViaDefs), apCost(in.apCost), nearestAPDist(in.nearestAPDist), 
                                                    conflict(in.conflict), preferred(in.preferred) {}
    // deprecated
    //FlexAccessPattern(const FlexAccessPattern &in, const frTransform &xform) {
    //  frPoint bpIn, epIn;
    //  in.getPoints(bpIn, epIn);
    //  bpIn.transform(xform);
    //  epIn.transform(xform);
    //  beginPoint.set(bpIn);
    //  endPoint.set(epIn);
    //  beginLayerNum = in.getBeginLayerNum();
    //  endLayerNum = in.getEndLayerNum();
    //  //APCost = in.getCost();
    //  //pathsegs = in.getPathSegs();
    //  //for (auto pathSeg: in.getPathSegs()) {
    //  //  pathSeg.move(xform);
    //  //  pathsegs.push_back(pathSeg);
    //  //}
    //  //// vias = in.getVias();
    //  //for (auto via: in.getVias()) {
    //  //  via.move(xform);
    //  //  vias.push_back(via);
    //  //}
    //  pin = in.getPin();
    //  net = in.getNet();
    //  insts = in.getInsts();
    //  validAccess = in.getValidAccess();
    //  accessViaDefs = in.getAccessViaDefs();
    //}

    // getters
    //void getPoint(frPoint &in) const {
    //  in.set(beginPoint);
    //}
    void getPoints(frPoint &bpIn, frPoint &epIn) const {
      bpIn.set(beginPoint);
      //epIn.set(endPoint);
    }
    frLayerNum getBeginLayerNum() const {
      return beginLayerNum;
    }
    //frLayerNum getEndLayerNum() const {
    //  return endLayerNum;
    //}
    //frCost getCost() const {
    //  return APCost;
    //}
    //frList<frPathSeg>& getPathSegs() {
    //  return pathsegs;
    //}
    //const frList<frPathSeg>& getPathSegs() const {
    //  return pathsegs;
    //}
    //frList<frVia>& getVias() {
    //  return vias;
    //}
    //const frList<frVia>& getVias() const {
    //  return vias;
    //}
    frPin* getPin() const {
      return pin;
    }
    frNet* getNet() const {
      return net;
    }
    const std::set<frInst*>& getInsts() const {
      return insts;
    }
    std::set<frInst*>& getInsts() {
      return insts;
    }

    const std::vector<bool>& getValidAccess() const {
      return validAccess;
    }
    std::vector<bool>& getValidAccess() {
      return validAccess;
    }

    const std::vector<std::vector<frViaDef*> >& getAccessViaDefs() const {
      return accessViaDefs;
    }
    std::vector<std::vector<frViaDef*> >& getAccessViaDefs() {
      return accessViaDefs;
    }

    int getCost() {
      return apCost;
    }

    frCoord getNearestAPDist() {
      return nearestAPDist;
    }

    bool hasInst(frInst* inst) const {
      return (!(insts.find(inst) == insts.end()));
    }

    bool hasValidAccess(const frDirEnum &dir) {
      switch (dir) {
        case (frDirEnum::E):
          return validAccess[0];
          break;
        case (frDirEnum::S):
          return validAccess[1];
          break;
        case (frDirEnum::W):
          return validAccess[2];
          break;
        case (frDirEnum::N):
          return validAccess[3];
          break;
        case (frDirEnum::U):
          return validAccess[4];
          break;
        case (frDirEnum::D):
          return validAccess[5];
          break;
        default:
          return false;
      }
    }

    bool hasValidAccess() {
      return (validAccess[0] || validAccess[1] || validAccess[2] || validAccess[3] || validAccess[4] || validAccess[5]);
    }

    // frViaDef* getAccessViaDef(const frDirEnum &dir) {
    //   if (dir ==  frDirEnum::U) {
    //     return accessViaDefs[0];
    //   } else if (dir == frDirEnum::D) {
    //     return accessViaDefs[1];
    //   } else {
    //     return nullptr;
    //   }
    // }
    bool isConflict() {
      return conflict;
    }

    bool isPreferred() {
      return preferred;
    }

    std::vector<frViaDef*>& getAccessViaDef(const frDirEnum &dir) {
      if (dir == frDirEnum::U) {
        return accessViaDefs[0];
      } else {
        return accessViaDefs[1];
      }
    }
    
    const std::vector<frViaDef*>& getAccessViaDef(const frDirEnum &dir) const {
      if (dir == frDirEnum::U) {
        return accessViaDefs[0];
      } else {
        return accessViaDefs[1];
      }
    }

    // setters
    void setPoints(const frPoint &bpIn, const frPoint &epIn) {
      beginPoint.set(bpIn);
      //endPoint.set(epIn);
    }
    void setBeginLayerNum(frLayerNum in) {
      beginLayerNum = in;
    }
    //void setEndLayerNum(frLayerNum in) {
    //  endLayerNum = in;
    //}
    //void setCost(frCost in) {
    //  APCost = in;
    //}

    void setValidAccess(const frDirEnum &dir, const bool isValid) {
      switch (dir) {
        case (frDirEnum::E):
          validAccess[0] = isValid;
          break;
        case (frDirEnum::S):
          validAccess[1] = isValid;
          break;
        case (frDirEnum::W):
          validAccess[2] = isValid;
          break;
        case (frDirEnum::N):
          validAccess[3] = isValid;
          break;
        case (frDirEnum::U):
          validAccess[4] = isValid;
          break;
        case (frDirEnum::D):
          validAccess[5] = isValid;
          break;
        default:
          std::cout << "Error: unexpected direction in setValidAccess\n";
      }
    }

    // void setAccessViaDef(const frDirEnum dir, frViaDef *viaDef) {
    //   if (dir == frDirEnum::U) {
    //     accessViaDefs[0] = viaDef;
    //   } else if (dir == frDirEnum::D) {
    //     accessViaDefs[1] = viaDef;
    //   } else {
    //     std::cout << "Error: unexpected direction in setAccessVia\n";
    //   }
    // }

    void addAccessViaDef(const frDirEnum dir, frViaDef *viaDef) {
      if (dir == frDirEnum::U) {
        accessViaDefs[0].push_back(viaDef);
      } else if (dir == frDirEnum::D) {
        accessViaDefs[1].push_back(viaDef);
      } else {
        std::cout << "Error: unexpected direction in addAccessViaDef\n";
      }
    }


    //frListIter<frPathSeg> addPathSeg(const frPathSeg &in) {
    //  pathsegs.push_back(in);
    //  return (--pathsegs.end());
    //}
    //void removePathSeg(frListIter<frPathSeg> &in);
    //frListIter<frVia> addVia(const frVia &in) {;
    //  vias.push_back(in);
    //  return (--vias.end());
    //}
    //void removeVia(frListIter<frVia> &in);
    void addToPin(frPin* in) {
      pin = in;
    }
    void addToNet(frNet* in) {
      net = in;
    }
    void addInst(frInst *inst) {
      insts.insert(inst);
    }
    void addInsts(std::set<frInst*> instsIn) {
      insts.insert(instsIn.begin(), instsIn.end());
    }
    virtual frBlockObjectEnum typeId() const override {
      return frcAccessPattern;
    }
    void setCost(int in) {
      apCost = in;
    }
    void setNearestAPDist(frCoord in) {
      nearestAPDist = in;
    }
    void setConflict(bool in) {
      conflict = in;
    }
    void setPreferred(bool in) {
      preferred = in;
    }

  protected:
    frPoint beginPoint;
    frLayerNum beginLayerNum;
    frPin* pin;
    frNet* net;
    std::set<frInst*>    insts;
    std::vector<bool> validAccess; // 0 = E, 1 = S, 2 = W, 3 = N, 4 = U, 5 = D
    // std::vector<frViaDef*> accessViaDefs; // 0 = U, 1 = D
    std::vector<std::vector<frViaDef*> > accessViaDefs;
    int apCost;
    frCoord nearestAPDist;
    bool conflict, preferred;
  };

  struct APComp {
    bool operator()(const std::unique_ptr<FlexAccessPattern> &a, const std::unique_ptr<FlexAccessPattern> &b) {
      if (a->isPreferred() && !b->isPreferred()) {
        return true;
      } else if (!a->isPreferred() && b->isPreferred()) {
        return false;
      } else {
        if (!a->isConflict() && b->isConflict()) {
          return true;
        } else if (a->isConflict() && !b->isConflict()) {
          return false;
        } else {
          return (a.get() < b.get());
        }
      }
    }
  };
}




#endif
