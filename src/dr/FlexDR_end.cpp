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

#include "dr/FlexDR.h"

using namespace std;
using namespace fr;

void FlexDRWorker::endGetModNets(set<frNet*, frBlockObjectComp> &modNets) {
  for (auto &net: nets) {
    if (net->isModified()) {
      modNets.insert(net->getFrNet());
    }
  }
  // change modified flag to true if another subnet get routed
  for (auto &net: nets) {
    if (!net->isModified() && modNets.find(net->getFrNet()) != modNets.end()) {
      net->setModified(true);
    }
  }
}

void FlexDRWorker::endRemoveNets_pathSeg(frPathSeg* pathSeg, 
                                         set<pair<frPoint, frLayerNum> > &boundPts) {
  bool enableOutput = false;
  frPoint begin, end;
  pathSeg->getPoints(begin, end);
  auto routeBox = getRouteBox();
  auto net = pathSeg->getNet();
  auto regionQuery = getRegionQuery();
  // vertical seg
  if (begin.x() == end.x()) {
    // if cross routeBBox
    bool condition1 = isInitDR() ? (begin.x() < routeBox.right()) : (begin.x() <= routeBox.right());
    if (routeBox.left() <= begin.x() && condition1 &&
        //!(begin.y() >= routeBox.top() || end.y() <= routeBox.bottom())) {
        !(begin.y() > routeBox.top() || end.y() < routeBox.bottom())) {
      // bottom seg to ext
      if (begin.y() < routeBox.bottom()) {
        auto uPathSeg = make_unique<frPathSeg>(*pathSeg);
        frPoint boundPt(end.x(), min(end.y(), routeBox.bottom()));
        uPathSeg->setPoints(begin, boundPt);
        unique_ptr<frShape> uShape(std::move(uPathSeg));
        auto sptr = uShape.get();
        net->addShape(uShape);
        regionQuery->addDRObj(sptr);
        // add cutSegs
        auto lNum = sptr->getLayerNum();
        //cutSegs[make_pair(boundPt, lNum)].insert(sptr);
        boundPts.insert(make_pair(boundPt, lNum));
        if (enableOutput) {
          cout <<"trim pathseg to ext bottom" <<endl;
        }
      }
      // top seg to ext
      if (end.y() > routeBox.top()) {
        auto uPathSeg = make_unique<frPathSeg>(*pathSeg);
        frPoint boundPt(begin.x(), max(begin.y(), routeBox.top()));
        uPathSeg->setPoints(boundPt, end);
        unique_ptr<frShape> uShape(std::move(uPathSeg));
        auto sptr = uShape.get();
        net->addShape(uShape);
        regionQuery->addDRObj(sptr);
        // add cutSegs
        auto lNum = sptr->getLayerNum();
        //cutSegs[make_pair(boundPt, lNum)].insert(sptr);
        boundPts.insert(make_pair(boundPt, lNum));
        if (enableOutput) {
          cout <<"trim pathseg to ext top" <<endl;
        }
      }
      //std::cout << "  removingPathSeg " << &(*pathSeg) << " (" << begin.x() << ", " << begin.y() 
      //          << ") -- (" << end.x() << ", " << end.y() << ") " << drNet->getName() <<  "\n" << std::flush;
      regionQuery->removeDRObj(pathSeg); // delete rq
      net->removeShape(pathSeg); // delete segment
    }
  // horizontal seg
  } else if (begin.y() == end.y()) {
    // if cross routeBBox
    bool condition1 = isInitDR() ? (begin.y() < routeBox.top()) : (begin.y() <= routeBox.top());
    if (routeBox.bottom() <= begin.y() && condition1 &&
        //!(begin.x() >= routeBox.right() || end.x() <= routeBox.left())) {
        !(begin.x() > routeBox.right() || end.x() < routeBox.left())) {
      // left seg to ext
      if (begin.x() < routeBox.left()) {
        auto uPathSeg = make_unique<frPathSeg>(*pathSeg);
        frPoint boundPt(min(end.x(), routeBox.left()), end.y());
        uPathSeg->setPoints(begin, boundPt);
        unique_ptr<frShape> uShape(std::move(uPathSeg));
        auto sptr = uShape.get();
        net->addShape(uShape);
        regionQuery->addDRObj(sptr);
        // add cutSegs
        auto lNum = sptr->getLayerNum();
        //cutSegs[make_pair(boundPt, lNum)].insert(sptr);
        boundPts.insert(make_pair(boundPt, lNum));
        if (enableOutput) {
          cout <<"trim pathseg to ext left" <<endl;
        }
      }
      // right seg to ext
      if (end.x() > routeBox.right()) {
        auto uPathSeg = make_unique<frPathSeg>(*pathSeg);
        frPoint boundPt(max(begin.x(), routeBox.right()), begin.y());
        uPathSeg->setPoints(boundPt, end);
        unique_ptr<frShape> uShape(std::move(uPathSeg));
        auto sptr = uShape.get();
        net->addShape(uShape);
        regionQuery->addDRObj(sptr);
        // add cutSegs
        auto lNum = sptr->getLayerNum();
        //cutSegs[make_pair(boundPt, lNum)].insert(sptr);
        boundPts.insert(make_pair(boundPt, lNum));
        if (enableOutput) {
          cout <<"trim pathseg to ext right" <<endl;
        }
      }
      regionQuery->removeDRObj(pathSeg); // delete rq
      net->removeShape(pathSeg); // delete segment
    }
  }
}

void FlexDRWorker::endRemoveNets_via(frVia* via) {
  //bool enableOutput = true;
  bool enableOutput = false;
  auto gridBBox = getRouteBox();
  auto regionQuery = getRegionQuery();
  auto net = via->getNet();
  frPoint viaPoint;
  via->getOrigin(viaPoint);
  bool condition1 = isInitDR() ? 
                    (viaPoint.x() <  gridBBox.right() && viaPoint.y() <  gridBBox.top()) : 
                    (viaPoint.x() <= gridBBox.right() && viaPoint.y() <= gridBBox.top());
  if (viaPoint.x() >= gridBBox.left() && viaPoint.y() >= gridBBox.bottom() && condition1) {
    regionQuery->removeDRObj(via); // delete rq
    net->removeVia(via);
    if (enableOutput) {
      cout <<"delete via in route" <<endl;
    }
  }
}

void FlexDRWorker::endRemoveNets_patchWire(frPatchWire* pwire) {
  //bool enableOutput = true;
  bool enableOutput = false;
  auto gridBBox = getRouteBox();
  auto regionQuery = getRegionQuery();
  auto net = pwire->getNet();
  frPoint origin;
  pwire->getOrigin(origin);
  bool condition1 = isInitDR() ? 
                    (origin.x() <  gridBBox.right() && origin.y() <  gridBBox.top()) : 
                    (origin.x() <= gridBBox.right() && origin.y() <= gridBBox.top());
  if (origin.x() >= gridBBox.left() && origin.y() >= gridBBox.bottom() && condition1) {
    regionQuery->removeDRObj(pwire); // delete rq
    net->removePatchWire(pwire);
    if (enableOutput) {
      cout <<"delete pwire in route" <<endl;
    }
  }
}

void FlexDRWorker::endRemoveNets(set<frNet*, frBlockObjectComp> &modNets, 
                                 map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &boundPts) {
  vector<frBlockObject*> result;
  getRegionQuery()->queryDRObj(getRouteBox(), result);
  for (auto rptr: result) {
    if (rptr->typeId() == frcPathSeg) {
      auto cptr = static_cast<frPathSeg*>(rptr);
      if (cptr->hasNet()) {
        if (modNets.find(cptr->getNet()) != modNets.end()) {
          endRemoveNets_pathSeg(cptr, boundPts[cptr->getNet()]);
        }
      } else {
        cout <<"Error: endRemoveNet hasNet() empty" <<endl;
      }
    } else if (rptr->typeId() == frcVia) {
      auto cptr = static_cast<frVia*>(rptr);
      if (cptr->hasNet()) {
        if (modNets.find(cptr->getNet()) != modNets.end()) {
          endRemoveNets_via(cptr);
        }
      } else {
        cout <<"Error: endRemoveNet hasNet() empty" <<endl;
      }
    } else if (rptr->typeId() == frcPatchWire) {
      auto cptr = static_cast<frPatchWire*>(rptr);
      if (cptr->hasNet()) {
        if (modNets.find(cptr->getNet()) != modNets.end()) {
          endRemoveNets_patchWire(cptr);
        }
      } else {
        cout <<"Error: endRemoveNet hasNet() empty" <<endl;
      }
    } else {
      cout <<"Error: endRemoveNets unsupported type" <<endl;
    }
  }
}

void FlexDRWorker::endAddNets_pathSeg(drPathSeg* pathSeg/*, map<pair<frPoint, frLayerNum>, set<frBlockObject*> > &cutSegs*/) {
  //cout <<"add pathseg" <<endl;
  auto net = pathSeg->getNet()->getFrNet();
  /*
  frPoint bp, ep;
  pathSeg->getPoints(bp, ep);
  auto lNum = pathSeg->getLayerNum();
  bool isHorzSeg = (bp.y() == ep.y());
  auto &rbox = getRouteBox();
  //bool merged = false;
  // horizontal segment
  if (isHorzSeg) {
    // check if it can be merged on left boundary
    frPathSeg* mergeWithL = nullptr;
    frPoint    pL;
    frPathSeg* mergeWithR = nullptr;
    frPoint    pR;
    if (bp.x() == rbox.left() && cutSegs.find(make_pair(bp, lNum)) != cutSegs.end()) {
      for (auto &obj: cutSegs.find(make_pair(bp, lNum))->second) {
        if (obj->typeId() == frcPathSeg) {
          auto nbrSeg = static_cast<frPathSeg*>(obj);
          frPoint nbrBp, nbrEp;
          nbrSeg->getPoints(nbrBp, nbrEp);
          if (nbrBp.y() == nbrEp.y()) {
            mergeWithL = nbrSeg;
            pL.set(nbrBp);
            break;
          }
        }
      }
    }
    // check if it can be merged on right boundary
    if (ep.x() == rbox.right() && cutSegs.find(make_pair(ep, lNum)) != cutSegs.end()) {
      for (auto &obj: cutSegs.find(make_pair(ep, lNum))->second) {
        if (obj->typeId() == frcPathSeg) {
          auto nbrSeg = static_cast<frPathSeg*>(obj);
          frPoint nbrBp, nbrEp;
          nbrSeg->getPoints(nbrBp, nbrEp);
          if (nbrBp.y() == nbrEp.y()) {
            mergeWithR = nbrSeg;
            pR.set(nbrEp);
            break;
          }
        }
      }
    }
    if (mergeWithL) {
      bp.set(pL);
      getRegionQuery()->removeDRObj(mergeWithL);
      net->removeShape(mergeWithL);
    }
    if (mergeWithR) {
      ep.set(pR);
      getRegionQuery()->removeDRObj(mergeWithR);
      net->removeShape(mergeWithR);
    }
  // vertical segment
  } else {
    // check if it can be merged on bottom boundary
    frPathSeg* mergeWithB = nullptr;
    frPoint    pB;
    frPathSeg* mergeWithT = nullptr;
    frPoint    pT;
    if (bp.y() == rbox.bottom() && cutSegs.find(make_pair(bp, lNum)) != cutSegs.end()) {
      for (auto &obj: cutSegs.find(make_pair(bp, lNum))->second) {
        if (obj->typeId() == frcPathSeg) {
          auto nbrSeg = static_cast<frPathSeg*>(obj);
          frPoint nbrBp, nbrEp;
          nbrSeg->getPoints(nbrBp, nbrEp);
          if (nbrBp.x() == nbrEp.x()) {
            mergeWithB = nbrSeg;
            pB.set(nbrBp);
            break;
          }
        }
      }
    }
    // check if it can be merged on right boundary
    if (ep.y() == rbox.top() && cutSegs.find(make_pair(ep, lNum)) != cutSegs.end()) {
      for (auto &obj: cutSegs.find(make_pair(ep, lNum))->second) {
        if (obj->typeId() == frcPathSeg) {
          auto nbrSeg = static_cast<frPathSeg*>(obj);
          frPoint nbrBp, nbrEp;
          nbrSeg->getPoints(nbrBp, nbrEp);
          if (nbrBp.x() == nbrEp.x()) {
            mergeWithT = nbrSeg;
            pT.set(nbrEp);
            break;
          }
        }
      }
    }
    if (mergeWithB) {
      bp.set(pB);
      getRegionQuery()->removeDRObj(mergeWithB);
      net->removeShape(mergeWithB);
    }
    if (mergeWithT) {
      ep.set(pT);
      getRegionQuery()->removeDRObj(mergeWithT);
      net->removeShape(mergeWithT);
    }
  }
  pathSeg->setPoints(bp, ep);
  */
  //bool enableOutput = true;
  //if (enableOutput) {
  //  if (pathSeg->getNet()->getFrNet()->getName() == string("net30")) {
  //    frPoint bp, ep;
  //    pathSeg->getPoints(bp, ep);
  //    auto lNum = pathSeg->getLayerNum();
  //    if (lNum == 2 && bp.x() == 294200 && ep.x() == 294200) {
  //      cout <<"found@@@" <<bp <<" " <<ep <<endl;
  //    }
  //  }
  //}
  unique_ptr<frShape> uShape = make_unique<frPathSeg>(*pathSeg);
  auto rptr = uShape.get();
  net->addShape(uShape);
  getRegionQuery()->addDRObj(rptr);
}

void FlexDRWorker::endAddNets_via(drVia* via) {
  auto net = via->getNet()->getFrNet();
  unique_ptr<frVia> uVia = make_unique<frVia>(*via);
  auto rptr = uVia.get();
  net->addVia(uVia);
  //double dbu = getDesign()->getTopBlock()->getDBUPerUU();
  //if (getDRIter() == 2 &&
  //    rptr->getNet()->getName() == string("net221") &&
  //    getRouteBox().left()    == 63     * dbu && 
  //    getRouteBox().right()   == 84     * dbu && 
  //    getRouteBox().bottom()  == 139.65 * dbu && 
  //    getRouteBox().top()     == 159.6  * dbu) { 
  //  auto obj = rptr;
  //  frPoint pt;
  //  obj->getOrigin(pt);
  //  cout <<"write back via (" 
  //       <<pt.x() / dbu <<", " <<pt.y() / dbu <<") "
  //       <<obj->getViaDef()->getName() 
  //       <<endl;
  //}
  getRegionQuery()->addDRObj(rptr);
}

void FlexDRWorker::endAddNets_patchWire(drPatchWire* pwire) {
  auto net = pwire->getNet()->getFrNet();
  unique_ptr<frShape> uShape = make_unique<frPatchWire>(*pwire);
  auto rptr = uShape.get();
  net->addPatchWire(uShape);
  getRegionQuery()->addDRObj(rptr);
}

void FlexDRWorker::endAddNets_merge(frNet* net, set<pair<frPoint, frLayerNum> > &boundPts) {
  vector<rq_rptr_value_t<frBlockObject> > result;
  vector<frBlockObject*> drObjs;
  vector<frPathSeg*> horzPathSegs;
  vector<frPathSeg*> vertPathSegs;
  bool hasPatchMetal = false;
  auto regionQuery = getRegionQuery();
  for (auto &[pt, lNum]: boundPts) {
    hasPatchMetal = false;
    bool skip = false;
    result.clear();
    regionQuery->query(frBox(pt, pt), lNum, result);
    for (auto &[bx, obj]: result) {
      if (obj->typeId() == frcInstTerm) {
        auto instTerm = static_cast<frInstTerm*>(obj);
        if (instTerm->getNet() == net) {
          skip = true;
          break;
        }
      } else if (obj->typeId() == frcTerm) {
        auto term = static_cast<frTerm*>(obj);
        if (term->getNet() == net) {
          skip = true;
          break;
        }
      }
    }
    if (skip) {
      continue;
    }
    // get all drObjs
    drObjs.clear();
    horzPathSegs.clear();
    vertPathSegs.clear();
    regionQuery->queryDRObj(frBox(pt, pt), lNum, drObjs);
    for (auto &obj: drObjs) {
      if (obj->typeId() == frcPathSeg) {
        auto ps = static_cast<frPathSeg*>(obj);
        if (!(ps->getNet() == net)) {
          continue;
        }
        frPoint bp, ep;
        ps->getPoints(bp, ep);
        if (bp == pt || ep == pt) {
          // vertical
          if (bp.x() == ep.x()) {
            vertPathSegs.push_back(ps);
          // horizontal
          } else {
            horzPathSegs.push_back(ps);
          }
        }
      } else if (obj->typeId() == frcPatchWire) {
        auto pwire = static_cast<frPatchWire*>(obj);
        if (!(pwire->getNet() == net)) {
          continue;
        }
        frPoint bp;
        pwire->getOrigin(bp);
        if (bp == pt) {
          hasPatchMetal = true;
          break;
        }
      }
    }
    // merge horz pathseg
    if ((int)horzPathSegs.size() == 2 && vertPathSegs.empty() && !hasPatchMetal) {
      unique_ptr<frShape> uShape = make_unique<frPathSeg>(*horzPathSegs[0]);
      auto rptr = static_cast<frPathSeg*>(uShape.get());
      frPoint bp1, ep1, bp2, ep2;
      horzPathSegs[0]->getPoints(bp1, ep1);
      horzPathSegs[1]->getPoints(bp2, ep2);
      frPoint bp(min(bp1.x(), bp2.x()), bp1.y());
      frPoint ep(max(ep1.x(), ep2.x()), ep1.y());
      rptr->setPoints(bp, ep);
      
      regionQuery->removeDRObj(horzPathSegs[0]);
      regionQuery->removeDRObj(horzPathSegs[1]);
      net->removeShape(horzPathSegs[0]);
      net->removeShape(horzPathSegs[1]);

      net->addShape(uShape);
      regionQuery->addDRObj(rptr);
    }
    if ((int)vertPathSegs.size() == 2 && horzPathSegs.empty() && !hasPatchMetal) {
      unique_ptr<frShape> uShape = make_unique<frPathSeg>(*vertPathSegs[0]);
      auto rptr = static_cast<frPathSeg*>(uShape.get());
      frPoint bp1, ep1, bp2, ep2;
      vertPathSegs[0]->getPoints(bp1, ep1);
      vertPathSegs[1]->getPoints(bp2, ep2);
      frPoint bp(bp1.x(), min(bp1.y(), bp2.y()));
      frPoint ep(ep1.x(), max(ep1.y(), ep2.y()));
      rptr->setPoints(bp, ep);
      
      regionQuery->removeDRObj(vertPathSegs[0]);
      regionQuery->removeDRObj(vertPathSegs[1]);
      net->removeShape(vertPathSegs[0]);
      net->removeShape(vertPathSegs[1]);

      net->addShape(uShape);
      regionQuery->addDRObj(rptr);
    }
  }
}

void FlexDRWorker::endAddNets(map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &boundPts) {
  //bool enableOutput = true;
  for(auto &net: nets) {
    if (!net->isModified()) {
      continue;
    }
    //if (enableOutput) {
    //  if (net->getFrNet()->getName() == string("net30")) {
    //    cout <<"write back net@@@" <<endl;
    //  }
    //}
    //double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    //if (getDRIter() == 2 &&
    //    getRouteBox().left()    == 63     * dbu && 
    //    getRouteBox().right()   == 84     * dbu && 
    //    getRouteBox().bottom()  == 139.65 * dbu && 
    //    getRouteBox().top()     == 159.6  * dbu) { 
    //  cout <<"write back net " <<net->getFrNet()->getName() <<endl;
    //}
    //auto fNet = net->getFrNet();
    //for (auto &connFig: net->getRouteConnFigs()) {
    for (auto &connFig: net->getBestRouteConnFigs()) {
      if (connFig->typeId() == drcPathSeg) {
        endAddNets_pathSeg(static_cast<drPathSeg*>(connFig.get())/*, cutSegs[fNet]*/);
      } else if (connFig->typeId() == drcVia) {
        endAddNets_via(static_cast<drVia*>(connFig.get()));
      } else if (connFig->typeId() == drcPatchWire) {
        endAddNets_patchWire(static_cast<drPatchWire*>(connFig.get()));
      } else {
        cout <<"Error: endAddNets unsupported type" <<endl;
      }
    }
  }
  for (auto &[net, bPts]: boundPts) {
    endAddNets_merge(net, bPts);
  }
}

/*
void FlexDRWorker::endFilterCutSegs(map<frNet*, map<pair<frPoint, frLayerNum>, set<frBlockObject*> > > &cutSegs) {
  vector<rq_rptr_value_t<frBlockObject> > result;
  auto regionQuery = getRegionQuery();
  // remove merging point if there is a pin
  for (auto &[net, mp]: cutSegs) {
    //for (auto &[pr, objS]: mp) {
    for (auto it = mp.begin(); it != mp.end(); ) {
      auto localIt = it;
      ++it;
      auto &pr = localIt->first;
      //auto &objS = localIt->second;
      result.clear();
      auto &[pt, lNum] = pr;
      regionQuery->query(frBox(pt, pt), lNum, result);
      for (auto &[bx, obj]: result) {
        if (obj->typeId() == frcInstTerm) {
          auto instTerm = static_cast<frInstTerm*>(obj);
          if (instTerm->getNet() == net) {
            mp.erase(localIt);
            break;
          }
        } else if (obj->typeId() == frcTerm) {
          auto term = static_cast<frTerm*>(obj);
          if (term->getNet() == net) {
            mp.erase(localIt);
            break;
          }
        }
      }
    }
  }
  // remove merging point if there is a crossing segment 
}
*/

void FlexDRWorker::endRemoveMarkers() {
  auto regionQuery = getRegionQuery();
  auto topBlock = getDesign()->getTopBlock();
  vector<frMarker*> result;
  regionQuery->queryMarker(getDrcBox(), result);
  for (auto mptr: result) {
    regionQuery->removeMarker(mptr);
    topBlock->removeMarker(mptr);
  }
}

void FlexDRWorker::endAddMarkers() {
  auto regionQuery = getRegionQuery();
  auto topBlock = getDesign()->getTopBlock();
  frBox mBox;
  //for (auto &m: getMarkers()) {
  for (auto &m: getBestMarkers()) {
    m.getBBox(mBox);
    if (getDrcBox().overlaps(mBox)) {
      auto uptr = make_unique<frMarker>(m);
      auto ptr = uptr.get();
      regionQuery->addMarker(ptr);
      topBlock->addMarker(uptr);
    }
  }
}

void FlexDRWorker::end() {
  // skip if current clip does not have input DRCs
  // ripupMode = 0 must have enableDRC = true in previous iteration
  if (isEnableDRC() && getRipupMode() == 0 && getInitNumMarkers() == 0) {
    return;
  // do not write back if current clip is worse than input
  } else if (isEnableDRC() && getRipupMode() == 0 && getBestNumMarkers() > getInitNumMarkers()) {
    //cout <<"skip clip with #init/final = " <<getInitNumMarkers() <<"/" <<getNumMarkers() <<endl;
    return;
  } else if (isEnableDRC() && getRipupMode() == 0 && getBestNumMarkers() == getInitNumMarkers()) {
    //cout <<"write back clip with no improvement #final = " <<getNumMarkers() <<endl;
  } else if (isEnableDRC() && getRipupMode() == 0 && getBestNumMarkers() < getInitNumMarkers()) {
    //cout <<"write back clip with #init/final = " <<getInitNumMarkers() <<"/" <<getNumMarkers() <<endl;
  }
  set<frNet*, frBlockObjectComp> modNets;
  endGetModNets(modNets);
  // get lock
  map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> boundPts;
  endRemoveNets(modNets, boundPts);
  //endRemoveNets(modNets);
  //endFilterCutSegs(cutSegs);
  endAddNets(boundPts); // if two subnets have diff isModified() status, then should always write back
  //endAddNets();
  if (isEnableDRC()) {
    endRemoveMarkers();
    endAddMarkers();
  }
  // release lock

}

int FlexDRWorker::getNumQuickMarkers() {
  int totNum = 0;
  for (auto &net: nets) {
    totNum += net->getNumMarkers();
  }
  return totNum;
}
