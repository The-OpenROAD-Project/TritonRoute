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

#include "io/frAPG.h"

using namespace boost::polygon::operators;

void fr::APGWorker::init() {
  //bool enableOutput = true;
  for (auto &uinstTerm: inst->getInstTerms()) {
    frInstTerm* instTerm = uinstTerm.get();
    auto instTermType = instTerm->getTerm()->getType();
    if (instTermType != frTermEnum::frcNormalTerm && instTermType != frTermEnum::frcClockTerm) {
      continue;
    }
    // pupulate map and rtree
    for (auto &upin: instTerm->getTerm()->getPins()) {
      bool hasViaAccess = false;
      frPin* pin = upin.get();
      // unConnectedPins.push_back(pin);
      frTransform instXform; 
      inst->getUpdatedXform(instXform);
      for (auto &uap: pin->getAccessPatterns(instXform.orient())) {
        FlexAccessPattern *ap = uap.get();
        if (!(ap->hasInst(inst))) {
          continue;
        }
        if (!ap->getAccessViaDef(frDirEnum::U).empty()) {
          // std::cout << "empty\n";
          hasViaAccess = true;
          pin2APs[pin].push_back(ap);
          frPoint bp, ep;
          ap->getPoints(bp, ep);
          layer2APRTree[ap->getBeginLayerNum()].insert(std::make_pair(point_t(bp.x(), bp.y()), ap));
          // std::cout << "  " << pin->getTerm()->getName() << " insert ap at (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ") on layer " << ap->getBeginLayerNum() << "\n";
        }
      }
      if (hasViaAccess == false) {
        if (true) {
          std::cout << "Warning: " << inst->getName() << "/" << pin->getTerm()->getName() 
                    << " has no via access found during APG. Not considered for APG\n";
        }
      } else {
        unConnectedPins.insert(pin);
      }
    }
  }
  // 

}

// get the pin order
void fr::APGWorker::setup() {
  //bool enableOutput = true;
  auto unProcessedLayer2APRtree = layer2APRTree;
  // first sort the unconnected pins to get the start pin
  // std::set<std::pair<frCoord, frPin*> > areaPinPairs;
  // std::set<std::tuple<frCoord, Point, frPin*> > pinTuples;
  std::set<std::tuple<int, frCoord, Point, frPin*> > pinTuples;
  for (auto &pin: unConnectedPins) {
    frCoord pinArea = 0;
    PolygonSet pinPS;
    auto &layer2PolySet = pin->getLayer2PolySet();
    for (auto layerIt = layer2PolySet.begin(); layerIt != layer2PolySet.end(); ++layerIt) {
      pinPS += layerIt->second;
      pinArea += area(layerIt->second);
    }
    Rectangle pinBBoxRect;
    extents(pinBBoxRect, pinPS);
    Point pinBBoxCenter;
    center(pinBBoxCenter, pinBBoxRect);
    pinTuples.insert(std::make_tuple(pin2APs[pin].size(), pinArea, pinBBoxCenter, pin));
    // areaPinPairs.insert(std::make_pair(pinArea, pin));
  }
  // get the order of pins for AP expansion
  // start from the smallest pin, gradually expand to find the 
  // "bottleneck" of conflicting pins
  // frPin* currPin = areaPinPairs.begin()->second;
  frPin* currPin = std::get<3>(*pinTuples.begin());
  // std::cout << "  " << currPin->getTerm()->getName() << "\n";
  pinOrder.push_back(currPin);
  if (unConnectedPins.find(currPin) == unConnectedPins.end()) {
    std::cout << "Error: not found in unConnectedPins\n";
  }
  unConnectedPins.erase(currPin);
  frPoint bp, ep;
  pin2APs[currPin].front()->getPoints(bp, ep);
  Point apOrigin(bp.x(), bp.y());
  boost::polygon::set_points(apBBoxRect, apOrigin, apOrigin);
  for (auto layerIt = unProcessedLayer2APRtree.begin(); layerIt != unProcessedLayer2APRtree.end(); ++layerIt) {
    auto &apRTree = layerIt->second;
    std::vector<std::pair<point_t, FlexAccessPattern*> > removeBuffer;
    for (auto rtreeIt = apRTree.begin(); rtreeIt != apRTree.end(); ++rtreeIt) {
      auto loc = rtreeIt->first;
      auto ap = rtreeIt->second;
      // remove the ap from rtree if ap has the same pin as the currPin
      if (ap->getPin() == currPin) {
        // apRTree.remove(std::make_pair(loc, ap));
        removeBuffer.push_back(std::make_pair(loc, ap));
        // std::cout << "    removed ap from rtree\n";
      }
    }
    for (auto &element: removeBuffer) {
      apRTree.remove(element);
    }
  }
  // remaining pins
  // std::cout << "  start from second pin\n";
  while (!unConnectedPins.empty()) {
    box_t apBBox(point_t(xl(apBBoxRect), yl(apBBoxRect)), point_t(xh(apBBoxRect), yh(apBBoxRect)));
    frCoord minDist = std::numeric_limits<int>::max();
    Point nextAPLoc;
    // find next pin to add to order
    for (auto layerIt = unProcessedLayer2APRtree.begin(); layerIt != unProcessedLayer2APRtree.end(); ++layerIt) {
      auto &apRTree = layerIt->second;
      if (apRTree.empty()) {
        continue;
      }
      std::vector<std::pair<point_t, FlexAccessPattern*> > qq;
      apRTree.query(bgi::nearest(apBBox, 1), std::back_inserter(qq));
      for (auto &apPair: qq) {
        auto tmpDist = bg::distance(apBBox, apPair.first);
        if (tmpDist < minDist) {
          minDist = tmpDist;
          currPin = (apPair.second)->getPin();
          nextAPLoc = Point(apPair.first.x(), apPair.first.y());
        }
      }
    }
    // std::cout << "  " << currPin->getTerm()->getName() << "\n";

    // add the closest pin to order and update unconnected info
    pinOrder.push_back(currPin);
    unConnectedPins.erase(currPin);
    boost::polygon::encompass(apBBoxRect, nextAPLoc);
    for (auto layerIt = unProcessedLayer2APRtree.begin(); layerIt != unProcessedLayer2APRtree.end(); ++layerIt) {
      auto &apRTree = layerIt->second;
      if (apRTree.empty()) {
        continue;
      }
      std::vector<std::pair<point_t, FlexAccessPattern*> > removeBuffer;
      for (auto rtreeIt = apRTree.begin(); rtreeIt != apRTree.end(); ++rtreeIt) {
        auto loc = rtreeIt->first;
        auto ap = rtreeIt->second;
        // remove the ap from rtree if ap has the same pin as the currPin
        if (ap->getPin() == currPin) {
          // apRTree.remove(std::make_pair(loc, ap));
          removeBuffer.push_back(std::make_pair(loc, ap));
          // std::cout << "    removed ap from rtree\n";
        }
      }
      for (auto &element: removeBuffer) {
        apRTree.remove(element);
      }
    }
    // 

  }

}

// according to pin order, find the best AP combination and find other usable AP (regarding to best combination)
void fr::APGWorker::main() {
  //bool enableOutput = true;
  auto unProcessedLayer2APRtree = layer2APRTree;
  frVector<FlexAccessPattern*> currBestAPComb;
  frVector<frPoint> apBps;
  std::vector<std::unique_ptr<frVia> > currObjs;
  std::set<FlexAccessPattern*> currValidAPs;
  FlexAccessPattern* bestAP;
  // boost::polygon::center(prevAPLoc, apBBoxRect);
  // get best APC
  for (auto &currPin: pinOrder) {
    int bestVioCnt = -1;
    std::map<FlexAccessPattern*, frCoord> ap2MinDist;
    if (apBps.empty()) {
      Point apBBoxRectCenter;
      boost::polygon::center(apBBoxRectCenter, apBBoxRect);
      for (auto &ap: pin2APs[currPin]) {
        frPoint bp, ep;
        ap->getPoints(bp, ep);
        Point apBp(bp.x(), bp.y());
        frCoord tmpDist = boost::polygon::euclidean_distance(apBp, apBBoxRectCenter);
        ap2MinDist[ap] = tmpDist;
      }
    } else {
      for (auto &ap: pin2APs[currPin]) {
        for (auto &bap: currBestAPComb) {
          frPoint bp, ep, bbp, bep;
          ap->getPoints(bp, ep);
          bap->getPoints(bbp, bep);
          Point apBp(bp.x(), bp.y());
          Point bapBp(bbp.x(), bbp.y());
          frCoord tmpDist = boost::polygon::euclidean_distance(apBp, bapBp);
          if (ap2MinDist.find(ap) == ap2MinDist.end() || tmpDist < ap2MinDist[ap]) {
            ap2MinDist[ap] = tmpDist;
          }
        }
      }
    }
    //
    std::vector<std::pair<frCoord, FlexAccessPattern*> > distAPs;
    for (auto it = ap2MinDist.begin(); it != ap2MinDist.end(); ++it) {
      distAPs.push_back(std::make_pair(it->second, it->first));
    }
    auto apComp = [](const std::pair<frCoord, FlexAccessPattern*> &a, const std::pair<frCoord, FlexAccessPattern*> &b) {
                    frPoint bp1, ep1, bp2, ep2;
                    a.second->getPoints(bp1, ep1);
                    b.second->getPoints(bp2, ep2);
                    return (a.first == b.first) ? (bp1 < bp2) : (a.first < b.first);
                  };
    std::sort(distAPs.begin(), distAPs.end(), apComp);
    // greedily start check from the nearest one, find the one with smallest cost and proceed
    // for (auto &distAP: distAPs) {
    for (int i = 0; i < (int)distAPs.size(); ++i) {
      auto &ap = distAPs[i].second;
      std::vector<frBlockObject*> tmpObjs;
      for (auto &obj: currObjs) {
        tmpObjs.push_back(obj.get());
      }
      std::unique_ptr<frVia> tmpVia = std::make_unique<frVia>((ap->getAccessViaDef(frDirEnum::U).front()));
      frPoint bp, ep;
      ap->getPoints(bp, ep);
      tmpVia->setOrigin(bp);
      frTransform viaXform(bp);
      tmpVia->setTransform(viaXform);
      tmpVia->addToPin(currPin);
      tmpObjs.push_back(tmpVia.get());
      DRCWorker drcWorker(design, tmpObjs);
      drcWorker.addIgnoredConstraintType(frConstraintTypeEnum::frcAreaConstraint);
      drcWorker.init();
      drcWorker.setup();
      // drcWorker.main();
      drcWorker.check();
      if (drcWorker.getViolations().empty()) {
        bestVioCnt = 0;
        bestAP = ap;
      } else {
        int numVio = drcWorker.getViolations().size();
        if (bestVioCnt == -1 || numVio < bestVioCnt) {
          bestVioCnt = numVio;
          bestAP = ap;
        }
      }
      // update if clean ap found or all ap visited
      if (bestVioCnt == 0 || (i + 1) == (int)distAPs.size()) {
        apBps.push_back(bp);
        currBestAPComb.push_back(bestAP);
        currObjs.push_back(std::move(tmpVia));
        break;
      }

    }

  }

  for (auto &ap: currBestAPComb) {
    frPoint bp, ep;
    ap->getPoints(bp, ep);
    ap->setPreferred(true);
    std::cout << "  ap in bestAPComb: (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
    currValidAPs.insert(ap);
  }

  // find all other APs which are not conflicting with best APC
  for (auto it = pin2APs.begin(); it != pin2APs.end(); ++it) {
    auto &pin = it->first;
    auto &aps = it->second;
    std::vector<frBlockObject*> otherObjs;
    // get the aps in APC except the current one
    for (auto &via: currObjs) {
      if (via->hasPin() && via->getPin() != pin) {
        otherObjs.push_back(via.get());
      }
    }
    // check ap of the current pin one by one
    for (auto &ap: aps) {
      auto tmpObjs = otherObjs;
      std::unique_ptr<frVia> tmpVia = std::make_unique<frVia>(*(ap->getAccessViaDef(frDirEnum::U).begin()));
      frPoint bp, ep;
      ap->getPoints(bp, ep);
      tmpVia->setOrigin(bp);
      frTransform viaXform(bp);
      tmpVia->setTransform(viaXform);
      tmpVia->addToPin(pin);
      tmpObjs.push_back(tmpVia.get());
      DRCWorker drcWorker(design, tmpObjs);
      drcWorker.init();
      drcWorker.setup();
      // drcWorker.main();
      drcWorker.check();
      if (drcWorker.getViolations().empty()) {
        currValidAPs.insert(ap);
      }
    }
  }

  bestAPComb = currBestAPComb;
  validAPs = currValidAPs;

  updateAPStatus();

}

void fr::APGWorker::updateAPStatus() {
  for (auto it = pin2APs.begin(); it != pin2APs.end(); ++it) {
    auto &aps = it->second;
    for (auto &ap: aps) {
      if (validAPs.find(ap) != validAPs.end()) {
        ap->setConflict(false);
      }
    }
  }
}

void fr::APGWorker::end() {
  for (auto &uinstTerm: inst->getInstTerms()) {
    frInstTerm* instTerm = uinstTerm.get();
    auto instTermType = instTerm->getTerm()->getType();
    if (instTermType != frTermEnum::frcNormalTerm && instTermType != frTermEnum::frcClockTerm) {
      continue;
    }
    for (auto &upin: instTerm->getTerm()->getPins()) {
      frPin* pin = upin.get();
      frTransform instXform; 
      inst->getUpdatedXform(instXform);
      auto &aps = pin->getAccessPatterns(instXform.orient());
      std::sort(aps.begin(), aps.end(), APComp());
    }
  }
}
