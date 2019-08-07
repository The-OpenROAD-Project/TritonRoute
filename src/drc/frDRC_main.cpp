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

#include "drc/frDRC.h"
#include "io/frShapeUtil.h"
#include <memory>
#include <iterator>
#include <iostream>
#include <math.h>
#include <fstream>
#include <chrono>
#include <set>

using namespace std::chrono;
using namespace boost::polygon::operators;

void fr::DRCWorker::check() {
  targetNetId = -1;
  resetMarkers();
  lastMarkerNum = uMarkers.size();
  main();
  // clearMarkerRTree();
  // buildMarkerRTree();
}

void fr::DRCWorker::check(frBlockObject *obj) {
  targetNetId = getNetId(obj);
  if (targetNetId != -1) {
    removeMarkers(targetNetId);
    lastMarkerNum = uMarkers.size();
    main();
    // int currMarkerNum = uMarkers.size();
    // for (int i = lastMarkerNum; i < currMarkerNum; ++i) {
    //   auto &umarker = uMarkers[i];
    //   auto marker = umarker.get();
    //   updateMarkerRTree(marker);
    // }
  } else {
    std::cout << "Error: net not found in DRC data model. Rules not checked...\n";
  }
}

void fr::DRCWorker::main() {
  checkMetalShort();
  checkCutShort();
  checkMetalSpacingRules();
  //checkMetalMinAreaRule();
  checkMetalMinStepRule();
  checkMinWidthRule();
  checkCutSpacingRules();
}

void fr::DRCWorker::checkMetalMinAreaRule() {
  if (ignoredConstraintTypes.find(frConstraintTypeEnum::frcAreaConstraint) != ignoredConstraintTypes.end()) {
    return;
  }
  for (auto &drcNet: drcNets) {
    checkMetalMinAreaRule_net(drcNet.get());
  }
}

void fr::DRCWorker::checkMetalMinAreaRule_net(DRCNet* drcNet) {
  std::set<frLayerNum> layerNums;
  auto &layer2FixedPolySet = drcNet->getLayer2FixedPolySet();
  auto &layer2RoutePolySet = drcNet->getLayer2RoutePolySet();
  // std::map<frLayerNum, PolygonSet> layer2MergedPolySet;
  for (auto layerIt = layer2FixedPolySet.begin(); layerIt != layer2FixedPolySet.end(); ++layerIt) {
    layerNums.insert(layerIt->first);
  }
  for (auto layerIt = layer2RoutePolySet.begin(); layerIt != layer2RoutePolySet.end(); ++layerIt) {
    layerNums.insert(layerIt->first);
  }
  for (auto layerNum: layerNums) {
    auto minAreaConstraint = design->getTech()->getLayer(layerNum)->getAreaConstraint();
    if (minAreaConstraint == nullptr) {
      continue;
    }
    frCoord layerMinArea = minAreaConstraint->getMinArea();
    PolygonSet mergedPS;
    mergedPS += layer2FixedPolySet[layerNum];
    mergedPS += layer2RoutePolySet[layerNum];

    // check if minArea is violated

    for (auto &poly: mergedPS) {
      if (area(poly) < layerMinArea) {
        std::vector<Rectangle> maxRects;
        PolygonSet tempPS;
        tempPS += poly;
        bool isCoveredByFixed = true;
        get_max_rectangles(maxRects, tempPS);
        Rectangle minMaxRect;
        frCoord minMaxRectArea = -1;
        for (auto &maxRect: maxRects) {
          frCoord tempMinMaxRectArea = area(maxRect);
          if (minMaxRectArea == -1 || tempMinMaxRectArea < minMaxRectArea) {
            minMaxRectArea = tempMinMaxRectArea;
            minMaxRect = maxRect;
          }
          if (isCoveredByFixed) {
            box_t maxBox(point_t(xl(maxRect), yl(maxRect)), point_t(xh(maxRect), yh(maxRect)));
            std::vector<std::pair<box_t, int> > fixedQueryResult;
            layer2FixedRTree[layerNum].query(bgi::intersects(maxBox), back_inserter(fixedQueryResult));
            bool isMaxRectCoveredByFixed = false;
            for (auto &fixedPair: fixedQueryResult) {
              auto fixedPairNetId = fixedPair.second;
              auto &fixedBox = fixedPair.first;
              if (fixedPairNetId != drcNet->getId()) {
                continue;
              }
              if (bg::covered_by(maxBox, fixedBox)) {
                isMaxRectCoveredByFixed = true;
                break;
              }
            }
            if (!isMaxRectCoveredByFixed) {
              isCoveredByFixed = false;
            }
          } else {
            continue;
          }

        }
        // if not covered by fixed, then it is a violation
        if (!isCoveredByFixed) {
          // push to violation
          // frMarker minAreaMarker;
          auto minAreaMarker = std::make_unique<frMarker>();
          frBox minAreaBBox(xl(minMaxRect), yl(minMaxRect), xh(minMaxRect), yh(minMaxRect));
          minAreaMarker->setConstraint(minAreaConstraint);
          minAreaMarker->setBBox(minAreaBBox);
          minAreaMarker->setLayerNum(layerNum);
          minAreaMarker->addSrc(drcNet->getSrc());
          minAreaMarker->addSrcId(drcNet->getId());
          uMarkers.push_back(std::move(minAreaMarker));
        }

      }
    }

  }



}

void fr::DRCWorker::checkMetalMinStepRule() {
  // std::cout << "checkMetalMinStepRule\n";
  for (auto &drcNet: drcNets) {
    // std::cout << "  check " << drcNet->getId() << "\n";
    if (targetNetId != -1 && drcNet.get()->getId() != targetNetId) {
      continue;
    }
    checkMetalMinStepRule_net(drcNet.get());
  }
}

void fr::DRCWorker::checkMetalMinStepRule_net(DRCNet* drcNet) {
  // auto layer2Seg2Edge = drcNet->getLayerSegEdgeMap();
  for (auto &uedge: drcNet->getDRCEdges()) {
    auto edge = uedge.get();
    frLayerNum layerNum = edge->getLayerNum();
    // auto &seg2Edge = layerIt->second;
    auto constraint = design->getTech()->getLayer(layerNum)->getMinStepConstraint();
    if (constraint) {
      // std::cout << "check minStep" << constraint->hasInsideCorner() << " " << constraint->hasOutsideCorner()
      //           << " " << constraint->hasStep() << "\n";
      frCoord minStepLength = constraint->getMinStepLength();
      // begin check
      if (edge->getNextEdge() == nullptr || edge->getPrevEdge() == nullptr) {
        continue;
      }
      // minStep check starts with an edge longer than minStepLength
      if (length(edge->getSegment()) < minStepLength) {
        continue;
      }
      // skip if next edge is also longer than minStepLength
      if (length(edge->getNextEdge()->getSegment()) >= minStepLength) {
        continue;
      }
      // push potential violating edges into a queue and find next non-violating edge
      frVector<Segment> segs;
      frVector<DRCEdge*> edges;
      frCoord sumLength = 0;
      auto edgePtr = edge;
      while (length(edgePtr->getNextEdge()->getSegment()) < minStepLength) {
        segs.push_back(edgePtr->getNextEdge()->getSegment());
        edges.push_back(edgePtr->getNextEdge());
        sumLength += length(edgePtr->getNextEdge()->getSegment());
        edgePtr = edgePtr->getNextEdge();
      }
      edgePtr = edgePtr->getNextEdge();
      // edgePtr points to the last edge
      // skip if there is only one edge longer than minStepLength
      if (edgePtr->getNextEdge() == edge) {
        // std::cout << "single edge skip\n";
        continue;
      }
      bool checkMinStep = false;
      auto startSeg = edge->getSegment();
      auto endSeg = edgePtr->getSegment();
      if (constraint->hasMaxEdges()) {
        checkMinStep = true;
      } else if (constraint->hasInsideCorner() || constraint->hasOutsideCorner()) {
        // parallel continue
        if (orientation(startSeg, endSeg) == 0) {
          // std::cout << "parallel skip\n";
          continue;
        }
        if (constraint->hasInsideCorner()) {
          if ((edgePtr->getWinding() == boost::polygon::HIGH && orientation(edge->getSegment(), endSeg) == 1) || 
              (edgePtr->getWinding() == boost::polygon::LOW && orientation(edge->getSegment(), endSeg) == -1)) {
            checkMinStep = true;
          }
          // for (int i = 0; i < (int)segs.size() - 1; ++i) {
          //   if ((orientation(segs[i], segs[i+1]) == 1 && edges[i]->getWinding() == boost::polygon::LOW) ||
          //       (orientation(segs[i], segs[i+1]) == -1 && edges[i]->getWinding() == boost::polygon::HIGH)) {
          //     checkMinStep = true;
          //   }
          // }
        } else {
          if ((edgePtr->getWinding() == boost::polygon::LOW && orientation(edge->getSegment(), endSeg) == 1) || 
              (edgePtr->getWinding() == boost::polygon::HIGH && orientation(edge->getSegment(), endSeg) == -1)) {
            checkMinStep = true;
          }
          // for (int i = 0; i < (int)segs.size() - 1; ++i) {
          //   if (orientation(segs[i], segs[i+1]) == -1 && edges[i]->getWinding() == boost::polygon::LOW ||
          //       orientation(segs[i], segs[i+1]) == 1 && edges[i]->getWinding() == boost::polygon::HIGH) {
          //     checkMinStep = true;
          //   }
          // }
        }
      } else {
        if (orientation(startSeg, endSeg) != 0) {
          continue;
        }
      }
      if (!checkMinStep) {
        continue;
      }
      // check maxEdges
      if (constraint->hasMaxEdges() && (int)segs.size() <= constraint->getMaxEdges()) {
        // std::cout << "maxEdges skip\n";
        continue;
      }
      // check maxLength
      if (constraint->hasMaxLength() && constraint->getMaxLength() >= sumLength) {
        // std::cout << "maxLength skip\n";
        continue;
      }
      // potential violation, a real violation contains at least one non-fixed edge
      bool hasNonFixed = false;
      for (auto currEdgePtr = edge; currEdgePtr->getPrevEdge() != edgePtr; currEdgePtr = currEdgePtr->getNextEdge()) {
        if (currEdgePtr->getType() == drcEdgeTypeEnum::ROUTE) {
          hasNonFixed = true;
          break;
        }
      }
      if (hasNonFixed) {
        // push violation
        // frMarker minStepMarker;
        auto minStepMarker = std::make_unique<frMarker>();
        Point lowPt = std::min(high(startSeg), low(endSeg));
        Point highPt = std::max(high(startSeg), low(endSeg));
        frBox minStepBBox(x(lowPt), y(lowPt), x(highPt), y(highPt));
        minStepMarker->setConstraint(constraint);
        minStepMarker->setBBox(minStepBBox);
        minStepMarker->setLayerNum(layerNum);
        minStepMarker->addSrc(drcNet->getSrc());
        minStepMarker->addSrcId(drcNet->getId());

        uMarkers.push_back(std::move(minStepMarker));
        // std::cout << "start seg: (" << x(low(startSeg)) / 2000.0 << ", " << y(low(startSeg)) / 2000.0 << ") -- ("
        //           << x(high(startSeg)) / 2000.0 << ", " << y(high(startSeg)) / 2000.0 
        //           << ", end seg (" << x(low(endSeg)) / 2000.0 << ", " << y(low(endSeg)) / 2000.0 << ") -- ("
        //           << x(high(endSeg)) / 2000.0 << ", " << y(high(endSeg)) / 2000.0 << ")\n";
      } else {
        // std::cout << "fixed error, skipped\n";
      }
    }
  }
}


void fr::DRCWorker::checkMinWidthRule() {
  for (auto &drcNet: drcNets) {
    if (targetNetId != -1 && targetNetId != drcNet.get()->getId()) {
      continue;
    } 
    checkMinWidthRule_net(drcNet.get());
  }
}

// this assumes minWidth violation only happens to overlap between
void fr::DRCWorker::checkMinWidthRule_net(DRCNet* drcNet) {
  std::set<frLayerNum> fixedLayers, routeLayers;
  std::vector<frLayerNum> commonLayers;
  auto &layer2FixedPolySet = drcNet->getLayer2FixedPolySet();
  auto &layer2RoutePolySet = drcNet->getLayer2RoutePolySet();
  for (auto layerIt = layer2FixedPolySet.begin(); layerIt != layer2FixedPolySet.end(); ++layerIt) {
    fixedLayers.insert(layerIt->first);
  }
  for (auto layerIt = layer2RoutePolySet.begin(); layerIt != layer2RoutePolySet.end(); ++layerIt) {
    routeLayers.insert(layerIt->first);
  }
  std::set_intersection(fixedLayers.begin(), fixedLayers.end(), routeLayers.begin(), routeLayers.end(), std::back_inserter(commonLayers));
  for (auto &layerNum: commonLayers) {
    frCoord minWidth = design->getTech()->getLayer(layerNum)->getMinWidth();
    auto minWidthConstraint = design->getTech()->getLayer(layerNum)->getMinWidthConstraint();
    // PolygonSet ovlpPS;
    auto &fixedPolySet = layer2FixedPolySet[layerNum];
    auto &routePolySet = layer2RoutePolySet[layerNum];
    auto ovlpPS = fixedPolySet & routePolySet;
    std::vector<Rectangle> maxRects;
    get_max_rectangles(maxRects, ovlpPS);
    for (auto &rect: maxRects) {
      auto actWidth = rectDiagLength(rect);
      if (actWidth < minWidth) {
        // frMarker minWidthMarker;
        auto minWidthMarker = std::make_unique<frMarker>();
        frBox minWidthVioBox(xl(rect), yl(rect), xh(rect), yh(rect));
        minWidthMarker->setConstraint(minWidthConstraint);
        minWidthMarker->setBBox(minWidthVioBox);
        minWidthMarker->addSrc(drcNet->getSrc());
        minWidthMarker->addSrcId(drcNet->getId());
        uMarkers.push_back(std::move(minWidthMarker));
      }
    }

  }
}

void fr::DRCWorker::checkCutShort() {
  for (auto layerIt = layer2CutRectRTree.begin(); layerIt != layer2CutRectRTree.end(); ++layerIt) {
    frLayerNum layerNum = layerIt->first;
    if (design->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::CUT) {
      continue;
    }

    frShortConstraint* shortConstraint = design->getTech()->getLayer(layerNum)->getShortConstraint();

    if (shortConstraint == nullptr) {
      std::cout << "Error: short constraint is not defined on layer " << layerNum << "\n";
      continue;
    }

    auto &cutRectRTree = layerIt->second;
    for (auto it = cutRectRTree.begin(); it != cutRectRTree.end(); ++it) {
      box_t objBox = it->first;
      int objNetId = it->second;
      if (targetNetId != -1 && targetNetId != objNetId) {
        continue;
      }
      std::vector<std::pair<box_t, int> > queryResult;
      cutRectRTree.query(bgi::intersects(objBox), back_inserter(queryResult));
      for (auto &shortPair: queryResult) {
        if (shortPair.second == objNetId && bg::equals(shortPair.first, objBox)) {
          continue;
        }
        int shortNetId = shortPair.second;
        // just to remove compile warning
        box_t shortArea = box_t();
        bg::intersection(objBox, shortPair.first, shortArea);
        std::vector<std::pair<box_t, int> > fixedQueryResult;
        layer2FixedRTree[layerNum].query(bgi::intersects(shortArea), back_inserter(fixedQueryResult));
        bool inFixedObj1 = false, inFixedObj2 = false;
        for (auto &fixedPair: fixedQueryResult) {
          auto fixedPairNetId = fixedPair.second;
          if (!inFixedObj1 && fixedPairNetId == objNetId && bg::covered_by(shortArea, fixedPair.first)) {
            inFixedObj1 = true;
          }
          if (!inFixedObj2 && fixedPairNetId == shortNetId && bg::covered_by(shortArea, fixedPair.first)) {
            inFixedObj2 = true;
          }
          if (inFixedObj1 && inFixedObj2) {
            break;
          }
        }
        if (!(inFixedObj1 && inFixedObj2)) {
          // push to violation
          // frMarker shortMarker;
          auto shortMarker = std::make_unique<frMarker>();
          frBox shortBBox(shortArea.min_corner().get<0>(), 
                          shortArea.min_corner().get<1>(), 
                          shortArea.max_corner().get<0>(), 
                          shortArea.max_corner().get<1>());
          shortMarker->setConstraint(shortConstraint);
          shortMarker->setBBox(shortBBox);
          shortMarker->setLayerNum(layerNum);
          // shortMarker.addNet(drcNets[objNetId]->getNet());
          // shortMarker.addNet(drcNets[shortNetId]->getNet());
          shortMarker->addSrc(drcNets[objNetId]->getSrc());
          shortMarker->addSrcId(objNetId);
          shortMarker->addSrc(drcNets[shortNetId]->getSrc());
          shortMarker->addSrcId(shortNetId);

          uMarkers.push_back(std::move(shortMarker));

        }
        else {
          // std::cout << "shortArea completely in fixed shape\n";
        }
      }
    }


  }
}

void fr::DRCWorker::checkMetalShort() {
  // double dbu = design->getTech()->getDBUPerUU();
  for (auto layerIt = layer2MergedRTree.begin(); layerIt != layer2MergedRTree.end(); ++layerIt) {
    frLayerNum layerNum = layerIt->first;
    if (design->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }

    frShortConstraint* shortConstraint = design->getTech()->getLayer(layerNum)->getShortConstraint();

    if (shortConstraint == nullptr) {
      std::cout << "Error: short constraint is not defined on layer " << layerNum << "\n";
      continue;
    }
    // std::cout << "check short on layer" << layerNum <<"\n";
    auto &mergedRTree = layerIt->second;
    for (auto it = mergedRTree.begin(); it != mergedRTree.end(); ++it) {
      box_t objBox = it->first;
      int objNetId = it->second;
      if (targetNetId != -1 && targetNetId != objNetId) {
        continue;
      }
      // std::cout << "check maxRect (" << objBox.min_corner().get<0>() / dbu << ", " << objBox.min_corner().get<1>() / dbu 
      //           << ") -- (" << objBox.max_corner().get<0>() / dbu << ", " << objBox.max_corner().get<1>() / dbu << ") net " << objNetId << "\n";
      std::vector<std::pair<box_t, int> > queryResult;
      mergedRTree.query(bgi::intersects(objBox), back_inserter(queryResult));
      for (auto &shortPair: queryResult) {
        // skip same net
        if (shortPair.second == objNetId) {
          continue;
        }
        // std::cout << "  against diffnet maxRect" << shortPair.first.min_corner().get<0>() / dbu << ", " << shortPair.first.min_corner().get<1>() / dbu 
        //         << ") -- (" << shortPair.first.max_corner().get<0>() / dbu << ", " << shortPair.first.max_corner().get<1>() / dbu << ") net " << shortPair.second << "\n";
        int shortNetId = shortPair.second;
        // just to remove compile warning
        box_t shortArea = box_t();
        bg::intersection(objBox, shortPair.first, shortArea);
        std::vector<std::pair<box_t, int> > fixedQueryResult;
        layer2FixedRTree[layerNum].query(bgi::intersects(shortArea), back_inserter(fixedQueryResult));
        bool inFixedObj1 = false, inFixedObj2 = false;
        for (auto &fixedPair: fixedQueryResult) {
          auto fixedPairNetId = fixedPair.second;
          if (!inFixedObj1 && fixedPairNetId == objNetId && bg::covered_by(shortArea, fixedPair.first)) {
            inFixedObj1 = true;
          }
          if (!inFixedObj2 && fixedPairNetId == shortNetId && bg::covered_by(shortArea, fixedPair.first)) {
            inFixedObj2 = true;
          }
          if (inFixedObj1 && inFixedObj2) {
            break;
          }
        }
        if (!(inFixedObj1 && inFixedObj2)) {
          // push to violation
          // frMarker shortMarker;
          auto shortMarker = std::make_unique<frMarker>();
          frBox shortBBox(shortArea.min_corner().get<0>(), 
                          shortArea.min_corner().get<1>(), 
                          shortArea.max_corner().get<0>(), 
                          shortArea.max_corner().get<1>());
          shortMarker->setConstraint(shortConstraint);
          shortMarker->setBBox(shortBBox);
          shortMarker->setLayerNum(layerNum);
          // shortMarker.addNet(drcNets[objNetId]->getNet());
          // shortMarker.addNet(drcNets[shortNetId]->getNet());
          shortMarker->addSrc(drcNets[objNetId]->getSrc());
          shortMarker->addSrcId(objNetId);
          shortMarker->addSrc(drcNets[shortNetId]->getSrc());
          shortMarker->addSrcId(objNetId);

          uMarkers.push_back(std::move(shortMarker));

        }
        else {
          // std::cout << "shortArea completely in fixed shape\n";
        }
      }
      // end for shortPair
    }
    // end for mergedRTree element
  }
}


void fr::DRCWorker::checkCutSpacingRules() {
  for (auto layerIt = layer2EdgeRTree.begin(); layerIt != layer2EdgeRTree.end(); ++layerIt) {
    frLayerNum layerNum = layerIt->first;
    auto currLayer = design->getTech()->getLayer(layerNum);
    if (currLayer->getType() != frLayerTypeEnum::CUT) {
      continue;
    }
    // conventional cut spacing rule
    if (currLayer->hasCutSpacing()) {
      for (auto constraint: currLayer->getCutConstraint()) {
        if (constraint->getCenterToCenter()) {
          std::cout << "Warning: center-to-center to be implemented\n";
          continue;
        }

        checkCutSpacingRule(layer2EdgeRTree[layerNum], constraint);


      }
    }
  }
}

void fr::DRCWorker::checkCutSpacingRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,
                                        frCutSpacingConstraint* constraint) {
  // double dbu = design->getTech()->getDBUPerUU();
  for (auto segEdgeIt = edgeRTree.begin(); segEdgeIt != edgeRTree.end(); ++segEdgeIt) {
    frCoord spacingVal = constraint->getCutSpacing();
    auto &seg = segEdgeIt->first;
    Rectangle segSpanRect;
    auto &edge = segEdgeIt->second;
    auto edgeDir = edge->getDir();
    box_t testBox;
    getSpacingTableTestBox(seg, edge, spacingVal, testBox);
    std::vector<std::pair<segment_t, DRCEdge*> > edgeQueryResult;
    getEdgeSpanRect(seg, edgeDir, edge->getSpan(), segSpanRect);
    if (edge->getPrevEdge() == nullptr || edge->getNextEdge() == nullptr) {
      continue;
    }
    // skip if triggering condition is not met
    //bool hasOBS = false;
    //bool isAdjCut = false;
    // adjacent cuts
    if (constraint->getAdjacentCuts() != -1) {
      int adjCutCnt = -1; // to exclude self count
      frCoord cutWithin = constraint->getCutWithin();
      Rectangle adjCutQueryRect = segSpanRect;
      adjCutQueryRect = bloat(adjCutQueryRect, cutWithin);
      box_t adjCutQueryBox = box_t(point_t(xl(adjCutQueryRect), yl(adjCutQueryRect)), point_t(xh(adjCutQueryRect), yh(adjCutQueryRect)));
      box_t adjCutBox = box_t(point_t(xl(segSpanRect), yl(segSpanRect)), point_t(xh(segSpanRect), yh(segSpanRect)));
      std::vector<std::pair<box_t, int> > adjCutQueryResult;
      layer2MergedRTree[edge->getLayerNum()].query(bgi::intersects(adjCutQueryBox), back_inserter(adjCutQueryResult));

      for (auto &adjCutBoxPair: adjCutQueryResult) {
        auto boxDist = frBox2BoxDist(adjCutBoxPair.first, adjCutBox);
        if (boxDist < cutWithin) {
          // cut OBS always violates adjcut as long as it's within
          if (edge->getNetId() == int(drcSpecialNetIdEnum::OBS) || adjCutBoxPair.second == int(drcSpecialNetIdEnum::OBS)) {
            adjCutCnt += 5;
            //isAdjCut = true;
            //hasOBS = true;
            spacingVal = cutWithin; // < cut OBS within always results in violation
          } else {
            ++adjCutCnt;
          }
        }
      }

      if (adjCutCnt < constraint->getAdjacentCuts()) {
        continue;
      }
    }
    // second layer
    if (constraint->getSecondLayerName().empty()) {
      edgeRTree.query(bgi::intersects(testBox), back_inserter(edgeQueryResult));
    } else {
      auto secondLayer = design->getTech()->getLayer(constraint->getSecondLayerName());
      if (secondLayer == nullptr) {
        std::cout << "Error: second layer not found\n";
        return;
      }
      layer2EdgeRTree[secondLayer->getLayerNum()].query(bgi::intersects(testBox), back_inserter(edgeQueryResult));
    }    
    // get potential violation
    for (auto &querySegEdge: edgeQueryResult) {
      auto &querySeg = querySegEdge.first;
      auto &queryEdge = querySegEdge.second;
      int netId1 = edge->getNetId();
      int netId2 = queryEdge->getNetId();
      if (targetNetId != -1 && netId1 != targetNetId && netId2 != targetNetId) {
        continue;
      }
      // only check polygon edge
      if (queryEdge->getPrevEdge() == nullptr || queryEdge->getNextEdge() == nullptr) {
        continue;
      }
      // only opposite-dir edges are counted as spacing violation
      if (!isOppositeDir(edgeDir, queryEdge->getDir())) {
        continue;
      }
      Rectangle queryEdgeSpanRect;
      getEdgeSpanRect(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanRect);
      bool isHorizontal = (edgeDir == frDirEnum::N || edgeDir == frDirEnum::S);
      frCoord prl = -1;
      frCoord actSpacing;
      Interval segIntv, querySegIntv;
      // Rectangle tmpSegSpanRect = segSpanRect;
      Rectangle tmpQueryEdgeSpanRect = queryEdgeSpanRect;
      Rectangle gapRect = generalized_intersect(tmpQueryEdgeSpanRect, segSpanRect);
      if (isHorizontal) {
        segIntv = horizontal(segSpanRect);
        querySegIntv = horizontal(queryEdgeSpanRect);
      } else {
        segIntv = vertical(segSpanRect);
        querySegIntv = vertical(queryEdgeSpanRect);
      }
      // std::cout << "   segIntv: " << low(segIntv) << " -- " << high(segIntv) << "\n";
      // std::cout << "   querySegIntv: " << low(querySegIntv) << " -- " << high(querySegIntv) << "\n";
      if (intersects(segIntv, querySegIntv)) {
        prl = delta(generalized_intersect(querySegIntv, segIntv));
        if (isHorizontal) {
          actSpacing = (yh(gapRect) - yl(gapRect));
        } else {
          actSpacing = (xh(gapRect) - xl(gapRect));
        }
      } else {
        prl = -delta(generalized_intersect(querySegIntv, segIntv));
        actSpacing = rectDiagLength(gapRect);
      }
      // skip if PARALLELOVERLAP specified but not met
      if (constraint->getParallelOverlap()) {
        // negative / zero prl does not trigger
        if (prl <= 0) {
          continue;
        }
        // enclosure
        frCoord bboxLlx, bboxLly, bboxUrx, bboxUry;
        bboxLlx = std::min(xl(segSpanRect), xl(queryEdgeSpanRect));
        bboxLly = std::min(yl(segSpanRect), yl(queryEdgeSpanRect));
        bboxUrx = std::max(xh(segSpanRect), xh(queryEdgeSpanRect));
        bboxUry = std::max(yh(segSpanRect), yh(queryEdgeSpanRect));
        box_t cutBboxQueryBox(point_t(bboxLlx, bboxLly), point_t(bboxUrx, bboxUry));
        std::vector<std::pair<box_t, int> > adjCutQueryResultLower, adjCutQueryResultUpper;
        bool cutCovered = false;
        layer2MergedRTree[edge->getLayerNum() - 1].query(bgi::intersects(cutBboxQueryBox), back_inserter(adjCutQueryResultLower));
        for (auto &metalPair: adjCutQueryResultLower) {
          if (bg::covered_by(cutBboxQueryBox, metalPair.first)) {
            cutCovered = true;
            break;
          }
        }
        layer2MergedRTree[edge->getLayerNum() + 1].query(bgi::intersects(cutBboxQueryBox), back_inserter(adjCutQueryResultUpper));
        for (auto &metalPair: adjCutQueryResultUpper) {
          if (bg::covered_by(cutBboxQueryBox, metalPair.first)) {
            cutCovered = true;
            break;
          }
        }
        if (cutCovered) {
          continue;
        }

      }
      if (actSpacing < spacingVal) {
        // skip on violation with both edge fixed
        if (edge->getType() == drcEdgeTypeEnum::FIXED && queryEdge->getType() == drcEdgeTypeEnum::FIXED) {
        
        } else {
          bool isVio = true;

          // if the violation box is completely covered by rect of either involved net, then it is not considered as a violation
          box_t cutVioQueryBox(point_t(xl(gapRect), yl(gapRect)), point_t(xh(gapRect), yh(gapRect)));
          std::vector<std::pair<box_t, int> > cutQueryResult;
          auto &mergedRTree = layer2MergedRTree[edge->getLayerNum()];
          mergedRTree.query(bgi::intersects(cutVioQueryBox), back_inserter(cutQueryResult));
          int netId1 = edge->getNetId();
          int netId2 = queryEdge->getNetId();

          for (auto &cutVioPair: cutQueryResult) {
            if (cutVioPair.second != netId1 && cutVioPair.second != netId2) {
              continue;
            }
            if (bg::covered_by(cutVioQueryBox, cutVioPair.first)) {
              isVio = false;
              break;
            }
          }
          if (isVio) {
            // frMarker cutVioMarker;
            auto cutVioMarker = std::make_unique<frMarker>();
            frBox cutVioBox(xl(gapRect), yl(gapRect), xh(gapRect), yh(gapRect));
            cutVioMarker->setConstraint(constraint);
            cutVioMarker->setBBox(cutVioBox);
            cutVioMarker->setLayerNum(edge->getLayerNum());
            cutVioMarker->addSrc(drcNets[edge->getNetId()]->getSrc());
            cutVioMarker->addSrcId(edge->getNetId());
            cutVioMarker->addSrc(drcNets[queryEdge->getNetId()]->getSrc());
            cutVioMarker->addSrcId(queryEdge->getNetId());

            uMarkers.push_back(std::move(cutVioMarker));
          }

        }
      }

    }


  }
}


void fr::DRCWorker::checkMetalSpacingRules() {
  for (auto layerIt = layer2EdgeRTree.begin(); layerIt != layer2EdgeRTree.end(); ++layerIt) {
    frLayerNum layerNum = layerIt->first;
    auto currLayer = design->getTech()->getLayer(layerNum);
    if (currLayer->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    // for (auto constraint: design->getTech()->getLayer(layerNum)->getConstraints()) {
    //   switch (constraint->typeId()) {
    //     case frConstraintTypeEnum::frcSpacingTableConstraint:
    //       checkMetalSpacingTableRule(layer2EdgeRTree[layerNum], (std::static_pointer_cast<frSpacingTableConstraint>(constraint)).get());
    //       break;
    //     case frConstraintTypeEnum::frcSpacingEndOfLineConstraint:
    //       checkMetalSpacingEndOfLineRule(layer2EdgeRTree[layerNum], (std::static_pointer_cast<frSpacingEndOfLineConstraint>(constraint)).get());
    //       break;
    //     default:
    //       continue;
    //   }
    // }

    // min spacing rule
    if (currLayer->hasMinSpacing()) {
      auto constraint = currLayer->getMinSpacing();
      switch (constraint->typeId()) {
        case frConstraintTypeEnum::frcSpacingTablePrlConstraint:
          // std::cout << "check prl spacing\n";
          checkMetalSpacingTablePrlRule(layer2EdgeRTree[layerNum], static_cast<frSpacingTablePrlConstraint*>(constraint));
          break;
        default:
          std::cout << "Warning: Unsupported MinSpacing rule\n";
      }
    }
    if (currLayer->hasEolSpacing()) {
      for (auto constraint: currLayer->getEolSpacing()) {
        // std::cout << "check eol spacing\n";
        switch (constraint->typeId()) {
          case frConstraintTypeEnum::frcSpacingEndOfLineConstraint:
            checkMetalSpacingEndOfLineRule(layer2EdgeRTree[layerNum], static_cast<frSpacingEndOfLineConstraint*>(constraint));
            break;
          default:
            std::cout << "Warning: Unsupported EOL rule\n";
        }
      }
    }
  }
}

void fr::DRCWorker::checkMetalSpacingTablePrlRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,  
                                                  frSpacingTablePrlConstraint* constraint) {
  // double dbu = design->getTech()->getDBUPerUU();
  auto prlConstraint = constraint->getLookupTbl();
  frCoord maxSpacingVal = prlConstraint.find(prlConstraint.getRows().back(), prlConstraint.getCols().back());
  for (auto segEdgeIt = edgeRTree.begin(); segEdgeIt != edgeRTree.end(); ++segEdgeIt) {
    auto &seg = segEdgeIt->first;
    Rectangle segSpanRect;
    auto &edge = segEdgeIt->second;
    auto edgeDir = edge->getDir();
    box_t testBox;
    getSpacingTableTestBox(seg, edge, maxSpacingVal, testBox);
    std::vector<std::pair<segment_t, DRCEdge*> > edgeQueryResult;
    edgeRTree.query(bgi::intersects(testBox), back_inserter(edgeQueryResult));
    getEdgeSpanRect(seg, edgeDir, edge->getSpan(), segSpanRect);
    // std::cout << "  check edge (" << bg::get<0,0>(seg) / dbu << ", " << bg::get<0,1>(seg) / dbu
    //             << ") -- (" << bg::get<1,0>(seg) / dbu << ", " << bg::get<1,1>(seg) / dbu
    //             << ") (" << int(edge->getDir()) << "), type = " << (int)edge->getType() << ", queryBox = (" 
    //             << bg::get<bg::min_corner, 0>(testBox) / dbu << ", " << bg::get<bg::min_corner, 1>(testBox) / dbu << ") -- ("
    //             << bg::get<bg::max_corner, 0>(testBox) / dbu << ", " << bg::get<bg::max_corner, 1>(testBox) / dbu << ")\n";
    for (auto &querySegEdge: edgeQueryResult) {
      auto &querySeg = querySegEdge.first;
      auto &queryEdge = querySegEdge.second;
      int netId1 = edge->getNetId();
      int netId2 = queryEdge->getNetId();
      if (targetNetId != -1 && targetNetId != netId1 && targetNetId != netId2) {
        continue;
      }
      // only opposite-dir edges are counted as prl violation
      if (!isOppositeDir(edgeDir, queryEdge->getDir())) {
        continue;
      }
      box_t queryEdgeSpanBox;
      getEdgeSpanBox(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanBox);
      // std::cout << "    against edge (" << bg::get<0,0>(querySeg) / dbu << ", " << bg::get<0,1>(querySeg) / dbu
      //           << ") -- (" << bg::get<1,0>(querySeg) / dbu << ", " << bg::get<1,1>(querySeg) / dbu
      //           << ") (" << int(edge->getDir()) << "), type = " << (int)queryEdge->getType() << ", spanBox = (" 
      //           << bg::get<bg::min_corner, 0>(queryEdgeSpanBox) / dbu << ", " << bg::get<bg::min_corner, 1>(queryEdgeSpanBox) / dbu << ") -- ("
      //           << bg::get<bg::max_corner, 0>(queryEdgeSpanBox) / dbu << ", " << bg::get<bg::max_corner, 1>(queryEdgeSpanBox) / dbu << ")\n";
      if (!bg::intersects(queryEdgeSpanBox, testBox)) {
        continue;
      } else {
        box_t overlapBox;
        bg::intersection(queryEdgeSpanBox, testBox, overlapBox);
        if (bg::area(overlapBox) == 0) {
          continue;
        }
      }
      // there might be violation, do exact check
      // frCoord effWidth = std::max(std::min(frCoord(bg::length(querySeg)), queryEdge->getSpan()), std::min(frCoord(bg::length(seg)), edge->getSpan()));
      frCoord queryEdgeWidth = queryEdge->getWidth();
      frCoord edgeWidth = edge->getWidth();
      // minspacing obs
      if (USEMINSPACING_OBS && queryEdge->getNetId() == int(drcSpecialNetIdEnum::OBS)) {
        queryEdgeWidth = design->getTech()->getLayer(queryEdge->getLayerNum())->getWidth();
      }
      if (USEMINSPACING_OBS && edge->getNetId() == int(drcSpecialNetIdEnum::OBS)) {
        edgeWidth = design->getTech()->getLayer(edge->getLayerNum())->getWidth();
      }

      // frCoord effWidth = std::max(queryEdge->getWidth(), edge->getWidth());
      frCoord effWidth = std::max(queryEdgeWidth, edgeWidth);
      Rectangle queryEdgeSpanRect;
      getEdgeSpanRect(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanRect);
      // std::cout << "     double-check " << xl(queryEdgeSpanRect) << " " << yl(queryEdgeSpanRect) << " " << xh(queryEdgeSpanRect) << " " << yh(queryEdgeSpanRect) << "\n";
      bool isHorizontal = (edgeDir == frDirEnum::N || edgeDir == frDirEnum::S);
      frCoord prl = -1;
      frCoord actSpacing;
      Interval segIntv, querySegIntv;
      // Rectangle tmpSegSpanRect = segSpanRect;
      Rectangle tmpQueryEdgeSpanRect = queryEdgeSpanRect;
      Rectangle gapRect = generalized_intersect(tmpQueryEdgeSpanRect, segSpanRect);
      if (isHorizontal) {
        segIntv = horizontal(segSpanRect);
        querySegIntv = horizontal(queryEdgeSpanRect);
      } else {
        segIntv = vertical(segSpanRect);
        querySegIntv = vertical(queryEdgeSpanRect);
      }
      // std::cout << "   segIntv: " << low(segIntv) << " -- " << high(segIntv) << "\n";
      // std::cout << "   querySegIntv: " << low(querySegIntv) << " -- " << high(querySegIntv) << "\n";
      if (intersects(segIntv, querySegIntv)) {
        prl = delta(generalized_intersect(querySegIntv, segIntv));
        if (isHorizontal) {
          actSpacing = (yh(gapRect) - yl(gapRect));
        } else {
          actSpacing = (xh(gapRect) - xl(gapRect));
        }
      } else {
        prl = -delta(generalized_intersect(querySegIntv, segIntv));
        actSpacing = rectDiagLength(gapRect);
      }
      // Rectangle gapRect = generalized_intersect(queryEdgeSpanRect, segSpanRect);
      // frCoord prl = (edgeDir == frDirEnum::N || edgeDir == frDirEnum::S) ? (xh(gapRect) - xl(gapRect)) : (yh(gapRect) - yl(gapRect));
      // frCoord actSpacing = (edgeDir == frDirEnum::N || edgeDir == frDirEnum::S) ? (yh(gapRect) - yl(gapRect)) : (xh(gapRect) - xl(gapRect));
      frCoord reqSpacing = prlConstraint.find(effWidth, prl);
      
      // std::cout << "  checking between edge (" << bg::get<0,0>(seg) / dbu << ", " << bg::get<0,1>(seg) / dbu
      //           << ") -- (" << bg::get<1,0>(seg) / dbu << ", " << bg::get<1,1>(seg) / dbu
      //           << ") (" << int(edge->getDir()) << ") and edge (" << bg::get<0,0>(querySeg) / dbu << ", " << bg::get<0,1>(querySeg) / dbu
      //           << ") -- (" << bg::get<1,0>(querySeg) / dbu << ", " << bg::get<1,1>(querySeg) / dbu << ") (" << int(queryEdge->getDir()) << "), "
      //           << "reqSpacing = " << reqSpacing << ", actSpacing = " << actSpacing <<"\n";
          



      // there is a violation

      if (actSpacing < reqSpacing) {
        // skip on violation with both edge fixed
        if (edge->getType() == drcEdgeTypeEnum::FIXED && queryEdge->getType() == drcEdgeTypeEnum::FIXED) {
        
        } else {
          bool isVio = true;
          // std::cout << "      violation\n";
          // std::cout << "prl violation between edge (" << bg::get<0,0>(seg) / dbu << ", " << bg::get<0,1>(seg) / dbu
          //           << ") -- (" << bg::get<1,0>(seg) / dbu << ", " << bg::get<1,1>(seg) / dbu
          //           << ") (" << int(edge->getDir()) << ") and edge (" << bg::get<0,0>(querySeg) / dbu << ", " << bg::get<0,1>(querySeg) / dbu
          //           << ") -- (" << bg::get<1,0>(querySeg) / dbu << ", " << bg::get<1,1>(querySeg) / dbu << ") (" << int(queryEdge->getDir()) << ")\n";
          // std::cout << "  prl = " << prl << ", width = " << effWidth << ", actSpacing = " << actSpacing << ", reqSpacing = " << reqSpacing << "\n";          
          // std::cout << "    segSpan = " << edge->getSpan() / dbu << ", queryEdgeSpan = " << queryEdge->getSpan() / dbu << "\n";
          // std::cout << "    segSpanRect = (" << xl(segSpanRect) / dbu << ", " << yl(segSpanRect) / dbu << ") -- (" << xh(segSpanRect) / dbu << ", " << yh(segSpanRect) / dbu << ")\n";
          // std::cout << "    queryEdgeSpanRect = (" << xl(queryEdgeSpanRect) / dbu << ", " << yl(queryEdgeSpanRect) / dbu << ") -- (" << xh(queryEdgeSpanRect) / dbu << ", " << yh(queryEdgeSpanRect) / dbu << ")\n";
          
          // if the violation box is completely covered by rect of either involved net, then it is not considered as a violation
          box_t prlVioQueryBox(point_t(xl(gapRect), yl(gapRect)), point_t(xh(gapRect), yh(gapRect)));
          std::vector<std::pair<box_t, int> > prlQueryResult;
          auto &mergedRTree = layer2MergedRTree[edge->getLayerNum()];
          mergedRTree.query(bgi::intersects(prlVioQueryBox), back_inserter(prlQueryResult));
          int netId1 = edge->getNetId();
          int netId2 = queryEdge->getNetId();

          for (auto &prlVioPair: prlQueryResult) {
            if (prlVioPair.second != netId1 && prlVioPair.second != netId2) {
              continue;
            }
            if (bg::covered_by(prlVioQueryBox, prlVioPair.first)) {
              isVio = false;
              break;
            }
          }

          if (isVio) {
            // frMarker prlVioMarker;
            auto prlVioMarker = std::make_unique<frMarker>();
            frBox prlVioBox(xl(gapRect), yl(gapRect), xh(gapRect), yh(gapRect));
            prlVioMarker->setConstraint(constraint);
            prlVioMarker->setBBox(prlVioBox);
            prlVioMarker->setLayerNum(edge->getLayerNum());
            // prlVioMarker.addNet(drcNets[edge->getNetId()]->getNet());
            // prlVioMarker.addNet(drcNets[queryEdge->getNetId()]->getNet());
            prlVioMarker->addSrc(drcNets[edge->getNetId()]->getSrc());
            prlVioMarker->addSrcId(edge->getNetId());
            prlVioMarker->addSrc(drcNets[queryEdge->getNetId()]->getSrc());
            prlVioMarker->addSrcId(queryEdge->getNetId());
            prlVioMarker->setHasDir(true);
            prlVioMarker->setIsH(isHorizontal);
            uMarkers.push_back(std::move(prlVioMarker));
          }
        }
      }
    }
  }
}

void fr::DRCWorker::checkMetalSpacingEndOfLineRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,  
                                                   frSpacingEndOfLineConstraint* constraint) {
  // double dbu = design->getTech()->getDBUPerUU();
  // frSpacingEndOfLineParallelEdgeConstraint* eolParallelEdgeConstraint = (constraint->hasParallelEdge()) ? constraint->getParallelEdge().get() : nullptr;
  for (auto segEdgeIt = edgeRTree.begin(); segEdgeIt != edgeRTree.end(); ++segEdgeIt) {
    bool isTriggered = !constraint->hasParallelEdge(); // if no parallel edge, always triggered
    bool isTriggeredByRouteObj = false;
    auto &seg = segEdgeIt->first;
    auto eolWithin = constraint->getEolWithin();
    auto eolWidth = constraint->getEolWidth();
    auto eolSpace = constraint->getMinSpacing();
    Rectangle segSpanRect;
    auto &edge = segEdgeIt->second;
    auto edgeDir = edge->getDir();
    box_t testBox, trigBox1, trigBox2; // left / bottom trigger box and right / top trigger box
    // skip if not eol edge
    if (!isEolEdge(edge)) {
      continue;
    }
    // skip if eolWidth not satisfied
    if (frCoord(bg::length(seg)) >= eolWidth) {
      continue;
    }
    getEdgeSpanRect(seg, edgeDir, edge->getSpan(), segSpanRect);    
    // get trigger box if specified
    if (constraint->hasParallelEdge()) {
      bool isLeftTriggered = false, isRightTriggered = false;
      getParallelEdgeBoxs(seg, edge, constraint->getParSpace(), constraint->getParWithin(), eolWithin, trigBox1, trigBox2);
      std::vector<std::pair<segment_t, DRCEdge*> > trigBox1QueryResult, trigBox2QueryResult;
      edgeRTree.query(bgi::intersects(trigBox1), back_inserter(trigBox1QueryResult));
      edgeRTree.query(bgi::intersects(trigBox2), back_inserter(trigBox2QueryResult));
      // check left
      for (auto &querySegEdge: trigBox1QueryResult) {
        auto &querySeg = querySegEdge.first;
        auto &queryEdge = querySegEdge.second;
        // only opposite-dir edges are counted as prl violation
        if (!isOppositeDir(leftEdgeDir(edgeDir), queryEdge->getDir())) {
          continue;
        }
        box_t queryEdgeSpanBox;
        getEdgeSpanBox(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanBox);
        if (!bg::intersects(queryEdgeSpanBox, trigBox1)) {
          continue;
        } else {
          box_t overlapBox;
          bg::intersection(queryEdgeSpanBox, trigBox1, overlapBox);
          if (bg::area(overlapBox) == 0) {
            continue;
          } else {
            isLeftTriggered = true;
            if (queryEdge->getType() == drcEdgeTypeEnum::ROUTE) {
              isTriggeredByRouteObj = true;
            }
          }
        }
        if (isLeftTriggered && isTriggeredByRouteObj) {
          break;
        }
        // left is triggered
        // isLeftTriggered = true;
        // break;
      }
      // check right
      for (auto &querySegEdge: trigBox2QueryResult) {
        auto &querySeg = querySegEdge.first;
        auto &queryEdge = querySegEdge.second;
        // only opposite-dir edges are counted as prl violation
        if (!isOppositeDir(rightEdgeDir(edgeDir), queryEdge->getDir())) {
          continue;
        }
        box_t queryEdgeSpanBox;
        getEdgeSpanBox(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanBox);
        if (!bg::intersects(queryEdgeSpanBox, trigBox2)) {
          continue;
        } else {
          box_t overlapBox;
          bg::intersection(queryEdgeSpanBox, trigBox2, overlapBox);
          if (bg::area(overlapBox) == 0) {
            continue;
          } else {
            isRightTriggered = true;
            if (queryEdge->getType() == drcEdgeTypeEnum::ROUTE) {
              isTriggeredByRouteObj = true;
            }
          }
        }
        if (isRightTriggered && isTriggeredByRouteObj) {
          break;
        }
        // left is triggered
        // isRightTriggered = true;
        // break;
      }
      if ((constraint->hasTwoEdges() && (isRightTriggered && isLeftTriggered)) ||
          (!constraint->hasTwoEdges() && (isRightTriggered || isLeftTriggered)) ) {
        isTriggered = true;
      }
    }

    // check eol spacing if it is triggered
    if (isTriggered) {
      getEOLTestBox(seg, edge, eolSpace, eolWithin, testBox);
      std::vector<std::pair<segment_t, DRCEdge*> > edgeQueryResult;
      edgeRTree.query(bgi::intersects(testBox), back_inserter(edgeQueryResult));
      // std::cout << "  check edge (" << bg::get<0,0>(seg) / dbu << ", " << bg::get<0,1>(seg) / dbu
      //             << ") -- (" << bg::get<1,0>(seg) / dbu << ", " << bg::get<1,1>(seg) / dbu
      //             << ") (" << int(edge->getDir()) << "), type = " << (int)edge->getType() << ", queryBox = (" 
      //             << bg::get<bg::min_corner, 0>(testBox) / dbu << ", " << bg::get<bg::min_corner, 1>(testBox) / dbu << ") -- ("
      //             << bg::get<bg::max_corner, 0>(testBox) / dbu << ", " << bg::get<bg::max_corner, 1>(testBox) / dbu << ")\n"; 
      for (auto &querySegEdge: edgeQueryResult) {
        auto &querySeg = querySegEdge.first;
        auto &queryEdge = querySegEdge.second;
        int netId1 = edge->getNetId();
        int netId2 = queryEdge->getNetId();
        if (targetNetId != -1 && targetNetId != netId1 && targetNetId != netId2) {
          continue;
        }
        // only opposite-dir edges are counted as eol violation
        if (!isOppositeDir(edgeDir, queryEdge->getDir())) {
          continue;
        }
        box_t queryEdgeSpanBox;
        getEdgeSpanBox(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanBox);
        // std::cout << "    against edge (" << bg::get<0,0>(querySeg) / dbu << ", " << bg::get<0,1>(querySeg) / dbu
        //           << ") -- (" << bg::get<1,0>(querySeg) / dbu << ", " << bg::get<1,1>(querySeg) / dbu
        //           << ") (" << int(edge->getDir()) << "), type = " << (int)queryEdge->getType() << ", spanBox = (" 
        //           << bg::get<bg::min_corner, 0>(queryEdgeSpanBox) / dbu << ", " << bg::get<bg::min_corner, 1>(queryEdgeSpanBox) / dbu << ") -- ("
        //           << bg::get<bg::max_corner, 0>(queryEdgeSpanBox) / dbu << ", " << bg::get<bg::max_corner, 1>(queryEdgeSpanBox) / dbu << ")\n";
        if (!bg::intersects(queryEdgeSpanBox, testBox)) {
          continue;
        } else {
          box_t overlapBox;
          bg::intersection(queryEdgeSpanBox, testBox, overlapBox);
          if (bg::area(overlapBox) == 0) {
            continue;
          }
        }
        // it is a violation
        if (edge->getType() == drcEdgeTypeEnum::FIXED && queryEdge->getType() == drcEdgeTypeEnum::FIXED && !isTriggeredByRouteObj) {
          continue;
        }
        // std::cout << "eol violation between edge (" << bg::get<0,0>(seg) / dbu << ", " << bg::get<0,1>(seg) / dbu
        //           << ") -- (" << bg::get<1,0>(seg) / dbu << ", " << bg::get<1,1>(seg) / dbu
        //           << ") and edge (" << bg::get<0,0>(querySeg) / dbu << ", " << bg::get<0,1>(querySeg) / dbu
        //           << ") -- (" << bg::get<1,0>(querySeg) / dbu << ", " << bg::get<1,1>(querySeg) / dbu << ")\n";
        Rectangle queryEdgeSpanRect;
        getEdgeSpanRect(querySeg, queryEdge->getDir(), queryEdge->getSpan(), queryEdgeSpanRect);
        Rectangle gapRect = generalized_intersect(queryEdgeSpanRect, segSpanRect);
        
        bool isNotCoveredByFixedShape = true;

        box_t eolVioQueryBox(point_t(xl(gapRect), yl(gapRect)), point_t(xh(gapRect), yh(gapRect)));
        std::vector<std::pair<box_t, int> > eolQueryResult;
        auto &mergedRTree = layer2MergedRTree[edge->getLayerNum()];
        mergedRTree.query(bgi::intersects(eolVioQueryBox), back_inserter(eolQueryResult));
        

        for (auto &eolVioPair: eolQueryResult) {
          if (eolVioPair.second != netId1 && eolVioPair.second != netId2) {
            continue;
          }
          if (bg::covered_by(eolVioQueryBox, eolVioPair.first)) {
            isNotCoveredByFixedShape = false;
            break;
          }
        }

        if (isNotCoveredByFixedShape || isTriggeredByRouteObj) {
          // frMarker eolVioMarker;
          auto eolVioMarker = std::make_unique<frMarker>();
          frBox eolVioBox(xl(gapRect), yl(gapRect), xh(gapRect), yh(gapRect));
          eolVioMarker->setConstraint(constraint);
          eolVioMarker->setBBox(eolVioBox);
          eolVioMarker->setLayerNum(edge->getLayerNum());
          // eolVioMarker.addNet(drcNets[edge->getNetId()]->getNet());
          // eolVioMarker.addNet(drcNets[queryEdge->getNetId()]->getNet());
          eolVioMarker->addSrc(drcNets[edge->getNetId()]->getSrc());
          eolVioMarker->addSrcId(edge->getNetId());
          eolVioMarker->addSrc(drcNets[queryEdge->getNetId()]->getSrc());
          eolVioMarker->addSrcId(queryEdge->getNetId());
          eolVioMarker->setHasDir(true);
          bool isHorizontal = (edgeDir == frDirEnum::N || edgeDir == frDirEnum::S);
          eolVioMarker->setIsH(isHorizontal);
          uMarkers.push_back(std::move(eolVioMarker));
        }
      }
    }
  }
}

// void fr::DRCWorker::clearMarkerRTree() {
//   layer2MarkerRTree.clear();
// }

// void fr::DRCWorker::buildMarkerRTree() {
//   for (auto &umarker: uMarkers) {
//     auto marker = umarker.get();
//     updateMarkerRTree(marker, true);
//   }
// }

void fr::DRCWorker::resetMarkers() {
  uMarkers.clear();
}

// void fr::DRCWorker::updateMarkerRTree(frMarker *marker, bool isAdd) {
//   auto layerNum = marker->getLayerNum();
//   frBox markerBBox;
//   marker->getBBox(markerBBox);
//   box_t boostb = (point_t(markerBBox.left(), markerBBox.bottom()), point(markerBBox.right(), markerBBox.top()));
//   if (isAdd) {
//     layer2MarkerRTree[layerNum].insert(std::make_pair(boostb, marker));
//   } else {
//     layer2MarkerRTree[layerNum].remove(std::make_pair(boostb, marker));
//   }
// }

// void fr::DRCWorker::removeMarkers(frBlockObject* obj) {
//   switch (obj->typeId()) {
//     case frcNet: 
//     {
//       frNet *net = static_cast<frNet*>(obj);
//       removeMarkers(net);
//       break;
//     }
//     case drcNet:
//     {
//       drNet *net = static_cast<drNet*>(obj);
//       removeMarkers(net);
//       break;
//     }
//     case frcTerm:
//     {
//       frTerm *term = static_cast<frTerm*>(obj);
//       removeMarkers(term);
//       break;
//     }
//     case frcInstTerm:
//     {
//       frInstTerm *instTerm = static_cast<frInstTerm*>(obj);
//       frTerm *term = instTerm->getTerm();
//       removeMarkers(term);
//       break;
//     }
//     default:
//       std::cout << "Error: unexpected type in removeMarker\n";
//   }
// }

// void fr::DRCWorker::removeMarkers(frTerm *term) {
//   std::vector<int> victims;
//   for (int i = 0; i < (int)markers.size(); ++i) {
//     auto &umarker = uMarkers[i];
//     auto &srcs = umarker.getSrcs();
//     if (srcs.find(term) != srcs.end()) {
//       victims.push_back(i);
//     }
//   }
//   // erase violations related to term
//   // remove from both marker vector and marker RTree
//   for (it = victims.rbegin(); it != victims.rend(); ++it) {
//     int idx = *it;
//     auto &umarker = uMarkers[idx];
//     auto marker = umarker.get();
//     // updateMarkerRTree(marker, false);
//     uMarkers.erase(markers.begin() + idx);
//   }
//   lastMarkerNum = uMarkers.size();
// }

// void fr::DRCWorker::removeMarkers(frNet *net) {
//   std::vector<int> victims;
//   for (int i = 0; i < (int)markers.size(); ++i) {
//     auto &marker = markers[i];
//     auto &srcs = marker.getSrcs();
//     if (srcs.find(net) != srcs.end()) {
//       victims.push_back(i);
//     }
//   }
//   // erase violations related to frNet
//   for (it = victims.rbegin(); it != victims.rend(); ++it) {
//     int idx = *it;
//     auto &umarker = uMarkers[idx];
//     auto marker = umarker.get();
//     // updateMarkerRTree(marker, false);
//     uMarkers.erase(uMarkers.begin() + idx);
//   }
//   lastMarkerNum = uMarkers.size();
// }

// void fr::DRCWorker::removeMarkers(drNet *net) {
//   std::vector<int> victims;
//   auto ownerNet = net->getFrNet();
//   for (int i = 0; i < (int)uMarkers.size(); ++i) {
//     auto &umarker = uMarkers[i];
//     auto &srcs = umarker.getSrcs();
//     if (srcs.find(ownerNet) != srcs.end()) {
//       victims.push_back(i);
//     }
//   }
//   // erase violations related to frNet
//   for (it = victims.rbegin(); it != victims.rend(); ++it) {
//     int idx = *it;
//     auto &umarker = uMarkers[idx];
//     auto marker = umarker.get();
//     // updateMarkerRTree(marker, false);
//     uMarkers.erase(uMarkers.begin() + idx);
//   }
//   lastMarkerNum = uMarkers.size();
// }

void fr::DRCWorker::removeMarkers(int netId) {
  std::vector<int> victims;
  for (int i = 0; i < (int)uMarkers.size(); ++i) {
    auto &uMarker = uMarkers[i];
    auto &srcIds = uMarker->getSrcIds();
    if (srcIds.find(netId) != srcIds.end()) {
      victims.push_back(i);
    }
  }
  // erase violations related to frNet
  for (auto it = victims.rbegin(); it != victims.rend(); ++it) {
    int idx = *it;
    // auto &uMarker = uMarkers[idx];
    // auto marker = uMarker.get();
    // updateMarkerRTree(marker, false);
    uMarkers.erase(uMarkers.begin() + idx);
  }
  lastMarkerNum = uMarkers.size();
}
