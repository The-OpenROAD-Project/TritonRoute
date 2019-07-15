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

using namespace std::chrono;

void fr::DRCWorker::printEdgeRTree() {
  double dbu = design->getTech()->getDBUPerUU();
  for (auto mapIt = layer2EdgeRTree.begin(); mapIt != layer2EdgeRTree.end(); ++mapIt) {
    // frLayerNum layerNum = mapIt->first;
    // std::cout << "  layerNum = " << layerNum << "\n";
    auto &edgeRTree = mapIt->second;
    for (auto rtreeIt = edgeRTree.begin(); rtreeIt != edgeRTree.end(); ++rtreeIt) {
      segment_t seg = rtreeIt->first;
      auto edge = rtreeIt->second;
      point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
      point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
      point_t leftPt, rightPt;
      if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
        leftPt = pt2;
        rightPt = pt1;
      } else {
        leftPt = pt1;
        rightPt = pt2;
      }
      frCoord llx = 0, lly = 0, urx = 0, ury = 0;
      llx = bg::get<0>(leftPt);
      urx = bg::get<0>(rightPt);
      lly = bg::get<1>(leftPt);
      ury = bg::get<1>(rightPt);
      std::cout << "    net " << edge->getNetId() << " (" << llx / dbu << ", " << lly / dbu
                << ") - (" << urx / dbu << ", " << ury / dbu << "), dir = " << (int)edge->getDir() 
                << ", edgeType = " << (int)edge->getType() << "\n";
    }
  }
}



void fr::DRCWorker::getEdgeSpanRect(const segment_t &seg, const frDirEnum &dir, const frCoord &span, Rectangle &edgeSpanRect) {
  // double dbu = design->getTech()->getDBUPerUU();
  point_t leftPt, rightPt; // left is the smaller one, right is the larger one, true for vertical as well
  point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
  point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
  if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
    leftPt = pt2;
    rightPt = pt1;
  } else {
    leftPt = pt1;
    rightPt = pt2;
  }
  // std::cout << "  getEdgeSpanRect: from (" << bg::get<0>(leftPt) / dbu<< ", " << bg::get<1>(leftPt) / dbu<< ") to "
  //           << "(" << bg::get<0>(rightPt) / dbu << ", " << bg::get<1>(rightPt) / dbu << ")\n";
  frCoord llx = -1, lly = -1, urx = -1, ury = -1;
  if (dir == frDirEnum::E) {
    urx = bg::get<0>(rightPt);
    llx = urx - span;
    lly = bg::get<1>(leftPt);
    ury = bg::get<1>(rightPt);
  } else if (dir == frDirEnum::S) {
    llx = bg::get<0>(leftPt);
    urx = bg::get<0>(rightPt);
    lly = bg::get<1>(leftPt);
    ury = lly + span;
  } else if (dir == frDirEnum::W) {
    llx = bg::get<0>(leftPt);
    urx = llx + span;
    lly = bg::get<1>(leftPt);
    ury = bg::get<1>(rightPt);
  } else if (dir == frDirEnum::N) {
    llx = bg::get<0>(leftPt);
    urx = bg::get<0>(rightPt);
    ury = bg::get<1>(leftPt);
    lly = ury - span;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getEdgeSpanRect\n";
  }
  // std::cout << "   " << llx << " " << lly << " " << urx << " " << ury << "\n";
  edgeSpanRect = Rectangle(llx, lly, urx, ury);
}

void fr::DRCWorker::getEdgeSpanBox(const segment_t &seg, const frDirEnum &dir, const frCoord &span, box_t &edgeSpanBox) {
  point_t leftPt, rightPt; // left is the smaller one, right is the larger one, true for vertical as well
  point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
  point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
  if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
    leftPt = pt2;
    rightPt = pt1;
  } else {
    leftPt = pt1;
    rightPt = pt2;
  }
  frCoord llx = 0, lly = 0, urx = 0, ury = 0;
  if (dir == frDirEnum::E) {
    urx = bg::get<0>(leftPt);
    llx = urx - span;
    lly = bg::get<1>(leftPt);
    ury = bg::get<1>(rightPt);
  } else if (dir == frDirEnum::S) {
    llx = bg::get<0>(leftPt);
    urx = bg::get<0>(rightPt);
    lly = bg::get<1>(leftPt);
    ury = lly + span;
  } else if (dir == frDirEnum::W) {
    llx = bg::get<0>(leftPt);
    urx = llx + span;
    lly = bg::get<1>(leftPt);
    ury = bg::get<1>(rightPt);
  } else if (dir == frDirEnum::N) {
    llx = bg::get<0>(leftPt);
    urx = bg::get<0>(rightPt);
    ury = bg::get<1>(leftPt);
    lly = ury - span;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getEdgeSpanBox\n";
  }

  edgeSpanBox = box_t(point_t(llx, lly), point_t(urx, ury));
}

bool fr::DRCWorker::isOppositeDir(frDirEnum dir1, frDirEnum dir2) {
  if (dir1 == frDirEnum::E && dir2 == frDirEnum::W || 
      dir1 == frDirEnum::S && dir2 == frDirEnum::N || 
      dir1 == frDirEnum::W && dir2 == frDirEnum::E || 
      dir1 == frDirEnum::N && dir2 == frDirEnum::S) {
    return true;
  } else {
    return false;
  }
}

void fr::DRCWorker::getSpacingTableTestBox(const segment_t &seg, DRCEdge* prlEdge, frCoord maxSpacingVal, box_t &testBox) {
  auto dir = prlEdge->getDir();
  point_t leftPt, rightPt; // left is the smaller one, right is the larger one, true for vertical as well
  point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
  point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
  if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
    leftPt = pt2;
    rightPt = pt1;
  } else {
    leftPt = pt1;
    rightPt = pt2;
  }
  frCoord llx = 0, lly = 0, urx = 0, ury = 0;
  if (dir == frDirEnum::E) {
    llx = bg::get<0>(leftPt);
    urx = llx + maxSpacingVal;
    lly = bg::get<1>(leftPt) - maxSpacingVal;
    ury = bg::get<1>(rightPt) + maxSpacingVal;
  } else if (dir == frDirEnum::S) {
    llx = bg::get<0>(leftPt) - maxSpacingVal;
    urx = bg::get<0>(rightPt) + maxSpacingVal;
    ury = bg::get<1>(leftPt);
    lly = ury - maxSpacingVal;
  } else if (dir == frDirEnum::W) {
    urx = bg::get<0>(leftPt);
    llx = urx - maxSpacingVal;
    lly = bg::get<1>(leftPt) - maxSpacingVal;
    ury = bg::get<1>(rightPt) + maxSpacingVal;
  } else if (dir == frDirEnum::N) {
    llx = bg::get<0>(leftPt) - maxSpacingVal;
    urx = bg::get<0>(rightPt) + maxSpacingVal;
    lly = bg::get<1>(leftPt);
    ury = lly + maxSpacingVal;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getSpacingTableTestBox\n";
  }
  testBox = box_t(point_t(llx, lly), point_t(urx, ury));

}

// left or bottom direction
fr::frDirEnum fr::DRCWorker::leftEdgeDir(const frDirEnum &dirIn) {
  if (dirIn == frDirEnum::N || dirIn == frDirEnum::S) {
    return frDirEnum::W;
  } else {
    return frDirEnum::S;
  }
}

fr::frDirEnum fr::DRCWorker::rightEdgeDir(const frDirEnum &dirIn) {
  if (dirIn == frDirEnum::N || dirIn == frDirEnum::S) {
    return frDirEnum::E;
  } else {
    return frDirEnum::N;
  }
}

bool fr::DRCWorker::isEolEdge(DRCEdge* edge) {
  // if not polygon edge, return false
  if (edge->getPrevEdge() == nullptr || edge->getNextEdge() == nullptr) {
    return false;
  }
  auto prevEdge = edge->getPrevEdge();
  auto nextEdge = edge->getNextEdge();
  // return true iff one of the following is true
  if ((prevEdge->getDir() == frDirEnum::W && edge->getDir() == frDirEnum::N && nextEdge->getDir() == frDirEnum::E) ||
      (prevEdge->getDir() == frDirEnum::S && edge->getDir() == frDirEnum::W && nextEdge->getDir() == frDirEnum::N) || 
      (prevEdge->getDir() == frDirEnum::E && edge->getDir() == frDirEnum::S && nextEdge->getDir() == frDirEnum::W) ||
      (prevEdge->getDir() == frDirEnum::N && edge->getDir() == frDirEnum::E && nextEdge->getDir() == frDirEnum::S)) {
    return true;
  } else {
    return false;
  }
}

// bool fr::DRCWorker::isConvecEdge(DRCEdge* edge) {
//   // if not polygon edge, return false
//   if (edge->getPrevEdge() == nullptr || edge->getNextEdge() == nullptr) {
//     return false;
//   }
//   auto prevEdge = edge->getPrevEdge();
//   auto nextEdge = edge->getNextEdge();
//   // return true iff one of the following is true
//   if ((prevEdge->getDir() == frDirEnum::W && edge->getDir() == frDirEnum::N && nextEdge->getDir() == frDirEnum::E) ||
//       (prevEdge->getDir() == frDirEnum::S && edge->getDir() == frDirEnum::W && nextEdge->getDir() == frDirEnum::N) || 
//       (prevEdge->getDir() == frDirEnum::E && edge->getDir() == frDirEnum::S && nextEdge->getDir() == frDirEnum::W) ||
//       (prevEdge->getDir() == frDirEnum::N && edge->getDir() == frDirEnum::E && nextEdge->getDir() == frDirEnum::S)) {
//     return true;
//   } else {
//     return false;
//   }
// }

// bool fr::DRCWorker::isConcaveEdge(DRCEdge* edge) {
//   // if not polygon edge, return false
//   if (edge->getPrevEdge() == nullptr || edge->getNextEdge() == nullptr) {
//     return false;
//   }
//   auto prevEdge = edge->getPrevEdge();
//   auto nextEdge = edge->getNextEdge();
//   // return true iff one of the following is true
//   if ((prevEdge->getDir() == frDirEnum::E && edge->getDir() == frDirEnum::N && nextEdge->getDir() == frDirEnum::W) ||
//       (prevEdge->getDir() == frDirEnum::N && edge->getDir() == frDirEnum::W && nextEdge->getDir() == frDirEnum::S) || 
//       (prevEdge->getDir() == frDirEnum::W && edge->getDir() == frDirEnum::S && nextEdge->getDir() == frDirEnum::E) ||
//       (prevEdge->getDir() == frDirEnum::S && edge->getDir() == frDirEnum::E && nextEdge->getDir() == frDirEnum::N)) {
//     return true;
//   } else {
//     return false;
//   }
// }

void fr::DRCWorker::getEOLTestBox(const segment_t &seg, DRCEdge* eolEdge, const frCoord &eolSpace, const frCoord &eolWithin, box_t &testBox) {
  auto dir = eolEdge->getDir();
  point_t leftPt, rightPt; // left is the smaller one, right is the larger one, true for vertical as well
  point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
  point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
  if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
    leftPt = pt2;
    rightPt = pt1;
  } else {
    leftPt = pt1;
    rightPt = pt2;
  }
  frCoord llx = 0, lly = 0, urx = 0, ury = 0;
  if (dir == frDirEnum::E) {
    llx = bg::get<0>(leftPt);
    urx = llx + eolSpace;
    lly = bg::get<1>(leftPt) - eolWithin;
    ury = bg::get<1>(rightPt) + eolWithin;
  } else if (dir == frDirEnum::S) {
    llx = bg::get<0>(leftPt) - eolWithin;
    urx = bg::get<0>(rightPt) + eolWithin;
    ury = bg::get<1>(leftPt);
    lly = ury - eolSpace;
  } else if (dir == frDirEnum::W) {
    urx = bg::get<0>(leftPt);
    llx = urx - eolSpace;
    lly = bg::get<1>(leftPt) - eolWithin;
    ury = bg::get<1>(rightPt) + eolWithin;
  } else if (dir == frDirEnum::N) {
    llx = bg::get<0>(leftPt) - eolWithin;
    urx = bg::get<0>(rightPt) + eolWithin;
    lly = bg::get<1>(leftPt);
    ury = lly + eolSpace;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getSpacingTableTestBox\n";
  }
  testBox = box_t(point_t(llx, lly), point_t(urx, ury));
}

void fr::DRCWorker::getParallelEdgeBoxs(const segment_t &seg,
                                         DRCEdge *eolEdge, 
                                         const frCoord &parSpace,
                                         const frCoord &parWithin,
                                         const frCoord &eolWithin,
                                         box_t &parallelEdgeBoxLeft,
                                         box_t &parallelEdgeBoxRight) {
  auto dir = eolEdge->getDir();
  point_t leftPt, rightPt;
  point_t pt1(bg::get<0,0>(seg), bg::get<0,1>(seg));
  point_t pt2(bg::get<1,0>(seg), bg::get<1,1>(seg));
  if (bg::get<0>(pt1) > bg::get<0>(pt2) || bg::get<1>(pt1) > bg::get<1>(pt2)) {
    leftPt = pt2;
    rightPt = pt1;
  } else {
    leftPt = pt1;
    rightPt = pt2;
  }
  frCoord llx = 0, lly = 0, urx = 0, ury = 0;
  // left / bottom parallelEdge rect
  if (dir == frDirEnum::E) {
    ury = bg::get<1>(leftPt);
    lly = ury - parSpace;
    llx = bg::get<0>(leftPt) - parWithin;
    urx = bg::get<0>(leftPt) + eolWithin;
  } else if (dir == frDirEnum::S) {
    urx = bg::get<0>(leftPt);
    llx = urx - parSpace;
    ury = bg::get<1>(leftPt) + parWithin;
    lly = bg::get<1>(leftPt) - eolWithin;
  } else if (dir == frDirEnum::W) {
    ury = bg::get<1>(leftPt);
    lly = ury - parSpace;
    llx = bg::get<0>(leftPt) - eolWithin;
    urx = bg::get<0>(leftPt) + parWithin;
  } else if (dir == frDirEnum::N) {
    urx = bg::get<0>(leftPt);
    llx = urx - parSpace;
    ury = bg::get<1>(leftPt) + eolWithin;
    lly = bg::get<1>(leftPt) - parWithin;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getParallelEdgeRect\n";
  }
  parallelEdgeBoxLeft =  box_t(point_t(llx, lly), point_t(urx, ury));
  // right / top parallelEddge rect
  if (dir == frDirEnum::E) {
    lly = bg::get<1>(rightPt);
    ury = lly + parSpace;
    llx = bg::get<0>(rightPt) - parWithin;
    urx = bg::get<0>(rightPt) + eolWithin;
  } else if (dir == frDirEnum::S) {
    llx = bg::get<0>(rightPt);
    urx = llx + parSpace;
    ury = bg::get<1>(rightPt) + parWithin;
    lly = bg::get<1>(rightPt) - eolWithin;
  } else if (dir == frDirEnum::W) {
    lly = bg::get<1>(rightPt);
    ury = lly + parSpace;
    llx = bg::get<0>(rightPt) - eolWithin;
    urx = bg::get<0>(rightPt) + parWithin;
  } else if (dir == frDirEnum::N) {
    llx = bg::get<0>(rightPt);
    urx = llx + parSpace;
    ury = bg::get<1>(rightPt) + eolWithin;
    lly = bg::get<1>(rightPt) - parWithin;
  } else {
    std::cout << "Error: UNKNOWN edge direction in getParallelEdgeRect\n";
  }
  parallelEdgeBoxRight = box_t(point_t(llx, lly), point_t(urx, ury));
  return;
}


bool fr::DRCWorker::covered_by(const boost::icl::discrete_interval<frCoord, std::less> &intv,
                               const boost::icl::interval_set<frCoord> &intvSet) {
  auto intvItRes = intvSet.equal_range(intv);
  for (auto intvIt = intvItRes.first; intvIt != intvItRes.second; ++intvIt) {
    return (boost::icl::contains(*intvIt, intv));
  }
  return false;
}

fr::frCoord fr::DRCWorker::rectDiagLength(const Rectangle &rect) {
  frCoord deltaX = (xh(rect) - xl(rect));
  frCoord deltaY = (yh(rect) - yl(rect));
  return frCoord(sqrt(deltaX * deltaX + deltaY * deltaY));
}

std::vector<fr::frMarker*>& fr::DRCWorker::getViolations() {
  markers.clear();
  for (auto &umarker: uMarkers) {
    markers.push_back(umarker.get());
  }
  return markers;
}

// std::vector<frMarker*>& fr::DRCWorker::getViolations(drNet *net) {
//   markers.clear();
//   // region query violation that is related to the drNet
//   frNet* ownerNet = net->getFrNet();
//   if (drNet2Layer2PolySet.find(net) != drNet2Layer2PolySet.end()) {
//     // query violation layer by layer
//     for (auto layerIt = drNet2Layer2PolySet[net].begin(); layerIt != drNet2Layer2PolySet[net].end(); ++layerIt) {
//       auto &layerNum = layerIt->first;
//       auto &polySet = layerIt->second;
//       vector<Rectangle> polySetRects;
//       get_rectangles(polySetRects, polySet);
//       for (auto &rect: polySetRects) {
//         box_t queryBox(point_t(xl(rect), yl(rect)), point_t(xh(rect), yh(rect)));
//         std::vector<std::pair<box_t, frMarker*> > queryResult;
//         layer2MarkerRtree[layerNum].query(bgi::intersects(queryBox), back_inserter(queryResult));
//         for (auto &markerPair: queryResult) {
//           auto marker = markerPair.second;
//           auto &srcs = marker.getSrcs();
//           if (srcs.find(ownerNet) != srcs.end()) {
//             markers.push_back(marker);
//           }
//         }
//       }
//     }
//   }
//   return markers;
// }

std::vector<fr::frMarker*>& fr::DRCWorker::getIncreViolations() {
  markers.clear();
  for (auto i = lastMarkerNum; i < (int)uMarkers.size(); ++i) {
    auto &umaker = uMarkers[i];
    markers.push_back(umaker.get());
  }
  return markers;
}

void fr::DRCWorker::report() {
  double dbu = design->getTech()->getDBUPerUU();
  std::cout << DRC_RPT_FILE << "\n";
  if (DRC_RPT_FILE != std::string("")) {
    std::ofstream drcRpt(DRC_RPT_FILE.c_str());
    if (drcRpt.is_open()) {
      for (auto &marker: uMarkers) {
        drcRpt << "  violation type: " << int(marker->getConstraint()->typeId()) << "\n";
        // get source(s) of violation
        drcRpt << "    srcs: ";
        for (auto src: marker->getSrcs()) {
          if (src) {
            switch (src->typeId()) {
              case frcNet:
                drcRpt << (static_cast<frNet*>(src))->getName() << " ";
                break;
              case frcInstTerm: {
                frInstTerm* instTerm = (static_cast<frInstTerm*>(src));
                // drcRpt << instTerm->getInst()->getName() 
                       // << "/" << instTerm->getTerm()->getName() << " ";
                drcRpt << "Pin of Cell " << instTerm->getInst()->getName() << " ";
                break;
              }
              case frcTerm: {
                frTerm* term = (static_cast<frTerm*>(src));
                drcRpt << term->getName() << " ";
                break;
              }
              default:
                std::cout << "Error: unexpected src type in marker\n";
            }
          }
        }
        drcRpt << "\n";
        // get violation bbox
        frBox bbox;
        marker->getBBox(bbox);
        drcRpt << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
               << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    } else {
      std::cout << "Error: Fail to open DRC report file\n";
    }
  } else {
    std::cout << "Error: DRC report file is not specified\n";
  }
}

int fr::DRCWorker::getNetId(frBlockObject *obj) {
  int netId = -1;
  switch (obj->typeId()) {
    case frcPathSeg:
    {
      frPathSeg* pathSeg = static_cast<frPathSeg*>(obj);
      if (pathSeg->hasNet()) {
        if (net2Id.find(pathSeg->getNet()) == net2Id.end()) {
          netId = net2Id[pathSeg->getNet()];
        }
      } else if (pathSeg->hasPin()) {
        for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
          if (netId != -1) {
            break;
          }
          auto term = mapIt->first;
          for (auto &pin: term->getPins()) {
            auto pinPtr = pin.get();
            if (pinPtr == pathSeg->getPin()) {
              netId = mapIt->second;
              break;
            }
          }
        }
      }
    }
    case frcVia:
    {
      frVia* via = static_cast<frVia*>(obj);
      if (via->hasNet()) {
        if (net2Id.find(via->getNet()) == net2Id.end()) {
          netId = net2Id[via->getNet()];
        }
      } else if (via->hasPin()) {
        for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
          if (netId != -1) {
            break;
          }
          auto term = mapIt->first;
          for (auto &pin: term->getPins()) {
            auto pinPtr = pin.get();
            if (pinPtr == via->getPin()) {
              netId = mapIt->second;
              break;
            }
          }
        }
      }
    }
    case frcPatchWire:
    {
      frPatchWire* pwire = static_cast<frPatchWire*>(obj);
      if (pwire->hasNet()) {
        if (net2Id.find(pwire->getNet()) != net2Id.end()) {
          netId = net2Id[pwire->getNet()];
        }
      } else if (pwire->hasPin()) {
        for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
          if (netId != -1) {
            break;
          }
          auto term = mapIt->first;
          for (auto &pin: term->getPins()) {
            auto pinPtr = pin.get();
            if (pinPtr == pwire->getPin()) {
              netId = mapIt->second;
              break;
            }
          }
        }
      }
    }
    case drcPathSeg:
    {
      drPathSeg* pathSeg = static_cast<drPathSeg*>(obj);
      if (pathSeg->hasNet()) {
        if (net2Id.find(pathSeg->getNet()->getFrNet()) == net2Id.end()) {
          netId = net2Id[pathSeg->getNet()->getFrNet()];
        }
      }
      break;
    }
    case drcVia:
    {
      drVia *via = static_cast<drVia*>(obj);
      if (via->hasNet()) {
        if (net2Id.find(via->getNet()->getFrNet()) != net2Id.end()) {
          netId = net2Id[via->getNet()->getFrNet()];
        }
      }
      break;
    }
    case drcPatchWire:
    {
      drPatchWire* pwire = static_cast<drPatchWire*>(obj);
      if (pwire->hasNet()) {
        if (net2Id.find(pwire->getNet()->getFrNet()) != net2Id.end()) {
          netId = net2Id[pwire->getNet()->getFrNet()];
        }
      }
      break;
    }
    case frcInstTerm:
    {
      frInstTerm *instTerm = static_cast<frInstTerm*>(obj);
      auto term = instTerm->getTerm();
      if (instTerm->hasNet()) {
        if (net2Id.find(instTerm->getNet()) == net2Id.end()) {
          netId = net2Id[instTerm->getNet()];
        }
      } else {
        if (term->getType() == frTermEnum::frcNormalTerm || 
            term->getType() == frTermEnum::frcClockTerm) {
          if (term2Id.find(term) != term2Id.end()) {
            netId = term2Id[term];
          }
        } else if (term->getType() == frTermEnum::frcPowerTerm) {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATPOWER);
        } else if (term->getType() == frTermEnum::frcGroundTerm) {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATGROUND);
        }
      }
    }
    case frcTerm:
    {
      frTerm* term = static_cast<frTerm*>(obj);
      if (term->hasNet()) {
        if (net2Id.find(term->getNet()) != net2Id.end()) {
          netId = net2Id[term->getNet()];
        }
      } else {
        if (term->getType() == frTermEnum::frcNormalTerm || 
            term->getType() == frTermEnum::frcClockTerm) {
          if (term2Id.find(term) != term2Id.end()) {
            netId = term2Id[term];
          }
        } else if (term->getType() == frTermEnum::frcPowerTerm) {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATPOWER);
        } else if (term->getType() == frTermEnum::frcGroundTerm) {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATGROUND);
        }
      }
    }
    case frcNet:
    {
      frNet *net = static_cast<frNet*>(obj);
      if (net2Id.find(net) != net2Id.end()) {
        netId = net2Id[net];
      }
    }
    case drcNet:
    {
      drNet *net = static_cast<drNet*>(obj);
      if (net2Id.find(net->getFrNet()) != net2Id.end()) {
        netId = net2Id[net->getFrNet()];
      }
    }
    default:
      ;
  }
  return netId;
}