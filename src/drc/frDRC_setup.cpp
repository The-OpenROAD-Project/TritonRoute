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
using namespace boost::polygon::operators;


void fr::DRCWorker::setup() {
  setupEdgeRTree();
  // printEdgeRTree();
  setupMaxRectRTree();
  setupCutRectRTree();
}

void fr::DRCWorker::setupCutRectRTree() {
  for (auto &drcNet: drcNets) {
    setupCutRectRTree_net(drcNet.get(), true);
  }
}

void fr::DRCWorker::setupCutRectRTree_net(DRCNet *net, bool isAdd) {
  int netId = net->getId();
  auto &layer2CutRects = net->getLayer2CutRects();
  for (auto layerIt = layer2CutRects.begin(); layerIt != layer2CutRects.end(); ++layerIt) {
    auto layerNum = layerIt->first;
    auto &cutRects = layerIt->second;
    for (auto &cutRect: cutRects) {
      auto boostb = box_t(point_t(xl(cutRect), yl(cutRect)), point_t(xh(cutRect), yh(cutRect)));
      if (isAdd) {
        layer2CutRectRTree[layerNum].insert(std::make_pair(boostb, netId));
      } else {
        layer2CutRectRTree[layerNum].remove(std::make_pair(boostb, netId));
      }
    }
  }
}

void fr::DRCWorker::setupEdgeRTree() {
  for (auto &drcNet: drcNets) {
    setupEdges_net(drcNet.get());
    setupEdgeRTree_net(drcNet.get(), true);
  }
}


void fr::DRCWorker::setupEdges_net(DRCNet* net) {
  // double dbu = design->getTech()->getDBUPerUU();
  int netId = net->getId();
  std::map<frLayerNum, std::map<frCoord, boost::icl::interval_set<frCoord> > > layer2HorzEdgeIntvSet, layer2VertEdgeIntvSet; // HorzEdge: mapped from yLoc
  std::map<frLayerNum, std::map<frCoord, boost::icl::interval_set<frCoord> > > layer2MergedHorzEdgeIntvSet, layer2MergedVertEdgeIntvSet; // HorzEdge: mapped from yLoc
  std::set<frLayerNum> layerNums;
  std::map<frLayerNum, PolygonSet> layer2MergedPolySet;
  auto &layer2FixedPolySet = net->getLayer2FixedPolySet();
  auto &layer2RoutePolySet = net->getLayer2RoutePolySet();
  // auto &layer2Seg2Edge = net->getLayerSegEdgeMap();
  std::map<frLayerNum, std::map<Segment, DRCEdge*> > layer2Seg2Edge;

  // get set of merged layerNums
  for (auto it = layer2FixedPolySet.begin(); it != layer2FixedPolySet.end(); ++it) {
    layerNums.insert(it->first);
  }
  for (auto it = layer2RoutePolySet.begin(); it != layer2RoutePolySet.end(); ++it) {
    layerNums.insert(it->first);
  }

  // build fixed edge intv set
  for (auto it = layer2FixedPolySet.begin(); it != layer2FixedPolySet.end(); ++it) {
    frLayerNum layerNum = it->first;
    for (auto &origPoly: it->second) {
      std::vector<Polygon> polys;
      getPolyWithHole(origPoly, polys);
      if (!polys.empty() && winding(polys.back()) != boost::polygon::LOW) {
        std::cout << "Error: outline poly must be clockwise\n";
      }
      for (auto &poly:polys) {
        frVector<Point> vertices;
        for (auto ptIt = begin_points(poly); ptIt != end_points(poly); ++ptIt) {
          vertices.push_back(*ptIt);
        }
        for (int i = 0; i < (int)vertices.size(); ++i) {
          auto pt1 = vertices[i];
          auto pt2 = ((i + 1) == (int)vertices.size()) ? vertices[0] : vertices[i + 1];
          // always from smaller pt to larger pt
          if (pt1 > pt2) {
            auto tmpPt = pt2;
            pt2 = pt1;
            pt1 = tmpPt;
          }
          // insert to intv set
          if (pt1.x() == pt2.x()) {
            // vertical intv
            auto edgeIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.y(), pt2.y());
            layer2VertEdgeIntvSet[layerNum][pt1.x()].insert(edgeIntv);
          } else if (pt1.y() == pt2.y()) {
            // horizontal intv
            auto edgeIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.x(), pt2.x());
            layer2HorzEdgeIntvSet[layerNum][pt1.y()].insert(edgeIntv);
          } else {
            std::cout << "Error: non-colinear pts in setupEdgeRTree_net, skipped\n";
            continue;
          }
        }
      }
    }
  }

  // get layer2MergedPolySet
  for (auto layerNum: layerNums) {
    if (layer2FixedPolySet.find(layerNum) != layer2FixedPolySet.end()) {
      layer2MergedPolySet[layerNum] += layer2FixedPolySet[layerNum];
    }
    if (layer2RoutePolySet.find(layerNum) != layer2RoutePolySet.end()) {
      layer2MergedPolySet[layerNum] += layer2RoutePolySet[layerNum];
    }
  }

  // build merged edge intv set
  for (auto it = layer2MergedPolySet.begin(); it != layer2MergedPolySet.end(); ++it) {
    frLayerNum layerNum = it->first;
    for (auto &origPoly: it->second) {
      std::vector<Polygon> polys;
      getPolyWithHole(origPoly, polys);
      if (!polys.empty() && winding(polys.back()) != boost::polygon::LOW) {
        std::cout << "Error: outline poly must be clockwise\n";
      }
      for (auto &poly: polys) {
        frVector<Point> vertices;
        for (auto ptIt = begin_points(poly); ptIt != end_points(poly); ++ptIt) {
          vertices.push_back(*ptIt);
        }
        for (int i = 0; i < (int)vertices.size(); ++i) {
          auto pt1 = vertices[i];
          auto pt2 = ((i + 1) == (int)vertices.size()) ? vertices[0] : vertices[i + 1];
          // always from smaller pt to larger pt
          if (pt1 > pt2) {
            auto tmpPt = pt2;
            pt2 = pt1;
            pt1 = tmpPt;
          }
          // insert to intv set
          if (pt1.x() == pt2.x()) {
            // vertical intv
            auto edgeIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.y(), pt2.y());
            layer2MergedVertEdgeIntvSet[layerNum][pt1.x()].insert(edgeIntv);
          } else if (pt1.y() == pt2.y()) {
            // horizontal intv
            auto edgeIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.x(), pt2.x());
            layer2MergedHorzEdgeIntvSet[layerNum][pt1.y()].insert(edgeIntv);
          } else {
            std::cout << "Error: non-colinear pts in setupEdgeRTree_net, skipped\n";
            continue;
          }
        }
      }
    }
  }


  // build edges 
  for (auto it = layer2MergedPolySet.begin(); it != layer2MergedPolySet.end(); ++it) {
    frLayerNum layerNum = it->first;
    auto &polySet = it->second;
    // the poly may have hole, need to call getPolyWithHole
    for (auto &origPoly: polySet) {
      std::vector<Polygon> polys;
      getPolyWithHole(origPoly, polys);
      if (!polys.empty() && winding(polys.back()) != boost::polygon::LOW) {
        std::cout << "Error: outline poly must be clockwise\n";
      }
      // the last one is outline and the others are holes
      for (auto &poly: polys) {
        // build edge based on polygon
        // std::cout << "@@@ new poly@@@\n";
        frVector<Point> vertices;
        frVector<Segment> segments;
        for (auto ptIt = begin_points(poly); ptIt != end_points(poly); ++ptIt) {
          vertices.push_back(*ptIt);
        }
        for (int i = 0; i < (int)vertices.size(); ++i) {
          auto pt1 = vertices[i];
          // std::cout << "@@@debug@@@: visit vertex (" << pt1.x() / dbu << ", " << pt1.y() / dbu << ")\n";
          auto pt2 = ((i + 1) == (int)vertices.size()) ? vertices[0] : vertices[i + 1];
          Segment directedEdgeSeg(pt1, pt2);
          // always from smaller pt to larger pt
          if (pt1 > pt2) {
            auto tmpPt = pt2;
            pt2 = pt1;
            pt1 = tmpPt;
          }
          // create new edges
          bool isFixed = false;
          bool isHorizontal = (pt1.y() == pt2.y());
          DRCEdge *drcEdge = net->addDRCEdge();
          Segment drcEdgeSeg(pt1, pt2);
          segments.push_back(drcEdgeSeg);
          // std::cout << "@@@debug@@@: add edge (" << pt1.x() / dbu << ", " << pt1.y() / dbu 
          //           << ") -- (" << pt2.x() / dbu << ", " << pt2.y() / dbu << ")\n";
          layer2Seg2Edge[layerNum][drcEdgeSeg] = drcEdge;
          drcEdge->setId(netId);
          drcEdge->setSegment(directedEdgeSeg);
          drcEdge->setLayerNum(layerNum);
          drcEdge->setWinding(winding(poly));
          // check whether this is fixed edge
          if (isHorizontal) {
            auto tempIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.x(), pt2.x());
            if (layer2HorzEdgeIntvSet.find(layerNum) == layer2HorzEdgeIntvSet.end() ||
                layer2HorzEdgeIntvSet[layerNum].find(pt1.y()) == layer2HorzEdgeIntvSet[layerNum].end() ||
                (!boost::icl::contains(layer2HorzEdgeIntvSet[layerNum][pt1.y()], tempIntv))) {
              isFixed = false;
            } else {
              isFixed = true;
            }
          } else {
            auto tempIntv = boost::icl::interval<frCoord>::interval_type::right_open(pt1.y(), pt2.y());
            if (layer2VertEdgeIntvSet.find(layerNum) == layer2VertEdgeIntvSet.end() ||
                layer2VertEdgeIntvSet[layerNum].find(pt1.x()) == layer2VertEdgeIntvSet[layerNum].end() ||
                (!boost::icl::contains(layer2VertEdgeIntvSet[layerNum][pt1.x()], tempIntv))) {
              isFixed = false;
            } else {
              isFixed = true;
            }
          }

          if (isFixed) {
            drcEdge->setType(drcEdgeTypeEnum::FIXED);
          } else {
            drcEdge->setType(drcEdgeTypeEnum::ROUTE);
          }
          // dir, prev, next, span will be set later
        }
        // set prev next pointer
        for (int i = 0; i < (int)segments.size(); ++i) {
          DRCEdge* drcEdge = layer2Seg2Edge[layerNum][segments[i]];
          DRCEdge *nextDRCEdge, *prevDRCEdge;
          // if (winding(poly) == boost::polygon::LOW) {
            nextDRCEdge = layer2Seg2Edge[layerNum][segments[(i+1) % ((int)segments.size())]];
            prevDRCEdge = layer2Seg2Edge[layerNum][segments[(i-1+(int)segments.size()) % ((int)segments.size())]];
          // } else {
          //   nextDRCEdge = layer2Seg2Edge[layerNum][segments[(i-1+(int)segments.size()) % ((int)segments.size())]];
          //   prevDRCEdge = layer2Seg2Edge[layerNum][segments[(i+1) % ((int)segments.size())]];
          // }
          drcEdge->setNext(nextDRCEdge);
          drcEdge->setPrev(prevDRCEdge);
        }
      }

      // build edge based max rectangles
      std::vector<Rectangle> maxRects;
      PolygonSet tmpPolySet;
      tmpPolySet += origPoly;
      get_max_rectangles(maxRects, tmpPolySet);
      for (auto rect: maxRects) {
        frCoord horzSpan = xh(rect) - xl(rect);
        frCoord vertSpan = yh(rect) - yl(rect);
        // left side
        Segment leftSeg(ll(rect), ul(rect));
        auto leftIntv = boost::icl::interval<frCoord>::interval_type::right_open(yl(rect), yh(rect));
        auto intvItResLeft = layer2MergedVertEdgeIntvSet[layerNum][xl(rect)].equal_range(leftIntv);
        for (auto intvIt = intvItResLeft.first; intvIt != intvItResLeft.second; ++intvIt) {
          auto &segIntv = *intvIt;
          // update existing edge
          if (boost::icl::contains(leftIntv, segIntv)) {
            Segment drcEdgeSeg(Point(xl(rect), intvIt->lower()), Point(xl(rect), intvIt->upper()));
            DRCEdge* drcEdge = layer2Seg2Edge[layerNum][drcEdgeSeg];
            drcEdge->setDir(frDirEnum::W);
            if (drcEdge->getSpan() < horzSpan) {
              drcEdge->setSpan(horzSpan);
            }
            frCoord tmpWidth = std::min(vertSpan, horzSpan);
            if (drcEdge->getWidth() < tmpWidth) {
              drcEdge->setWidth(tmpWidth);
            }
          } 
          // add new edge if any
          else {
            auto ovlpIntv = boost::icl::interval<frCoord>::interval_type::right_open(std::max(intvIt->lower(), leftIntv.lower()), std::min(intvIt->upper(), leftIntv.upper()));
            Segment ovlpEdgeSeg(Point(xl(rect), ovlpIntv.lower()),Point(xl(rect), ovlpIntv.upper()));
            if (layer2Seg2Edge[layerNum].find(ovlpEdgeSeg) != layer2Seg2Edge[layerNum].end()) {
              DRCEdge* drcEdge = layer2Seg2Edge[layerNum][ovlpEdgeSeg];
              if (drcEdge->getSpan() < horzSpan) {
                drcEdge->setSpan(horzSpan);
              }
              drcEdge->setDir(frDirEnum::W);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              if (drcEdge->getWidth() < tmpWidth) {
                drcEdge->setWidth(tmpWidth);
              }
            } else {
              DRCEdge *drcEdge = net->addDRCEdge();
              layer2Seg2Edge[layerNum][ovlpEdgeSeg] = drcEdge;
              drcEdge->setId(netId);
              drcEdge->setSegment(ovlpEdgeSeg);
              drcEdge->setLayerNum(layerNum);
              if (boost::icl::contains(layer2VertEdgeIntvSet[layerNum][xl(rect)], ovlpIntv)) {
                drcEdge->setType(drcEdgeTypeEnum::FIXED);
              } else {
                drcEdge->setType(drcEdgeTypeEnum::ROUTE);
              }
              drcEdge->setDir(frDirEnum::W);
              drcEdge->setSpan(horzSpan);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              drcEdge->setWidth(tmpWidth);
            }
          }
        }

        // bottom side
        Segment bottomSeg(ll(rect), lr(rect));
        auto bottomIntv = boost::icl::interval<frCoord>::interval_type::right_open(xl(rect), xh(rect));
        auto intvItResBottom = layer2MergedHorzEdgeIntvSet[layerNum][yl(rect)].equal_range(bottomIntv);
        for (auto intvIt = intvItResBottom.first; intvIt != intvItResBottom.second; ++intvIt) {
          auto &segIntv = *intvIt;
          // update existing edge
          if (boost::icl::contains(bottomIntv, segIntv)) {
            Segment drcEdgeSeg(Point(intvIt->lower(), yl(rect)), Point(intvIt->upper(), yl(rect)));
            DRCEdge* drcEdge = layer2Seg2Edge[layerNum][drcEdgeSeg];
            drcEdge->setDir(frDirEnum::S);
            if (drcEdge->getSpan() < vertSpan) {
              drcEdge->setSpan(vertSpan);
            }
            frCoord tmpWidth = std::min(vertSpan, horzSpan);
            if (drcEdge->getWidth() < tmpWidth) {
              drcEdge->setWidth(tmpWidth);
            }
          }
          // add new edge if any
          else {
            auto ovlpIntv = boost::icl::interval<frCoord>::interval_type::right_open(std::max(intvIt->lower(), bottomIntv.lower()), std::min(intvIt->upper(), bottomIntv.upper()));
            Segment ovlpEdgeSeg(Point(ovlpIntv.lower(), yl(rect)),Point(ovlpIntv.upper(), yl(rect)));
            if (layer2Seg2Edge[layerNum].find(ovlpEdgeSeg) != layer2Seg2Edge[layerNum].end()) {
              DRCEdge* drcEdge = layer2Seg2Edge[layerNum][ovlpEdgeSeg];
              if (drcEdge->getSpan() < vertSpan) {
                drcEdge->setSpan(vertSpan);
              }
              drcEdge->setDir(frDirEnum::S);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              if (drcEdge->getWidth() < tmpWidth) {
                drcEdge->setWidth(tmpWidth);
              }
            } else {
              DRCEdge *drcEdge = net->addDRCEdge();
              layer2Seg2Edge[layerNum][ovlpEdgeSeg] = drcEdge;
              drcEdge->setId(netId);
              drcEdge->setSegment(ovlpEdgeSeg);
              drcEdge->setLayerNum(layerNum);
              if (boost::icl::contains(layer2HorzEdgeIntvSet[layerNum][yl(rect)], ovlpIntv)) {
                drcEdge->setType(drcEdgeTypeEnum::FIXED);
              } else {
                drcEdge->setType(drcEdgeTypeEnum::ROUTE);
              }
              drcEdge->setDir(frDirEnum::S);
              drcEdge->setSpan(vertSpan);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              drcEdge->setWidth(tmpWidth);
            }
          }
        }


        // right side
        Segment rightSeg(lr(rect), ur(rect));
        auto rightIntv = boost::icl::interval<frCoord>::interval_type::right_open(yl(rect), yh(rect));
        auto intvItResRight = layer2MergedVertEdgeIntvSet[layerNum][xh(rect)].equal_range(rightIntv);
        for (auto intvIt = intvItResRight.first; intvIt != intvItResRight.second; ++intvIt) {
          auto &segIntv = *intvIt;
          // update existing edge
          if (boost::icl::contains(rightIntv, segIntv)) {
            Segment drcEdgeSeg(Point(xh(rect), intvIt->lower()), Point(xh(rect), intvIt->upper()));
            DRCEdge* drcEdge = layer2Seg2Edge[layerNum][drcEdgeSeg];
            drcEdge->setDir(frDirEnum::E);
            if (drcEdge->getSpan() < horzSpan) {
              drcEdge->setSpan(horzSpan);
            }
            frCoord tmpWidth = std::min(vertSpan, horzSpan);
            if (drcEdge->getWidth() < tmpWidth) {
              drcEdge->setWidth(tmpWidth);
            }
          }
          // add new edge if any
          else {
            auto ovlpIntv = boost::icl::interval<frCoord>::interval_type::right_open(std::max(intvIt->lower(), rightIntv.lower()), std::min(intvIt->upper(), rightIntv.upper()));
            Segment ovlpEdgeSeg(Point(xh(rect), ovlpIntv.lower()),Point(xh(rect), ovlpIntv.upper()));
            if (layer2Seg2Edge[layerNum].find(ovlpEdgeSeg) != layer2Seg2Edge[layerNum].end()) {
              DRCEdge* drcEdge = layer2Seg2Edge[layerNum][ovlpEdgeSeg];
              if (drcEdge->getSpan() < horzSpan) {
                drcEdge->setSpan(horzSpan);
              }
              drcEdge->setDir(frDirEnum::E);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              if (drcEdge->getWidth() < tmpWidth) {
                drcEdge->setWidth(tmpWidth);
              }
            } else {
              DRCEdge *drcEdge = net->addDRCEdge();
              layer2Seg2Edge[layerNum][ovlpEdgeSeg] = drcEdge;
              drcEdge->setId(netId);
              drcEdge->setSegment(ovlpEdgeSeg);
              drcEdge->setLayerNum(layerNum);
              if (boost::icl::contains(layer2VertEdgeIntvSet[layerNum][xh(rect)], ovlpIntv)) {
                drcEdge->setType(drcEdgeTypeEnum::FIXED);
              } else {
                drcEdge->setType(drcEdgeTypeEnum::ROUTE);
              }
              drcEdge->setDir(frDirEnum::E);
              drcEdge->setSpan(horzSpan);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              drcEdge->setWidth(tmpWidth);
            }
          }
        }


        // top side
        Segment topSeg(ul(rect), ur(rect));
        auto topIntv = boost::icl::interval<frCoord>::interval_type::right_open(xl(rect), xh(rect));
        auto intvItResTop = layer2MergedHorzEdgeIntvSet[layerNum][yh(rect)].equal_range(topIntv);
        for (auto intvIt = intvItResTop.first; intvIt != intvItResTop.second; ++intvIt) {
          auto &segIntv = *intvIt;
          // update existing edge
          if (boost::icl::contains(topIntv, segIntv)) {
            Segment drcEdgeSeg(Point(intvIt->lower(), yh(rect)), Point(intvIt->upper(), yh(rect)));
            DRCEdge* drcEdge = layer2Seg2Edge[layerNum][drcEdgeSeg];
            drcEdge->setDir(frDirEnum::N);
            if (drcEdge->getSpan() < vertSpan) {
              drcEdge->setSpan(vertSpan);
            }
            frCoord tmpWidth = std::min(vertSpan, horzSpan);
            if (drcEdge->getWidth() < tmpWidth) {
              drcEdge->setWidth(tmpWidth);
            }
          }
          // add new edge if any
          else {
            auto ovlpIntv = boost::icl::interval<frCoord>::interval_type::right_open(std::max(intvIt->lower(), topIntv.lower()), std::min(intvIt->upper(), topIntv.upper()));
            Segment ovlpEdgeSeg(Point(ovlpIntv.lower(), yh(rect)),Point(ovlpIntv.upper(), yh(rect)));
            if (layer2Seg2Edge[layerNum].find(ovlpEdgeSeg) != layer2Seg2Edge[layerNum].end()) {
              DRCEdge* drcEdge = layer2Seg2Edge[layerNum][ovlpEdgeSeg];
              if (drcEdge->getSpan() < vertSpan) {
                drcEdge->setSpan(vertSpan);
              }
              drcEdge->setDir(frDirEnum::N);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              if (drcEdge->getWidth() < tmpWidth) {
                drcEdge->setWidth(tmpWidth);
              }
            } else {
              DRCEdge *drcEdge = net->addDRCEdge();
              layer2Seg2Edge[layerNum][ovlpEdgeSeg] = drcEdge;
              drcEdge->setId(netId);
              drcEdge->setSegment(ovlpEdgeSeg);
              drcEdge->setLayerNum(layerNum);
              if (boost::icl::contains(layer2HorzEdgeIntvSet[layerNum][yh(rect)], ovlpIntv)) {
                drcEdge->setType(drcEdgeTypeEnum::FIXED);
              } else {
                drcEdge->setType(drcEdgeTypeEnum::ROUTE);
              }
              drcEdge->setDir(frDirEnum::N);
              drcEdge->setSpan(vertSpan);
              frCoord tmpWidth = std::min(vertSpan, horzSpan);
              drcEdge->setWidth(tmpWidth);
            }
          }
        }


      }
    }
  }

  // push edges to RTree
  
  for (auto layerIt = layer2Seg2Edge.begin(); layerIt != layer2Seg2Edge.end(); ++layerIt) {
    frLayerNum layerNum = layerIt->first;
    auto &seg2Edge = layerIt->second;
    for (auto segIt = seg2Edge.begin(); segIt != seg2Edge.end(); ++segIt) {
      auto &edgeSeg = segIt->first;
      if (segIt->second->getDir() == frDirEnum::UNKNOWN) {
        std::cout << "Error: edge with UNKNOWN direction being pushed from netId = " << net->getId() << "\n";
        if (net->getSrc()->typeId() == frcNet) {
          std::cout << "  " << static_cast<frNet*>(net->getSrc())->getName() << "\n";
        } else if (net->getSrc()->typeId() == drcNet) {
          std::cout << "  " << static_cast<drNet*>(net->getSrc())->getFrNet()->getName() << "\n";
        } else if (net->getSrc()->typeId() == frcInstTerm) {
          auto instTerm = static_cast<frInstTerm*>(net->getSrc());
          std::cout << "  " << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() << "\n";
        } else if (net->getSrc()->typeId() == frcTerm) {
          auto term = static_cast<frTerm*>(net->getSrc());
          std::cout << "  " << term->getName() << "\n";
        } else {
          std::cout << "  I don't know what the source is... TypeId = " << (int)(net->getSrc()->typeId()) << "\n";
        }
        auto tmpSeg = segIt->second->getSegment();
        std::cout << "  seg: (" << low(edgeSeg).x() / 2000.0 << ", " << low(edgeSeg).y() / 2000.0 << ") -- ("
                  << high(edgeSeg).x() / 2000.0 << ", " << high(edgeSeg).y() / 2000.0 << ") on layer " << layerNum << "\n"; 
        std::cout << "  edge: (" << low(tmpSeg).x() / 2000.0 << ", " << low(tmpSeg).y() / 2000.0 << ") -- ("
                  << high(tmpSeg).x() / 2000.0 << ", " << high(tmpSeg).y() / 2000.0 << ")\n"; 
      }
      // auto &edge = segIt->second;
      // frCoord tmpLowX = std::min(low(edge->getSegment()).x(), high(edge->getSegment()).x());
      // frCoord tmpLowY = std::min(low(edge->getSegment()).y(), high(edge->getSegment()).y());
      // frCoord tmpHighX = std::max(low(edge->getSegment()).x(), high(edge->getSegment()).x());
      // frCoord tmpHighY = std::max(low(edge->getSegment()).y(), high(edge->getSegment()).y());
      // Segment testSeg(Point(tmpLowX, tmpLowY), Point(tmpHighX, tmpHighY));
      // segment_t rtreeSeg(point_t(low(edgeSeg).x(), low(edgeSeg).y()), point_t(high(edgeSeg).x(), high(edgeSeg).y()));
      // if (edgeSeg != testSeg) {
      //   std::cout << "Error: it was that moment, he knew he fxxked up...2\n";
      // }
      // layer2EdgeRTree[layerNum].insert(std::make_pair(rtreeSeg, segIt->second));
    }
  }
  
}

void fr::DRCWorker::setupEdgeRTree_net(DRCNet* net, bool isAdd) {
  // auto &layer2Seg2Edge = net->getLayerSegEdgeMap();
  auto &drcEdges = net->getDRCEdges();
  for (auto &udrcEdge: drcEdges) {
    auto drcEdge = udrcEdge.get();

    if (drcEdge->getDir() == frDirEnum::UNKNOWN) {
      std::cout << "Error: edge with UNKNOWN direction being pushed from netId = " << net->getId() << "\n";
      if (net->getSrc()->typeId() == frcNet) {
        std::cout << "  " << static_cast<frNet*>(net->getSrc())->getName() << "\n";
      } else if (net->getSrc()->typeId() == drcNet) {
        std::cout << "  " << static_cast<drNet*>(net->getSrc())->getFrNet()->getName() << "\n";
      } else if (net->getSrc()->typeId() == frcInstTerm) {
        auto instTerm = static_cast<frInstTerm*>(net->getSrc());
        std::cout << "  " << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() << "\n";
      } else if (net->getSrc()->typeId() == frcTerm) {
        auto term = static_cast<frTerm*>(net->getSrc());
        std::cout << "  " << term->getName() << "\n";
      } else {
        std::cout << "  I don't know what the source is... TypeId = " << (int)(net->getSrc()->typeId()) << "\n";
      }
      auto tmpSeg = drcEdge->getSegment();
      // std::cout << "  seg: (" << low(edgeSeg).x() / 2000.0 << ", " << low(edgeSeg).y() / 2000.0 << ") -- ("
      //           << high(edgeSeg).x() / 2000.0 << ", " << high(edgeSeg).y() / 2000.0 << ") on layer " << layerNum << "\n"; 
      std::cout << "  edge: (" << low(tmpSeg).x() / 2000.0 << ", " << low(tmpSeg).y() / 2000.0 << ") -- ("
                << high(tmpSeg).x() / 2000.0 << ", " << high(tmpSeg).y() / 2000.0 << ") layerNum = " << drcEdge->getLayerNum() << "\n"; 
    }



    auto layerNum = drcEdge->getLayerNum();
    auto edgeSeg = drcEdge->getSegment();
    frCoord tmpLowX = std::min(low(edgeSeg).x(), high(edgeSeg).x());
    frCoord tmpLowY = std::min(low(edgeSeg).y(), high(edgeSeg).y());
    frCoord tmpHighX = std::max(low(edgeSeg).x(), high(edgeSeg).x());
    frCoord tmpHighY = std::max(low(edgeSeg).y(), high(edgeSeg).y());
    segment_t rtreeSeg(point_t(tmpLowX, tmpLowY), point_t(tmpHighX, tmpHighY));
    if (isAdd) {
      layer2EdgeRTree[layerNum].insert(std::make_pair(rtreeSeg, drcEdge));
    } else {
      layer2EdgeRTree[layerNum].remove(std::make_pair(rtreeSeg, drcEdge));
    }
  }
}

void fr::DRCWorker::setupMaxRectRTree() {
  for (auto &drcNet: drcNets) {
    setupMaxRectRTree_net(drcNet.get());
  }
}

void fr::DRCWorker::setupFixedMaxRectRTree_net(DRCNet* net) {
  int netId = net->getId();
  auto &layer2FixedPolySet = net->getLayer2FixedPolySet();
  // build fixedRTree
  updateMaxRectRTree(netId, layer2FixedPolySet, layer2FixedRTree, true);
}

void fr::DRCWorker::setupMergedMaxRectRTree_net(DRCNet* net, bool isAdd) {
  int netId = net->getId();
  std::set<frLayerNum> layerNums;
  std::map<frLayerNum, PolygonSet> layer2MergedPolySet;
  auto &layer2FixedPolySet = net->getLayer2FixedPolySet();
  auto &layer2RoutePolySet = net->getLayer2RoutePolySet();
  // get set of merged layerNums
  for (auto it = layer2FixedPolySet.begin(); it != layer2FixedPolySet.end(); ++it) {
    layerNums.insert(it->first);
  }
  for (auto it = layer2RoutePolySet.begin(); it != layer2RoutePolySet.end(); ++it) {
    layerNums.insert(it->first);
  }
  // merge
  for (auto layerNum: layerNums) {
    if (layer2FixedPolySet.find(layerNum) != layer2FixedPolySet.end()) {
      layer2MergedPolySet[layerNum] += layer2FixedPolySet[layerNum];
    }
    if (layer2RoutePolySet.find(layerNum) != layer2RoutePolySet.end()) {
      layer2MergedPolySet[layerNum] += layer2RoutePolySet[layerNum];
    }
  }
  updateMaxRectRTree(netId, layer2MergedPolySet, layer2MergedRTree, isAdd);
 
}

// divide this funciton to separate two functions, one for fixed, one for route
// when done, reset route poly set
void fr::DRCWorker::setupMaxRectRTree_net(DRCNet* net) {
  setupFixedMaxRectRTree_net(net);
  setupMergedMaxRectRTree_net(net, true);
  // int netId = net->getId();
  // std::set<frLayerNum> layerNums;
  // std::map<frLayerNum, PolygonSet> layer2MergedPolySet;
  // auto &layer2FixedPolySet = net->getLayer2FixedPolySet();
  // auto &layer2RoutePolySet = net->getLayer2RoutePolySet();
  
  // // build fixedRTree
  // updateMaxRectRTree(netId, layer2FixedPolySet, layer2FixedRTree, true);

  // // build mergedRTree

  // // get set of merged layerNums
  // for (auto it = layer2FixedPolySet.begin(); it != layer2FixedPolySet.end(); ++it) {
  //   layerNums.insert(it->first);
  // }
  // for (auto it = layer2RoutePolySet.begin(); it != layer2RoutePolySet.end(); ++it) {
  //   layerNums.insert(it->first);
  // }

  // for (auto layerNum: layerNums) {
  //   if (layer2FixedPolySet.find(layerNum) != layer2FixedPolySet.end()) {
  //     layer2MergedPolySet[layerNum] += layer2FixedPolySet[layerNum];
  //   }
  //   if (layer2RoutePolySet.find(layerNum) != layer2RoutePolySet.end()) {
  //     layer2MergedPolySet[layerNum] += layer2RoutePolySet[layerNum];
  //   }
  // }
  // updateMaxRectRTree(netId, layer2MergedPolySet, layer2MergedRTree, true);

}

void fr::DRCWorker::updateMaxRectRTree(int netId, 
                                       const std::map<frLayerNum, PolygonSet> &layer2PolySet, 
                                       std::map<frLayerNum, bgi::rtree<std::pair<box_t, int>, bgi::quadratic<16> > > &maxRectRTree,
                                       bool isAdd) {
  for (auto it = layer2PolySet.begin(); it != layer2PolySet.end(); ++it) {
    frLayerNum layerNum = it->first;
    auto &polySet = it->second;
    std::vector<Rectangle> maxRects;
    get_max_rectangles(maxRects, polySet);
    for (auto &maxRect: maxRects) {
      box_t boostb = box_t(point_t(xl(maxRect), yl(maxRect)), point_t(xh(maxRect), yh(maxRect)));
      if (isAdd) {
        maxRectRTree[layerNum].insert(std::make_pair(boostb, netId));
      } else {
        maxRectRTree[layerNum].remove(std::make_pair(boostb, netId));
      }
    }
  }
}


