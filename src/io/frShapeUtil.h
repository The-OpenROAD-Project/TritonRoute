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

#include "frBaseTypes.h"
#include "db/obj/frShape.h"
#include <algorithm>
#include "db/obj/frBlockage.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp> 
#include <boost/foreach.hpp>
#include <tuple>
#include <iostream>

namespace fr {
  void frRect2Poly(frRect &rectIn, boostPolygon &polyOut);
  void frPolygon2Poly(frPolygon &polygonIn, boostPolygon &polyOut);
  void frBox2Poly(frBox &boxIn, boostPolygon &polyOut);
  void frBox2Rectangle(frBox &boxIn, Rectangle &rectOut);
  void frRect2Rectangle(frRect &rectIn, Rectangle &rectOut);
  double frBox2BoxDist(frBox &box1, frBox &box2);
  double frBox2BoxDist(box_t &box1, box_t &box2);
  //void frBlockage2Poly(frBlockage &blockageIn, boostPolygon &polyOut);
  bool isSamePoint(boostPoint &pt1, boostPoint &pt2);
  void polyCovering(boostPolygon &polygon, frCollection<frRect> &rects);
  void maintainMaxRects(std::tuple<int, int, int, int> &newRect, std::set<std::tuple<int, int, int, int> > &maxRectSet);
  bool processNewRect(std::tuple<int, int, int, int> &newRect, std::set<std::tuple<int, int, int, int> > &maxRectSet);
  bool getMaxHSlice(int xIdx, int yIdx, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::pair<int, int> &retPt);
  bool getMaxVSlice(int xIdx, int yIdx, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::pair<int, int> &retPt);
  void getMaxHRect(std::pair<int, int> startPt, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::set<std::tuple<int, int, int, int> > &maxRectSet);
  void getMaxVRect(std::pair<int, int> startPt, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::set<std::tuple<int, int, int, int> > &maxRectSet);
  frCoord getEdgeWidth(boostEdge &edge);
  void getPolyWithHole(const std::vector<Point> &vertices, Polygon &outline, std::vector<Polygon> &holes);
  void getPolyWithHole(const std::vector<Point> &vertices, std::vector<Polygon> &polys);
  void getPolyWithHole(const Polygon &polyIn, std::vector<Polygon> &polys);
  void getPolyWithHole_new(const Polygon &polyIn, std::vector<Polygon> &polys);
  bool isColinear(const Point &pt1, const Point &pt2, const Point &pt3);

}
