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

#include "io/frShapeUtil.h"
#include <boost/icl/interval_map.hpp>

namespace fr {
  void frRect2Poly(frRect &rectIn, boostPolygon &polyOut) {
    frBox tmpBox;
    rectIn.getBBox(tmpBox);
    polyOut = boostPolygon();
    bg::append(polyOut, boostPoint(tmpBox.left(), tmpBox.bottom()));
    bg::append(polyOut, boostPoint(tmpBox.left(), tmpBox.top()));
    bg::append(polyOut, boostPoint(tmpBox.right(), tmpBox.top()));
    bg::append(polyOut, boostPoint(tmpBox.right(), tmpBox.bottom()));
    bg::append(polyOut, boostPoint(tmpBox.left(), tmpBox.bottom()));
  }

  void frPolygon2Poly(frPolygon &polygonIn, boostPolygon &polyOut) {
    polyOut = boostPolygon();
    auto polyPoints = polygonIn.getPoints();
    for (auto &point: polyPoints) {
      bg::append(polyOut, boostPoint((int)point.x(), (int)point.y()));
    }
  }

  void frBox2Poly(frBox &boxIn, boostPolygon &polyOut) {
    polyOut = boostPolygon();
    bg::append(polyOut, boostPoint(boxIn.left(), boxIn.bottom()));
    bg::append(polyOut, boostPoint(boxIn.left(), boxIn.top()));
    bg::append(polyOut, boostPoint(boxIn.right(), boxIn.top()));
    bg::append(polyOut, boostPoint(boxIn.right(), boxIn.bottom()));
    bg::append(polyOut, boostPoint(boxIn.left(), boxIn.bottom()));
  }

  void frBox2Rectangle(frBox &boxIn, Rectangle &rectOut) {
    rectOut = Rectangle(boxIn.left(), boxIn.bottom(), boxIn.right(), boxIn.top());
  }

  void frRect2Rectangle(frRect &rectIn, Rectangle &rectOut) {
    frBox rectInBox;
    rectIn.getBBox(rectInBox);
    rectOut = Rectangle(rectInBox.left(), rectInBox.bottom(), rectInBox.right(), rectInBox.top());
  }

  //void frBlockage2Poly(frBlockage &blockageIn, boostPolygon &polyOut) {
  //  auto points = blockageIn.getPoints();
  //  polyOut = boostPolygon();
  //  for (auto point: points) {
  //    bg::append(polyOut, boostPoint(point.x(), point.y()));
  //  }
  //}

  // Use before checking overlapping
  double frBox2BoxDist(frBox &box1, frBox &box2) {
    frCoord xDist = std::max(0, std::max(box1.left(), box2.left()) - std::min(box1.right(), box2.right()) );
    frCoord yDist = std::max(0, std::max(box1.bottom(), box2.bottom()) - std::min(box1.top(), box2.top()));
    return std::sqrt(xDist * xDist + yDist * yDist);
  }

  double frBox2BoxDist(box_t &box1, box_t &box2) {
    frCoord llx1, lly1, urx1, ury1, llx2, lly2, urx2, ury2;
    llx1 = bg::get<bg::min_corner, 0>(box1);
    lly1 = bg::get<bg::min_corner, 1>(box1);
    urx1 = bg::get<bg::max_corner, 0>(box1);
    ury1 = bg::get<bg::max_corner, 1>(box1);
    llx2 = bg::get<bg::min_corner, 0>(box2);
    lly2 = bg::get<bg::min_corner, 1>(box2);
    urx2 = bg::get<bg::max_corner, 0>(box2);
    ury2 = bg::get<bg::max_corner, 1>(box2);
    frCoord xDist = std::max(0, std::max(llx1, llx2) - std::min(urx1, urx2));
    frCoord yDist = std::max(0, std::max(lly1, lly2) - std::min(ury1, ury2));
    return std::sqrt(xDist * xDist + yDist * yDist);
  }

  bool isSamePoint(boostPoint &pt1, boostPoint &pt2) {
    auto x1 = bg::get<0>(pt1);
    auto y1 = bg::get<1>(pt1);
    auto x2 = bg::get<0>(pt2);
    auto y2 = bg::get<1>(pt2);
    return (x1 == x2 && y1 == y2);
  }

  frCoord getEdgeWidth(boostEdge &edge) {
    auto point1 = edge.first;
    auto point2 = edge.second;
    auto pt1X = bg::get<0>(point1);
    auto pt1Y = bg::get<1>(point1);
    auto pt2X = bg::get<0>(point2);
    auto pt2Y = bg::get<1>(point2);

    if (pt1X == pt2X) {
      return (std::abs(pt2Y - pt1Y));
    } else if (pt1Y == pt2Y) {
      return (std::abs(pt2X - pt1X));
    } else {
      std::cout << "Warning: edge is not orthogonal\n";
      return 0;
    }

  }

  void polyCovering(boostPolygon &polygon, frCollection<frRect> &rects) {
    // debug 
    // std::cout << "  start polyCovering\n" << std::flush; 
    frCollection<frCoord> x, y;
    frCollection<boostPoint> points;
    std::set<frCoord> xIntv, yIntv;
    std::vector<frCoord> xTick, yTick;
    std::set<std::pair<int,int> > HSliceStart, VSliceStart;
    std::vector<std::vector<bool> > isOccupied;
    std::set<std::pair<int, int> > maxHSlice, maxVSlice;
    std::set<std::tuple<int, int, int, int> > maxRectSet;
    std::pair<int, int> tmpPoint;



    
    // exterior ring
    auto ring = bg::exterior_ring(polygon);
    for (auto it = ring.begin(); it != ring.end(); ++it) {
      points.push_back(*it);
    }
    // interior rings
    BOOST_FOREACH(auto & ring, bg::interior_rings(polygon)) {
      for (auto it = ring.begin(); it != ring.end(); ++it) {
        points.push_back(*it);
      }
    }

    // push_back to x and y 
    for (auto &point: points) {
      x.push_back(point.get<0>());
      y.push_back(point.get<1>());
    }
    xIntv = std::set<frCoord>(x.begin(), x.end());
    yIntv = std::set<frCoord>(y.begin(), y.end());

    std::copy(xIntv.begin(), xIntv.end(), std::back_inserter(xTick));
    std::copy(yIntv.begin(), yIntv.end(), std::back_inserter(yTick));

    // std::cout << "    polyCovering: num points = " << points.size() << std::endl;
    // std::cout << "    polyCovering: |xTick| = " << xTick.size() << ", |yTick| = " << yTick.size() << std::endl;

    isOccupied = std::vector<std::vector<bool> >(xTick.size() - 1, std::vector<bool>(yTick.size() - 1, false));

    // debug 
    // std::cout << "    start building k-map\n" << std::flush; 
    // build k-map
    for (int i = 0; i < (int)xTick.size() - 1; i++) {
      for (int j = 0; j < (int)yTick.size() - 1; j++) {
        frCoord tempX = (xTick.at(i) + xTick.at(i + 1)) / 2;
        frCoord tempY = (yTick.at(j) + yTick.at(j + 1)) / 2;
        boostPoint tempPt(tempX, tempY);
        if (bg::covered_by(tempPt, polygon)) {
          isOccupied[i][j] = true;
        }
      }
    }

    // printBoolMatrix(isOccupied);

    // debug 
    // std::cout << "    start building slices\n" << std::flush; 
    // build slices
    for (int i = 0; i < (int)isOccupied.size(); i++) {
      for (int j = 0; j < (int)isOccupied[i].size(); j++) {
        if (isOccupied[i][j] == false) {
          continue;
        } else {
          // horizontal
          // debug
          // std::cout << "      H slices\n" << std::flush;
          if (getMaxHSlice(i, j, isOccupied, maxHSlice, tmpPoint)) {
            // std::cout << "        getting max H rect\n" << std::flush;
            // std::cout << "          starting point = (" << tmpPoint.first << ", " << tmpPoint.second << ")\n";
            getMaxHRect(tmpPoint, isOccupied, maxHSlice, maxRectSet);
            // std::cout << "        finish getting max H rect\n" << std::flush;
          }
          // vertical
          // debug
          // std::cout << "      V slices\n" << std::flush;
          if (getMaxVSlice(i, j, isOccupied, maxVSlice, tmpPoint)) {
            // std::cout << "        getting max V rect\n" << std::flush;
            // std::cout << "          starting point = (" << tmpPoint.first << ", " << tmpPoint.second << ")\n";
            getMaxVRect(tmpPoint, isOccupied, maxVSlice, maxRectSet);
            // std::cout << "        finish getting max V rect\n" << std::flush;
          }
        }
      }
    }

    // debug 
    // std::cout << "    start outputing result\n" << std::flush; 
    // std::cout << "      num rects = " << maxRectSet.size() << std::endl;
    for (auto it = maxRectSet.begin(); it != maxRectSet.end(); ++it) {
      auto &tmpTuple = *it;
      // std::cout << "       (" << get<0>(tmpTuple) << ", " << get<1>(tmpTuple) << ") -- (" 
      //           << get<2>(tmpTuple) << ", " << get<3>(tmpTuple) << ")" << std::endl;
      frBox tmpBox;
      frRect tmpRect;
      tmpBox.set(xTick[get<0>(tmpTuple)], yTick[get<1>(tmpTuple)], xTick[get<2>(tmpTuple) + 1], yTick[get<3>(tmpTuple) + 1]);
      tmpRect.setBBox(tmpBox);
      rects.push_back(tmpRect);
    }

  }

  // every rect in maxRectSet is maximum among all rects seen so far
  void maintainMaxRects(std::tuple<int, int, int, int> &newRect, std::set<std::tuple<int, int, int, int> > &maxRectSet) {
    // std::cout << "processNewRect of (" << std::get<0>(newRect) << ", " << std::get<1>(newRect) << ") -- (" 
    //           << std::get<2>(newRect) << ", " << std::get<3>(newRect) << ")" << std::endl;

    boostBox tmpNewRect(boostPoint(std::get<0>(newRect), std::get<1>(newRect)), boostPoint(std::get<2>(newRect), std::get<3>(newRect)));
    
    if (maxRectSet.size() == 0) {
      // std::cout << "@@@debug@@@: insert (" << std::get<0>(newRect) << ", " << std::get<1>(newRect) 
      //           << ") -- (" << std::get<2>(newRect) << "," << std::get<3>(newRect) << ")\n";
      
      maxRectSet.insert(newRect);
      return;
    }

    if (maxRectSet.find(newRect) != maxRectSet.end()) {
      return;
    }
    
    std::vector<std::tuple<int, int, int, int> > coveredRects;
    // isSkip = false;
    for (auto it = maxRectSet.begin(); it != maxRectSet.end(); it++) {
      boostBox tmpRect(boostPoint(std::get<0>(*it), std::get<1>(*it)), boostPoint(std::get<2>(*it), std::get<3>(*it)));
      // std::cout << "    checking against (" << std::get<0>(*it) << ", " << std::get<1>(*it) << ") -- (" 
            // << std::get<2>(*it) << ", " << std::get<3>(*it) << ")" << std::endl;
      if (bg::covered_by(tmpNewRect, tmpRect)) {
        // std::cout << "@@@debug@@@: covered_by (" << std::get<0>(*it) << ", " << std::get<1>(*it) 
        //           << ") -- (" << std::get<2>(*it) << "," << std::get<3>(*it) << ")\n";
      
        // newRect is strictly within some existing Rect
        return;
      } else if (bg::covered_by(tmpRect, tmpNewRect)) {
        coveredRects.push_back(*it);
        // maxRectSet.erase(it);
        // maxRectSet.insert(newRect);
        // break;
      }
    }

    if (!coveredRects.empty()) {
      for (int i = 0; i < (int)coveredRects.size(); i++) {
        auto it = maxRectSet.find(coveredRects[i]);
        // std::cout << "@@@debug@@@: erase (" << std::get<0>(coveredRects[i]) << ", " << std::get<1>(coveredRects[i]) 
        //         << ") -- (" << std::get<2>(coveredRects[i]) << "," << std::get<3>(coveredRects[i]) << ")\n";
      
        maxRectSet.erase(it);
      }
      
    }
    // std::cout << "@@@debug@@@: insert (" << std::get<0>(newRect) << ", " << std::get<1>(newRect) 
    //             << ") -- (" << std::get<2>(newRect) << "," << std::get<3>(newRect) << ")\n";
      
    maxRectSet.insert(newRect);
    return;
  }

  bool processNewRect(std::tuple<int, int, int, int> &newRect, std::set<std::tuple<int, int, int, int> > &maxRectSet) {
    // std::cout << "processNewRect of (" << std::get<0>(newRect) << ", " << std::get<1>(newRect) << ") -- (" 
    //           << std::get<2>(newRect) << ", " << std::get<3>(newRect) << ")" << std::endl;

    boostBox tmpNewRect(boostPoint(std::get<0>(newRect), std::get<1>(newRect)), boostPoint(std::get<2>(newRect), std::get<3>(newRect)));
    bool isChanged = true;
    if (maxRectSet.find(newRect) != maxRectSet.end()) {
      return false;
    }
    if (maxRectSet.size() == 0) {
      // std::cout << "@@@debug@@@: insert (" << std::get<0>(newRect) << ", " << std::get<1>(newRect) 
      //           << ") -- (" << std::get<2>(newRect) << "," << std::get<3>(newRect) << ")\n";
      maxRectSet.insert(newRect);

      return true;
    }
    
    isChanged = false;
    std::vector<std::tuple<int, int, int, int> > coveredRects;
    // isSkip = false;
    for (auto it = maxRectSet.begin(); it != maxRectSet.end(); it++) {
      boostBox tmpRect(boostPoint(std::get<0>(*it), std::get<1>(*it)), boostPoint(std::get<2>(*it), std::get<3>(*it)));
      // std::cout << "    checking against (" << std::get<0>(*it) << ", " << std::get<1>(*it) << ") -- (" 
            // << std::get<2>(*it) << ", " << std::get<3>(*it) << ")" << std::endl;
      if (bg::covered_by(tmpNewRect, tmpRect)) {
        // newRect is strictly within some existing Rect
        return false;
      } else if (bg::covered_by(tmpRect, tmpNewRect)) {
        coveredRects.push_back(*it);
        // maxRectSet.erase(it);
        // maxRectSet.insert(newRect);
        isChanged = true;
        break;
      }
    }
    if (isChanged == true) {
      for (int i = 0; i < (int)coveredRects.size(); i++) {
        auto it = maxRectSet.find(coveredRects[i]);
        maxRectSet.erase(it);
      }
      return true;
    }

    return false;
  }

  bool getMaxHSlice(int xIdx, int yIdx, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::pair<int, int> &retPt) {
    // std::cout << "getMaxHSlice" << std::endl;
    int xl, xh;
    xl = xh = xIdx;
    while (xl - 1 >= 0 && isOccupied[xl - 1][yIdx] == true) {
      xl--;
    }
    while (xh + 1 < (int)isOccupied.size() && isOccupied[xh + 1][yIdx] == true) {
      xh++;
    }
    std::pair<int, int> tmpPt = std::make_pair(xl, yIdx);
    retPt = tmpPt;
    if (sliceSet.find(tmpPt) != sliceSet.end()) {
      return false;
    } else {
      sliceSet.insert(tmpPt);
      return true;
    }
  }



  bool getMaxVSlice(int xIdx, int yIdx, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::pair<int, int> &retPt) {
    // std::cout << "getMaxVSlice" << std::endl;
    int yl, yh;
    yl = yh = yIdx;
    while (yl - 1 >= 0 && isOccupied[xIdx][yl - 1] == true) {
      yl--;
    }
    while (yh + 1 < (int)isOccupied[xIdx].size() && isOccupied[xIdx][yh + 1] == true) {
      yh++;
    }
    std::pair<int, int> tmpPt = std::make_pair(xIdx, yl);
    retPt = tmpPt;
    if (sliceSet.find(tmpPt) != sliceSet.end()) {
      return false;
    } else {
      sliceSet.insert(tmpPt);
      return true;
    }
  }


  void getMaxHRect(std::pair<int, int> startPt, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::set<std::tuple<int, int, int, int> > &maxRectSet) {
    // std::cout << "getMaxHRect start pnt = (" << startPt.first << ", " << startPt.second << ")" << std::endl;
    int xl, xh, yl, yh;
    xl = xh = startPt.first;
    yl = yh = startPt.second;
    while (xh + 1 < (int)isOccupied.size() && isOccupied[xh + 1][startPt.second] == true) {
      xh++;
    }
    

    // upward
    bool flag = true;
    while (flag) {
      // std::cout << "      upward\n" << std::flush;
      if (yh + 1 < (int)isOccupied[xl].size()) {
        for (int tempX = xl; tempX <= xh; tempX++) {
          if (isOccupied[tempX][yh + 1] == false) {
            flag = false;
            break;
          }
        }
        if (flag == true) {
          yh++;
        }
      } else {
        flag = false;
        break;
      }
    }

    // downward
    flag = true;
    while (flag) {
      // std::cout << "      downward\n" << std::flush;
      if (yl - 1 >= 0) {
        // yl--;
        for (int tempX = xl; tempX <= xh; tempX++) {
          if (isOccupied[tempX][yl - 1] == false) {
            flag = false;
            break;
          }
        }
        if (flag == true) {
          yl--;
        }
      } else {
        flag = false;
        break;
      }
    }
    // std::cout << "xl = " << xl << ", " << "xh = " << xh << std::endl;
    // std::cout << "yl = " << yl << ", yh = " << yh << std::endl;
    std::tuple<int, int, int, int> tmpTuple = std::make_tuple(xl, yl, xh, yh);
    // printRect(tmpTuple);
    maintainMaxRects(tmpTuple, maxRectSet);
    // return (maintainRects(tmpTuple, maxRectSet));

  }


  void getMaxVRect(std::pair<int, int> startPt, std::vector<std::vector<bool> > &isOccupied, std::set<std::pair<int, int> > &sliceSet, std::set<std::tuple<int, int, int, int> > &maxRectSet) {
    // std::cout << "getMaxVRect start pnt = (" << startPt.first << ", " << startPt.second << ")" << std::endl;
    int xl, xh, yl, yh;
    xl = xh = startPt.first;
    yl = yh = startPt.second;
    while (yh + 1 < (int)isOccupied[startPt.first].size() && isOccupied[startPt.first][yh + 1]) {
      yh++;
    }

    // upward
    bool flag = true;
    while (flag) {
      if (xh + 1 < (int)isOccupied.size()) {
        // xh++;
        for (int tempY = yl; tempY <= yh; tempY++) {
          if (isOccupied[xh + 1][tempY] == false) {
            flag = false;
            break;
          }
        }
        if (flag == true) {
          xh++;
        }
      } else {
        flag = false;
      }
    }
    // downward
    flag = true;
    while (flag) {
      if (xl - 1 >= 0) {
        // xl--;
        for (int tempY = yl; tempY <= yh; tempY++) {
          if (isOccupied[xl - 1][tempY] == false) {
            flag = false;
            break;
          }
        }
        if (flag == true) {
          xl--;
        }
      } else {
        flag = false;
      }
    }
    
    // std::cout << "xl = " << xl << ", " << "xh = " << xh << std::endl;
    // std::cout << "yl = " << yl << ", " << "yh = " << yh << std::endl;
    std::tuple<int, int, int, int> tmpTuple = std::make_tuple(xl, yl, xh, yh);
    maintainMaxRects(tmpTuple, maxRectSet);
    // return (processNewRect(tmpTuple, maxRectSet));

  }

  void getPolyWithHole(const std::vector<Point> &vertices, Polygon &outline, std::vector<Polygon> &holes) {
    typedef boost::icl::interval_map<int, int> IntvMap;
    std::vector<Polygon> polys;
    std::vector<std::pair<Point, Point> > edges;
    std::vector<bool> validEdges;
    std::map<int, IntvMap> horzIntv2Edge, vertIntv2Edge;
    for (int i = 0; i < (int)vertices.size(); ++i) {
      Point currPt = vertices[i];
      Point nextPt = vertices[(i + 1) % vertices.size()];
      edges.push_back(std::make_pair(currPt, nextPt));
      validEdges.push_back(true);

      int edgeIdx = -1;
      bool isHorizontal = true;
      auto itResHorz = horzIntv2Edge[nextPt.y()].equal_range(boost::icl::interval<int>::closed(nextPt.x(), nextPt.x()));
      int horzCnt = 0;
      for (auto it = itResHorz.first; it != itResHorz.second; ++it) {
        if (horzCnt > 0) {
          std::cout << "Error: more than one interval found\n";
          break;
        }
        if ((it->second - 1) > edgeIdx && validEdges[(it->second  - 1)] == true) {
          edgeIdx = it->second - 1;
          isHorizontal = true;
        }
        horzCnt++;
      }
      // vert
      auto itResVert = vertIntv2Edge[nextPt.x()].equal_range(boost::icl::interval<int>::closed(nextPt.y(), nextPt.y()));
      int vertCnt = 0;
      for (auto it = itResVert.first; it != itResVert.second; ++it) {
        if (vertCnt > 0) {
          std::cout << "Error: more than one interval found\n";
          break;
        }
        if ((it->second  - 1) > edgeIdx && validEdges[(it->second  - 1)] == true) {
          edgeIdx = it->second - 1;
          isHorizontal = false;
        }
        vertCnt++;
      }
      // get poly
      if (edgeIdx == -1) {
        ;
      } else if ((i - edgeIdx) > 2) {
        std::vector<Point> vertices;
        int edgeCnt = 0;
        for (int currEdgeIdx = edgeIdx; currEdgeIdx <= i; ++currEdgeIdx) {
          if (validEdges[currEdgeIdx] == false) {
            continue;
          }
          edgeCnt++;
          vertices.push_back(edges[currEdgeIdx].second);
          if (currEdgeIdx == edgeIdx) {
            if (nextPt == edges[currEdgeIdx].first) {
              // cout << "  invalidate " << currEdgeIdx << endl;
              validEdges[currEdgeIdx] = false;
            } else {
              Point tmpEndPoint = edges[currEdgeIdx].second;
              edges[currEdgeIdx].second = nextPt;
              if (isHorizontal) {
                horzIntv2Edge[nextPt.y()] -= std::make_pair(boost::icl::interval<int>::right_open(std::min(tmpEndPoint.x(), nextPt.x()), std::max(tmpEndPoint.x(), nextPt.x())), edgeIdx);
                // cout << "  erase y = " << nextPt.y() << ", x = [" << min(tmpEndPoint.x(), nextPt.x()) << ", " << max(tmpEndPoint.x(), nextPt.x()) << "] edgeIdx = " << edgeIdx << "\n";
              } else {
                vertIntv2Edge[nextPt.x()] -= std::make_pair(boost::icl::interval<int>::right_open(std::min(tmpEndPoint.y(), nextPt.y()), std::max(tmpEndPoint.y(), nextPt.y())), edgeIdx);
                // cout << "  erase x = " << nextPt.x() << ", x = [" << min(tmpEndPoint.y(), nextPt.y()) << ", " << max(tmpEndPoint.y(), nextPt.y()) << "] edgeIdx = " << edgeIdx << "\n";
              }
            }
          } else {
            // cout << "  invalidate " << currEdgeIdx << endl;
            validEdges[currEdgeIdx] = false;
          }
        }
        if (edgeCnt > 2) {
          // cout << "poly startIdx = " << edgeIdx << ", endIdx = " << i << endl;
          Polygon tmpPoly;
          Point pt1, pt2, pt3;
          pt1 = vertices.back();
          pt2 = vertices.front();
          pt3 = vertices[vertices.size() - 2];
          // cout << "xxx2\n";

          if ((pt1.x() == pt2.x() && pt2.x() == pt3.x()) ||
              (pt1.y() == pt2.y() && pt2.y() == pt3.y())) {
            vertices.pop_back();
          }
          set_points(tmpPoly, vertices.begin(), vertices.end());
          polys.push_back(tmpPoly);
        }
      } else {
        // cout << "erase startIdx = " << edgeIdx << ", endIdx = " << i << endl;
        for (int currEdgeIdx = edgeIdx; currEdgeIdx <= i; ++currEdgeIdx) {
          if (validEdges[currEdgeIdx] == false) {
            continue;
          }
          if (currEdgeIdx == edgeIdx) {
            if (nextPt == edges[currEdgeIdx].first) {
              validEdges[currEdgeIdx] = false;
            } else {
              edges[currEdgeIdx].second = nextPt;
            }
          } else {
            validEdges[currEdgeIdx] = false;
          }
        }
      }

      // insert to interval map
      if (edgeIdx == -1) {
        if (currPt.x() == nextPt.x()) {
          auto edgeIntv = boost::icl::interval<int>::closed(std::min(currPt.y(), nextPt.y()), std::max(currPt.y(), nextPt.y()));
          vertIntv2Edge[nextPt.x()] += std::make_pair(edgeIntv, i + 1);
          // cout << "add x = " << currPt.x() << ", y = [" << edgeIntv.lower() << ", " << edgeIntv.upper() << "]\n";
        } 
        if (currPt.y() == nextPt.y()) {
          auto edgeIntv = boost::icl::interval<int>::closed(std::min(currPt.x(), nextPt.x()), std::max(currPt.x(), nextPt.x()));
          horzIntv2Edge[nextPt.y()] += std::make_pair(edgeIntv, i + 1);
          // cout << "add y = " << currPt.y() << ", x = [" << edgeIntv.lower() << ", " << edgeIntv.upper() << "]\n";
        }
      }

    }
    
    std::vector<Point> tmpPts;
    for (int currEdgeIdx = 0; currEdgeIdx < int(edges.size()); ++currEdgeIdx) {
      if (validEdges[currEdgeIdx] == false) {
        continue;
      }
      tmpPts.push_back(edges[currEdgeIdx].second);
      // std::cout << "pushing (" << tmpPts.back().x() << ", " << tmpPts.back().y() << "\n";
    }

    for (int i = 0; i < (int)polys.size() - 1; ++i) {
      holes.push_back(polys[i]);
    }
    if (polys.size() > 0) {
      outline = polys.back();
    }


  }

  bool isColinear(const Point &pt1, const Point &pt2, const Point &pt3) {
    return ((pt1.x() == pt2.x() && pt2.x() == pt3.x()) ||
            (pt1.y() == pt2.y() && pt2.y() == pt3.y()));
  }

  void getNonColinearVertex(const std::vector<Point> &in, std::vector<Point> &out) {
    int inSize = in.size();
    int cornerIdx = -1;
    int startIdx, endIdx;
    Point lastCorner;
    std::vector<int> cornerIdxs;
    if (inSize < 3) {
      out = in;
      return;
    }
    Point tmpPt1 = in[0];
    Point tmpPt2 = in[1];
    Point tmpPt3;
    for (int i = 2; i < inSize; ++i) {
      tmpPt3 = in[i];
      if (!isColinear(tmpPt1, tmpPt2, tmpPt3)) {
        cornerIdx = i - 1;
        break;
      }
    }
    if (cornerIdx == -1) {
      out.push_back(in.front());
      out.push_back(in.back());
      return;
    } else {
      cornerIdxs.push_back(cornerIdx);
      lastCorner = in[cornerIdx];
    }
    //
    for (int i = cornerIdx+1; i < cornerIdx + inSize; ++i) {
      startIdx = i % inSize;
      tmpPt1 = in[startIdx];
      // find next corner
      for (int j = i + 1; j <= cornerIdx + inSize; ++j) {
        endIdx = j % inSize;
        tmpPt2 = in[endIdx];
        if (!isColinear(lastCorner, tmpPt1, tmpPt2)) {
          int lastCornerIdx = (endIdx + inSize - 1) % inSize;
          cornerIdxs.push_back(lastCornerIdx);
          lastCorner = in[lastCornerIdx];
          i = j-1;
          break;
        }
      }
    }
    for (int i = 0; i < (int)cornerIdxs.size(); ++i) {
      out.push_back(in[cornerIdxs[i]]);
    }
    return;

  }

  void getPolyWithHole_new(const Polygon &polyIn, std::vector<Polygon> &polys) {
    using namespace boost::polygon::operators;
    using PolygonX = boost::polygon::polygon_90_with_holes_data<int>;
    using PolygonSetData = boost::polygon::polygon_90_set_data<int>;
    PolygonSetData psd;
    psd += polyIn;
    std::vector<PolygonX> psx;
    psd.get(psx);
    
    for (auto &poly: psx) {
      Polygon outlinePoly;
      std::vector<Point> outlinePolyVtx;
      for (auto it = poly.begin(); it != poly.end(); it++) {
        outlinePolyVtx.push_back(*it);
      }
      boost::polygon::set_points(outlinePoly, outlinePolyVtx.rbegin(), outlinePolyVtx.rend());
      for (auto it1 = poly.begin_holes(); it1 != poly.end_holes(); it1++) {
        std::vector<Point> vtx;
        Polygon holePoly;
        for (auto it2 = (*it1).begin(); it2 != (*it1).end(); it2++) {
          vtx.push_back(*it2);
        }
        boost::polygon::set_points(holePoly, vtx.rbegin(), vtx.rend());
        polys.push_back(holePoly);
      }
      polys.push_back(outlinePoly);
    }
  }

  void getPolyWithHole(const Polygon &polyIn, std::vector<Polygon> &polys) {
    typedef boost::icl::interval_map<int, int> IntvMap;
    std::vector<Point> origVertices, vertices;
    for (auto ptIt = begin_points(polyIn); ptIt != end_points(polyIn); ++ptIt) {
      origVertices.push_back(*ptIt);
    }

    // getNonColinearVertex(origVertices, vertices);
    vertices = origVertices;
    std::vector<std::pair<Point, Point> > edges;
    std::vector<bool> validEdges;
    std::map<int, IntvMap> horzIntv2Edge, vertIntv2Edge;
    for (int i = 0; i < (int)vertices.size(); ++i) {
      Point currPt = vertices[i];
      Point nextPt = vertices[(i + 1) % vertices.size()];
      edges.push_back(std::make_pair(currPt, nextPt));
      validEdges.push_back(true);

      int edgeIdx = -1;
      bool isHorizontal = true;
      auto itResHorz = horzIntv2Edge[nextPt.y()].equal_range(boost::icl::interval<int>::closed(nextPt.x(), nextPt.x()));
      int horzCnt = 0;
      for (auto it = itResHorz.first; it != itResHorz.second; ++it) {
        if (horzCnt > 0) {
          std::cout << "Error: more than one interval found\n";
          break;
        }
        if ((it->second - 1) > edgeIdx && validEdges[(it->second  - 1)] == true) {
          edgeIdx = it->second - 1;
          isHorizontal = true;
        }
        horzCnt++;
      }
      // vert
      auto itResVert = vertIntv2Edge[nextPt.x()].equal_range(boost::icl::interval<int>::closed(nextPt.y(), nextPt.y()));
      int vertCnt = 0;
      for (auto it = itResVert.first; it != itResVert.second; ++it) {
        if (vertCnt > 0) {
          std::cout << "Error: more than one interval found\n";
          break;
        }
        if ((it->second  - 1) > edgeIdx && validEdges[(it->second  - 1)] == true) {
          edgeIdx = it->second - 1;
          isHorizontal = false;
        }
        vertCnt++;
      }
      // get poly
      if (edgeIdx == -1) {
        ;
      } else if ((i - edgeIdx) > 2) {
        // std::cout << "push new poly\n";
        std::vector<Point> vertices;
        int edgeCnt = 0;
        for (int currEdgeIdx = edgeIdx; currEdgeIdx <= i; ++currEdgeIdx) {
          if (validEdges[currEdgeIdx] == false) {
            continue;
          }
          if (vertices.size() < 2) {
            edgeCnt++;
            vertices.push_back(edges[currEdgeIdx].second);
            // std::cout << "pushing (" << vertices.back().x() / 2000.0 << ", " << vertices.back().y() / 2000.0 << ")\n";
          } else if (!isColinear(vertices.back(), vertices[vertices.size() - 2], edges[currEdgeIdx].second)) {
            edgeCnt++;
            vertices.push_back(edges[currEdgeIdx].second);
            // std::cout << "pushing (" << vertices.back().x() / 2000.0 << ", " << vertices.back().y() / 2000.0 << ")\n";
          } else {
            // std::cout << "pop (" << vertices.back().x() / 2000.0 << ", " << vertices.back().y() / 2000.0 << ")\n";
            vertices.pop_back();
            vertices.push_back(edges[currEdgeIdx].second);
            // std::cout << "push (" << vertices.back().x() / 2000.0 << ", " << vertices.back().y() / 2000.0 << ")\n";
          }
          if (currEdgeIdx == edgeIdx) {
            if (nextPt == edges[currEdgeIdx].first) {
              // cout << "  invalidate " << currEdgeIdx << endl;
              validEdges[currEdgeIdx] = false;
            } else {
              Point tmpEndPoint = edges[currEdgeIdx].second;
              edges[currEdgeIdx].second = nextPt;
              if (isHorizontal) {
                horzIntv2Edge[nextPt.y()] -= std::make_pair(boost::icl::interval<int>::right_open(std::min(tmpEndPoint.x(), nextPt.x()), std::max(tmpEndPoint.x(), nextPt.x())), edgeIdx);
                // cout << "  erase y = " << nextPt.y() << ", x = [" << min(tmpEndPoint.x(), nextPt.x()) << ", " << max(tmpEndPoint.x(), nextPt.x()) << "] edgeIdx = " << edgeIdx << "\n";
              } else {
                vertIntv2Edge[nextPt.x()] -= std::make_pair(boost::icl::interval<int>::right_open(std::min(tmpEndPoint.y(), nextPt.y()), std::max(tmpEndPoint.y(), nextPt.y())), edgeIdx);
                // cout << "  erase x = " << nextPt.x() << ", x = [" << min(tmpEndPoint.y(), nextPt.y()) << ", " << max(tmpEndPoint.y(), nextPt.y()) << "] edgeIdx = " << edgeIdx << "\n";
              }
            }
          } else {
            // cout << "  invalidate " << currEdgeIdx << endl;
            validEdges[currEdgeIdx] = false;
          }
        }
        if (edgeCnt > 2) {
          // cout << "poly startIdx = " << edgeIdx << ", endIdx = " << i << endl;
          Polygon tmpPoly;
          Point pt1, pt2, pt3;
          pt1 = vertices.back();
          pt2 = vertices.front();
          pt3 = vertices[vertices.size() - 2];
          // cout << "xxx2\n";

          if ((pt1.x() == pt2.x() && pt2.x() == pt3.x()) ||
              (pt1.y() == pt2.y() && pt2.y() == pt3.y())) {
            vertices.pop_back();
          }
          set_points(tmpPoly, vertices.begin(), vertices.end());
          polys.push_back(tmpPoly);
        }
      } else {
        // cout << "erase startIdx = " << edgeIdx << ", endIdx = " << i << endl;
        for (int currEdgeIdx = edgeIdx; currEdgeIdx <= i; ++currEdgeIdx) {
          if (validEdges[currEdgeIdx] == false) {
            continue;
          }
          if (currEdgeIdx == edgeIdx) {
            if (nextPt == edges[currEdgeIdx].first) {
              validEdges[currEdgeIdx] = false;
            } else {
              edges[currEdgeIdx].second = nextPt;
            }
          } else {
            validEdges[currEdgeIdx] = false;
          }
        }
      }

      // insert to interval map
      if (edgeIdx == -1) {
        // vertical
        if (currPt.x() == nextPt.x()) {
          // fix bug for the case where the new interval intersects with old interval
          auto edgeIntv = boost::icl::interval<int>::closed(std::min(currPt.y(), nextPt.y()), std::max(currPt.y(), nextPt.y()));
          auto itResVert = vertIntv2Edge[nextPt.x()].equal_range(edgeIntv);
          std::vector<std::pair<int, int> > intvPairs;
          for (auto it = itResVert.first; it != itResVert.second; ++it) {
            auto &ovlpIntv = it->first;
            it->second = 0;
            intvPairs.push_back(std::make_pair(ovlpIntv.lower(), ovlpIntv.upper()));
          }
          for (auto &intvPair: intvPairs) {
            vertIntv2Edge[nextPt.x()] -= std::make_pair(boost::icl::interval<int>::right_open(intvPair.first, intvPair.second), -i - 1);
          }
          vertIntv2Edge[nextPt.x()] += std::make_pair(edgeIntv, i + 1);
          // cout << "add x = " << currPt.x() << ", y = [" << edgeIntv.lower() << ", " << edgeIntv.upper() << "]\n";
        } 
        if (currPt.y() == nextPt.y()) {
          // horizontal
          auto edgeIntv = boost::icl::interval<int>::closed(std::min(currPt.x(), nextPt.x()), std::max(currPt.x(), nextPt.x()));
          auto itResHorz = horzIntv2Edge[nextPt.y()].equal_range(edgeIntv);
          std::vector<std::pair<int, int> > intvPairs;
          for (auto it = itResHorz.first; it != itResHorz.second; ++it) {
            auto &ovlpIntv = it->first;
            it->second = 0;
            intvPairs.push_back(std::make_pair(ovlpIntv.lower(), ovlpIntv.upper()));
          }
          for (auto &intvPair: intvPairs) {
            horzIntv2Edge[nextPt.y()] -= std::make_pair(boost::icl::interval<int>::right_open(intvPair.first, intvPair.second), -i - 1);
          }
          horzIntv2Edge[nextPt.y()] += std::make_pair(edgeIntv, i + 1);
          // cout << "add y = " << currPt.y() << ", x = [" << edgeIntv.lower() << ", " << edgeIntv.upper() << "]\n";
        }
      }

    }
    
    // std::vector<Point> tmpPts;
    // for (int currEdgeIdx = 0; currEdgeIdx < edges.size(); ++currEdgeIdx) {
    //   if (validEdges[currEdgeIdx] == false) {
    //     continue;
    //   }
    //   tmpPts.push_back(edges[currEdgeIdx].second);
    //   // std::cout << "pushing (" << tmpPts.back().x() << ", " << tmpPts.back().y() << "\n";
    // }

    // for (int i = 0; i < (int)polys.size() - 1; ++i) {
    //   holes.push_back(polys[i]);
    // }
    // if (polys.size() > 0) {
    //   outline = polys.back();
    // }


  }

}


