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

#include <iostream>
#include "gc/FlexGC.h"

using namespace std;
using namespace fr;

int FlexGCWorker::test() {
  cout <<"test" <<endl;
  
  gcRect rect0;
  frBox box(1,2,3,4);
  rect0.setRect(box);
  //gtl::xl(rect0, 1);
  //gtl::yl(rect0, 2);
  //gtl::xh(rect0, 3);
  //gtl::yh(rect0, 4);
  cout <<"out_rect(" <<gtl::xl(rect0) <<", "
                     <<gtl::yl(rect0) <<", "
                     <<gtl::xh(rect0) <<", "
                     <<gtl::yh(rect0) <<")" <<endl;
  
  gcPolygon poly0;
  vector<frPoint> points;
  points.push_back(frPoint(1,2));
  points.push_back(frPoint(3,2));
  points.push_back(frPoint(3,4));
  points.push_back(frPoint(1,4));
  poly0.setPolygon(points);
  //gtl::rectangle_data<frCoord> rect_0(0, 1, 2, 4);
  //gtl::rectangle_data<frCoord> rect_1(1, 2, 3, 4);
  //gtl::rectangle_data<frCoord> rect_2(-2, 3, 0, 5);
  //gtl::polygon_90_set_data<frCoord> ps;
  //{
  //  using namespace gtl::operators;
  //  ps += rect_0;
  //  ps += rect_1;
  //  ps += rect_2;
  //}
  //vector<gtl::polygon_90_with_holes_data<frCoord> > polys;
  //ps.get(polys);
  //gtl::polygon_90_with_holes_data<frCoord> test_poly;
  //test_poly = polys[0];
  //cout <<"test_poly:";
  //for (auto &pt: test_poly) {
  //  cout <<" (" <<pt.x() <<", " <<pt.y() <<")"; 
  //}
  //cout <<endl;
  //gcPolygon poly0(polys[0], 0, nullptr, nullptr);
  //gcPolygon poly0;
  //poly0.setPolygon(polys[0]);
  //cout <<"polys[0]:";
  //for (auto &pt: polys[0]) {
  //  cout <<" (" <<pt.x() <<", " <<pt.y() <<")"; 
  //}
  //cout <<endl;

  cout <<"poly0:";
  for (auto it = poly0.begin(); it != poly0.end(); it++) {
    auto &pt = (*it);
    cout <<" (" <<pt.x() <<", " <<pt.y() <<")"; 
  }
  cout <<endl;
  //cout <<"poly0:";
  //for (auto it = static_cast<gtl::polygon_90_with_holes_data<frCoord> >(poly0).begin(); 
  //     it != static_cast<gtl::polygon_90_with_holes_data<frCoord> >(poly0).end(); it++) {
  //  auto &pt = (*it);
  //  cout <<" (" <<pt.x() <<", " <<pt.y() <<")"; 
  //}
  //cout <<endl;
  
  gcPolygon poly1;
  points.clear();
  points.push_back(frPoint(1,2));
  points.push_back(frPoint(1,4));
  points.push_back(frPoint(3,4));
  points.push_back(frPoint(3,2));
  poly1.setPolygon(points);
  cout <<"poly1:";
  for (auto &pt: poly1) {
    cout <<" (" <<pt.x() <<", " <<pt.y() <<")"; 
  }
  cout <<endl;
  
  
  //gcNet net0(1);
  //
  //gtl::rectangle_data<frCoord> rect_0(0, 1, 2, 4);
  //gtl::rectangle_data<frCoord> rect_1(1, 2, 3, 4);
  //gtl::rectangle_data<frCoord> rect_2(-2, 3, 0, 5);
  //gtl::polygon_90_set_data<frCoord> ps;
  //using namespace gtl::operators;
  //ps += rect_0;
  //ps += rect_1;
  //ps += rect_2;
  //vector<gtl::polygon_90_with_holes_data<frCoord> > polys;
  //ps.get(polys);
  //for (auto &poly: polys) {
  //  net0.addPin(poly, 0);
  //}
  //cout <<"net == " <<&net0 <<endl;
  //for (auto &pin: net0.getPins(0)) {
  //  cout <<"  pin == " <<pin.get() <<", net == " <<pin->getNet() <<endl;
  //  auto &poly = pin->getPolygon();
  //  cout <<"    poly(layer/pin/net) == (" <<poly.getLayerNum() <<"/" <<poly.getPin() <<"/" <<poly.getNet() <<")";
  //  for (auto &pt: poly.get()) {
  //    cout <<" (" <<pt.x() <<", " <<pt.y() <<")";
  //  }
  //  cout <<endl;
  //  for (int i = 0; i < pin->numMaxRectangles(); i++) {
  //    auto &rect = pin->getMaxRectangle(i);
  //    cout <<"    maxrect(layer/pin/net) == (" <<rect.getLayerNum() <<"/" <<rect.getPin() <<"/" <<rect.getNet() <<") ("
  //         <<gtl::xl(rect.get()) <<", " <<gtl::yl(rect.get()) <<") ("
  //         <<gtl::xh(rect.get()) <<", " <<gtl::yh(rect.get()) <<")" <<endl;
  //  }
  //}
  return 0;
}

