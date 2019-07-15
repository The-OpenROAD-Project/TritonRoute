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

#include <chrono>
#include <fstream>
#include <boost/io/ios_state.hpp>
//#include <taskflow/taskflow.hpp>
#include "dr/FlexDR.h"
#include "io/io.h"
#include "db/infra/frTime.h"
//#include <omp.h>

using namespace std;
using namespace fr;

// std::chrono::duration<double> time_span_init(0);
// std::chrono::duration<double> time_span_init0(0);
// std::chrono::duration<double> time_span_init1(0);
// std::chrono::duration<double> time_span_init2(0);
// std::chrono::duration<double> time_span_init3(0);
// std::chrono::duration<double> time_span_route(0);
// std::chrono::duration<double> time_span_end(0);

int FlexDRWorker::main() {
  using namespace std::chrono;
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  if (VERBOSE > 1) {
    frBox scaledBox;
    stringstream ss;
    ss <<endl <<"start DR worker (BOX) "
                <<"( " <<routeBox.left()   * 1.0 / getTech()->getDBUPerUU() <<" "
                <<routeBox.bottom() * 1.0 / getTech()->getDBUPerUU() <<" ) ( "
                <<routeBox.right()  * 1.0 / getTech()->getDBUPerUU() <<" "
                <<routeBox.top()    * 1.0 / getTech()->getDBUPerUU() <<" )" <<endl;
    cout <<ss.str() <<flush;
  }
  //return 0;

  init();
  //FlexMazeRoute mazeInst(getDesign(), routeBox, extBox);
  //mazeInst.setInitDR(isInitDR());
  //mazeInst.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //mazeInst.route();
  //if (getDRIter() == 0) {
    route();
  //} else {
  //  route_2();
  //}
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  end();
  //mazeInst.end();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();

  duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);
  //time_span_init  += time_span0;
  duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
  //time_span_route += time_span1;
  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
  //time_span_end   += time_span2;

  if (VERBOSE > 1) {
    stringstream ss;
    ss   <<"time (INIT/ROUTE/POST) " <<time_span0.count() <<" " 
                                     <<time_span1.count() <<" "
                                     <<time_span2.count() <<" "
                                     <<endl;
    cout <<ss.str() <<flush;
  }
  return 0;
}

void FlexDR::initFromTA() {
  bool enableOutput = false;
  // initialize lists
  int cnt = 0;
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    for (auto &guide: net->getGuides()) {
      for (auto &connFig: guide->getRoutes()) {
        if (connFig->typeId() == frcPathSeg) {
          unique_ptr<frShape> ps = make_unique<frPathSeg>(*(static_cast<frPathSeg*>(connFig.get())));
          frPoint bp, ep;
          static_cast<frPathSeg*>(ps.get())->getPoints(bp, ep);
          if (ep.x() - bp.x() + ep.y() - bp.y() == 1) {
            ; // skip TA dummy segment
          } else {
            net->addShape(ps);
          }
        } else {
          cout <<"Error: initFromTA unsupported shape" <<endl;
        }
      }
    }
    //net->clearGuides(); // should not clear guide because of initGCellBoundary
    cnt++;
    //if (cnt < 100000) {
    //  if (cnt % 10000 == 0) {
    //    cout <<"  initFromTA complete " <<cnt <<" nets" <<endl;
    //  }
    //} else {
    //  if (cnt % 100000 == 0) {
    //    cout <<"  initFromTA complete " <<cnt <<" nets" <<endl;
    //  }
    //}
  }
  //getRegionQuery()->clearGuides(); // should not clear guide because of initGCellBoundary

  if (enableOutput) {
    for (auto &net: getDesign()->getTopBlock()->getNets()) {
      cout <<"net " <<net->getName() <<" has " <<net->getShapes().size() <<" shape" <<endl;
    }
  }
}

void FlexDR::initGCell2BoundaryPin() {
  bool enableOutput = false;
  // initiailize size
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  //frCoord GCELLOFFSETX = xgp.getStartCoord();
  //frCoord GCELLOFFSETY = ygp.getStartCoord();
  //frCoord GCELLCNTX    = xgp.getCount();
  //frCoord GCELLCNTY    = ygp.getCount();
  auto tmpVec = vector<map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> >((int)ygp.getCount());
  gcell2BoundaryPin = vector<vector<map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> > >((int)xgp.getCount(), tmpVec);
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    auto netPtr = net.get();
    for (auto &guide: net->getGuides()) {
      for (auto &connFig: guide->getRoutes()) {
        if (connFig->typeId() == frcPathSeg) {
          auto ps = static_cast<frPathSeg*>(connFig.get());
          frLayerNum layerNum;
          frPoint bp, ep;
          ps->getPoints(bp, ep);
          layerNum = ps->getLayerNum();
          // skip TA dummy segment
          if (ep.x() - bp.x() + ep.y() - bp.y() == 1 || ep.x() - bp.x() + ep.y() - bp.y() == 0) {
            continue; 
          }
          frPoint idx1, idx2;
          getDesign()->getTopBlock()->getGCellIdx(bp, idx1);
          getDesign()->getTopBlock()->getGCellIdx(ep, idx2);
          //frBox gcellBox1, gcellBox2;
          //getDesign()->getTopBlock()->getGCellBox(idx1, gcellBox1);
          //getDesign()->getTopBlock()->getGCellBox(idx2, gcellBox2);
          // update gcell2BoundaryPin
          // horizontal
          if (bp.y() == ep.y()) {
            //int x1 = (bp.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int x2 = (ep.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int y  = (bp.y() - ygp.getStartCoord()) / ygp.getSpacing();
            int x1 = idx1.x();
            int x2 = idx2.x();
            int y  = idx1.y();
            //if (x1 < 0 || x2 < 0 || y < 0) {
            //  cout <<"Warning: initGCell2BoundaryPin < 0";
            //  exit(1);
            //}
            //if (x1 >= (int)GCELLCNTX) {
            //  x1 = (int)GCELLCNTX - 1;
            //}
            //if (x2 >= (int)GCELLCNTX) {
            //  x2 = (int)GCELLCNTX - 1;
            //}
            //if (y >= (int)GCELLCNTY) {
            //  y = (int)GCELLCNTY - 1;
            //}
            for (auto x = x1; x <= x2; ++x) {
              frBox gcellBox;
              getDesign()->getTopBlock()->getGCellBox(frPoint(x, y), gcellBox);
              //frCoord leftBound  = x * xgp.getSpacing() + xgp.getStartCoord();
              //frCoord rightBound = (x == (int)GCELLCNTX - 1) ? dieBox.right() : (x + 1) * xgp.getSpacing() + xgp.getStartCoord();
              frCoord leftBound  = gcellBox.left();
              frCoord rightBound = gcellBox.right();
              bool hasLeftBound  = true;
              bool hasRightBound = true;
              if (bp.x() < leftBound) {
                hasLeftBound = true;
              } else {
                hasLeftBound = false;
              }
              if (ep.x() >= rightBound) {
                hasRightBound = true;
              } else {
                hasRightBound = false;
              }
              if (hasLeftBound) {
                frPoint boundaryPt(leftBound, bp.y());
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init left boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
              if (hasRightBound) {
                frPoint boundaryPt(rightBound, ep.y());
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init right boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
            }
          } else if (bp.x() == ep.x()) {
            //int x  = (bp.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int y1 = (bp.y() - ygp.getStartCoord()) / ygp.getSpacing();
            //int y2 = (ep.y() - ygp.getStartCoord()) / ygp.getSpacing();
            int x  = idx1.x();
            int y1 = idx1.y();
            int y2 = idx2.y();
            //if (y1 < 0 || y2 < 0 || x < 0) {
            //  cout <<"Warning: initGCell2BoundaryPin < 0";
            //  exit(1);
            //}
            //if (x >= (int)GCELLCNTX) {
            //  x = (int)GCELLCNTX - 1;
            //}
            //if (y1 >= (int)GCELLCNTY) {
            //  y1 = (int)GCELLCNTY - 1;
            //}
            //if (y2 >= (int)GCELLCNTY) {
            //  y2 = (int)GCELLCNTY - 1;
            //}
            for (auto y = y1; y <= y2; ++y) {
              frBox gcellBox;
              getDesign()->getTopBlock()->getGCellBox(frPoint(x, y), gcellBox);
              //frCoord bottomBound = y * ygp.getSpacing() + ygp.getStartCoord();
              //frCoord topBound    = (y == (int)GCELLCNTY - 1) ? dieBox.top() : (y + 1) * ygp.getSpacing() + ygp.getStartCoord();
              frCoord bottomBound = gcellBox.bottom();
              frCoord topBound    = gcellBox.top();
              bool hasBottomBound = true;
              bool hasTopBound    = true;
              if (bp.y() < bottomBound) {
                hasBottomBound = true;
              } else {
                hasBottomBound = false;
              }
              if (ep.y() >= topBound) {
                hasTopBound = true;
              } else {
                hasTopBound = false;
              }
              if (hasBottomBound) {
                frPoint boundaryPt(bp.x(), bottomBound);
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init bottom boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
              if (hasTopBound) {
                frPoint boundaryPt(ep.x(), topBound);
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init top boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
            }
          } else {
            cout << "Error: non-orthogonal pathseg in initGCell2BoundryPin\n";
          }
        }
      }
    }
  }
}

frCoord FlexDR::init_via2viaMinLen_minimumcut1(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2) {
  if (!(viaDef1 && viaDef2)) {
    //cout <<"hehehehe" <<endl;
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  //frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  bool isVia1Above = false;
  frVia via1(viaDef1);
  frBox viaBox1, cutBox1;
  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
    isVia1Above = true;
  } else {
    via1.getLayer2BBox(viaBox1);
    isVia1Above = false;
  }
  via1.getCutBBox(cutBox1);
  auto width1    = viaBox1.width();
  auto length1   = viaBox1.length();
  //bool isVia1Fat = isH ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  //auto prl1      = isH ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  bool isVia2Above = false;
  frVia via2(viaDef2);
  frBox viaBox2, cutBox2;
  if (viaDef2->getLayer1Num() == lNum) {
    via2.getLayer1BBox(viaBox2);
    isVia2Above = true;
  } else {
    via2.getLayer2BBox(viaBox2);
    isVia2Above = false;
  }
  via2.getCutBBox(cutBox2);
  auto width2    = viaBox2.width();
  auto length2   = viaBox2.length();
  //bool isVia2Fat = isH ? (viaBox2.top() - viaBox2.bottom() > defaultWidth) : (viaBox2.right() - viaBox2.left() > defaultWidth);
  //auto prl2      = isH ? (viaBox2.top() - viaBox2.bottom()) : (viaBox2.right() - viaBox2.left());

  for (auto &con: getDesign()->getTech()->getLayer(lNum)->getMinimumcutConstraints()) {
    //// two via enclsoure <= width, minimumcut rule not applied
    //if (defaultWidth <= con->getWidth() && width1 <= con->getWidth() && width2 <= con->getWidth()) {
    //  continue;
    //}
    //// skip if hasConnection but no via satisfies
    //if (con->hasConnection() && con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE &&
    //    !isVia1Above && !isVia2Above) {
    //  continue;
    //}
    //if (con->hasConnection() && con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW &&
    //    isVia1Above && isVia2Above) {
    //  continue;
    //}
    // check via2cut to via1metal
    // no length OR metal1 shape satisfies --> check via2
    if ((!con->hasLength() || (con->hasLength() && length1 > con->getLength())) && 
        width1 > con->getWidth()) {
      bool checkVia2 = false;
      if (!con->hasConnection()) {
        checkVia2 = true;
      } else {
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia2Above) {
          checkVia2 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia2Above) {
          checkVia2 = true;
        }
      }
      if (!checkVia2) {
        continue;
      }
      if (isH) {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox2.right() - 0 + 0 - viaBox1.left(), 
                           viaBox1.right() - 0 + 0 - cutBox2.left()));
      } else {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox2.top() - 0 + 0 - viaBox1.bottom(), 
                           viaBox1.top() - 0 + 0 - cutBox2.bottom()));
      }
    } 
    // check via1cut to via2metal
    if ((!con->hasLength() || (con->hasLength() && length2 > con->getLength())) && 
        width2 > con->getWidth()) {
      bool checkVia1 = false;
      if (!con->hasConnection()) {
        checkVia1 = true;
      } else {
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia1Above) {
          checkVia1 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia1Above) {
          checkVia1 = true;
        }
      }
      if (!checkVia1) {
        continue;
      }
      if (isH) {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox1.right() - 0 + 0 - viaBox2.left(), 
                           viaBox2.right() - 0 + 0 - cutBox1.left()));
      } else {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox1.top() - 0 + 0 - viaBox2.bottom(), 
                           viaBox2.top() - 0 + 0 - cutBox1.bottom()));
      }
    }
  }

  return sol;
}

bool FlexDR::init_via2viaMinLen_minimumcut2(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2) {
  if (!(viaDef1 && viaDef2)) {
    return true;
  }
  // skip if same-layer via
  if (viaDef1 == viaDef2) {
    return true;
  }

  bool sol = true;

  bool isVia1Above = false;
  frVia via1(viaDef1);
  frBox viaBox1, cutBox1;
  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
    isVia1Above = true;
  } else {
    via1.getLayer2BBox(viaBox1);
    isVia1Above = false;
  }
  via1.getCutBBox(cutBox1);
  auto width1    = viaBox1.width();

  bool isVia2Above = false;
  frVia via2(viaDef2);
  frBox viaBox2, cutBox2;
  if (viaDef2->getLayer1Num() == lNum) {
    via2.getLayer1BBox(viaBox2);
    isVia2Above = true;
  } else {
    via2.getLayer2BBox(viaBox2);
    isVia2Above = false;
  }
  via2.getCutBBox(cutBox2);
  auto width2    = viaBox2.width();

  for (auto &con: getDesign()->getTech()->getLayer(lNum)->getMinimumcutConstraints()) {
    if (con->hasLength()) {
      continue;
    }
    // check via2cut to via1metal
    if (width1 > con->getWidth()) {
      bool checkVia2 = false;
      if (!con->hasConnection()) {
        checkVia2 = true;
      } else {
        // has length rule
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia2Above) {
          checkVia2 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia2Above) {
          checkVia2 = true;
        }
      }
      if (!checkVia2) {
        continue;
      }
      sol = false;
      break;
    } 
    // check via1cut to via2metal
    if (width2 > con->getWidth()) {
      bool checkVia1 = false;
      if (!con->hasConnection()) {
        checkVia1 = true;
      } else {
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia1Above) {
          checkVia1 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia1Above) {
          checkVia1 = true;
        }
      }
      if (!checkVia1) {
        continue;
      }
      sol = false;
      break;
    }
  }
  return sol;
}

frCoord FlexDR::init_via2viaMinLen_minSpc(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2) {
  if (!(viaDef1 && viaDef2)) {
    //cout <<"hehehehehe" <<endl;
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  frVia via1(viaDef1);
  frBox viaBox1;
  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
  } else {
    via1.getLayer2BBox(viaBox1);
  }
  auto width1    = viaBox1.width();
  bool isVia1Fat = isH ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  auto prl1      = isH ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  frVia via2(viaDef2);
  frBox viaBox2;
  if (viaDef2->getLayer1Num() == lNum) {
    via2.getLayer1BBox(viaBox2);
  } else {
    via2.getLayer2BBox(viaBox2);
  }
  auto width2    = viaBox2.width();
  bool isVia2Fat = isH ? (viaBox2.top() - viaBox2.bottom() > defaultWidth) : (viaBox2.right() - viaBox2.left() > defaultWidth);
  auto prl2      = isH ? (viaBox2.top() - viaBox2.bottom()) : (viaBox2.right() - viaBox2.left());

  frCoord reqDist = 0;
  if (isVia1Fat && isVia2Fat) {
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con) {
      if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
        reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
        reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), min(prl1, prl2));
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
        reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, min(prl1, prl2));
      }
    }
    if (isH) {
      reqDist += max((viaBox1.right() - 0), (0 - viaBox1.left()));
      reqDist += max((viaBox2.right() - 0), (0 - viaBox2.left()));
    } else {
      reqDist += max((viaBox1.top() - 0), (0 - viaBox1.bottom()));
      reqDist += max((viaBox2.top() - 0), (0 - viaBox2.bottom()));
    }
    sol = max(sol, reqDist);
  }

  // check min len in layer2 if two vias are in same layer
  if (viaDef1 != viaDef2) {
    return sol;
  }

  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer2BBox(viaBox1);
    lNum = lNum + 2;
  } else {
    via1.getLayer1BBox(viaBox1);
    lNum = lNum - 2;
  }
  width1    = viaBox1.width();
  prl1      = isH ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());
  reqDist   = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), prl1);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, prl1);
    }
  }
  if (isH) {
    reqDist += (viaBox1.right() - 0) + (0 - viaBox1.left());
  } else {
    reqDist += (viaBox1.top() - 0) + (0 - viaBox1.bottom());
  }
  sol = max(sol, reqDist);
  
  return sol;
}

void FlexDR::init_via2viaMinLen() {
  //bool enableOutput = false;
  bool enableOutput = true;
  auto bottomLayerNum = getDesign()->getTech()->getBottomLayerNum();
  auto topLayerNum    = getDesign()->getTech()->getTopLayerNum();
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    vector<frCoord> via2viaMinLenTmp(4, 0);
    vector<bool>    via2viaZeroLen(4, true);
    via2viaMinLen.push_back(make_pair(via2viaMinLenTmp, via2viaZeroLen));
  }
  // check prl
  int i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
      //cout <<"d via found " <<downVia <<endl;
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
      //cout <<"u via found " <<upVia <<endl;
    //} else {
    //  cout <<"lNum=" <<lNum <<endl;
    //  cout <<"topLayerNum=" <<getDesign()->getTech()->getTopLayerNum() <<endl;
    }
    //vector<frCoord> via2viaMinLenTmp(4, 0);
    //cout <<"@@@0" <<endl;
    (via2viaMinLen[i].first)[0] = max((via2viaMinLen[i].first)[0], init_via2viaMinLen_minSpc(lNum, downVia, downVia));
    //cout <<"@@@1" <<endl;
    (via2viaMinLen[i].first)[1] = max((via2viaMinLen[i].first)[1], init_via2viaMinLen_minSpc(lNum, downVia, upVia));
    //cout <<"@@@2" <<endl;
    (via2viaMinLen[i].first)[2] = max((via2viaMinLen[i].first)[2], init_via2viaMinLen_minSpc(lNum, upVia, downVia));
    //cout <<"@@@3" <<endl;
    (via2viaMinLen[i].first)[3] = max((via2viaMinLen[i].first)[3], init_via2viaMinLen_minSpc(lNum, upVia, upVia));
    if (enableOutput) {
      cout <<"initVia2ViaMinLen_minSpc " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (d2d, d2u, u2d, u2u) = (" 
           <<(via2viaMinLen[i].first)[0] <<", "
           <<(via2viaMinLen[i].first)[1] <<", "
           <<(via2viaMinLen[i].first)[2] <<", "
           <<(via2viaMinLen[i].first)[3] <<")" <<endl;
    }
    i++;
  }

  // check minimumcut
  i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    vector<frCoord> via2viaMinLenTmp(4, 0);
    //cout <<"@@@0" <<endl;
    (via2viaMinLen[i].first)[0] = max((via2viaMinLen[i].first)[0], init_via2viaMinLen_minimumcut1(lNum, downVia, downVia));
    //cout <<"@@@1" <<endl;
    (via2viaMinLen[i].first)[1] = max((via2viaMinLen[i].first)[1], init_via2viaMinLen_minimumcut1(lNum, downVia, upVia));
    //cout <<"@@@2" <<endl;
    (via2viaMinLen[i].first)[2] = max((via2viaMinLen[i].first)[2], init_via2viaMinLen_minimumcut1(lNum, upVia, downVia));
    //cout <<"@@@3" <<endl;
    (via2viaMinLen[i].first)[3] = max((via2viaMinLen[i].first)[3], init_via2viaMinLen_minimumcut1(lNum, upVia, upVia));
    (via2viaMinLen[i].second)[0] = (via2viaMinLen[i].second)[0] && init_via2viaMinLen_minimumcut2(lNum, downVia, downVia);
    (via2viaMinLen[i].second)[1] = (via2viaMinLen[i].second)[1] && init_via2viaMinLen_minimumcut2(lNum, downVia, upVia);
    (via2viaMinLen[i].second)[2] = (via2viaMinLen[i].second)[2] && init_via2viaMinLen_minimumcut2(lNum, upVia, downVia);
    (via2viaMinLen[i].second)[3] = (via2viaMinLen[i].second)[3] && init_via2viaMinLen_minimumcut2(lNum, upVia, upVia);
    if (enableOutput) {
      cout <<"initVia2ViaMinLen_minimumcut " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (d2d, d2u, u2d, u2u) = (" 
           <<(via2viaMinLen[i].first)[0] <<", "
           <<(via2viaMinLen[i].first)[1] <<", "
           <<(via2viaMinLen[i].first)[2] <<", "
           <<(via2viaMinLen[i].first)[3] <<")" <<endl;
      cout <<"initVia2ViaMinLen_minimumcut " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" zerolen (b, b, b, b) = (" 
           <<(via2viaMinLen[i].second)[0] <<", "
           <<(via2viaMinLen[i].second)[1] <<", "
           <<(via2viaMinLen[i].second)[2] <<", "
           <<(via2viaMinLen[i].second)[3] <<")" 
           <<endl;
    }
    i++;
  }
}

frCoord FlexDR::init_via2viaMinLenNew_minimumcut1(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2, bool isCurrDirY) {
  if (!(viaDef1 && viaDef2)) {
    //cout <<"hehehehe" <<endl;
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  //bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  bool isCurrDirX = !isCurrDirY;
  //frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  bool isVia1Above = false;
  frVia via1(viaDef1);
  frBox viaBox1, cutBox1;
  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
    isVia1Above = true;
  } else {
    via1.getLayer2BBox(viaBox1);
    isVia1Above = false;
  }
  via1.getCutBBox(cutBox1);
  auto width1    = viaBox1.width();
  auto length1   = viaBox1.length();
  //bool isVia1Fat = isH ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  //auto prl1      = isH ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  bool isVia2Above = false;
  frVia via2(viaDef2);
  frBox viaBox2, cutBox2;
  if (viaDef2->getLayer1Num() == lNum) {
    via2.getLayer1BBox(viaBox2);
    isVia2Above = true;
  } else {
    via2.getLayer2BBox(viaBox2);
    isVia2Above = false;
  }
  via2.getCutBBox(cutBox2);
  auto width2    = viaBox2.width();
  auto length2   = viaBox2.length();
  //bool isVia2Fat = isH ? (viaBox2.top() - viaBox2.bottom() > defaultWidth) : (viaBox2.right() - viaBox2.left() > defaultWidth);
  //auto prl2      = isH ? (viaBox2.top() - viaBox2.bottom()) : (viaBox2.right() - viaBox2.left());

  for (auto &con: getDesign()->getTech()->getLayer(lNum)->getMinimumcutConstraints()) {
    //// two via enclsoure <= width, minimumcut rule not applied
    //if (defaultWidth <= con->getWidth() && width1 <= con->getWidth() && width2 <= con->getWidth()) {
    //  continue;
    //}
    //// skip if hasConnection but no via satisfies
    //if (con->hasConnection() && con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE &&
    //    !isVia1Above && !isVia2Above) {
    //  continue;
    //}
    //if (con->hasConnection() && con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW &&
    //    isVia1Above && isVia2Above) {
    //  continue;
    //}
    // check via2cut to via1metal
    // no length OR metal1 shape satisfies --> check via2
    if ((!con->hasLength() || (con->hasLength() && length1 > con->getLength())) && 
        width1 > con->getWidth()) {
      bool checkVia2 = false;
      if (!con->hasConnection()) {
        checkVia2 = true;
      } else {
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia2Above) {
          checkVia2 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia2Above) {
          checkVia2 = true;
        }
      }
      if (!checkVia2) {
        continue;
      }
      if (isCurrDirX) {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox2.right() - 0 + 0 - viaBox1.left(), 
                           viaBox1.right() - 0 + 0 - cutBox2.left()));
      } else {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox2.top() - 0 + 0 - viaBox1.bottom(), 
                           viaBox1.top() - 0 + 0 - cutBox2.bottom()));
      }
    } 
    // check via1cut to via2metal
    if ((!con->hasLength() || (con->hasLength() && length2 > con->getLength())) && 
        width2 > con->getWidth()) {
      bool checkVia1 = false;
      if (!con->hasConnection()) {
        checkVia1 = true;
      } else {
        if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia1Above) {
          checkVia1 = true;
        } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia1Above) {
          checkVia1 = true;
        }
      }
      if (!checkVia1) {
        continue;
      }
      if (isCurrDirX) {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox1.right() - 0 + 0 - viaBox2.left(), 
                           viaBox2.right() - 0 + 0 - cutBox1.left()));
      } else {
        sol = max(sol, (con->hasLength() ? con->getDistance() : 0) + 
                       max(cutBox1.top() - 0 + 0 - viaBox2.bottom(), 
                           viaBox2.top() - 0 + 0 - cutBox1.bottom()));
      }
    }
  }

  return sol;
}

frCoord FlexDR::init_via2viaMinLenNew_minSpc(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2, bool isCurrDirY) {
  if (!(viaDef1 && viaDef2)) {
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  //bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  bool isCurrDirX = !isCurrDirY;
  frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  frVia via1(viaDef1);
  frBox viaBox1;
  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
  } else {
    via1.getLayer2BBox(viaBox1);
  }
  auto width1    = viaBox1.width();
  bool isVia1Fat = isCurrDirX ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  auto prl1      = isCurrDirX ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  frVia via2(viaDef2);
  frBox viaBox2;
  if (viaDef2->getLayer1Num() == lNum) {
    via2.getLayer1BBox(viaBox2);
  } else {
    via2.getLayer2BBox(viaBox2);
  }
  auto width2    = viaBox2.width();
  bool isVia2Fat = isCurrDirX ? (viaBox2.top() - viaBox2.bottom() > defaultWidth) : (viaBox2.right() - viaBox2.left() > defaultWidth);
  auto prl2      = isCurrDirX ? (viaBox2.top() - viaBox2.bottom()) : (viaBox2.right() - viaBox2.left());

  frCoord reqDist = 0;
  if (isVia1Fat && isVia2Fat) {
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con) {
      if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
        reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
        reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), min(prl1, prl2));
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
        reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, min(prl1, prl2));
      }
    }
    if (isCurrDirX) {
      reqDist += max((viaBox1.right() - 0), (0 - viaBox1.left()));
      reqDist += max((viaBox2.right() - 0), (0 - viaBox2.left()));
    } else {
      reqDist += max((viaBox1.top() - 0), (0 - viaBox1.bottom()));
      reqDist += max((viaBox2.top() - 0), (0 - viaBox2.bottom()));
    }
    sol = max(sol, reqDist);
  }

  // check min len in layer2 if two vias are in same layer
  if (viaDef1 != viaDef2) {
    return sol;
  }

  if (viaDef1->getLayer1Num() == lNum) {
    via1.getLayer2BBox(viaBox1);
    lNum = lNum + 2;
  } else {
    via1.getLayer1BBox(viaBox1);
    lNum = lNum - 2;
  }
  width1    = viaBox1.width();
  prl1      = isCurrDirX ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());
  reqDist   = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), prl1);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, prl1);
    }
  }
  if (isCurrDirX) {
    reqDist += (viaBox1.right() - 0) + (0 - viaBox1.left());
  } else {
    reqDist += (viaBox1.top() - 0) + (0 - viaBox1.bottom());
  }
  sol = max(sol, reqDist);
  
  return sol;
}

void FlexDR::init_via2viaMinLenNew() {
  //bool enableOutput = false;
  bool enableOutput = true;
  auto bottomLayerNum = getDesign()->getTech()->getBottomLayerNum();
  auto topLayerNum    = getDesign()->getTech()->getTopLayerNum();
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    vector<frCoord> via2viaMinLenTmp(8, 0);
    via2viaMinLenNew.push_back(via2viaMinLenTmp);
  }
  // check prl
  int i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    via2viaMinLenNew[i][0] = max(via2viaMinLenNew[i][0], init_via2viaMinLenNew_minSpc(lNum, downVia, downVia, false));
    via2viaMinLenNew[i][1] = max(via2viaMinLenNew[i][1], init_via2viaMinLenNew_minSpc(lNum, downVia, downVia, true ));
    via2viaMinLenNew[i][2] = max(via2viaMinLenNew[i][2], init_via2viaMinLenNew_minSpc(lNum, downVia, upVia,   false));
    via2viaMinLenNew[i][3] = max(via2viaMinLenNew[i][3], init_via2viaMinLenNew_minSpc(lNum, downVia, upVia,   true ));
    via2viaMinLenNew[i][4] = max(via2viaMinLenNew[i][4], init_via2viaMinLenNew_minSpc(lNum, upVia,   downVia, false));
    via2viaMinLenNew[i][5] = max(via2viaMinLenNew[i][5], init_via2viaMinLenNew_minSpc(lNum, upVia,   downVia, true ));
    via2viaMinLenNew[i][6] = max(via2viaMinLenNew[i][6], init_via2viaMinLenNew_minSpc(lNum, upVia,   upVia,   false));
    via2viaMinLenNew[i][7] = max(via2viaMinLenNew[i][7], init_via2viaMinLenNew_minSpc(lNum, upVia,   upVia,   true ));
    if (enableOutput) {
      cout <<"initVia2ViaMinLenNew_minSpc " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (d2d-x, d2d-y, d2u-x, d2u-y, u2d-x, u2d-y, u2u-x, u2u-y) = (" 
           <<via2viaMinLenNew[i][0] <<", "
           <<via2viaMinLenNew[i][1] <<", "
           <<via2viaMinLenNew[i][2] <<", "
           <<via2viaMinLenNew[i][3] <<", "
           <<via2viaMinLenNew[i][4] <<", "
           <<via2viaMinLenNew[i][5] <<", "
           <<via2viaMinLenNew[i][6] <<", "
           <<via2viaMinLenNew[i][7] <<")" <<endl;
    }
    i++;
  }

  // check minimumcut
  i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    via2viaMinLenNew[i][0] = max(via2viaMinLenNew[i][0], init_via2viaMinLenNew_minimumcut1(lNum, downVia, downVia, false));
    via2viaMinLenNew[i][1] = max(via2viaMinLenNew[i][1], init_via2viaMinLenNew_minimumcut1(lNum, downVia, downVia, true ));
    via2viaMinLenNew[i][2] = max(via2viaMinLenNew[i][2], init_via2viaMinLenNew_minimumcut1(lNum, downVia, upVia,   false));
    via2viaMinLenNew[i][3] = max(via2viaMinLenNew[i][3], init_via2viaMinLenNew_minimumcut1(lNum, downVia, upVia,   true ));
    via2viaMinLenNew[i][4] = max(via2viaMinLenNew[i][4], init_via2viaMinLenNew_minimumcut1(lNum, upVia,   downVia, false));
    via2viaMinLenNew[i][5] = max(via2viaMinLenNew[i][5], init_via2viaMinLenNew_minimumcut1(lNum, upVia,   downVia, true ));
    via2viaMinLenNew[i][6] = max(via2viaMinLenNew[i][6], init_via2viaMinLenNew_minimumcut1(lNum, upVia,   upVia,   false));
    via2viaMinLenNew[i][7] = max(via2viaMinLenNew[i][7], init_via2viaMinLenNew_minimumcut1(lNum, upVia,   upVia,   true ));
    if (enableOutput) {
      cout <<"initVia2ViaMinLenNew_minimumcut " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (d2d-x, d2d-y, d2u-x, d2u-y, u2d-x, u2d-y, u2u-x, u2u-y) = (" 
           <<via2viaMinLenNew[i][0] <<", "
           <<via2viaMinLenNew[i][1] <<", "
           <<via2viaMinLenNew[i][2] <<", "
           <<via2viaMinLenNew[i][3] <<", "
           <<via2viaMinLenNew[i][4] <<", "
           <<via2viaMinLenNew[i][5] <<", "
           <<via2viaMinLenNew[i][6] <<", "
           <<via2viaMinLenNew[i][7] <<")" <<endl;
    }
    i++;
  }
}

void FlexDR::init_halfViaEncArea() {
  auto bottomLayerNum = getDesign()->getTech()->getBottomLayerNum();
  auto topLayerNum    = getDesign()->getTech()->getTopLayerNum();
  //vector<frCoord> via2viaMinLenTmp(4, 0);
  for (int i = bottomLayerNum; i <= topLayerNum; i++) {
    if (getDesign()->getTech()->getLayer(i)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    if (i + 1 <= topLayerNum && getDesign()->getTech()->getLayer(i + 1)->getType() == frLayerTypeEnum::CUT) {
      auto viaDef = getTech()->getLayer(i + 1)->getDefaultViaDef();
      frVia via(viaDef);
      frBox layer1Box;
      frBox layer2Box;
      via.getLayer1BBox(layer1Box);
      via.getLayer2BBox(layer2Box);
      auto layer1HalfArea = layer1Box.width() * layer1Box.length() / 2;
      auto layer2HalfArea = layer2Box.width() * layer2Box.length() / 2;
      // cout <<"z = " <<zCoords.size() <<" " <<"layer1/2HalfArea = " <<layer1HalfArea <<"/" <<layer2HalfArea <<endl;
      halfViaEncArea.push_back(make_pair(layer1HalfArea, layer2HalfArea));
    } else {
      halfViaEncArea.push_back(make_pair(0,0));
    }
    //via2viaMinLen.push_back(via2viaMinLenTmp);
  }
  //init_via2ViaMinLen();
}



frCoord FlexDR::init_via2turnMinLen_minSpc(frLayerNum lNum, frViaDef* viaDef, bool isCurrDirY) {
  if (!viaDef) {
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  //bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  bool isCurrDirX = !isCurrDirY;
  frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  frVia via1(viaDef);
  frBox viaBox1;
  if (viaDef->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
  } else {
    via1.getLayer2BBox(viaBox1);
  }
  auto width1    = viaBox1.width();
  bool isVia1Fat = isCurrDirX ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  auto prl1      = isCurrDirX ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  frCoord reqDist = 0;
  if (isVia1Fat) {
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con) {
      if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
        reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
        reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, defaultWidth), prl1);
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
        reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, defaultWidth, prl1);
      }
    }
    if (isCurrDirX) {
      reqDist += max((viaBox1.right() - 0), (0 - viaBox1.left()));
      reqDist += defaultWidth;
    } else {
      reqDist += max((viaBox1.top() - 0), (0 - viaBox1.bottom()));
      reqDist += defaultWidth;
    }
    sol = max(sol, reqDist);
  }

  return sol;
}

frCoord FlexDR::init_via2turnMinLen_minStp(frLayerNum lNum, frViaDef* viaDef, bool isCurrDirY) {
  if (!viaDef) {
    return 0;
  }

  frCoord sol = 0;

  // check min len in lNum assuming pre dir routing
  //bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  bool isCurrDirX = !isCurrDirY;
  frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  frVia via1(viaDef);
  frBox viaBox1;
  if (viaDef->getLayer1Num() == lNum) {
    via1.getLayer1BBox(viaBox1);
  } else {
    via1.getLayer2BBox(viaBox1);
  }
  //auto width1    = viaBox1.width();
  bool isVia1Fat = isCurrDirX ? (viaBox1.top() - viaBox1.bottom() > defaultWidth) : (viaBox1.right() - viaBox1.left() > defaultWidth);
  //auto prl1      = isCurrDirX ? (viaBox1.top() - viaBox1.bottom()) : (viaBox1.right() - viaBox1.left());

  frCoord reqDist = 0;
  if (isVia1Fat) {
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinStepConstraint();
    if (con && con->hasMaxEdges()) { // currently only consider maxedge violation
      reqDist = con->getMinStepLength();
      if (isCurrDirX) {
        reqDist += max((viaBox1.right() - 0), (0 - viaBox1.left()));
        reqDist += defaultWidth;
      } else {
        reqDist += max((viaBox1.top() - 0), (0 - viaBox1.bottom()));
        reqDist += defaultWidth;
      }
      sol = max(sol, reqDist);
    }
  }
  return sol;
}


void FlexDR::init_via2turnMinLen() {
  bool enableOutput = false;
  //bool enableOutput = true;
  auto bottomLayerNum = getDesign()->getTech()->getBottomLayerNum();
  auto topLayerNum    = getDesign()->getTech()->getTopLayerNum();
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    vector<frCoord> via2turnMinLenTmp(4, 0);
    via2turnMinLen.push_back(via2turnMinLenTmp);
  }
  // check prl
  int i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    via2turnMinLen[i][0] = max(via2turnMinLen[i][0], init_via2turnMinLen_minSpc(lNum, downVia, false));
    via2turnMinLen[i][1] = max(via2turnMinLen[i][1], init_via2turnMinLen_minSpc(lNum, downVia, true));
    via2turnMinLen[i][2] = max(via2turnMinLen[i][2], init_via2turnMinLen_minSpc(lNum, upVia, false));
    via2turnMinLen[i][3] = max(via2turnMinLen[i][3], init_via2turnMinLen_minSpc(lNum, upVia, true));
    if (enableOutput) {
      cout <<"initVia2TurnMinLen_minSpc " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (down->x->y, down->y->x, up->x->y, up->y->x) = (" 
           <<via2turnMinLen[i][0] <<", "
           <<via2turnMinLen[i][1] <<", "
           <<via2turnMinLen[i][2] <<", "
           <<via2turnMinLen[i][3] <<")" <<endl;
    }
    i++;
  }

  // check minstep
  i = 0;
  for (auto lNum = bottomLayerNum; lNum <= topLayerNum; lNum++) {
    if (getDesign()->getTech()->getLayer(lNum)->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    vector<frCoord> via2turnMinLenTmp(4, 0);
    via2turnMinLen[i][0] = max(via2turnMinLen[i][0], init_via2turnMinLen_minStp(lNum, downVia, false));
    via2turnMinLen[i][1] = max(via2turnMinLen[i][1], init_via2turnMinLen_minStp(lNum, downVia, true));
    via2turnMinLen[i][2] = max(via2turnMinLen[i][2], init_via2turnMinLen_minStp(lNum, upVia, false));
    via2turnMinLen[i][3] = max(via2turnMinLen[i][3], init_via2turnMinLen_minStp(lNum, upVia, true));
    if (enableOutput) {
      cout <<"initVia2TurnMinLen_minstep " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (down->x->y, down->y->x, up->x->y, up->y->x) = (" 
           <<via2turnMinLen[i][0] <<", "
           <<via2turnMinLen[i][1] <<", "
           <<via2turnMinLen[i][2] <<", "
           <<via2turnMinLen[i][3] <<")" <<endl;
    }
    i++;
  }
}



void FlexDR::init() {
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<"start routing data preparation" <<endl;
  }
  //initFromTA(); // this does not help much for conservative boundary patch metal in initDR
  initGCell2BoundaryPin();
  //if (VERBOSE > 0) {
  //  cout <<endl <<"init dr objs ..." <<endl;
  //}
  getRegionQuery()->initDRObj(getTech()->getLayers().size());
  //getRegionQuery()->printDRObj();

  init_halfViaEncArea();
  init_via2viaMinLen();
  init_via2viaMinLenNew();
  init_via2turnMinLen();

  if (VERBOSE > 0) {
    t.print();
  }
}

void FlexDR::removeGCell2BoundaryPin() {
  gcell2BoundaryPin.clear();
  gcell2BoundaryPin.shrink_to_fit();
}

map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> FlexDR::initDR_mergeBoundaryPin(int startX, int startY, int size, const frBox &routeBox) {
  map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> bp;
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  for (int i = startX; i < (int)xgp.getCount() && i < startX + size; i++) {
    for (int j = startY; j < (int)ygp.getCount() && j < startY + size; j++) {
      auto &currBp = gcell2BoundaryPin[i][j];
      for (auto &[net, s]: currBp) {
        for (auto &[pt, lNum]: s) {
          if (pt.x() == routeBox.left()   || pt.x() == routeBox.right() ||
              pt.y() == routeBox.bottom() || pt.y() == routeBox.top()) {
            bp[net].insert(make_pair(pt, lNum));
          }
        }
      }
    }
  }
  return bp;
}

void FlexDR::initDR(int size, bool enableDRC) {
  bool TEST = false;
  //bool TEST = true;
  //cout <<"sizeof listiter   = " <<sizeof(frListIter<frPathSeg>) <<endl;
  //cout <<"sizeof raw ptr    = " <<sizeof(frPathSeg*) <<endl;
  //cout <<"sizeof unique ptr = " <<sizeof(unique_ptr<frPathSeg>) <<endl;
  //cout <<"sizeof shared_ptr = " <<sizeof(shared_ptr<frPathSeg>) <<endl;
  //exit(0);

  //FlexGridGraph gg(getTech(), getDesign());
  ////frBox testBBox(225000, 228100, 228000, 231100); // net1702 in ispd19_test1
  //frBox testBBox(0, 0, 2000, 2000); // net1702 in ispd19_test1
  ////gg.setBBox(testBBox);
  //gg.init(testBBox);
  //gg.print();
  //exit(1);

  frTime t;

  if (VERBOSE > 0) {
    cout <<endl <<"start initial detail routing ..." <<endl;
  }
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);

  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);

  int numQuickMarkers = 0;
  if (TEST) {
    //FlexDRWorker worker(getDesign());
    FlexDRWorker worker(this);
    //frBox routeBox;
    //routeBox.set(0*2000, 0*2000, 1*2000, 1*2000);
    frCoord xl = 63 * 2000;
    frCoord yl = 84 * 2000;
    //frCoord xh = 129 * 2000;
    //frCoord yh = 94.05 * 2000;
    frPoint idx;
    getDesign()->getTopBlock()->getGCellIdx(frPoint(xl, yl), idx);
    if (VERBOSE > 1) {
      cout <<"(i,j) = (" <<idx.x() <<", " <<idx.y() <<")" <<endl;
    }
    //getDesign()->getTopBlock()->getGCellBox(idx, routeBox);
    frBox routeBox1;
    getDesign()->getTopBlock()->getGCellBox(frPoint(idx.x(), idx.y()), routeBox1);
    frBox routeBox2;
    getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, idx.x() + size-1), 
                                                    min((int)ygp.getCount(), idx.y() + size-1)), routeBox2);
    frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
    auto bp = initDR_mergeBoundaryPin(idx.x(), idx.y(), size, routeBox);
    //routeBox.set(129*2000, 94.05*2000, 132*2000, 96.9*2000);
    worker.setRouteBox(routeBox);
    frBox extBox;
    routeBox.bloat(2000, extBox);
    frBox drcBox;
    routeBox.bloat(500, drcBox);
    worker.setRouteBox(routeBox);
    worker.setExtBox(extBox);
    worker.setDrcBox(drcBox);
    worker.setFollowGuide(false);
    worker.setCost(DRCCOST, 0, 0, 0);
    //int i = (129   * 2000 - xgp.getStartCoord()) / xgp.getSpacing();
    //int j = (94.05 * 2000 - ygp.getStartCoord()) / ygp.getSpacing();
    //worker.setDRIter(0, gcell2BoundaryPin[idx.x()][idx.y()]);
    worker.setDRIter(0, bp);
    worker.setEnableDRC(enableDRC);
    //worker.setTest(true);
    worker.main();
    cout <<"done"  <<endl <<flush;
    /*
    {
    //FlexDRWorker worker(getDesign());
    FlexDRWorker worker(this);
    //frBox routeBox;
    //routeBox.set(0*2000, 0*2000, 1*2000, 1*2000);
    frCoord xl = 84 * 2000;
    frCoord yl = 0 * 2000;
    //frCoord xh = 129 * 2000;
    //frCoord yh = 94.05 * 2000;
    frPoint idx;
    getDesign()->getTopBlock()->getGCellIdx(frPoint(xl, yl), idx);
    if (VERBOSE > 1) {
      cout <<"(i,j) = (" <<idx.x() <<", " <<idx.y() <<")" <<endl;
    }
    //getDesign()->getTopBlock()->getGCellBox(idx, routeBox);
    frBox routeBox1;
    getDesign()->getTopBlock()->getGCellBox(frPoint(idx.x(), idx.y()), routeBox1);
    frBox routeBox2;
    getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, idx.x() + size-1), 
                                                    min((int)ygp.getCount(), idx.y() + size-1)), routeBox2);
    frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
    auto bp = initDR_mergeBoundaryPin(idx.x(), idx.y(), size, routeBox);
    //routeBox.set(129*2000, 94.05*2000, 132*2000, 96.9*2000);
    worker.setRouteBox(routeBox);
    frBox extBox;
    routeBox.bloat(1000, extBox);
    frBox drcBox;
    routeBox.bloat(500, drcBox);
    worker.setRouteBox(routeBox);
    worker.setExtBox(extBox);
    worker.setDrcBox(drcBox);
    worker.setFollowGuide(false);
    worker.setCost(DRCCOST, 0, 0, 0);
    //int i = (129   * 2000 - xgp.getStartCoord()) / xgp.getSpacing();
    //int j = (94.05 * 2000 - ygp.getStartCoord()) / ygp.getSpacing();
    //worker.setDRIter(0, gcell2BoundaryPin[idx.x()][idx.y()]);
    worker.setDRIter(0, bp);
    worker.setEnableDRC(enableDRC);
    //worker.setTest(true);
    worker.main();
    cout <<"done"  <<endl <<flush;
    }
    */
  } else {
    //vector<FlexDRWorker> workers;
    int cnt = 0;
    //int tot = (int)xgp.getCount() * (int)ygp.getCount();
    int tot = (((int)xgp.getCount() - 1) / size + 1) * (((int)ygp.getCount() - 1) / size + 1);
    int prev_perc = 0;
    bool isExceed = false;
    for (int i = 0; i < (int)xgp.getCount(); i += size) {
      for (int j = 0; j < (int)ygp.getCount(); j += size) {
    //for (int i = 310; i < 330; i++) {
    //  for (int j = 300; j < 320; j++) {
        //FlexDRWorker worker(getDesign());
        FlexDRWorker worker(this);
        frBox routeBox1;
        getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox1);
        frBox routeBox2;
        getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, i + size-1), 
                                                        min((int)ygp.getCount(), j + size-1)), routeBox2);
        //frBox routeBox;
        frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
        //getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox);
        //if (!(routeBox.left()   >= 504 * 2000  && routeBox.right() <= 525*2000 &&
        //    routeBox.bottom() >= 538.65*2000 && routeBox.top()   <= 558.6*2000)) {
        //  continue;
        //}
        frBox extBox;
        routeBox.bloat(2000, extBox);
        frBox drcBox;
        routeBox.bloat(500, drcBox);
        worker.setRouteBox(routeBox);
        worker.setExtBox(extBox);
        worker.setDrcBox(drcBox);
        //workers.push_back(worker);
        //worker.setDRIter(0, &gcell2BoundaryPin[i][j]);
        auto bp = initDR_mergeBoundaryPin(i, j, size, routeBox);
        worker.setDRIter(0, bp);
        // set boundary pin
        worker.setEnableDRC(enableDRC);
        worker.setFollowGuide(false);
        //worker.setFollowGuide(true);
        worker.setCost(DRCCOST, 0, 0, 0);
        worker.main();
        numQuickMarkers += worker.getNumQuickMarkers();
        //cout <<"i/j = " <<i <<", " <<j <<endl;
        cnt++;
        if (VERBOSE > 0) {
          if (cnt * 1.0 / tot >= prev_perc / 100.0 + 0.1) {
            if (prev_perc == 0 && t.isExceed(0)) {
              isExceed = true;
            }
            prev_perc += 10;
            //if (true) {
            if (isExceed) {
              if (enableDRC) {
                cout <<"    completing " <<prev_perc <<"% with " <<getDesign()->getTopBlock()->getNumMarkers() <<" violations" <<endl;
              } else {
                cout <<"    completing " <<prev_perc <<"% with " <<numQuickMarkers <<" quick violations" <<endl;
              }
              cout <<"    " <<t <<endl <<flush;
            }
          }
        }
        //if (cnt % 10000 == 0) {
        //  cout <<"    complete " <<cnt <<"/" <<tot <<endl;
        //  cout <<"    " <<t <<endl;
        //  //cout <<"  completing XX% with YY violations" <<endl;
        //  //cout <<"  elapsed time = 00:04:55, memory = ZZZZ.ZZ (MB)" <<endl;
        //}
      }
    }
    checkConnectivity();
  }

  //cout <<"  number of violations = " <<numMarkers <<endl;
  removeGCell2BoundaryPin();

  if (VERBOSE > 0) {
    if (enableDRC) {
      cout <<"  number of violations = "       <<getDesign()->getTopBlock()->getNumMarkers() <<endl;
    } else {
      cout <<"  number of quick violations = " <<numQuickMarkers <<endl;
    }
    //cout <<"    by layer and type :" <<endl;
    //cout <<"           MetSpc EOLSpc Loop CutSpc AdjCut CorSpc Others Totals" <<endl;
    //cout <<"    Metal1      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"    Totals      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"cpu time = 00:00:00, elapsed time = 00:00:00, memory = 0000.00 (MB), peak = 0000.00 (MB)" <<endl;
    t.print();
    cout <<flush;
  }
}

void FlexDR::searchRepair(int iter, int size, int offset, int mazeEndIter, 
                          frUInt4 workerDRCCost, frUInt4 workerMarkerCost, 
                          frUInt4 workerMarkerBloatWidth, frUInt4 workerMarkerBloatDepth,
                          bool enableDRC, int ripupMode, bool followGuide, 
                          int fixMode, bool TEST) {
  frTime t;
  //bool TEST = false;
  //bool TEST = true;
  if (VERBOSE > 0) {
    cout <<endl <<"start " <<iter;
    string suffix;
    if (iter == 1 || (iter > 20 && iter % 10 == 1)) {
      suffix = "st";
    } else if (iter == 2 || (iter > 20 && iter % 10 == 2)) {
      suffix = "nd";
    } else if (iter == 3 || (iter > 20 && iter % 10 == 3)) {
      suffix = "rd";
    } else {
      suffix = "th";
    }
    cout <<suffix <<" optimization iteration ..." <<endl;
  }
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);
  //getRegionQuery()->initDRObj(getTech()->getLayers().size());
  //getRegionQuery()->printDRObj();
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  int numQuickMarkers = 0;
  if (TEST) {
    cout <<"search and repair test mode" <<endl <<flush;
    //FlexDRWorker worker(getDesign());
    FlexDRWorker worker(this);
    frBox routeBox;
    //frCoord xl = 148.5 * 2000;
    //frCoord yl = 570 * 2000;
    //frPoint idx;
    //getDesign()->getTopBlock()->getGCellIdx(frPoint(xl, yl), idx);
    //if (VERBOSE > 1) {
    //  cout <<"(i,j) = (" <<idx.x() <<", " <<idx.y() <<")" <<endl;
    //}
    //getDesign()->getTopBlock()->getGCellBox(idx, routeBox);
    //routeBox.set(156*2000, 108.3*2000, 177*2000, 128.25*2000);
    // routeBox.set(175*2000, 3.5*2000, 185*2000, 13.5*2000);
    // routeBox.set(0*2000, 0*2000, 200*2000, 200*2000);
    routeBox.set(126*2000, 126*2000, 147*2000, 147*2000);
    worker.setRouteBox(routeBox);
    frBox extBox;
    frBox drcBox;
    routeBox.bloat(2000, extBox);
    routeBox.bloat(500, drcBox);
    worker.setRouteBox(routeBox);
    worker.setExtBox(extBox);
    worker.setDrcBox(drcBox);
    worker.setMazeEndIter(mazeEndIter);
    worker.setTest(true);
    worker.setQuickDRCTest(true);
    //worker.setDRCTest(true);
    worker.setDRIter(iter);
    worker.setEnableDRC(enableDRC);
    worker.setRipupMode(ripupMode);
    worker.setFollowGuide(followGuide);
    worker.setFixMode(fixMode);
    //worker.setNetOrderingMode(netOrderingMode);
    worker.setCost(workerDRCCost, workerMarkerCost, workerMarkerBloatWidth, workerMarkerBloatDepth);
    worker.main();
    numQuickMarkers += worker.getNumQuickMarkers();
    cout <<"done"  <<endl <<flush;
  } else {
    //vector<FlexDRWorker> workers;
    int clipSize = size;
    int cnt = 0;
    int tot = (((int)xgp.getCount() - 1 - offset) / clipSize + 1) * (((int)ygp.getCount() - 1 - offset) / clipSize + 1);
    int prev_perc = 0;
    bool isExceed = false;
    //for (int i = offset; i < (int)xgp.getCount(); i += clipSize) {
    //  for (int j = offset; j < (int)ygp.getCount(); j += clipSize) {
    //    cnt++;
    //  }
    //}
    //cout <<"tot/cnt=" <<tot <<"/" <<cnt <<endl;
    //cnt = 0;
    for (int i = offset; i < (int)xgp.getCount(); i += clipSize) {
      for (int j = offset; j < (int)ygp.getCount(); j += clipSize) {
    //for (int i = 312; i < 330; i += 3) {
    //  for (int j = 300; j < 318; j += 3) {
        //FlexDRWorker worker(getDesign());
        FlexDRWorker worker(this);
        frBox routeBox1;
        getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox1);
        //double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        //std::cout <<"routeBox1 (" <<routeBox1.left() / dbu <<", " <<routeBox1.bottom() / dbu <<") ("
        //                          <<routeBox1.right()/ dbu <<", " <<routeBox1.top()    / dbu <<")" <<std::endl;
        frBox routeBox2;
        getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, i + clipSize-1), 
                                                        min((int)ygp.getCount(), j + clipSize-1)), routeBox2);
        frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
        frBox extBox;
        frBox drcBox;
        routeBox.bloat(2000, extBox);
        routeBox.bloat(500, drcBox);
        worker.setRouteBox(routeBox);
        worker.setExtBox(extBox);
        worker.setDrcBox(drcBox);
        worker.setMazeEndIter(mazeEndIter);
        worker.setDRIter(iter);
        worker.setEnableDRC(enableDRC);
        worker.setRipupMode(ripupMode);
        worker.setFollowGuide(followGuide);
        //worker.setNetOrderingMode(netOrderingMode);
        worker.setFixMode(fixMode);
        worker.setCost(workerDRCCost, workerMarkerCost, workerMarkerBloatWidth, workerMarkerBloatDepth);
        //workers.push_back(worker);
        worker.main();
        //if (iter == 2) {
        //  double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        //  if (routeBox.left()    == 63     * dbu && 
        //      routeBox.right()   == 84     * dbu && 
        //      routeBox.bottom()  == 139.65 * dbu && 
        //      routeBox.top()     == 159.6  * dbu) { 
        //    return;
        //  }
        //}
        numQuickMarkers += worker.getNumQuickMarkers();
        cnt++;
        if (VERBOSE > 0) {
          if (cnt * 1.0 / tot >= prev_perc / 100.0 + 0.1) {
            if (prev_perc == 0 && t.isExceed(0)) {
              isExceed = true;
            }
            prev_perc += 10;
            //if (true) {
            if (isExceed) {
              if (enableDRC) {
                cout <<"    completing " <<prev_perc <<"% with " <<getDesign()->getTopBlock()->getNumMarkers() <<" violations" <<endl;
              } else {
                cout <<"    completing " <<prev_perc <<"% with " <<numQuickMarkers <<" quick violations" <<endl;
              }
              cout <<"    " <<t <<endl <<flush;
            }
          }
        }
        //if (cnt > 10000 && ) {
        //  cout <<"    completing " <<cnt <<"/" <<tot <<endl;
        //  cout <<"    " <<t <<endl;
        //  //cout <<"  completing XX% with YY violations" <<endl;
        //  //cout <<"  elapsed time = 00:04:55, memory = ZZZZ.ZZ (MB)" <<endl;
        //}
      }
    }
  }
  //cout <<"  number of violations = " <<numMarkers <<endl;
  checkConnectivity();
  if (VERBOSE > 0) {
    if (enableDRC) {
      cout <<"  number of violations = " <<getDesign()->getTopBlock()->getNumMarkers() <<endl;
    } else {
      cout <<"  number of quick violations = " <<numQuickMarkers <<endl;
    }
    //cout <<"    by layer and type :" <<endl;
    //cout <<"           MetSpc EOLSpc Loop CutSpc AdjCut CorSpc Others Totals" <<endl;
    //cout <<"    Metal1      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"    Totals      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"cpu time = 00:00:00, elapsed time = 00:00:00, memory = 0000.00 (MB), peak = 0000.00 (MB)" <<endl;
    t.print();
    cout <<flush;
  }
}

void FlexDR::end() {
  vector<unsigned long long> wlen(getTech()->getLayers().size(), 0);
  vector<unsigned long long> sCut(getTech()->getLayers().size(), 0);
  vector<unsigned long long> mCut(getTech()->getLayers().size(), 0);
  unsigned long long totWlen = 0;
  unsigned long long totSCut = 0;
  unsigned long long totMCut = 0;
  frPoint bp, ep;
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    for (auto &shape: net->getShapes()) {
      if (shape->typeId() == frcPathSeg) {
        auto obj = static_cast<frPathSeg*>(shape.get());
        obj->getPoints(bp, ep);
        auto lNum = obj->getLayerNum();
        frCoord psLen = ep.x() - bp.x() + ep.y() - bp.y();
        wlen[lNum] += psLen;
        totWlen += psLen;
      }
    }
    for (auto &via: net->getVias()) {
      auto lNum = via->getViaDef()->getCutLayerNum();
      if (via->getViaDef()->isMultiCut()) {
        ++mCut[lNum];
        ++totMCut;
      } else {
        ++sCut[lNum];
        ++totSCut;
      }
    }
  }
  if (VERBOSE > 0) {
    boost::io::ios_all_saver guard(std::cout);
    cout <<endl <<"total wire length = " <<totWlen / getDesign()->getTopBlock()->getDBUPerUU() <<" um" <<endl;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::ROUTING) {
        cout <<"total wire length on LAYER " <<getTech()->getLayer(i)->getName() <<" = " 
             <<wlen[i] / getDesign()->getTopBlock()->getDBUPerUU() <<" um" <<endl;
      }
    }
    cout <<"total number of vias = " <<totSCut + totMCut <<endl;
    if (totMCut > 0) {
      cout <<"total number of multi-cut vias = " <<totMCut 
           << " (" <<setw(5) <<fixed <<setprecision(1) <<totMCut * 100.0 / (totSCut + totMCut) <<"%)" <<endl;
      cout <<"total number of single-cut vias = " <<totSCut 
           << " (" <<setw(5) <<fixed <<setprecision(1) <<totSCut * 100.0 / (totSCut + totMCut) <<"%)" <<endl;
    }
    cout <<"up-via summary (total " <<totSCut + totMCut <<"):" <<endl;
    int nameLen = 0;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::CUT) {
        nameLen = max(nameLen, (int)getTech()->getLayer(i-1)->getName().size());
      }
    }
    int maxL = 1 + nameLen + 4 + (int)to_string(totSCut).length();
    if (totMCut) {
      maxL += 9 + 4 + (int)to_string(totMCut).length() + 9 + 4 + (int)to_string(totSCut + totMCut).length();
    }
    if (totMCut) {
      cout <<" " <<setw(nameLen + 4 + (int)to_string(totSCut).length() + 9) <<"single-cut";
      cout <<setw(4 + (int)to_string(totMCut).length() + 9) <<"multi-cut" 
           <<setw(4 + (int)to_string(totSCut + totMCut).length()) <<"total";
    }
    cout <<endl;
    for (int i = 0; i < maxL; i++) {
      cout <<"-";
    }
    cout <<endl;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::CUT) {
        cout <<" "    <<setw(nameLen) <<getTech()->getLayer(i-1)->getName() 
             <<"    " <<setw((int)to_string(totSCut).length()) <<sCut[i];
        if (totMCut) {
          cout <<" ("   <<setw(5) <<(double)((sCut[i] + mCut[i]) ? sCut[i] * 100.0 / (sCut[i] + mCut[i]) : 0.0) <<"%)";
          cout <<"    " <<setw((int)to_string(totMCut).length()) <<mCut[i] 
               <<" ("   <<setw(5) <<(double)((sCut[i] + mCut[i]) ? mCut[i] * 100.0 / (sCut[i] + mCut[i]) : 0.0) <<"%)"
               <<"    " <<setw((int)to_string(totSCut + totMCut).length()) <<sCut[i] + mCut[i];
        }
        cout <<endl;
      }
    }
    for (int i = 0; i < maxL; i++) {
      cout <<"-";
    }
    cout <<endl;
    cout <<" "    <<setw(nameLen) <<""
         <<"    " <<setw((int)to_string(totSCut).length()) <<totSCut;
    if (totMCut) {
      cout <<" ("   <<setw(5) <<(double)((totSCut + totMCut) ? totSCut * 100.0 / (totSCut + totMCut) : 0.0) <<"%)";
      cout <<"    " <<setw((int)to_string(totMCut).length()) <<totMCut 
           <<" ("   <<setw(5) <<(double)((totSCut + totMCut) ? totMCut * 100.0 / (totSCut + totMCut) : 0.0) <<"%)"
           <<"    " <<setw((int)to_string(totSCut + totMCut).length()) <<totSCut + totMCut;
    }
    cout <<endl <<endl <<flush;
    guard.restore();
  }
}

void FlexDR::reportDRC() {
  double dbu = design->getTech()->getDBUPerUU();
  cout << DRC_RPT_FILE << "\n";
  if (DRC_RPT_FILE != string("")) {
    ofstream drcRpt(DRC_RPT_FILE.c_str());
    if (drcRpt.is_open()) {
      for (auto &marker: getDesign()->getTopBlock()->getMarkers()) {
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
               << bbox.right() / dbu << ", " << bbox.top() / dbu << ") on Layer " 
               << getTech()->getLayer(marker->getLayerNum())->getName() << "\n";
      }
    } else {
      cout << "Error: Fail to open DRC report file\n";
    }
  } else {
    cout << "Error: DRC report file is not specified\n";
  }
}


int FlexDR::main() {
  init();
  //exit(1);
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<endl <<"start detail routing ...";
  }
  // initDR: enableDRC
  initDR(7, true);
  //cout   <<endl
  //       <<"time (INIT/ROUTE/POST) " <<time_span_init.count() <<" " 
  //                                   <<time_span_route.count() <<" "
  //                                   <<time_span_end.count() <<" "
  //                                   <<endl;
  //io::Writer writer(getDesign());
  //writer.writeFromDR("_init");
  // search and repair: iter, size, offset, mazeEndIter, workerDRCCost, workerMarkerCost, 
  //                    markerBloatWidth, markerBloatDepth, enableDRC, ripupMode, followGuide, fixMode, TEST
  // fixMode:
  //   0 - general fix
  //   1 - fat via short, spc to wire fix (keep via net), no permutation, increasing DRCCOST
  //   2 - fat via short, spc to wire fix (ripup via net), no permutation, increasing DRCCOST
  //   3 - general fix, ripup everything (bloat)
  //   4 - general fix, ripup left/bottom net (touching), currently DISABLED
  //   5 - general fix, ripup right/top net (touching), currently DISABLED
  //   6 - two-net viol
  // assume only mazeEndIter > 1 if enableDRC and ripupMode == 0 (partial ripup)
  //end();
  //searchRepair(1,  7, -4,  1, DRCCOST, 0,          0, 0, true, 1, false, 0, true); // test mode
  end();
  searchRepair(1,  7, -4,  1, DRCCOST, 0,          0, 0, true, 1, false, 0); // func as fully rerouting iter, no marker cost
  //end();
  //searchRepair(1,  7, 3,  1, DRCCOST, MARKERCOST, 1, 0, true, 1, false, 0); // func as fully rerouting iter, no marker cost
  //end();
  //searchRepair(2,  7, 0,  5, DRCCOST, 0,          0, 0, true, 0, false, 0); // true search and repair
  end();
  searchRepair(2,  7,  0, 4, DRCCOST, MARKERCOST,  0, 0, true, 0, false, 3); // true search and repair
  end();
  searchRepair(3,  7, -4, 4, DRCCOST, MARKERCOST,  0, 0, true, 0, false, 3); // true search and repair
  end();
  searchRepair(4,  7,  0, 4, DRCCOST, MARKERCOST,  8, 2, true, 0, false, 3); // true search and repair
  end();
  searchRepair(5,  7, -4, 4, DRCCOST, MARKERCOST,  8, 2, true, 0, false, 3); // true search and repair
  end();
  searchRepair(6,  7,  0, 4, DRCCOST, MARKERCOST,  8, 2, true, 0, false, 3); // true search and repair
  end();
  searchRepair(7,  7, -4, 4, DRCCOST, MARKERCOST,  8, 2, true, 0, false, 3); // true search and repair
  reportDRC();
  //time_span_init  = std::chrono::duration<double>(0);
  //time_span_route = std::chrono::duration<double>(0);
  //time_span_end   = std::chrono::duration<double>(0);
  //cout   <<endl 
  //       <<"time (INIT/ROUTE/POST) " <<time_span_init.count() <<" " 
  //                                   <<time_span_route.count() <<" "
  //                                   <<time_span_end.count() <<" "
  //                                   <<endl;
  if (VERBOSE > 0) {
    cout <<endl <<"complete detail routing";
    end();
  }
  if (VERBOSE > 0) {
    t.print();
    cout <<endl;
  }
  return 0;
}

