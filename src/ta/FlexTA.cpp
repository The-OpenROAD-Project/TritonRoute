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
#include <sstream>
#include <chrono>
#include "global.h"
#include "FlexTA.h"
#include "db/infra/frTime.h"
#include <algorithm>
//#include <omp.h>

using namespace std;
using namespace fr;
using namespace boost::polygon::operators;

int FlexTAWorker::main() {
  using namespace std::chrono;
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  if (VERBOSE > 1) {
    stringstream ss;
    //cout <<endl <<"start TA worker (BOX/LAYER) " <<routeBox <<" " <<layerNum <<endl;
    ss <<endl <<"start TA worker (BOX) ("
       <<routeBox.left()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", "
       <<routeBox.bottom() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") ("
       <<routeBox.right()  * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", "
       <<routeBox.top()    * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") ";
    if (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir) {
      ss <<"H";
    } else {
      ss <<"V";
    }
    ss <<endl;
    cout <<ss.str();
  }


  //assignIroutes();
  //reassignIroutes();
  //saveToGuides();
  //reportCosts();
  init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  assign();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  end();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();

  duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);
  duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);

  if (VERBOSE > 1) {
    stringstream ss;
    ss   <<"time (INIT/ASSIGN/POST) " <<time_span0.count() <<" " 
                                      <<time_span1.count() <<" "
                                      <<time_span2.count() <<" "
                                      <<endl;
    cout <<ss.str() <<flush;
  }
  return 0;
}

int FlexTA::initTA_helper(int iter, int size, int offset, bool isH, int &numPanels) {
  //frBox dieBox;
  //getDesign()->getTopBlock()->getBoundaryBBox(dieBox);

  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  int sol = 0;
  numPanels = 0;
  if (isH) {
    for (int i = offset; i < (int)ygp.getCount(); i += size) {
      FlexTAWorker worker(getDesign());
      frBox beginBox, endBox;
      getDesign()->getTopBlock()->getGCellBox(frPoint(0, i), beginBox);
      getDesign()->getTopBlock()->getGCellBox(frPoint((int)xgp.getCount() - 1, 
                                                      min(i + size - 1, (int)ygp.getCount() - 1)), endBox);
      frBox routeBox(beginBox.left(), beginBox.bottom(), endBox.right(), endBox.top());
      frBox extBox;
      routeBox.bloat(ygp.getSpacing() / 2, extBox);
      worker.setRouteBox(routeBox);
      worker.setExtBox(extBox);
      worker.setDir(frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
      worker.setTAIter(iter);
      worker.main();
      //int numAssigned = worker.getNumAssigned();
      sol += worker.getNumAssigned();
      numPanels++;
      //if (VERBOSE > 0) {
      //  cout <<"Done with " <<numAssigned <<"horizontal wires";
      //}
    }
  } else {
    for (int i = offset; i < (int)xgp.getCount(); i += size) {
      FlexTAWorker worker(getDesign());
      frBox beginBox, endBox;
      getDesign()->getTopBlock()->getGCellBox(frPoint(i, 0),                       beginBox);
      getDesign()->getTopBlock()->getGCellBox(frPoint(min(i + size - 1, (int)xgp.getCount() - 1),
                                                      (int)ygp.getCount() - 1), endBox);
      frBox routeBox(beginBox.left(), beginBox.bottom(), endBox.right(), endBox.top());
      frBox extBox;
      routeBox.bloat(xgp.getSpacing() / 2, extBox);
      worker.setRouteBox(routeBox);
      worker.setExtBox(extBox);
      worker.setDir(frPrefRoutingDirEnum::frcVertPrefRoutingDir);
      worker.setTAIter(iter);
      worker.main();
      //int numAssigned = worker.getNumAssigned();
      sol += worker.getNumAssigned();
      numPanels++;
      //if (VERBOSE > 0) {
      //  cout <<"Done with " <<numAssigned <<"vertical wires";
      //}
    }
  }
  return sol;
}

void FlexTA::initTA(int size) {
  frTime t;

  if (VERBOSE > 1) {
    cout <<endl <<"start initial track assignment ..." <<endl;
  }

  auto bottomLNum = getDesign()->getTech()->getBottomLayerNum();
  auto bottomLayer = getDesign()->getTech()->getLayer(bottomLNum);
  if (bottomLayer->getType() != frLayerTypeEnum::ROUTING) {
    bottomLNum++;
    bottomLayer = getDesign()->getTech()->getLayer(bottomLNum);
  }
  bool isBottomLayerH = (bottomLayer->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);

  int numAssigned = 0;
  int numPanels = 0;

  // H first
  if (isBottomLayerH) {
    numAssigned = initTA_helper(0, size, 0, true, numPanels);
    if (VERBOSE > 0) {
      cout <<"Done with " <<numAssigned <<" horizontal wires in " <<numPanels <<" frboxes and ";
    }
    initTA_helper(0, size, 0, false, numPanels);
    if (VERBOSE > 0) {
      cout <<numAssigned <<" vertical wires in " <<numPanels <<" frboxes." <<endl;
    }
  // V first
  } else {
    initTA_helper(0, size, 0, false, numPanels);
    if (VERBOSE > 0) {
      cout <<"Done with " <<numAssigned <<" vertical wires in " <<numPanels <<" frboxes and ";
    }
    initTA_helper(0, size, 0, true, numPanels);
    if (VERBOSE > 0) {
      cout <<numAssigned <<" horizontal wires in " <<numPanels <<" frboxes." <<endl;
    }
  }
}

void FlexTA::searchRepair(int iter, int size, int offset) {
  frTime t;

  if (VERBOSE > 1) {
    if (iter == -1) {
      cout <<endl <<"start polishing ..." <<endl;
    } else {
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
  }
  auto bottomLNum = getDesign()->getTech()->getBottomLayerNum();
  auto bottomLayer = getDesign()->getTech()->getLayer(bottomLNum);
  if (bottomLayer->getType() != frLayerTypeEnum::ROUTING) {
    bottomLNum++;
    bottomLayer = getDesign()->getTech()->getLayer(bottomLNum);
  }
  bool isBottomLayerH = (bottomLayer->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);

  //int sol = 0;
  int numAssigned = 0;
  int numPanels = 0;
  // H first
  if (isBottomLayerH) {
    numAssigned = initTA_helper(iter, size, offset, true, numPanels);
    if (VERBOSE > 0) {
      cout <<"Done with " <<numAssigned <<" horizontal wires in " <<numPanels <<" frboxes and ";
    }
    numAssigned = initTA_helper(iter, size, offset, false, numPanels);
    if (VERBOSE > 0) {
      cout <<numAssigned <<" vertical wires in " <<numPanels <<" frboxes." <<endl;
    }
  // V first
  } else {
    numAssigned = initTA_helper(iter, size, offset, false, numPanels);
    if (VERBOSE > 0) {
      cout <<"Done with " <<numAssigned <<" vertical wires in " <<numPanels <<" frboxes and ";
    }
    numAssigned = initTA_helper(iter, size, offset, true, numPanels);
    if (VERBOSE > 0) {
      cout <<numAssigned <<" horizontal wires in " <<numPanels <<" frboxes." <<endl;
    }
  }
}

int FlexTA::main() {
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<endl <<"start track assignment" <<endl;
  }
  initTA(50);
  searchRepair(1, 50, 0);
  //searchRepair(2, 50, 0);
  //searchRepair(2, 50, 0);
  //searchRepair(-1, 50, 0);

  if (VERBOSE > 0) {
    cout <<endl <<"complete track assignment";
    //end();
  }
  if (VERBOSE > 0) {
    cout <<endl;
    t.print();
    cout <<endl;
  }
  return 0;
}

/*
void FlexTAWorker::getAllGuides() {
  //bool enableOutput = true;
  bool enableOutput = false;
  vector<rq_rptr_value_t<frGuide> > result1;
  getDesign()->getRegionQuery()->queryGuide(routeBox, layerNum, result1);
  //cout <<endl <<"query1:" <<endl;
  for (auto &[boostb, guide]: result1) {
    frPoint pt1, pt2;
    guide->getPoints(pt1, pt2);
    if (enableOutput) {
      cout <<"found guide (" 
           <<pt1.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
           <<pt1.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") (" 
           <<pt2.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
           <<pt2.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") " 
           <<guide->getNet()->getName() << "\n";
    }
    guides.push_back(guide);
    //cout <<endl;
  }
  sort(guides.begin(), guides.end(), [](const frGuide *a, const frGuide *b) {return *a < *b;});
}
*/

/*
void FlexTAWorker::getAllTracks() {
  //bool enableOutput = true;
  bool enableOutput = false;
  // uPtr for tp
  for (auto &tp: getDesign()->getTopBlock()->getTrackPatterns(layerNum)) {
    if ((getTech()->getLayers().at(layerNum)->getDir() == frcHorzPrefRoutingDir &&
         tp->isHorizontal() == false) ||
        (getTech()->getLayers().at(layerNum)->getDir() == frcVertPrefRoutingDir &&
         tp->isHorizontal() == true)) {
      if (enableOutput) {
        cout <<"TRACKS " <<(tp->isHorizontal() ? string("X ") : string("Y "))
             <<tp->getStartCoord() <<" DO " <<tp->getNumTracks() <<" STEP "
             <<tp->getTrackSpacing() <<" LAYER " <<tp->getLayerNum() 
             <<" ;" <<endl;
      }
      if (getTech()->getLayers().at(layerNum)->getDir() == frcHorzPrefRoutingDir) {
        int trackNum = (routeBox.bottom() - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (trackNum < 0) {
          trackNum = 0;
        }
        if (trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < routeBox.bottom()) {
          trackNum++;
        }
        for (; trackNum < (int)tp->getNumTracks() && trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < routeBox.top(); trackNum++) {
          unique_ptr<FlexTrack> ft = make_unique<FlexTrack>();
          ft->setTrackPattern(tp.get());
          ft->setTrackLoc(trackNum * tp->getTrackSpacing() + tp->getStartCoord());
          tracks.push_back(std::move(ft));
        }
      } else {
        int trackNum = (routeBox.left() - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (trackNum < 0) {
          trackNum = 0;
        }
        if (trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < routeBox.left()) {
          trackNum++;
        }
        for (; trackNum < (int)tp->getNumTracks() && trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < routeBox.right(); trackNum++) {
          unique_ptr<FlexTrack> ft = make_unique<FlexTrack>();
          ft->setTrackPattern(tp.get());
          ft->setTrackLoc(trackNum * tp->getTrackSpacing() + tp->getStartCoord());
          tracks.push_back(std::move(ft));
        }
      }

    }
  }
  if (enableOutput) {
    for (auto &t: tracks) {
      cout <<"TRACKLOC " <<t->getTrackLoc() <<endl;
    }
  }
}
*/

// return_val = 0: left set, right set
// return_val = 1: left set, right not set
// return_val = 2: left not set, right set
// return_val = 3: left not set, right not set
/*
void FlexTAWorker::genIroutes_setBeginEnd(FlexIroute* iroute, bool isH) {
  auto guide = iroute->getGuide();
  auto net   = guide->getNet();
  if (guide) {
    ;
  } else {
    cout <<"Error: genIroutes_setBeginEnd cannot lock guide" <<endl;
    exit(1);
  }
  bool    hasMaxBegin = false;
  bool    hasMinEnd   = false;
  frCoord maxBegin    = 0;
  frCoord minEnd      = 0;
  frPoint segBegin, segEnd;
  
  frPoint bp, ep;
  guide->getPoints(bp, ep);
  frPoint cp;
  // layerNum in FlexTAWorker
  vector<frGuide*> nbrGuides;
  auto rq = getDesign()->getRegionQuery();
  int wlen = 0;
  frBox box;
  for (int i = 0; i < 2; i++) {
    nbrGuides.clear();
    // check left
    if (i == 0) {
      box.set(bp, bp);
      cp = bp;
    // check right
    } else {
      box.set(ep, ep);
      cp = ep;
    }
    if (layerNum - 2 >= 0) {
      rq->queryGuide(box, layerNum - 2, nbrGuides);
    } 
    if (layerNum + 2 < (int)design->getTech()->getLayers().size()) {
      rq->queryGuide(box, layerNum + 2, nbrGuides);
    } 
    for (auto &guide: nbrGuides) {
      if (guide->getNet() == net) {
        for (auto &connFig: guide->getRoutes()) {
          if (connFig->typeId() == frcPathSeg) {
            static_cast<frPathSeg*>(connFig.get())->getPoints(segBegin, segEnd);
            if (i == 0) {
              if (hasMaxBegin) {
                maxBegin = max(maxBegin, (isH ? segBegin.x() : segBegin.y()));
              } else {
                maxBegin = (isH ? segBegin.x() : segBegin.y());
                hasMaxBegin = true;
              }
            } else {
              if (hasMinEnd) {
                minEnd = min(minEnd, (isH ? segBegin.x() : segBegin.y()));
              } else {
                minEnd = (isH ? segBegin.x() : segBegin.y());
                hasMinEnd = true;
              }
            }
          }
        }
        frPoint nbrBp, nbrEp;
        if (cp == nbrEp) {
          wlen -= 1;
        } else {
          wlen += 1;
        }
      }
    }
  }

  frPoint guideBegin, guideEnd;
  guide->getPoints(guideBegin, guideEnd);
  if (!hasMaxBegin) {
    maxBegin = (isH ? guideBegin.x() : guideBegin.y());
  }
  if (!hasMinEnd) {
    minEnd = (isH ? guideEnd.x() : guideEnd.y());
  }
  if (maxBegin > minEnd) {
    swap(maxBegin, minEnd);
  }
  if (maxBegin == minEnd) {
    minEnd += 1;
  }

  iroute->setBegin(maxBegin);
  iroute->setEnd(minEnd);
  iroute->setWlenHelper(wlen);

}
*/

/*
void FlexTAWorker::genIroutes() {
  unique_ptr<FlexIroute> iroute;
  bool isH = (getDir() == frcHorzPrefRoutingDir) ? true : false;
  for (auto guide: guides) {
    if (guide) {
      ;
    } else {
      continue;
    }
    iroute = make_unique<FlexIroute>();
    iroute->setGuide(guide);
    genIroutes_setBeginEnd(iroute.get(), isH);
    iroutes.push_back(std::move(iroute));
  }
  sort(iroutes.begin(), iroutes.end(), 
       [](const unique_ptr<FlexIroute> &a, const unique_ptr<FlexIroute> &b) {
         if (a->getEnd() - a->getBegin() == b->getEnd() - b->getBegin()) {
           return *(a->getGuide()) < *(b->getGuide());
         } else {
           return (a->getEnd() - a->getBegin()) > (b->getEnd() - b->getBegin());
         }
       }
      );
}
*/
/*
void FlexTAWorker::init() {
  //bool enableOutput = true;
  bool enableOutput = false;
  getAllGuides();
  //if (enableOutput) {
  //  stringstream ss;
  //  ss <<"#guidess = " <<guides.size() <<endl;
  //  cout <<ss.str() <<flush;
  //}
  //sort(guides.begin(), guides.end(), 
  //     [](const weak_ptr<frGuide> &a, const weak_ptr<frGuide> &b) {
  //       frBegin

  //       return a.lock()->getTrackLoc() > b->getTrackLoc();
  //     }
  //    );

  genIroutes();

  if (enableOutput) {
    stringstream ss;
    ss <<"#guides = " <<guides.size() <<endl;
    cout <<ss.str() <<flush;
  }
  getAllTracks();
  sort(tracks.begin(), tracks.end(), 
       [](const unique_ptr<FlexTrack> &a, const unique_ptr<FlexTrack> &b) {
         return a->getTrackLoc() > b->getTrackLoc();
       }
      );
  auto last = unique(tracks.begin(), tracks.end(), 
                     [](const unique_ptr<FlexTrack> &a, const unique_ptr<FlexTrack> &b) {
                       return a->getTrackLoc() == b->getTrackLoc();
                      }
                     );
  tracks.erase(last, tracks.end());

}
*/

//bool FlexTAWorker::isViaOnPathSeg(const shared_ptr<frPathSeg> &pathSeg, const shared_ptr<frVia> &via) {
//  frPoint begin, end, viaPoint;
//  pathSeg->getPoints(begin, end);
//  via->getOrigin(viaPoint);
//  // vertical seg
//  if (begin.x() == end.x() && begin.x() == viaPoint.x() && begin.y() <= viaPoint.y() && end.y() >= viaPoint.y()) {
//    return true;
//  }
//  // horizontal seg
//  if (begin.y() == end.y() && begin.y() == viaPoint.y() && begin.x() <= viaPoint.x() && end.x() >= viaPoint.x()) {
//    return true;
//  }
//  return false;
//}

//bool FlexTAWorker::isViaOnTrack(const shared_ptr<FlexTrack> &track, const shared_ptr<frVia> &via) {
//  frPoint viaPoint;
//  via->getOrigin(viaPoint);
//  bool isVerticalLayer = track->getTrackPattern()->isHorizontal();
//  return ((isVerticalLayer ? viaPoint.x() : viaPoint.y()) == track->getTrackLoc());
//}

//bool FlexTAWorker::isPathSegOnTrack(const shared_ptr<FlexTrack> &track, const shared_ptr<frPathSeg> &pathSeg) {
//  frPoint begin, end;
//  pathSeg->getPoints(begin, end);
//  bool isVerticalLayer = track->getTrackPattern()->isHorizontal();
//  if (isVerticalLayer) {
//    if (begin.x() == end.x() && begin.x() == track->getTrackLoc()) {
//      return true;
//    } else {
//      return false;
//    }
//  } else {
//    if (begin.y() == end.y() && begin.y() == track->getTrackLoc()) {
//      return true;
//    } else {
//      return false;
//    }
//  }
//}

//bool FlexTAWorker::hasPathSegLimit(const shared_ptr<frGuide> &guide, const shared_ptr<frPathSeg> &pathSeg, 
//                                   bool checkLowerLayer, bool checkUpperLayer) {
//  bool init = false;
//  frCoord limit = 0;
//
//  auto v = guide->getBeginConn();
//  getPathSegLimit_helper(guide, pathSeg, checkLowerLayer, checkUpperLayer, true, v, init, limit);
//  v = guide->getEndConn();
//  getPathSegLimit_helper(guide, pathSeg, checkLowerLayer, checkUpperLayer, true, v, init, limit);
//  return init;
//}

// onlu used for upper/lower layer, keep existing same-layer connection
//void FlexTAWorker::getPathSegLimit_helper(const shared_ptr<frGuide> &guide, const shared_ptr<frPathSeg> &pathSeg, 
//  bool checkLowerLayer, bool checkUpperLayer, bool isBegin, const shared_ptr<frBlockObject> &v, bool &init, 
//  frCoord &limit) {
//
//  auto net   = guide->getNet();
//
//  frPoint begin, end, viaPoint;
//  pathSeg->getPoints(begin, end);
//  //auto pathSegLayerNum = pathSeg->getLayerNum();
//  
//  frCollection<shared_ptr<frGuide> > connGuides;
//  if (v->typeId() == frcSteiner) {
//    static_pointer_cast<frSteiner>(v)->getConnGuides(connGuides);
//  } else {
//    return;
//  }
//
//  auto nbrGuide = connGuides.at(2);
//  if (nbrGuide && checkLowerLayer && nbrGuide->hasRoutes()) {
//    for (auto &connFig: nbrGuide->getRoutes()) {
//      if (connFig->typeId() == frcVia) {
//        auto via = static_pointer_cast<frVia>(connFig);
//        if (isViaOnPathSeg(pathSeg, via)) {
//          via->getOrigin(viaPoint);
//          // assumes always preferred-direction routing
//          if (getTech()->getLayer(pathSeg->getLayerNum())->getDir() == frcHorzPrefRoutingDir) {
//            if (init) {
//              limit = isBegin ? min(limit, viaPoint.x()) : max(limit, viaPoint.x());
//            } else {
//              limit = viaPoint.x();
//              init = true;
//            }
//          } else {
//            if (init) {
//              limit = isBegin ? min(limit, viaPoint.y()) : max(limit, viaPoint.y());
//            } else {
//              limit = viaPoint.y();
//              init = true;
//            }
//          }
//        }
//      }
//    }
//  }
//  nbrGuide = connGuides.at(3);
//  if (nbrGuide && checkUpperLayer && nbrGuide->hasRoutes()) {
//    for (auto &connFig: nbrGuide->getRoutes()) {
//      if (connFig->typeId() == frcVia) {
//        auto via = static_pointer_cast<frVia>(connFig);
//        if (isViaOnPathSeg(pathSeg, via)) {
//          via->getOrigin(viaPoint);
//          // assumes always preferred-direction routing
//          if (getTech()->getLayer(pathSeg->getLayerNum())->getDir() == frcHorzPrefRoutingDir) {
//            if (init) {
//              limit = isBegin ? min(limit, viaPoint.x()) : max(limit, viaPoint.x());
//            } else {
//              limit = viaPoint.x();
//              init = true;
//            }
//          } else {
//            if (init) {
//              limit = isBegin ? min(limit, viaPoint.y()) : max(limit, viaPoint.y());
//            } else {
//              limit = viaPoint.y();
//              init = true;
//            }
//          }
//        }
//      }
//    }
//  }
//
//}


//frCoord FlexTAWorker::getPathSegLimit(const shared_ptr<frGuide> &guide, const shared_ptr<frPathSeg> &pathSeg, 
//                                      bool checkLowerLayer, bool checkUpperLayer, bool isBegin) {
//  bool init = false;
//  frCoord limit = 0;
//
//  auto v = guide->getBeginConn();
//  getPathSegLimit_helper(guide, pathSeg, checkLowerLayer, checkUpperLayer, isBegin, v, init, limit);
//  v = guide->getEndConn();
//  getPathSegLimit_helper(guide, pathSeg, checkLowerLayer, checkUpperLayer, isBegin, v, init, limit);
//  return limit;
//}

// left and bottom can equal
//bool FlexTAWorker::isTrackInGuide(const shared_ptr<frGuide> &guide, const shared_ptr<FlexTrack> &track) {
//  auto trackLoc = track->getTrackLoc();
//  frPoint begin, end;
//  guide->getPoints(begin, end);
//  // isHorizontal == 1 means vertical layer !!!
//  bool isVerticalLayer = track->getTrackPattern()->isHorizontal();
//  auto guideLoc  = isVerticalLayer ? begin.x() : begin.y();
//  auto guideLow  = guideLoc - (isVerticalLayer ? getGCELLGRIDX() / 2 : getGCELLGRIDY() / 2);
//  auto guideHigh = guideLoc + (isVerticalLayer ? getGCELLGRIDX() / 2 : getGCELLGRIDY() / 2);
//  return (trackLoc >= guideLow && trackLoc < guideHigh);
//}

//int FlexTAWorker::getWlenCost_helper(const shared_ptr<FlexIroute> &iroute) {
//  auto guide = iroute->getGuide();
//  if (guide && guide->isGlobal()) {
//    ;
//  } else {
//    return 0;
//  }
//  frCollection<shared_ptr<frGuide> > connGuides;
//  frCollection<shared_ptr<frGuide> > connGuides2;
//  shared_ptr<frSteiner> steiner;
//  shared_ptr<frSteiner> steiner2;
//  int wlen = 0;
//  // left and right steiners
//  for (int i = 0; i < 2; i++) {
//    if (i == 0) {
//      steiner = static_pointer_cast<frSteiner>(guide->getBeginConn());
//    } else {
//      steiner = static_pointer_cast<frSteiner>(guide->getEndConn());
//    }
//    steiner->getConnGuides(connGuides);
//    // down and up vias
//    for (int j = 2; j < 4; j++) {
//      if (connGuides.at(j)) {
//        if (j == 2) {
//          steiner2 = static_pointer_cast<frSteiner>(connGuides.at(j)->getBeginConn());
//        } else {
//          steiner2 = static_pointer_cast<frSteiner>(connGuides.at(j)->getEndConn());
//        }
//        steiner2->getConnGuides(connGuides2);
//        if (connGuides2.at(0) && connGuides2.at(0)->isGlobal()) {
//          wlen -= 1;
//        }
//        if (connGuides2.at(1) && connGuides2.at(1)->isGlobal()) {
//          wlen += 1;
//        }
//      }
//    }
//  }
//  return wlen;
//}

/*
frUInt4 FlexTAWorker::getWlenCost(FlexIroute* iroute, FlexTrack* track, 
                                  bool isH, int wlen_helper) {

  //auto &gp = design->getTopBlock()->getGCellPatterns();
  //auto &xgp = gp[0];
  //auto &ygp = gp[1];
  // use frCoord to prevent negative problems
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  //frCoord GCELLOFFSETX = xgp.getStartCoord();
  //frCoord GCELLOFFSETY = ygp.getStartCoord();
  //frCoord GCELLCNTX    = xgp.getCount();
  //frCoord GCELLCNTY    = ygp.getCount();
  auto guide = iroute->getGuide();
  //if (guide && guide->isGlobal()) {
  //  ;
  //} else {
  //  return 0;
  //}
  frPoint begin, end;
  guide->getPoints(begin, end);
  frBox endBox;
  frPoint idx;
  getDesign()->getTopBlock()->getGCellIdx(end, idx);
  getDesign()->getTopBlock()->getGCellBox(idx, endBox);
  int wlen = 0;
  if (wlen_helper <= 0) {
    if (isH) {
      wlen = abs(wlen_helper) * (track->getTrackLoc() - endBox.bottom());
    } else {
      wlen = abs(wlen_helper) * (track->getTrackLoc() - endBox.left());
    }
  } else {
    if (isH) {
      wlen = abs(wlen_helper) * (endBox.top() - track->getTrackLoc());
    } else {
      wlen = abs(wlen_helper) * (endBox.right() - track->getTrackLoc());
    }
  }
  if (wlen < 0) {
    cout <<"Error: getWlenCost has wlenCost < 0" <<endl;
    cout <<"  trackLoc " <<track->getTrackLoc() <<" box " <<endBox <<endl;
    return (frUInt4)0;
  } else {
    return (frUInt4)wlen;
  }
}
*/

/*
void FlexTAWorker::assignIroutes() {
  //bool enableOutput = true;
  bool enableOutput = false;
  bool isH = (getDir() == frcHorzPrefRoutingDir) ? true : false;
  for(auto &uIroute: iroutes) {
    auto iroute = uIroute.get();
    int bestTrack = -1;
    frUInt4 bestCost = UINT_MAX;
    //int wlen_helper  = getWlenCost_helper(iroute);
    int wlen_helper  = iroute->getWlenHelper();
    for (int i = 0; i < (int)tracks.size(); i++) {
      frUInt4 currCost = 0;
      auto track = tracks.at(i).get();
      currCost += 100*track->getOverlapCost(iroute->getBegin(), iroute->getEnd());
      currCost += getWlenCost(iroute, track, isH, wlen_helper);
      if (currCost < bestCost) {
        bestTrack = i;
        bestCost = currCost;
      }
    }
    if (bestTrack == -1) {
      cout <<"Error: cannot find a track" <<endl;
      exit(1);
    } else {
      iroute->setTrack(tracks.at(bestTrack).get());
      tracks.at(bestTrack)->addToOverlapCost(iroute->getBegin(), iroute->getEnd());
      if (enableOutput) {
        auto dbu = getDesign()->getTopBlock()->getDBUPerUU();
        auto layerName = getTech()->getLayer(layerNum)->getName();
        stringstream ss;
        if (isH) {
          ss   <<"assigned ( " 
               <<iroute->getBegin()                * 1.0 / dbu <<" "
               <<iroute->getTrack()->getTrackLoc() * 1.0 / dbu <<" ) ( "
               <<iroute->getEnd()                  * 1.0 / dbu <<" "
               <<iroute->getTrack()->getTrackLoc() * 1.0 / dbu <<" ) "
               <<layerName <<endl;
        } else {
          ss   <<"assigned ( " 
               <<iroute->getTrack()->getTrackLoc() * 1.0 / dbu <<" "
               <<iroute->getBegin()                * 1.0 / dbu <<" ) ( "
               <<iroute->getTrack()->getTrackLoc() * 1.0 / dbu <<" "
               <<iroute->getEnd()                  * 1.0 / dbu <<" ) "
               <<layerName <<endl;
        }
        cout <<ss.str();
      }
    }
  }
}
*/

/*
void FlexTAWorker::reassignIroutes() {
  ;
}
*/

/*
void FlexTAWorker::saveToGuides() {
  bool isH = (getDir() == frcHorzPrefRoutingDir) ? true : false;
  auto defaultSegStyle = getTech()->getLayer(layerNum)->getDefaultSegStyle();
  frPoint begin, end;
  for (auto &iroute: iroutes) {
    if (iroute->getTrack()) {
      unique_ptr<frPathSeg> pathSeg = make_unique<frPathSeg>();
      if (isH) {
        begin.set(iroute->getBegin(), iroute->getTrack()->getTrackLoc());
        end.set(iroute->getEnd(), iroute->getTrack()->getTrackLoc());
      } else {
        begin.set(iroute->getTrack()->getTrackLoc(), iroute->getBegin());
        end.set(iroute->getTrack()->getTrackLoc(), iroute->getEnd());
      }
      pathSeg->setPoints(begin, end);
      pathSeg->setLayerNum(layerNum);
      pathSeg->addToNet(iroute->getGuide()->getNet());
      pathSeg->setStyle(defaultSegStyle);
      auto guide = iroute->getGuide();
      if (guide) {
        vector<unique_ptr<frConnFig> > tmp;
        tmp.push_back(std::move(pathSeg));
        guide->setRoutes(tmp);
      }
      // modify upper/lower segs
      // upper/lower seg will have longest wirelength
    }
  }
}
*/

/*
void FlexTAWorker::reportCosts() {
  bool isH = (getDir() == frcHorzPrefRoutingDir) ? true : false;
  frCoord begin, end;
  if (isH) {
    begin = routeBox.left();
    end   = routeBox.right();
  } else {
    begin = routeBox.bottom();
    end   = routeBox.top();
  }
  frCoord wlen = 0;
  for (auto &iroute: iroutes) {
    wlen += iroute->getEnd() - iroute->getBegin();
  }
  frUInt4 overlapCost = 0;
  for (auto &track: tracks) {
    overlapCost += track->reportOverlapCost(begin, end);
  }
  if (VERBOSE > 1) {
    stringstream ss;
    ss <<"wlen/ovlp = " <<wlen <<"/" <<overlapCost <<endl; 
    cout <<ss.str();
  }
}
*/


