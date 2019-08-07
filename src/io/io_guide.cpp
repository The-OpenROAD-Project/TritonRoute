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

//#include <chrono>
#include "io/io.h"

using namespace std;
using namespace fr;
/* note: M1 guide special treatment. search "no M1 cross-gcell routing allowed" */

void io::Parser::genGuides_merge(vector<frRect> &rects, vector<map<frCoord, boost::icl::interval_set<frCoord> > > &intvs) {
  //bool enableOutput = true;
  bool enableOutput = false;
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

  for (auto &rect: rects) {
    frBox box;
    rect.getBBox(box);
    //frCoord x1 = (box.lowerLeft().x()  - GCELLOFFSETX)     / GCELLGRIDX;
    //frCoord y1 = (box.lowerLeft().y()  - GCELLOFFSETY)     / GCELLGRIDY;
    frPoint idx;
    frPoint pt(box.lowerLeft());
    design->getTopBlock()->getGCellIdx(pt, idx);
    frCoord x1 = idx.x();
    frCoord y1 = idx.y();
    //frCoord x2 = (box.upperRight().x() - GCELLOFFSETX - 1) / GCELLGRIDX;
    //frCoord y2 = (box.upperRight().y() - GCELLOFFSETY - 1) / GCELLGRIDY;
    pt.set(box.right() - 1, box.top() - 1);
    design->getTopBlock()->getGCellIdx(pt, idx);
    frCoord x2 = idx.x();
    frCoord y2 = idx.y();
    if (enableOutput) {
      cout << " rect " << rect << " " << x1 << " " << y1 << " " << x2 << " " << y2 << "\n";
    }
    auto layerNum = rect.getLayerNum();
    //if (x1 > x2 || y1 > y2) {
    //  cout <<"Error: genGuides start > end" <<endl;
    //  exit(1);
    //}
    //if (x1 >= (int)GCELLCNTX) {
    //  x1 = (int)GCELLCNTX - 1;
    //}
    //if (y1 >= (int)GCELLCNTY) {
    //  y1 = (int)GCELLCNTY - 1;
    //}
    //if (x2 >= (int)GCELLCNTX) {
    //  x2 = (int)GCELLCNTX - 1;
    //}
    //if (y2 >= (int)GCELLCNTY) {
    //  y2 = (int)GCELLCNTY - 1;
    //}
    if (tech->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
      for (auto i = y1; i <= y2; i++) {
        intvs[layerNum][i].insert(boost::icl::interval<frCoord>::closed(x1, x2));
        if (enableOutput) {
          cout <<"push " <<layerNum <<" " <<i <<" " <<x1 <<" " <<x2 <<endl;
        }
      }
    } else {
      for (auto i = x1; i <= x2; i++) {
        intvs[layerNum][i].insert(boost::icl::interval<frCoord>::closed(y1, y2));
        if (enableOutput) {
          cout <<"push " <<layerNum <<" " <<i <<" " <<y1 <<" " <<y2 <<endl;
        }
      }
    }
  }
  // trackIdx, beginIdx, endIdx, layerNum
  vector<tuple<frCoord, frCoord, frCoord, frLayerNum> > touchGuides;
  // append touching edges
  for (int lNum = 0; lNum < (int)intvs.size(); lNum++) {
    auto &m = intvs[lNum];
    int prevTrackIdx = -2;
    for (auto &[trackIdx, intvS]: m) {
      if (trackIdx == prevTrackIdx + 1) {
        auto &prevIntvS = m[prevTrackIdx];
        auto newIntv = intvS & prevIntvS;
        for (auto it = newIntv.begin(); it != newIntv.end(); it++) {
          auto beginIdx = it->lower();
          auto endIdx   = it->upper();
          bool haveLU = false;
          // lower layer intersection
          if (lNum - 2 >= 0) {
            auto nbrLayerNum = lNum - 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx); 
                      it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx; it2++) {
              if (boost::icl::contains(it2->second, trackIdx) &&
                  boost::icl::contains(it2->second, prevTrackIdx)) {
                haveLU = true;
                break;
              }
            }
          }
          if (lNum + 2 < (int)intvs.size() && !haveLU) {
            auto nbrLayerNum = lNum + 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx); 
                      it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx; it2++) {
              if (boost::icl::contains(it2->second, trackIdx) &&
                  boost::icl::contains(it2->second, prevTrackIdx)) {
                haveLU = true;
                break;
              }
            }
          }
          if (!haveLU) {
            // add touching guide;
            //cout <<"found touching guide" <<endl;
            if (lNum + 2 < (int)intvs.size()) {
              touchGuides.push_back(make_tuple(beginIdx, prevTrackIdx, trackIdx, lNum + 2));
            } else if (lNum - 2 >= 0) {
              touchGuides.push_back(make_tuple(beginIdx, prevTrackIdx, trackIdx, lNum - 2));
            } else {
              cout <<"Error: genGuides_merge cannot find touching layer" <<endl;
              exit(1);
            }
          }
        }
      }
      prevTrackIdx = trackIdx;
    }
  }

  for (auto &[trackIdx, beginIdx, endIdx, lNum]: touchGuides) {
    intvs[lNum][trackIdx].insert(boost::icl::interval<frCoord>::closed(beginIdx, endIdx));
  }
  if (enableOutput) {
    if (touchGuides.size()) {
      cout <<"found " <<touchGuides.size() <<" touching guides" <<endl;
    }
  }

}

void io::Parser::genGuides_split(vector<frRect> &rects, vector<map<frCoord, boost::icl::interval_set<frCoord> > > &intvs,
                                 map<pair<frPoint, frLayerNum>, set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap,
                                 map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap,
                                 bool retry) {

  rects.clear();
  //layerNum->trackIdx->beginIdx->set of obj
  vector<map<frCoord, map<frCoord, set<frBlockObject*, frBlockObjectComp> > > > pin_helper(design->getTech()->getLayers().size());
  for (auto &[pr, objS]: gCell2PinMap) {
    auto &point = pr.first;
    auto &lNum  = pr.second;
    if (design->getTech()->getLayer(lNum)->getDir() == frcHorzPrefRoutingDir) {
      pin_helper[lNum][point.y()][point.x()] = objS;
    } else {
      pin_helper[lNum][point.x()][point.y()] = objS;
    }
  }

  for (int layerNum = 0; layerNum < (int)intvs.size(); layerNum++) {
    auto dir = design->getTech()->getLayer(layerNum)->getDir();
    for (auto &[trackIdx, curr_intvs]: intvs[layerNum]) {
      // split by lower/upper seg
      for (auto it = curr_intvs.begin(); it != curr_intvs.end(); it++) {
        set<frCoord> lineIdx;
        auto beginIdx = it->lower();
        auto endIdx   = it->upper();
        // hardcode layerNum == 0 not used for GR
        if (!retry && layerNum == 0) {
          // split by pin
          if (pin_helper[layerNum].find(trackIdx) != pin_helper[layerNum].end()) {
            auto &pin_helper_map = pin_helper[layerNum][trackIdx];
            for (auto it2 = pin_helper_map.lower_bound(beginIdx); 
                 it2 != pin_helper_map.end() && it2->first <= endIdx; it2++) {
              //add pin2GCellmap
              for (auto obj: it2->second) {
                if (dir == frcHorzPrefRoutingDir) {
                  pin2GCellMap[obj].insert(make_pair(frPoint(it2->first, trackIdx), layerNum));
                } else {
                  pin2GCellMap[obj].insert(make_pair(frPoint(trackIdx, it2->first), layerNum));
                }
              }
              //cout <<"pin split" <<endl;
            }
          }
          for (int x = beginIdx; x <= endIdx; x++) {
            frRect tmpRect;
            if (dir == frcHorzPrefRoutingDir) {
              tmpRect.setBBox(frBox(x, trackIdx, x, trackIdx));
            } else {
              tmpRect.setBBox(frBox(trackIdx, x, trackIdx, x));
            }
            tmpRect.setLayerNum(layerNum);
            rects.push_back(tmpRect);
          }
        } else {
          //lineIdx.insert(beginIdx);
          //lineIdx.insert(endIdx);
          // lower layer intersection
          if (layerNum - 2 >= 0) {
            auto nbrLayerNum = layerNum - 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx); 
                      it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx; it2++) {
              if (boost::icl::contains(it2->second, trackIdx)) {
                lineIdx.insert(it2->first); //it2->first is intersection frCoord
                //cout <<"found split point" <<endl;
              }
            }
          }
          if (layerNum + 2 < (int)intvs.size()) {
            auto nbrLayerNum = layerNum + 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx); 
                      it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx; it2++) {
              if (boost::icl::contains(it2->second, trackIdx)) {
                lineIdx.insert(it2->first);
                //cout <<"found split point" <<endl;
              }
            }
          }
          // split by pin
          if (pin_helper[layerNum].find(trackIdx) != pin_helper[layerNum].end()) {
            auto &pin_helper_map = pin_helper[layerNum][trackIdx];
            for (auto it2 = pin_helper_map.lower_bound(beginIdx); 
                 it2 != pin_helper_map.end() && it2->first <= endIdx; it2++) {
              lineIdx.insert(it2->first);
              //add pin2GCellmap
              for (auto obj: it2->second) {
                if (dir == frcHorzPrefRoutingDir) {
                  pin2GCellMap[obj].insert(make_pair(frPoint(it2->first, trackIdx), layerNum));
                } else {
                  pin2GCellMap[obj].insert(make_pair(frPoint(trackIdx, it2->first), layerNum));
                }
              }
              //cout <<"pin split" <<endl;
            }
          }
          // add rect
          if (lineIdx.empty()) {
            cout <<"Error: genGuides_split lineIdx is empty" <<endl;
            exit(1);
          } else if (lineIdx.size() == 1) {
            auto x = *(lineIdx.begin());
            frRect tmpRect;
            if (dir == frcHorzPrefRoutingDir) {
              tmpRect.setBBox(frBox(x, trackIdx, x, trackIdx));
            } else {
              tmpRect.setBBox(frBox(trackIdx, x, trackIdx, x));
            }
            tmpRect.setLayerNum(layerNum);
            rects.push_back(tmpRect);
          } else {
            auto prevIt = lineIdx.begin();
            for (auto currIt = (++(lineIdx.begin())); currIt != lineIdx.end(); currIt++) {
              frRect tmpRect;
              if (dir == frcHorzPrefRoutingDir) {
                tmpRect.setBBox(frBox(*prevIt, trackIdx, *currIt, trackIdx));
              } else {
                tmpRect.setBBox(frBox(trackIdx, *prevIt, trackIdx, *currIt));
              }
              tmpRect.setLayerNum(layerNum);
              prevIt = currIt;
              rects.push_back(tmpRect);
            }
          }
        }
      }
    }
  }
  rects.shrink_to_fit();
}


void io::Parser::genGuides_gCell2TermMap(map<pair<frPoint, frLayerNum>, set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap, 
                                         map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap, 
                                         frTerm* term, frBlockObject* origTerm) {
  bool enableOutput = false;
  //bool enableOutput = true;
  //auto &gp = design->getTopBlock()->getGCellPatterns();
  //auto &xgp = gp[0];
  //auto &ygp = gp[1];
  // frCoord to prevent negative problems
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  //frCoord GCELLOFFSETX = xgp.getStartCoord();
  //frCoord GCELLOFFSETY = ygp.getStartCoord();
  //frCoord GCELLCNTX    = xgp.getCount();
  //frCoord GCELLCNTY    = ygp.getCount();

  for (auto &uPin: term->getPins()) {
    for (auto &uFig: uPin->getFigs()) {
      auto fig = uFig.get();
      if (fig->typeId() == frcRect) {
        auto shape = static_cast<frRect*>(fig);
        auto lNum  = shape->getLayerNum();
        frBox box;
        shape->getBBox(box);
        if (enableOutput) {
          cout <<"  box " <<box <<endl;
        }
        //frCoord x1 = (box.lowerLeft().x() - 1  - GCELLOFFSETX) / GCELLGRIDX;
        //frCoord y1 = (box.lowerLeft().y() - 1  - GCELLOFFSETY) / GCELLGRIDY;
        frPoint idx;
        frPoint pt(box.left() - 1, box.bottom() - 1);
        design->getTopBlock()->getGCellIdx(pt, idx);
        frCoord x1 = idx.x();
        frCoord y1 = idx.y();
        //frCoord x2 = (box.upperRight().x()     - GCELLOFFSETX) / GCELLGRIDX;
        //frCoord y2 = (box.upperRight().y()     - GCELLOFFSETY) / GCELLGRIDY;
        pt.set(box.upperRight());
        design->getTopBlock()->getGCellIdx(pt, idx);
        frCoord x2 = idx.x();
        frCoord y2 = idx.y();
        // ispd18_test4 and ispd18_test5 have zero overlap guide
        // excludes double-zero overlap area on the upper-right corner due to initDR requirements
        bool condition2 = false; // upper right corner has zero-length overlapped with gcell
        frBox gcellBox;
        frPoint tmpIdx;
        design->getTopBlock()->getGCellIdx(box.lowerLeft(), tmpIdx);
        design->getTopBlock()->getGCellBox(tmpIdx, gcellBox);
        if (box.lowerLeft() == gcellBox.lowerLeft()) {
          condition2 = true;
        }
        //if ((box.lowerLeft().x() == (x1 + 1) * (int)GCELLGRIDX + (int)GCELLOFFSETX) &&
        //    (box.lowerLeft().y() == (y1 + 1) * (int)GCELLGRIDY + (int)GCELLOFFSETY)) {
        //  condition2 = true;
        //}
        //if (x1 < 0) {
        //  cout <<"Warning: genGuides x1 < 0, automatically set to 0" <<endl;
        //  //exit(1);
        //  x1 = 0;
        //}
        //if (y1 < 0) {
        //  cout <<"Warning: genGuides y1 < 0, automatically set to 0" <<endl;
        //  //exit(1);
        //  y1 = 0;
        //}
        //if (x1 > x2 || y1 > y2) {
        //  cout <<"Error: genGuides_gCell2TermMap start > end" <<endl;
        //  cout <<"  x1/x2, y1/y2 = " <<x1 <<" " <<x2 <<" " <<y1 <<" " <<y2 <<endl;
        //  exit(1);
        //}
        //if (x1 >= (int)GCELLCNTX) {
        //  x1 = GCELLCNTX - 1;
        //}
        //if (x2 >= (int)GCELLCNTX) {
        //  //cout <<"Warning: genGuides x2 >= GCELLCNTX, automatically set to GCELLCNTX" <<endl;
        //  //cout <<"  bbox " <<box <<" " <<lNum <<endl;
        //  //exit(1);
        //  x2 = GCELLCNTX - 1;
        //}
        //if (y1 >= (int)GCELLCNTY) {
        //  y1 = GCELLCNTY - 1;
        //}
        //if (y2 >= (int)GCELLCNTY) {
        //  //cout <<"Warning: genGuides y2 >= GCELLCNTY, automatically set to GCELLCNTY" <<endl;
        //  //cout <<"  bbox " <<box <<" " <<lNum <<endl;
        //  //exit(1);
        //  y2 = GCELLCNTY - 1;
        //}
        for (int x = x1; x <= x2; x++) {
          for (int y = y1; y <= y2; y++) {
            if (condition2 && x == tmpIdx.x() - 1 && y == tmpIdx.y() - 1) {
              if (VERBOSE > 0) {
                cout <<"Warning: genGuides_gCell2TermMap avoid condition2, may result in guide open" <<endl;
              }
            } else {
              gCell2PinMap[make_pair(frPoint(x, y), lNum)].insert(origTerm);
              if (enableOutput) {
                cout <<" (x,y,lNum) = (" <<x <<", " <<y <<", " <<lNum <<")" <<endl;
              }
            }
            //pin2GCellMap[origTerm].insert(make_pair(frPoint(x,y), lNum));
          }
        }
      } else {
        cout <<"Error: genGuides_gCell2TermMap unsupoprted pinfig" <<endl;
        exit(1);
      }
    }
  }
}

void io::Parser::genGuides_gCell2PinMap(frNet* net, map<pair<frPoint, frLayerNum>, set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap,
    map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap) {
  bool enableOutput = false;
  //bool enableOutput = true;
  for (auto &instTerm: net->getInstTerms()) {
    if (enableOutput) {
      cout <<"instTerm " <<instTerm->getInst()->getName() <<"/" <<instTerm->getTerm()->getName() <<endl;
    }
    //frTransform xform;
    //instTerm->getInst()->getTransform(xform);
    //frBox   mbox;
    //instTerm->getInst()->getRefBlock()->getBoundaryBBox(mbox);
    //frPoint size(mbox.right(), mbox.top());
    //xform.updateXform(size);
    frTransform xform;
    instTerm->getInst()->getUpdatedXform(xform);
    auto origTerm = instTerm->getTerm();
    auto uTerm = make_unique<frTerm>(*origTerm, xform);
    auto term = uTerm.get();
    genGuides_gCell2TermMap(gCell2PinMap, pin2GCellMap, term, instTerm);
    //if (pin2GCellMap.find(origTerm) == pin2GCellMap.end() || pin2GCellMap[origTerm].empty()) {
    //  cout <<"Error: gcell not cover " <<instTerm->getInst()->getName() <<"/" <<origTerm->getName() <<endl;
    //  exit(1);
    //}
  }
  for (auto &term: net->getTerms()) {
    if (enableOutput) {
      cout <<"    term PIN/" <<term->getName() <<endl;
    }
    genGuides_gCell2TermMap(gCell2PinMap, pin2GCellMap, term, term);
    //if (pin2GCellMap.find(term) == pin2GCellMap.end() || pin2GCellMap[term].empty()) {
    //  cout <<"Error: gcell not cover PIN/" <<term->getName() <<endl;
    //  exit(1);
    //}
  }
}

void io::Parser::genGuides_initPin2GCellMap(frNet* net, std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap) {
  for (auto &instTerm: net->getInstTerms()) {
    pin2GCellMap[instTerm];
  }
  for (auto &term: net->getTerms()) {
    pin2GCellMap[term];
  }
}

void io::Parser::genGuides(frNet *net, vector<frRect> &rects) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //cout <<"net " <<net->getName() <<endl <<flush;
  vector<map<frCoord, boost::icl::interval_set<frCoord> > > intvs(tech->getLayers().size());
  genGuides_merge(rects, intvs); // merge and add touching guide
  //cout <<"merge done" <<endl <<flush;
  
  // gcell to pin map
  map<pair<frPoint, frLayerNum>, set<frBlockObject*, frBlockObjectComp> > gCell2PinMap;
  map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> pin2GCellMap; 
  genGuides_gCell2PinMap(net, gCell2PinMap, pin2GCellMap);
  //cout <<"gcell2pin map done" <<endl <<flush;
  genGuides_initPin2GCellMap(net, pin2GCellMap);
  //cout <<"init pin2gcell done" <<endl <<flush;

  bool retry = false;
  while(1) {
    genGuides_split(rects, intvs, gCell2PinMap, pin2GCellMap, retry); //split on LU intersecting guides and pins
    //cout <<"split done" <<endl <<flush;

    // filter pin2GCellMap with aps
    
    if (enableOutput) {
      for (auto &[gcell, objS]: gCell2PinMap) {
        cout <<"gcell (" <<gcell.first.x() <<", " <<gcell.first.y() <<") " <<gcell.second <<endl;
        for (auto obj: objS) {
          if (obj->typeId() == frcInstTerm) {
            auto ptr = static_cast<frInstTerm*>(obj);
            cout <<"  pin " <<ptr->getInst()->getName() <<"/" <<ptr->getTerm()->getName() <<endl; 
          } else if (obj->typeId() == frcTerm) {
            auto ptr = static_cast<frTerm*>(obj);
            cout <<"  pin PIN/" <<ptr->getName() <<endl; 
          } else {
            cout <<"Error: genGuides unknown type" <<endl; 
          }
        }
      }
      for (auto &[obj, locS]: pin2GCellMap) {
        if (obj->typeId() == frcInstTerm) {
          auto ptr = static_cast<frInstTerm*>(obj);
          cout <<"pin " <<ptr->getInst()->getName() <<"/" <<ptr->getTerm()->getName() <<endl; 
        } else if (obj->typeId() == frcTerm) {
          auto ptr = static_cast<frTerm*>(obj);
          cout <<"pin PIN/" <<ptr->getName() <<endl; 
        } else {
          cout <<"Error: genGuides unknown type" <<endl; 
        }
        for (auto &[pt, lNum]: locS) {
          cout <<"  gcell (" <<pt.x() <<", " <<pt.y() <<") " <<lNum <<endl;
        }
      }
      int i = 0;
      for (auto &rect: rects) {
        frBox box;
        rect.getBBox(box);
        cout <<"guide ms " <<i <<" " <<box <<" " <<design->getTech()->getLayer(rect.getLayerNum())->getName() <<endl;
        i++;
      }
    }
    

    if (pin2GCellMap.empty()) {
      cout <<"Error: genGuides empty pin2GCellMap" <<endl; 
      cout <<" gcell2pin.size() = " <<gCell2PinMap.size() <<endl;
    }
    for (auto &[obj, locS]: pin2GCellMap) {
      if (locS.empty()) {
        if (obj->typeId() == frcInstTerm) {
          auto ptr = static_cast<frInstTerm*>(obj);
          cout <<"Error: pin " <<ptr->getInst()->getName() <<"/" <<ptr->getTerm()->getName() <<" not covered by guide" <<endl; 
        } else if (obj->typeId() == frcTerm) {
          auto ptr = static_cast<frTerm*>(obj);
          cout <<"Error: pin PIN/" <<ptr->getName() <<" not covered by guide" <<endl; 
        } else {
          cout <<"Error: genGuides unknown type" <<endl; 
        }
      }
    }

    // steiner (i.e., gcell end and pin gcell idx) to guide idx (pin idx)
    map<pair<frPoint, frLayerNum>, set<int> > nodeMap;
    int gCnt = 0;
    int nCnt = 0;
    genGuides_buildNodeMap(nodeMap, gCnt, nCnt, rects, pin2GCellMap);
    //cout <<"build node map done" <<endl <<flush;

    vector<bool> adjVisited;
    vector<int>  adjPrevIdx;
    if (genGuides_astar(net, adjVisited, adjPrevIdx, nodeMap, gCnt, nCnt, false, retry)) {
      //cout <<"astar done" <<endl <<flush;
      genGuides_final(net, rects, adjVisited, adjPrevIdx, gCnt, nCnt, pin2GCellMap);
      break;
    } else {
      if (retry) {
        if (!ALLOW_PIN_AS_FEEDTHROUGH) {
          if (genGuides_astar(net, adjVisited, adjPrevIdx, nodeMap, gCnt, nCnt, true, retry)) {
            genGuides_final(net, rects, adjVisited, adjPrevIdx, gCnt, nCnt, pin2GCellMap);
            break;
          } else {
            cout <<"Error: critical error guide not connected, exit now 1!" <<endl;
            exit(1);
          }
        } else {
          cout <<"Error: critical error guide not connected, exit now 2!" <<endl;
          exit(1);
        }
      } else {
        retry = true;
      }
    }
    //build guide
    //map<int, int> pinGuideResidency;
    //genGuides_final(net, rects, adjVisited, adjPrevIdx, gCnt, nCnt, pin2GCellMap);
    //cout <<"final done" <<endl <<flush;
    //break;
  }
}

void io::Parser::genGuides_final(frNet *net, vector<frRect> &rects, vector<bool> &adjVisited, vector<int> &adjPrevIdx, int gCnt, int nCnt,
                                 map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap) {
  //bool enableOutput = true;
  bool enableOutput = false;
  vector<frBlockObject*> pin2ptr;
  for (auto &[obj, idxS]: pin2GCellMap) {
    pin2ptr.push_back(obj);
  }
  // find pin in which guide
  //vector<vector<int> > pinIdx2Guides(nCnt - gCnt);
  vector<vector<pair<frPoint, frLayerNum> > > pinIdx2GCellUpdated(nCnt - gCnt);
  vector<vector<int> > guideIdx2Pins(gCnt);
  for (int i = 0; i < (int)adjPrevIdx.size(); i++) {
    if (!adjVisited[i]) {
      continue;
    }
    if (i < gCnt && adjPrevIdx[i] >= gCnt) {
      // pin idx = adjPrevIdx[i] - gCnt
      // guide idx = i
      //pinInGuides[adjPrevIdx[i] - gCnt].push_back(i);
      auto pinIdx = adjPrevIdx[i] - gCnt;
      auto guideIdx = i;
      frBox box;
      auto &rect = rects[guideIdx];
      rect.getBBox(box);
      auto lNum = rect.getLayerNum();
      auto obj = pin2ptr[pinIdx];
      //cout <<" pin1 id " <<adjPrevIdx[i] <<" prev " <<i <<endl;
      if (pin2GCellMap[obj].find(make_pair(box.lowerLeft(), lNum)) != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(make_pair(box.lowerLeft(), lNum));
      } else if (pin2GCellMap[obj].find(make_pair(box.upperRight(), lNum)) != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(make_pair(box.upperRight(), lNum));
      } else {
        cout <<"Error: genGuides_final error 1" <<endl;
      }
      guideIdx2Pins[guideIdx].push_back(pinIdx);
    } else if (i >= gCnt && adjPrevIdx[i] >= 0 && adjPrevIdx[i] < gCnt) {
      // pin idx = i
      // guide idx = adjPrevIdx[i]
      //pinInGuides[i - gCnt].push_back(adjPrevIdx[i]);
      auto pinIdx = i - gCnt;
      auto guideIdx = adjPrevIdx[i];
      frBox box;
      auto &rect = rects[guideIdx];
      rect.getBBox(box);
      auto lNum = rect.getLayerNum();
      auto obj = pin2ptr[pinIdx];
      //cout <<" pin2 id " <<i <<" prev " <<adjPrevIdx[i] <<endl;
      if (pin2GCellMap[obj].find(make_pair(box.lowerLeft(), lNum)) != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(make_pair(box.lowerLeft(), lNum));
      } else if (pin2GCellMap[obj].find(make_pair(box.upperRight(), lNum)) != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(make_pair(box.upperRight(), lNum));
      } else {
        cout <<"Error: genGuides_final error 2" <<endl;
      }
      guideIdx2Pins[guideIdx].push_back(pinIdx);
    }
  }
  for (auto &guides: pinIdx2GCellUpdated) {
    if (guides.empty()) {
      cout <<"Error: genGuides_final pin not in any guide" <<endl;
    }
  }
  
  //auto &gp = design->getTopBlock()->getGCellPatterns();
  //auto &xgp = gp[0];
  //auto &ygp = gp[1];
  //// use frCoord to prevent negative problems
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  //frCoord GCELLOFFSETX = xgp.getStartCoord();
  //frCoord GCELLOFFSETY = ygp.getStartCoord();
  //frCoord GCELLCNTX    = xgp.getCount();
  //frCoord GCELLCNTY    = ygp.getCount();
   
  map<pair<frPoint, frLayerNum>, set<int> > updatedNodeMap;
  // pinIdx2GCellUpdated tells pin residency in gcell
  for (int i = 0; i < nCnt - gCnt; i++) {
    auto obj = pin2ptr[i];
    for (auto &[pt, lNum]: pinIdx2GCellUpdated[i]) {
      //frPoint absPt(pt.x() * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2,
      //              pt.y() * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2);
      frPoint absPt;
      design->getTopBlock()->getGCellCenter(pt, absPt);
      tmpGRPins.push_back(make_pair(obj, absPt));
      updatedNodeMap[make_pair(pt, lNum)].insert(i + gCnt);
      if (enableOutput) {
        cout <<"pin   final " <<i + gCnt <<" " <<pt <<" " <<design->getTech()->getLayer(lNum)->getName() <<endl;
      }
    }
  }
  for (int i = 0; i < gCnt; i++) {
    if (!adjVisited[i]) {
      continue;
    }
    auto &rect = rects[i];
    frBox box;
    rect.getBBox(box);
    updatedNodeMap[make_pair(frPoint(box.left(),  box.bottom()), rect.getLayerNum())].insert(i);
    updatedNodeMap[make_pair(frPoint(box.right(), box.top()),    rect.getLayerNum())].insert(i);
    //cout <<"add guide " <<i <<" to " <<frPoint(box.left(),  box.bottom()) <<" " <<rect.getLayerNum() <<endl;
    //cout <<"add guide " <<i <<" to " <<frPoint(box.right(), box.top())    <<" " <<rect.getLayerNum() <<endl;
  }
  for (auto &[pr, idxS]: updatedNodeMap) {
    auto &[pt, lNum] = pr;
    if ((int)idxS.size() == 1) {
      auto idx = *(idxS.begin());
      if (idx < gCnt) {
        // no upper/lower guide
        if (updatedNodeMap.find(make_pair(pt, lNum + 2)) == updatedNodeMap.end() &&
            updatedNodeMap.find(make_pair(pt, lNum - 2)) == updatedNodeMap.end()) {
          auto &rect = rects[idx];
          frBox box;
          rect.getBBox(box);
          if (box.lowerLeft() == pt) {
            rect.setBBox(frBox(box.right(), box.top(), box.right(), box.top()));
          } else {
            rect.setBBox(frBox(box.left(), box.bottom(), box.left(), box.bottom()));
          }
        }
      } else {
        cout <<"Error: pin dangling id " <<idx <<" " <<pt <<" " <<lNum <<endl;
        exit(1);
      }
    }
  }
  // guideIdx2Pins enables fiding from guide to pin
  // adjVisited tells guide to write back
  for (int i = 0; i < gCnt; i++) {
    if (!adjVisited[i]) {
      continue;
    }
    auto &rect = rects[i];
    frBox box;
    rect.getBBox(box);
    if (enableOutput) {
      cout <<"guide final " <<i <<" " <<box <<" " <<design->getTech()->getLayer(rect.getLayerNum())->getName() <<endl;
    }
    auto guide = make_unique<frGuide>();
    frPoint begin, end;
    //begin.set(box.left()   * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2, 
    //          box.bottom() * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2);
    design->getTopBlock()->getGCellCenter(box.lowerLeft(), begin);
    //end.set(box.right() * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2, 
    //        box.top()   * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2);
    design->getTopBlock()->getGCellCenter(box.upperRight(), end);
    guide->setPoints(begin, end);
    guide->setBeginLayerNum(rect.getLayerNum());
    guide->setEndLayerNum(rect.getLayerNum());
    guide->addToNet(net);
    net->addGuide(guide);
    //cout <<"add guide " <<begin <<" " <<end <<" " <<rect.getLayerNum() <<endl;
  }
}

void io::Parser::genGuides_buildNodeMap(map<pair<frPoint, frLayerNum>, set<int> > &nodeMap, int &gCnt, int &nCnt,
                                        vector<frRect> &rects, map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap) {
  bool enableOutput = false;
  // guideIdx
  //map<pair<frPoint, frLayerNum>, set<int> > nodeMap;
  for (int i = 0; i < (int)rects.size(); i++) {
    auto &rect = rects[i];
    frBox box;
    rect.getBBox(box);
    nodeMap[make_pair(box.lowerLeft(), rect.getLayerNum())].insert(i);
    nodeMap[make_pair(box.upperRight(), rect.getLayerNum())].insert(i);
  }
  gCnt = rects.size(); // total guide cnt
  int nodeIdx = rects.size();
  for (auto &[obj, locS]: pin2GCellMap) {
    if (enableOutput) {
      if (obj->typeId() == frcInstTerm) {
        auto instTerm = static_cast<frInstTerm*>(obj);
        cout <<"instTerm " <<nodeIdx <<" " <<instTerm->getInst()->getName() <<"/" <<instTerm->getTerm()->getName();
      } else if (obj->typeId() == frcTerm) {
        auto term = static_cast<frTerm*>(obj);
        cout <<"term " <<nodeIdx <<" PIN/" <<term->getName();
      }
    }
    for (auto &loc: locS) {
      nodeMap[loc].insert(nodeIdx);
      if (enableOutput) {
        cout <<" " <<loc.first <<" " <<design->getTech()->getLayer(loc.second)->getName();
      }
    }
    if (enableOutput) {
      cout <<endl;
    }
    nodeIdx++;
  }
  nCnt = nodeIdx; // total node cnt
}



bool io::Parser::genGuides_astar(frNet* net, vector<bool> &adjVisited, vector<int> &adjPrevIdx, 
                                 map<pair<frPoint, frLayerNum>, set<int> > &nodeMap, int &gCnt, int &nCnt,
                                 bool forceFeedThrough, bool retry) {
  //bool enableOutput = true;
  bool enableOutput = false;
  // a star search

  // node index, node visited
  vector<vector<int> > adjVec(nCnt, vector<int>());
  vector<bool> onPathIdx(nCnt, false);
  adjVisited.clear();
  adjPrevIdx.clear();
  adjVisited.resize(nCnt, false);
  adjPrevIdx.resize(nCnt, -1);
  //vector<bool> adjVisited(nCnt, false);
  //vector<int>  adjPrevIdx(nCnt, -1);
  for (auto &[pr, idxS]: nodeMap) {
    auto &[pt, lNum] = pr;
    for (auto it1 = idxS.begin(); it1 != idxS.end(); it1++) {
      auto it2 = it1;
      it2++;
      auto idx1 = *it1;
      for (; it2 != idxS.end(); it2++) {
        auto idx2 = *it2;
        // two pins, no edge
        if (idx1 >= gCnt && idx2 >= gCnt) {
          continue;
        // two gcells, has edge
        } else if (idx1 < gCnt && idx2 < gCnt) {
          // no M1 cross-gcell routing allowed
          if (lNum != 0) {
            adjVec[idx1].push_back(idx2);
            adjVec[idx2].push_back(idx1);
          }
          //cout <<"add edge1 " <<idx1 <<" " <<idx2 <<endl;
        // one pin, one gcell
        } else {
          auto gIdx = min(idx1, idx2);
          auto pIdx = max(idx1, idx2);
          // only out edge
          if (ALLOW_PIN_AS_FEEDTHROUGH || forceFeedThrough) {
            adjVec[pIdx].push_back(gIdx);
            adjVec[gIdx].push_back(pIdx);
          } else {
            if (pIdx == gCnt) {
              adjVec[pIdx].push_back(gIdx);
              //cout <<"add edge2 " <<pIdx <<" " <<gIdx <<endl;
            // only in edge
            } else {
              adjVec[gIdx].push_back(pIdx);
              //cout <<"add edge3 " <<gIdx <<" " <<pIdx <<endl;
            }
          }
        }
      }
      // add intersecting guide2guide edge excludes pin
      if (idx1 < gCnt && nodeMap.find(make_pair(pt, lNum + 2)) != nodeMap.end()) {
        for (auto nbrIdx: nodeMap[make_pair(pt, lNum + 2)]) {
          if (nbrIdx < gCnt) {
            adjVec[idx1].push_back(nbrIdx);
            adjVec[nbrIdx].push_back(idx1);
            //cout <<"add edge4 " <<idx1 <<" " <<nbrIdx <<endl;
            //cout <<"add edge5 " <<nbrIdx <<" " <<idx1 <<endl;
          }
        }
      }
    }
  }

  struct wf {
    int nodeIdx;
    int prevIdx;
    int cost;
    bool operator<(const wf &b) const {
      if (cost == b.cost) {
        return nodeIdx > b.nodeIdx;
      } else {
        return cost > b.cost;
      }
    }
  };
  for (int findNode = gCnt; findNode < nCnt - 1; findNode++) {
    //adjVisited = onPathIdx;
    //cout <<"finished " <<findNode <<" nodes" <<endl;
    priority_queue<wf> pq;
    if (enableOutput) {
      //cout <<"visit";
    }
    if (findNode == gCnt) {
      // push only first pin into pq
      pq.push({gCnt, -1, 0});
    } else {
      // push every visited node into pq
      for (int i = 0; i < nCnt; i++) {
        //if (adjVisited[i]) {
        if (onPathIdx[i]) {
          // penalize feedthrough in normal mode
          if (ALLOW_PIN_AS_FEEDTHROUGH && i >= gCnt) {
            pq.push({i, adjPrevIdx[i], 2});
          // penalize feedthrough in fallback mode
          } else if (forceFeedThrough && i >= gCnt) {
            pq.push({i, adjPrevIdx[i], 10});
          } else {
            pq.push({i, adjPrevIdx[i], 0});
          }
        }
      }
    }
    int lastNodeIdx = -1;
    while (!pq.empty()) {
      auto wfront = pq.top();
      pq.pop();
      if (!onPathIdx[wfront.nodeIdx] && adjVisited[wfront.nodeIdx]) {
        continue;
      }
      if (wfront.nodeIdx > gCnt && wfront.nodeIdx < nCnt && adjVisited[wfront.nodeIdx] == false) {
        adjVisited[wfront.nodeIdx] = true;
        adjPrevIdx[wfront.nodeIdx] = wfront.prevIdx;
        if (enableOutput) {
          //cout <<" " <<wfront.nodeIdx <<" (" <<wfront.cost <<"," <<wfront.prevIdx <<")" <<" exit" <<endl;
          cout <<"visit " <<wfront.nodeIdx <<" (" <<wfront.cost <<"," <<wfront.prevIdx <<")" <<" exit" <<endl;
        }
        lastNodeIdx = wfront.nodeIdx;
        break;
      }
      adjVisited[wfront.nodeIdx] = true;
      adjPrevIdx[wfront.nodeIdx] = wfront.prevIdx;
      if (enableOutput) {
        //cout <<" " <<wfront.nodeIdx <<" (" <<wfront.cost <<"," <<wfront.prevIdx <<")";
        cout <<"visit " <<wfront.nodeIdx <<" (" <<wfront.cost <<"," <<wfront.prevIdx <<")" <<endl;
      }
      // visit other nodes
      for (auto nbrIdx: adjVec[wfront.nodeIdx]) {
        if (!adjVisited[nbrIdx]) {
          pq.push({nbrIdx, wfront.nodeIdx, wfront.cost + 1});
          if (enableOutput) {
            cout <<"push " <<nbrIdx <<endl;
          }
        }
      }
    }
    // trace back path
    if (enableOutput) {
      cout <<"trace back id";
    }
    while ((lastNodeIdx != -1) && (!onPathIdx[lastNodeIdx])) {
      onPathIdx[lastNodeIdx] = true;
      if (enableOutput) {
        cout <<" " <<lastNodeIdx <<" (" <<adjPrevIdx[lastNodeIdx] <<")";
      }
      lastNodeIdx = adjPrevIdx[lastNodeIdx];
    }
    if (enableOutput) {
      cout <<endl;
    }
    adjVisited = onPathIdx;
  }
  // skip one-pin net
  if (nCnt == gCnt + 1) {
    return true;
  }
  if (enableOutput) {
  cout <<"stat: " <<net->getName() <<" #guide/#pin/#unused = " <<gCnt <<"/" <<nCnt - gCnt <<"/" 
       <<nCnt - count(adjVisited.begin(), adjVisited.end(), true) <<endl;
  }
  int pinVisited = count(adjVisited.begin() + gCnt, adjVisited.end(), true);
  // true error when allowing feedthrough
  if (pinVisited != nCnt - gCnt && (ALLOW_PIN_AS_FEEDTHROUGH || forceFeedThrough) && retry) {
    cout <<"Error: " <<net->getName() <<" " <<nCnt - gCnt - pinVisited <<" pin not visited #guides = " <<gCnt <<endl;
    if (enableOutput) {
      for (int i = gCnt; i < nCnt; i++) {
        if (!adjVisited[i]) {
          cout <<"  pin id = " <<i <<endl;
        }
      }
    }
  }
  // fallback to feedthrough in next iter
  if (pinVisited != nCnt - gCnt && !ALLOW_PIN_AS_FEEDTHROUGH && !forceFeedThrough && retry) {
    cout <<"Warning: " <<net->getName() <<" " <<nCnt - gCnt - pinVisited <<" pin not visited, fall back to feedrough mode" <<endl;
    //if (enableOutput) {
    //  for (int i = gCnt; i < nCnt; i++) {
    //    if (!adjVisited[i]) {
    //      cout <<"  pin id = " <<i <<endl;
    //    }
    //  }
    //}
  }
  // fallback to feedthrough in next iter
  //if (pinVisited != nCnt - gCnt && !retry) {
  //  cout <<"Warning: " <<net->getName() <<" " <<nCnt - gCnt - pinVisited <<" pin not visited, fall back to retry mode" <<endl;
  //  //if (enableOutput) {
  //  //  for (int i = gCnt; i < nCnt; i++) {
  //  //    if (!adjVisited[i]) {
  //  //      cout <<"  pin id = " <<i <<endl;
  //  //    }
  //  //  }
  //  //}
  //}
  if (pinVisited == nCnt - gCnt) {
    return true;
  } else {
    return false;
  }
}
