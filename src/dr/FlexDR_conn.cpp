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

#include "dr/FlexDR.h"
#include "io/io.h"

using namespace std;
using namespace fr;

// copied from FlexDRWorker::initNets_searchRepair_pin2epMap_helper
void FlexDR::checkConnectivity_pin2epMap_helper(frNet *net, const frPoint &bp, frLayerNum lNum, 
                                                map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap) {
  bool enableOutput = false;
  //bool enableOutput = true;
  auto regionQuery = getRegionQuery();
  vector<rq_rptr_value_t<frBlockObject> > result;
  //result.clear();
  regionQuery->query(frBox(bp, bp), lNum, result);
  for (auto &[bx, rqObj]: result) {
    if (rqObj->typeId() == frcInstTerm) {
      auto instTerm = static_cast<frInstTerm*>(rqObj);
      if (instTerm->getNet() == net) {
        if (enableOutput) {
          cout <<"    found " <<instTerm->getInst()->getName() <<"/" <<instTerm->getTerm()->getName() <<endl;
        }
        pin2epMap[rqObj].insert(make_pair(bp, lNum));
      } else {
        //if (enableOutput) {
        //  cout <<"    found other instTerm" <<endl;
        //}
      }
    } else if (rqObj->typeId() == frcTerm) {
      auto term = static_cast<frTerm*>(rqObj);
      if (term->getNet() == net) {
        if (enableOutput) {
          cout <<"    found PIN/" <<term->getName() <<endl;
        }
        pin2epMap[rqObj].insert(make_pair(bp, lNum));
      } else {
        //if (enableOutput) {
        //  cout <<"    found other term" <<endl;
        //}
      }
    }
  }
}

void FlexDR::checkConnectivity_pin2epMap(frNet* net, vector<frConnFig*> &netDRObjs,
                                         map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap) {
  bool enableOutput = false;
  //bool enableOutput = true;
  frPoint bp, ep;
  for (auto &connFig: netDRObjs) {
    if (connFig->typeId() == frcPathSeg) {
      auto obj = static_cast<frPathSeg*>(connFig);
      obj->getPoints(bp, ep);
      auto lNum = obj->getLayerNum();
      if (enableOutput) {
       cout <<"(bp, ep) (" <<bp.x() / 2000.0 <<" ," <<bp.y() / 2000.0 <<") ("
                           <<ep.x() / 2000.0 <<" ," <<ep.y() / 2000.0 <<") " 
            <<getTech()->getLayer(lNum)->getName() <<endl;
      }
      if (enableOutput) {
        cout <<"  query bp" <<endl;
      }
      checkConnectivity_pin2epMap_helper(net, bp, lNum, pin2epMap);
      if (enableOutput) {
        cout <<"  query ep" <<endl;
      }
      checkConnectivity_pin2epMap_helper(net, ep, lNum, pin2epMap);
    } else if (connFig->typeId() == frcVia) {
      auto obj = static_cast<frVia*>(connFig);
      obj->getOrigin(bp);
      auto l1Num = obj->getViaDef()->getLayer1Num();
      auto l2Num = obj->getViaDef()->getLayer2Num();
      if (enableOutput) {
        cout <<"pt (" <<bp.x() / 2000.0 <<" ," <<bp.y() / 2000.0 <<") "
             <<obj->getViaDef()->getName() <<endl;
      }
      if (enableOutput) {
        cout <<"  query pt l1" <<endl;
      }
      checkConnectivity_pin2epMap_helper(net, bp, l1Num, pin2epMap);
      if (enableOutput) {
        cout <<"  query pt l2" <<endl;
      }
      checkConnectivity_pin2epMap_helper(net, bp, l2Num, pin2epMap);
    //} else if (connFig->typeId() == frcPatchWire) {
    //  ;
    } else {
      cout <<"Error: checkConnectivity_pin2epMap unsupported type" <<endl;
    }
  }

}

void FlexDR::checkConnectivity_initDRObjs(frNet* net, vector<frConnFig*> &netDRObjs) {
  bool enableOutput = false;
  //bool enableOutput = true;
  for (auto &uPtr: net->getShapes()) {
    auto connFig = uPtr.get();
    if (connFig->typeId() == frcPathSeg) {
      netDRObjs.push_back(connFig);
      if (enableOutput) {
        auto obj = static_cast<frPathSeg*>(connFig);
        frPoint bp, ep;
        auto lNum = obj->getLayerNum();
        obj->getPoints(bp, ep);
        cout <<" (" <<bp.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
                    <<bp.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") ("
                    <<ep.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
                    <<ep.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") "
             <<getTech()->getLayer(lNum)->getName() <<endl;
      }
    } else {
      cout <<"Error: checkConnectivity_initDRObjs unsupported type" <<endl;
    }
  }
  for (auto &uPtr: net->getVias()) {
    auto connFig = uPtr.get();
    if (connFig->typeId() == frcVia) {
      netDRObjs.push_back(connFig);
      if (enableOutput) {
        auto obj = static_cast<frVia*>(connFig);
        frPoint bp;
        obj->getOrigin(bp);
        cout <<" (" <<bp.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
                    <<bp.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") "
             <<obj->getViaDef()->getName() <<endl;
      }
    } else {
      cout <<"Error: checkConnectivity_initDRObjs unsupported type" <<endl;
    }
  }
  //for (auto &uPtr: net->getPatchWires()) {
  //  auto connFig = uPtr.get();
  //  if (connFig->typeId() == frcPatchWire) {
  //    netDRObjs.push_back(connFig);
  //    if (enableOutput) {
  //      auto obj = static_cast<frPatchWire*>(connFig);
  //      frPoint bp;
  //      auto lNum = obj->getLayerNum();
  //      obj->getOrigin(bp);
  //      frBox box;
  //      obj->getOffsetBox(box);
  //      cout <<" (" <<bp.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", "
  //                  <<bp.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") RECT ("
  //                  <<box.left()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
  //                  <<box.bottom() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
  //                  <<box.right()  * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
  //                  <<box.top()    * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") "
  //           <<getTech()->getLayer(lNum)->getName() <<endl;
  //    }
  //  } else {
  //    cout <<"Error: checkConnectivity_initDRObjs unsupported type" <<endl;
  //  }
  //}
}

void FlexDR::checkConnectivity_nodeMap_routeObjEnd(frNet* net, vector<frConnFig*> &netRouteObjs,
                                                   map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  bool enableOutput = false;
  frPoint bp, ep;
  for (int i = 0; i < (int)netRouteObjs.size(); i++) {
    auto &connFig = netRouteObjs[i];
    if (connFig->typeId() == frcPathSeg) {
      auto obj = static_cast<frPathSeg*>(connFig);
      obj->getPoints(bp, ep);
      auto lNum = obj->getLayerNum();
      nodeMap[make_pair(bp, lNum)].insert(i);
      nodeMap[make_pair(ep, lNum)].insert(i);
      if (enableOutput) {
        cout <<"node idx = " <<i <<", (" <<bp.x() / 2000.0 <<", " <<bp.y() / 2000.0 <<") (" 
                                         <<ep.x() / 2000.0 <<", " <<ep.y() / 2000.0 <<") " 
             <<getTech()->getLayer(lNum)->getName() <<endl;
      }
    } else if (connFig->typeId() == frcVia) {
      auto obj = static_cast<frVia*>(connFig);
      obj->getOrigin(bp);
      auto l1Num = obj->getViaDef()->getLayer1Num();
      auto l2Num = obj->getViaDef()->getLayer2Num();
      nodeMap[make_pair(bp, l1Num)].insert(i);
      nodeMap[make_pair(bp, l2Num)].insert(i);
      if (enableOutput) {
        cout <<"node idx = " <<i <<", (" <<bp.x() / 2000.0 <<", " <<bp.y() / 2000.0 <<") "
             <<getTech()->getLayer(l1Num)->getName() <<" --> " <<getTech()->getLayer(l2Num)->getName() <<endl;
      }
    //} else if (connFig->typeId() == frcPatchWire) {
    //  auto obj = static_cast<frPatchWire*>(connFig);
    //  obj->getOrigin(bp);
    //  auto lNum = obj->getLayerNum();
    //  nodeMap[make_pair(bp, lNum)].insert(i);
    //  if (enableOutput) {
    //    cout <<"node idx = " <<i <<", (" <<bp.x() / 2000.0 <<", " <<bp.y() / 2000.0 <<") RECT " 
    //         <<getTech()->getLayer(lNum)->getName() <<endl;
    //  }
    } else {
      cout <<"Error: checkConnectivity_nodeMap_routeObjEnd unsupported type" <<endl;
    }
  }
}

void FlexDR::checkConnectivity_nodeMap_routeObjSplit_helper(const frPoint &crossPt, 
             frCoord trackCoord, frCoord splitCoord, frLayerNum lNum, 
             vector<map<frCoord, map<frCoord, pair<frCoord, int> > > > &mergeHelper,
             map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  auto it1 = mergeHelper[lNum].find(trackCoord);
  if (it1 != mergeHelper[lNum].end()) {
    auto &mp = it1->second; // map<ep, pair<bp, objIdx> >
    auto it2 = mp.lower_bound(splitCoord);
    if (it2 != mp.end()) {
      auto &endP = it2->first;
      auto &[beginP, objIdx] = it2->second;
      if (endP > splitCoord && beginP < splitCoord) {
        nodeMap[make_pair(crossPt, lNum)].insert(objIdx);
      }
    }
  }
}

void FlexDR::checkConnectivity_nodeMap_routeObjSplit(frNet* net, vector<frConnFig*> &netRouteObjs,
                                                     map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  frPoint bp, ep;
  // vector<map<track, map<ep, pair<bp, objIdx> > > > interval_map
  vector<map<frCoord, map<frCoord, pair<frCoord, int> > > > horzMergeHelper(getTech()->getLayers().size());
  vector<map<frCoord, map<frCoord, pair<frCoord, int> > > > vertMergeHelper(getTech()->getLayers().size());
  for (int i = 0; i < (int)netRouteObjs.size(); i++) {
    auto &connFig = netRouteObjs[i];
    if (connFig->typeId() == frcPathSeg) {
      auto obj = static_cast<frPathSeg*>(connFig);
      obj->getPoints(bp, ep);
      auto lNum = obj->getLayerNum();
      // vert seg
      if (bp.x() == ep.x()) {
        vertMergeHelper[lNum][bp.x()][ep.y()] = make_pair(bp.y(), i);
      // horz seg
      } else {
        horzMergeHelper[lNum][bp.y()][ep.x()] = make_pair(bp.x(), i);
      }
    }
  }
  for (int i = 0; i < (int)netRouteObjs.size(); i++) {
    auto &connFig = netRouteObjs[i];
    // ep on pathseg
    if (connFig->typeId() == frcPathSeg) {
      auto obj = static_cast<frPathSeg*>(connFig);
      obj->getPoints(bp, ep);
      auto lNum = obj->getLayerNum();
      // vert seg, find horz crossing seg
      if (bp.x() == ep.x()) {
        //find whether there is horz track at bp
        auto crossPt    = bp;
        auto trackCoord = bp.y();
        auto splitCoord = bp.x();
        checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, horzMergeHelper, nodeMap);
        //find whether there is horz track at ep
        crossPt    = ep;
        trackCoord = ep.y();
        splitCoord = ep.x();
        checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, horzMergeHelper, nodeMap);
      // horz seg
      } else {
        //find whether there is vert track at bp
        auto crossPt    = bp;
        auto trackCoord = bp.x();
        auto splitCoord = bp.y();
        checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, vertMergeHelper, nodeMap);
        //find whether there is vert track at ep
        crossPt    = ep;
        trackCoord = ep.x();
        splitCoord = ep.y();
        checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, vertMergeHelper, nodeMap);
      }
    } else if (connFig->typeId() == frcVia) {
      auto obj = static_cast<frVia*>(connFig);
      obj->getOrigin(bp);
      auto lNum = obj->getViaDef()->getLayer1Num();
      //find whether there is horz track at bp on layer1
      auto crossPt    = bp;
      auto trackCoord = bp.y();
      auto splitCoord = bp.x();
      checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, horzMergeHelper, nodeMap);
      //find whether there is vert track at bp on layer1
      crossPt    = bp;
      trackCoord = bp.x();
      splitCoord = bp.y();
      checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, vertMergeHelper, nodeMap);
      
      lNum = obj->getViaDef()->getLayer2Num();
      //find whether there is horz track at bp on layer2
      crossPt    = bp;
      trackCoord = bp.y();
      splitCoord = bp.x();
      checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, horzMergeHelper, nodeMap);
      //find whether there is vert track at bp on layer2
      crossPt    = bp;
      trackCoord = bp.x();
      splitCoord = bp.y();
      checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, vertMergeHelper, nodeMap);
    //} else if (connFig->typeId() == frcPatchWire) {
    //  auto obj = static_cast<frPatchWire*>(connFig);
    //  obj->getOrigin(bp);
    //  auto lNum = obj->getLayerNum();
    //  //find whether there is horz track at bp
    //  auto crossPt    = bp;
    //  auto trackCoord = bp.y();
    //  auto splitCoord = bp.x();
    //  checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, horzMergeHelper, nodeMap);
    //  //find whether there is vert track at bp
    //  crossPt    = bp;
    //  trackCoord = bp.x();
    //  splitCoord = bp.y();
    //  checkConnectivity_nodeMap_routeObjSplit_helper(crossPt, trackCoord, splitCoord, lNum, vertMergeHelper, nodeMap);
    }
  }
}

void FlexDR::checkConnectivity_nodeMap_pin(frNet* net, 
                                           vector<frConnFig*> &netRouteObjs,
                                           vector<frBlockObject*> &netPins,
                                           map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                           map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  bool enableOutput = false;
  int currCnt = (int)netRouteObjs.size();
  for (auto &[obj, locS]: pin2epMap) {
    netPins.push_back(obj);
    for (auto &pr: locS) {
      nodeMap[pr].insert(currCnt);
      if (enableOutput) {
        cout <<"pin idx = " <<currCnt <<", (" <<pr.first.x() <<", " <<pr.first.y() <<") " 
             <<getTech()->getLayer(pr.second)->getName() <<endl;
      }
    }
    ++currCnt;
  }
}

void FlexDR::checkConnectivity_nodeMap(frNet* net, 
                                       vector<frConnFig*> &netRouteObjs,
                                       vector<frBlockObject*> &netPins,
                                       map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                       map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  bool enableOutput = false;
  //bool enableOutput = true;
  checkConnectivity_nodeMap_routeObjEnd(net, netRouteObjs, nodeMap);
  checkConnectivity_nodeMap_routeObjSplit(net, netRouteObjs, nodeMap);
  checkConnectivity_nodeMap_pin(net, netRouteObjs, netPins, pin2epMap, nodeMap);
  if (enableOutput) {
    int idx = 0;
    for (auto connFig: netRouteObjs) {
      if (connFig->typeId() == frcPathSeg) {
        auto obj = static_cast<frPathSeg*>(connFig);
        frPoint bp, ep;
        auto lNum = obj->getLayerNum();
        obj->getPoints(bp, ep);
        cout <<"#" <<idx <<" (" 
             <<bp.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
             <<bp.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") ("
             <<ep.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
             <<ep.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") "
             <<getTech()->getLayer(lNum)->getName() <<endl;
      } else if (connFig->typeId() == frcVia) {
        auto obj = static_cast<frVia*>(connFig);
        frPoint bp;
        obj->getOrigin(bp);
        cout <<"#" <<idx <<" (" 
             <<bp.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
             <<bp.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") "
             <<obj->getViaDef()->getName() <<endl;
      //} else if (connFig->typeId() == frcPatchWire) {
      //  auto obj = static_cast<frPatchWire*>(connFig);
      //  frPoint bp;
      //  auto lNum = obj->getLayerNum();
      //  obj->getOrigin(bp);
      //  frBox box;
      //  obj->getOffsetBox(box);
      //  cout <<"#" <<idx <<" (" 
      //       <<bp.x() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<", "
      //       <<bp.y() * 1.0/ getDesign()->getTopBlock()->getDBUPerUU() <<") RECT ("
      //       <<box.left()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
      //       <<box.bottom() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
      //       <<box.right()  * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<" "
      //       <<box.top()    * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") "
      //       <<getTech()->getLayer(lNum)->getName() <<endl;
      } else {
        cout <<"Error: checkConnectivity_nodeMap unsupported type" <<endl;
      }
      idx++;
    }
    for (auto obj: netPins) {
      if (obj->typeId() == frcInstTerm) {
        auto instTerm = static_cast<frInstTerm*>(obj);
        cout <<"#" <<idx <<" " <<instTerm->getInst()->getName() <<"/" <<instTerm->getTerm()->getName() <<endl;
      } else if (obj->typeId() == frcTerm) {
        auto term = static_cast<frTerm*>(obj);
        cout <<"#" <<idx <<" PIN/" <<term->getName() <<endl;
      }
      idx++;
    }
  }
}

bool FlexDR::checkConnectivity_astar(frNet* net, vector<bool> &adjVisited, vector<int> &adjPrevIdx, 
                                     map<pair<frPoint, frLayerNum>, set<int> > &nodeMap, int &gCnt, int &nCnt) {
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
  for (auto &[pr, idxS]: nodeMap) {
    //auto &[pt, lNum] = pr;
    for (auto it1 = idxS.begin(); it1 != idxS.end(); it1++) {
      auto it2 = it1;
      it2++;
      auto idx1 = *it1;
      for (; it2 != idxS.end(); it2++) {
        auto idx2 = *it2;
        adjVec[idx1].push_back(idx2);
        adjVec[idx2].push_back(idx1);
        //cout <<"add edge #" <<idx1 <<" -- #" <<idx2 <<endl;
        // one pin, one gcell
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
          if (i >= gCnt) {
            pq.push({i, adjPrevIdx[i], 0});
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
  if (enableOutput) {
  cout <<"stat: " <<net->getName() <<" #guide/#pin/#unused = " <<gCnt <<"/" <<nCnt - gCnt <<"/" 
       <<nCnt - count(adjVisited.begin(), adjVisited.end(), true) <<endl;
  }
  int pinVisited = count(adjVisited.begin() + gCnt, adjVisited.end(), true);
  // true error when allowing feedthrough
  if (pinVisited != nCnt - gCnt) {
    cout <<"Error: " <<net->getName() <<" " <<nCnt - gCnt - pinVisited <<" pin not visited #guides = " <<gCnt <<endl;
    if (enableOutput) {
      for (int i = gCnt; i < nCnt; i++) {
        if (!adjVisited[i]) {
          cout <<"  pin id = " <<i <<endl;
        }
      }
    }
  }
  if (pinVisited == nCnt - gCnt) {
    return true;
  } else {
    return false;
  }
}

void FlexDR::checkConnectivity_final(frNet *net, vector<frConnFig*> &netRouteObjs, vector<frBlockObject*> &netPins,
                                     vector<bool> &adjVisited, int gCnt, int nCnt,
                                     map<pair<frPoint, frLayerNum>, set<int> > &nodeMap) {
  //bool enableOutput = true;
  bool enableOutput = false;
  
  auto regionQuery = getRegionQuery();

  // from obj to pt
  map<int, set<pair<frPoint, frLayerNum> > > reverseNodeMap;
  for (auto &[pr, idxS]: nodeMap) {
    for (auto &idx: idxS) {
      reverseNodeMap[idx].insert(pr);
    }
  }

  // nodeMap delete redundant objs
  for (int i = 0; i < (int)adjVisited.size(); i++) {
    if (adjVisited[i]) {
      continue;
    }
    for (auto &pr: reverseNodeMap[i]) {
      nodeMap[pr].erase(i);
    }
    if (i < gCnt) {
      if (netRouteObjs[i]->typeId() == frcPathSeg) {
        regionQuery->removeDRObj(static_cast<frShape*>(netRouteObjs[i]));
        net->removeShape(static_cast<frShape*>(netRouteObjs[i]));
        if (enableOutput) {
          cout <<"net " <<net->getName() <<" deleting pathseg" <<endl;
        }
      } else if (netRouteObjs[i]->typeId() == frcVia) {
        regionQuery->removeDRObj(static_cast<frVia*>(netRouteObjs[i]));
        net->removeVia(static_cast<frVia*>(netRouteObjs[i]));
        if (enableOutput) {
          cout <<"net " <<net->getName() <<" deleting via" <<endl;
        }
      //} else if (netRouteObjs[i]->typeId() == frcPatchWire) {
      //  regionQuery->removeDRObj(static_cast<frPatchWire*>(netRouteObjs[i]));
      //  net->removePatchWire(static_cast<frPatchWire*>(netRouteObjs[i]));
      //  if (enableOutput) {
      //    cout <<"net " <<net->getName() <<" deleting pwire" <<endl;
      //  }
      } else {
        cout <<"Error: checkConnectivity_final unsupporterd type" <<endl;
        exit(1);
      }
      netRouteObjs[i] = nullptr;
    } else {
      cout <<"Error: checkConnectivity_final i >= gCnt" <<endl;
      exit(1);
    }
  }
  
  // rebuild reverseNodeMap
  reverseNodeMap.clear();
  for (auto &[pr, idxS]: nodeMap) {
    if (idxS.size() == 1) {
      continue;
    }
    for (auto &idx: idxS) {
      reverseNodeMap[idx].insert(pr);
    }
  }

  for (auto &[idx, ptS]: reverseNodeMap) {
    if (idx < gCnt && netRouteObjs[idx]->typeId() == frcPathSeg) {
      auto ps = static_cast<frPathSeg*>(netRouteObjs[idx]);
      frPoint bp, ep;
      ps->getPoints(bp, ep);
      auto [minPr, maxPr] = minmax_element(ptS.begin(), ptS.end());
      auto &minPt = minPr->first;
      auto &maxPt = maxPr->first;
      // shrink segment
      if (bp < minPt || maxPt < ep) {
        regionQuery->removeDRObj(ps);
        ps->setPoints(minPt, maxPt);
        regionQuery->addDRObj(ps);
        if (enableOutput) {
          cout <<"net " <<net->getName() <<" shrinking pathseg" <<endl;
        }
      }
    }
  }

  // delete redundant pwires
  set<pair<frPoint, frLayerNum> > validPoints;
  frPoint bp, ep;
  frLayerNum lNum;
  for (auto &connFig: net->getShapes()) {
    if (connFig->typeId() == frcPathSeg) {
      auto obj = static_cast<frPathSeg*>(connFig.get());
      obj->getPoints(bp, ep);
      lNum = obj->getLayerNum();
      validPoints.insert(make_pair(bp, lNum));
      validPoints.insert(make_pair(ep, lNum));
    } else {
      cout <<"Error: checkConnectivity_final unsupporterd type" <<endl;
      exit(1);
    }
  }
  for (auto &connFig: net->getVias()) {
    if (connFig->typeId() == frcVia) {
      auto obj = static_cast<frVia*>(connFig.get());
      obj->getOrigin(bp);
      lNum = obj->getViaDef()->getLayer1Num();
      validPoints.insert(make_pair(bp, lNum));
      lNum = obj->getViaDef()->getLayer2Num();
      validPoints.insert(make_pair(bp, lNum));
    } else {
      cout <<"Error: checkConnectivity_final unsupporterd type" <<endl;
      exit(1);
    }
  }
  for (auto it = net->getPatchWires().begin(); it != net->getPatchWires().end();) {
    auto obj = static_cast<frPatchWire*>(it->get());
    it++;
    obj->getOrigin(bp);
    lNum = obj->getLayerNum();
    if (validPoints.find(make_pair(bp, lNum)) == validPoints.end()) {
      regionQuery->removeDRObj(obj);
      net->removePatchWire(obj);
      if (enableOutput) {
        cout <<"net " <<net->getName() <<" deleting pwire" <<endl;
      }
    }
  }
}




// feedthrough and loop check
void FlexDR::checkConnectivity() {
  //cout <<"checking connectivity " <<endl;

  bool isWrong = false;
  for (auto &uPtr: getDesign()->getTopBlock()->getNets()) {
    auto net = uPtr.get();
    //if (net->getName() != "net3097") {
    //  continue;
    //}
    vector<frConnFig*> netDRObjs;
    checkConnectivity_initDRObjs(net, netDRObjs);
    
    map<frBlockObject*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> pin2epMap;
    checkConnectivity_pin2epMap(net, netDRObjs, pin2epMap);

    vector<frBlockObject*> netPins;
    map<pair<frPoint, frLayerNum>, set<int> > nodeMap;
    checkConnectivity_nodeMap(net, netDRObjs, netPins, pin2epMap, nodeMap);

    int gCnt = (int)netDRObjs.size();
    int nCnt = (int)netDRObjs.size() + (int)netPins.size();
    vector<bool> adjVisited;
    vector<int>  adjPrevIdx;
    if (!checkConnectivity_astar(net, adjVisited, adjPrevIdx, nodeMap, gCnt, nCnt)) {
      cout <<"Error: checkConnectivity break, net " <<net->getName() <<endl;
      isWrong = true;
    } else {
      // get lock
      // delete / shrink netRouteObjs, 
      checkConnectivity_final(net, netDRObjs, netPins, adjVisited, gCnt, nCnt, nodeMap);
      // release lock
    }
  }
  if (isWrong) {
    io::Writer writer(getDesign());
    writer.writeFromDR("_conn");
    exit(1);
  }
}

