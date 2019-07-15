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

#include "dr/FlexGridGraph.h"
#include "dr/FlexDR.h"
#include <iostream>
#include <map>
#include <fstream>

using namespace std;
using namespace fr;

//void FlexGridGraph::resetCost(frMIdx x, frMIdx y, frMIdx z) {
//  resetGridCostE(x, y, z);
//  resetGridCostN(x, y, z);
//  resetGridCostU(x, y, z);
//  resetShapePlanar(x, y, z);
//  resetShapeVia(x, y, z);
//  resetDRCCostPlanar(x, y, z);
//  resetDRCCostVia(x, y, z);
//  resetMarkerCostPlanar(x, y, z);
//  resetMarkerCostVia(x, y, z);
//}

/*
frCoord FlexGridGraph::initVia2ViaMinLen_helper(frLayerNum lNum, frViaDef* viaDef1, frViaDef* viaDef2) {
  if (!(viaDef1 && viaDef2)) {
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

void FlexGridGraph::initVia2ViaMinLen() {
  bool enableOutput = false;
  for (int z = 0; z < (int)zCoords.size(); z++) {
    frViaDef* downVia = nullptr;
    frViaDef* upVia   = nullptr;
    auto lNum = zCoords[z];
    if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1) {
      downVia = getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef();
    }
    if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1) {
      upVia = getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef();
    }
    via2viaMinLen[z][0] = initVia2ViaMinLen_helper(lNum, downVia, downVia);
    via2viaMinLen[z][1] = initVia2ViaMinLen_helper(lNum, downVia, upVia);
    //via2viaMinLen[z][2] = initVia2ViaMinLen_helper(lNum, upVia, downVia);
    via2viaMinLen[z][2] = via2viaMinLen[z][1];
    via2viaMinLen[z][3] = initVia2ViaMinLen_helper(lNum, upVia, upVia);
    if (enableOutput) {
      cout <<"initVia2ViaMinLen " <<getDesign()->getTech()->getLayer(lNum)->getName()
           <<" (d2d, d2u, u2d, u2u) = (" 
           <<via2viaMinLen[z][0] <<", "
           <<via2viaMinLen[z][1] <<", "
           <<via2viaMinLen[z][2] <<", "
           <<via2viaMinLen[z][3] <<")" <<endl;
    }
  }
}
*/

void FlexGridGraph::initGrids(const map<frCoord, map<frLayerNum, frTrackPattern* > > &xMap,
                              const map<frCoord, map<frLayerNum, frTrackPattern* > > &yMap,
                              const map<frLayerNum, frPrefRoutingDirEnum> &zMap,
                              bool followGuide) {
  //bool enableOutput = true;
  bool enableOutput = false;
  // initialize coord vectors
  xCoords.clear();
  yCoords.clear();
  zCoords.clear();
  zHeights.clear();
  zDirs.clear();
  //halfViaEncArea.clear();
  //via2viaMinLen.clear();
  for (auto &[k, v]: xMap) {
    xCoords.push_back(k);
  }
  for (auto &[k, v]: yMap) {
    yCoords.push_back(k);
  }
  frCoord zHeight = 0;
  //vector<frCoord> via2viaMinLenTmp(4, 0);
  for (auto &[k, v]: zMap) {
    zCoords.push_back(k);
    zHeight += getTech()->getLayer(k)->getPitch() * VIACOST;
    zHeights.push_back(zHeight);
    zDirs.push_back((v == frPrefRoutingDirEnum::frcHorzPrefRoutingDir));
    //if (k + 1 <= getTech()->getTopLayerNum() && getTech()->getLayer(k + 1)->getType() == frLayerTypeEnum::CUT) {
    //  auto viaDef = getTech()->getLayer(k + 1)->getDefaultViaDef();
    //  frVia via(viaDef);
    //  frBox layer1Box;
    //  frBox layer2Box;
    //  via.getLayer1BBox(layer1Box);
    //  via.getLayer2BBox(layer2Box);
    //  auto layer1HalfArea = layer1Box.width() * layer1Box.length() / 2;
    //  auto layer2HalfArea = layer2Box.width() * layer2Box.length() / 2;
    //  // cout <<"z = " <<zCoords.size() <<" " <<"layer1/2HalfArea = " <<layer1HalfArea <<"/" <<layer2HalfArea <<endl;
    //  halfViaEncArea.push_back(make_pair(layer1HalfArea, layer2HalfArea));
    //} else {
    //  halfViaEncArea.push_back(make_pair(0,0));
    //}
    //via2viaMinLen.push_back(via2viaMinLenTmp);
  }
  // initialize all grids
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  bits.clear();
  bits.resize(xDim*yDim*zDim, 0);
  // new
  prevDirs.clear();
  srcs.clear();
  dsts.clear();
  prevDirs.resize(xDim*yDim*zDim*3, 0);
  srcs.resize(xDim*yDim*zDim, 0);
  dsts.resize(xDim*yDim*zDim, 0);
  guides.clear();
  if (followGuide) {
    guides.resize(xDim*yDim*zDim, 0);
  } else {
    guides.resize(xDim*yDim*zDim, 1);
  }

  //initVia2ViaMinLen();

  //pinBlksE.clear();
  //pinBlksN.clear();
  //pinBlksE.resize(xDim*yDim*zDim, 0);
  //pinBlksN.resize(xDim*yDim*zDim, 0);

  //for (int k = 0; k < (int)zDim; k++) {
  //  for (int j = 0; j < (int)yDim; j++) {
  //    for (int i = 0; i < (int)xDim; i++) {
  //      resetCost(i, j, k);
  //    }
  //  }
  //}
  if (enableOutput) {
    cout <<"x ";
    for (auto &k: xCoords) {
      cout <<" " <<k;
    }
    cout <<endl;
    cout <<"y ";   
    for (auto &k: yCoords) {
      cout <<" " <<k;
    }
    cout <<endl;
    cout <<"z ";   
    for (auto &k: zCoords) {
      cout <<" " <<k;
    }
    cout <<endl;
    cout <<"z height ";   
    for (auto &k: zHeights) {
      cout <<" " <<k;
    }
    cout <<endl;
  }
}

void FlexGridGraph::initEdges(const map<frCoord, map<frLayerNum, frTrackPattern* > > &xMap,
                              const map<frCoord, map<frLayerNum, frTrackPattern* > > &yMap,
                              const map<frLayerNum, frPrefRoutingDirEnum> &zMap,
                              const frBox &bbox, bool initDR) {
  //bool enableOutput = true;
  bool enableOutput = false;
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  // initialize grid graph
  frMIdx xIdx = 0, yIdx = 0, zIdx = 0;
  bool flag = false;

  // add cost to out-of-die edge
  //frBox blockBox;
  //getDesign()->getTopBlock()->getBoundaryBBox(blockBox);
  //frBox viaBox, planarBox;
  //frVia dVia(nullptr);
  //dVia.setOrigin(frPoint(0,0));
  //frCoord halfWidth = 0;
  //frTransform xform;

  for (auto &[layerNum, dir]: zMap) {
    frLayerNum nonPrefLayerNum;
    //viaBox.set(0,0,0,0);
    //halfWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth() / 2;
    //planarBox.set(-halfWidth, -halfWidth, halfWidth, halfWidth);
    if (layerNum + 2 <= getTech()->getTopLayerNum()) {
      nonPrefLayerNum = layerNum + 2;
      //dVia.setViaDef(getDesign()->getTech()->getLayer(layerNum + 1)->getDefaultViaDef());
      //dVia.getBBox(viaBox);
    } else if (layerNum - 2 >= getTech()->getBottomLayerNum()) {
      nonPrefLayerNum = layerNum - 2;
    } else {
      nonPrefLayerNum = layerNum;
    }
    yIdx = 0;
    for (auto &[yCoord, ySubMap]: yMap) {
      auto yIt  = ySubMap.find(layerNum);
      auto yIt2 = ySubMap.find(layerNum+2);
      auto yIt3 = ySubMap.find(nonPrefLayerNum);
      bool yFound  = (yIt != ySubMap.end());
      bool yFound2 = (yIt2 != ySubMap.end());
      bool yFound3 = (yIt3 != ySubMap.end());
      xIdx = 0;
      for (auto &[xCoord, xSubMap]: xMap) {
        auto xIt  = xSubMap.find(layerNum);
        auto xIt2 = xSubMap.find(layerNum+2);
        auto xIt3 = xSubMap.find(nonPrefLayerNum);
        bool xFound  = (xIt != xSubMap.end());
        bool xFound2 = (xIt2 != xSubMap.end());
        bool xFound3 = (xIt3 != xSubMap.end());

        // add cost to out-of-die edge
        //xform.set(frPoint(xCoord, yCoord));
        //viaBox.transform(xform);
        //planarBox.transform(xform);
        //bool outOfDiePlanar = !blockBox.contains(planarBox);
        //bool outOfDieVia    = !blockBox.contains(viaBox);
        bool outOfDiePlanar = false;
        bool outOfDieVia    = false;
      //for (auto &[layerNum, dir]: zMap) {
        // add edge for preferred direction
        if (dir == frcHorzPrefRoutingDir && yFound) {
          if (layerNum >= BOTTOM_ROUTING_LAYER) {
            if (enableOutput) {
              if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::E)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) already set" <<endl;
              }
            }
            flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::E, bbox, initDR);
            if (yIt->second == nullptr || outOfDiePlanar) {
              setGridCostE(xIdx, yIdx, zIdx);
            }
            if (enableOutput) {
              if (!flag) {
                cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E)" <<endl;
              } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::E)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) not set" <<endl;
              } else {
                if (hasGridCostE(xIdx, yIdx, zIdx) != ((yIt->second || outOfDiePlanar) ? false : true)) {
                  cout <<"check edge cost failed (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) != 1" <<endl;
                }
                cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E)" <<endl;
              }
            }
          }
          // via to upper layer
          if (xFound2) {
            if (enableOutput) {
              if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::U)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U) already set" <<endl;
              }
            }
            flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::U, bbox, initDR);
            bool condition = (yIt->second == nullptr || xIt2->second == nullptr);
            if (condition || outOfDieVia) {
              setGridCostU(xIdx, yIdx, zIdx);
            }
            if (enableOutput) {
              if (!flag) {
                cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U)" <<endl;
              } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::U)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U) not set" <<endl;
              } else {
                cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U)" <<endl;
              }
            }
          }
        } else if (dir == frcVertPrefRoutingDir && xFound) {
          if (layerNum >= BOTTOM_ROUTING_LAYER) {
            if (enableOutput) {
              if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::N)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) already set" <<endl;
              }
            }
            flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::N, bbox, initDR);
            if (xIt->second == nullptr || outOfDiePlanar) {
              setGridCostN(xIdx, yIdx, zIdx);
            }
            if (enableOutput) {
              if (!flag) {
                cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N)" <<endl;
              } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::N)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) not set" <<endl;
              } else {
                if (hasGridCostN(xIdx, yIdx, zIdx) != ((xIt->second || outOfDiePlanar) ? false : true)) {
                  cout <<"check edge cost failed (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) != 1" <<endl;
                }
                cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N)" <<endl;

              }
            }
          }
          // via to upper layer
          if (yFound2) {
            if (enableOutput) {
              if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::U)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U) already set" <<endl;
              }
            }
            flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::U, bbox, initDR);
            bool condition = (yIt2->second == nullptr || xIt->second == nullptr);
            if (condition || outOfDieVia) {
              setGridCostU(xIdx, yIdx, zIdx);
            }
            if (enableOutput) {
              if (!flag) {
                cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U)" <<endl;
              } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::U)) {
                cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U) not set" <<endl;
              } else {
                cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U)" <<endl;
              }
            }
          }
        }
        // get non pref track layer --> use upper layer pref dir track if possible
        if (USENONPREFTRACKS) {
          // add edge for non-preferred direction
          // vertical non-pref track
          if (dir == frcHorzPrefRoutingDir && xFound3) {
            if (layerNum >= BOTTOM_ROUTING_LAYER) {
              if (enableOutput) {
                if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::N)) {
                  cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) already set" <<endl;
                }
              }
              flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::N, bbox, initDR);
              setGridCostN(xIdx, yIdx, zIdx);
              if (enableOutput) {
                if (!flag) {
                  cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N)" <<endl;
                } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::N)) {
                  cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) not set" <<endl;
                } else {
                  if (!hasGridCostN(xIdx, yIdx, zIdx)) {
                    cout <<"check edge cost failed (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) != 1" <<endl;
                  }
                  cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N)" <<endl;
                }
              }
            }
          // horizontal non-pref track
          } else if (dir == frcVertPrefRoutingDir && yFound3) {
            if (layerNum >= BOTTOM_ROUTING_LAYER) {
              if (enableOutput) {
                if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::E)) {
                  cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) already set" <<endl;
                }
              }
              flag = addEdge(xIdx, yIdx, zIdx, frDirEnum::E, bbox, initDR);
              setGridCostE(xIdx, yIdx, zIdx);
              if (enableOutput) {
                if (!flag) {
                  cout <<"edge out of bound (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E)" <<endl;
                } else if (!hasEdge(xIdx, yIdx, zIdx, frDirEnum::E)) {
                  cout <<"Error: (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) not set" <<endl;
                } else {
                  if (!hasGridCostE(xIdx, yIdx, zIdx)) {
                    cout <<"check edge cost failed (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) != 1" <<endl;
                  }
                  cout <<"add edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E)" <<endl;
                }
              }
            }
          }
        }
        ++xIdx;
      }
      ++yIdx;
    }
    ++zIdx;
  }
}


// initialization: update grid graph topology, does not assign edge cost
void FlexGridGraph::init(const frBox &routeBBox, const frBox &extBBox,
                         map<frCoord, map<frLayerNum, frTrackPattern* > > &xMap,
                         map<frCoord, map<frLayerNum, frTrackPattern* > > &yMap,
                         bool initDR, bool followGuide) {
  bool enableOutput = false;

  halfViaEncArea = getDRWorker()->getDR()->getHalfViaEncArea();
  via2viaMinLen  = getDRWorker()->getDR()->getVia2ViaMinLen();
  via2turnMinLen = getDRWorker()->getDR()->getVia2TurnMinLen();
  via2viaMinLenNew = getDRWorker()->getDR()->getVia2ViaMinLenNew();

  // get tracks intersecting with the Maze bbox
  map<frLayerNum, frPrefRoutingDirEnum> zMap;
  initTracks(xMap, yMap, zMap, extBBox);
  initGrids(xMap, yMap, zMap, followGuide); // buildGridGraph
  initEdges(xMap, yMap, zMap, routeBBox, initDR); // add edges and edgeCost
  if (enableOutput) {
    for(int i = 0; i < (int)xCoords.size(); i++) {
      for(int j = 0; j < (int)yCoords.size(); j++) {
        for(int k = 0; k < (int)zCoords.size(); k++) {
          cout <<"test x/y/z/E/N/U = "  <<i <<" " <<j <<" " <<k <<" "
               <<hasGridCostE(i, j, k) <<" "
               <<hasGridCostN(i, j, k) <<" "
               <<"0" <<" " <<endl;
        }
      }
    }
  }
}

// initialization helpers
// get all tracks inetersecting with the Maze bbox, left/bottom are inclusive
void FlexGridGraph::initTracks(map<frCoord, map<frLayerNum, frTrackPattern* > > &xMap,
                               map<frCoord, map<frLayerNum, frTrackPattern* > > &yMap,
                               map<frLayerNum, frPrefRoutingDirEnum> &zMap,
                               const frBox &bbox) {
  bool enableOutput = false;
  //bool enableOutput = true;
  for (auto &layer: getTech()->getLayers()) {
    if (layer->getType() != frLayerTypeEnum::ROUTING) {
      continue;
    }
    if (enableOutput) {
      cout <<"initTracks: " <<layer->getName() <<endl <<flush;
    }
    frLayerNum currLayerNum = layer->getLayerNum();
    frPrefRoutingDirEnum currPrefRouteDir = layer->getDir();
    //bool hasTrack = false;
    for (auto &tp: getDesign()->getTopBlock()->getTrackPatterns(currLayerNum)) {
      bool flag = USENONPREFTRACKS ? 
                  (tp->isHorizontal()  && currPrefRouteDir == frcVertPrefRoutingDir ||
                  !tp->isHorizontal() && currPrefRouteDir == frcHorzPrefRoutingDir) : 
                  true;
      if (flag) {
        //cout <<"checkPoint 1" <<endl <<flush;
        int trackNum = ((tp->isHorizontal() ? bbox.left() : bbox.bottom()) - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (trackNum < 0) {
          trackNum = 0;
        }
        if (trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < (tp->isHorizontal() ? bbox.left() : bbox.bottom())) {
          ++trackNum;
        }
        for (; 
             trackNum < (int)tp->getNumTracks() && 
             trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < (tp->isHorizontal() ? bbox.right() : bbox.top()); 
             ++trackNum) {
          frCoord trackLoc = trackNum * tp->getTrackSpacing() + tp->getStartCoord();
          //cout <<"checkPoint 2" <<endl <<flush;
          //hasTrack = true;
          if (tp->isHorizontal()) {
            xMap[trackLoc][currLayerNum] = tp.get();
          } else {
            yMap[trackLoc][currLayerNum] = tp.get();
          }
        }
      }
    }
    //if (hasTrack) {
      //cout <<"checkPoint 3" <<endl <<flush;
      zMap[currLayerNum] = currPrefRouteDir;
    //}
  }
  if (enableOutput) {
    cout <<"finish initTracks" <<endl <<flush;
  }
}

void FlexGridGraph::resetStatus() {
  //frMIdx xDim, yDim, zDim;
  //getDim(xDim, yDim, zDim);
  //for (frMIdx z = 0; z < zDim; z++) {
  //  for (frMIdx y = 0; y < yDim; y++) {
  //    for (frMIdx x = 0; x < xDim; x++) {
  //      //resetSrc(x, y, z);
  //      //resetDst(x, y, z);
  //      setPrevAstarNodeDir(x, y, z, frDirEnum::UNKNOWN);
  //    }
  //  }
  //}
  resetSrc();
  resetDst();
  resetPrevNodeDir();
}

void FlexGridGraph::resetSrc() {
  srcs.assign(srcs.size(), 0);
}

void FlexGridGraph::resetDst() {
  dsts.assign(dsts.size(), 0);
}

void FlexGridGraph::resetPrevNodeDir() {
  //frMIdx xDim, yDim, zDim;
  //getDim(xDim, yDim, zDim);
  //for (frMIdx z = 0; z < zDim; z++) {
  //  for (frMIdx y = 0; y < yDim; y++) {
  //    for (frMIdx x = 0; x < xDim; x++) {
  //      setPrevAstarNodeDir(x, y, z, frDirEnum::UNKNOWN);
  //    }
  //  }
  //}
  prevDirs.assign(prevDirs.size(), 0);
}

// print the grid graph with edge and vertex for debug purpose
void FlexGridGraph::print() {
  ofstream mazeLog(OUT_MAZE_FILE.c_str());
  if (mazeLog.is_open()) {
    // print edges
    frBox gridBBox;
    getBBox(gridBBox);
    mazeLog << "printing Maze grid (" << gridBBox.left()  << ", " << gridBBox.bottom() << ") -- (" 
                                      << gridBBox.right() << ", " << gridBBox.top()    << ")\n";
    frMIdx xDim, yDim, zDim;
    getDim(xDim, yDim, zDim);

    if (xDim == 0 || yDim == 0 || zDim == 0) {
      cout << "Error: dimension == 0\n";
      return;
    } else {
      cout << "(xDim, yDim, zDim) = (" << xDim << ", " << yDim << ", " << zDim << ")\n";
    }

    // mazeLog << "xDim = " << xDim << ", yDim = " << yDim << ", zDim = " << zDim << "\n";

    frPoint p;
    for (frMIdx xIdx = 0; xIdx < xDim; ++xIdx) {
      // mazeLog << "xxxIdx = " << xIdx << "\n" << std::flush;
      for (frMIdx yIdx = 0; yIdx < yDim; ++yIdx) {
        for (frMIdx zIdx = 0; zIdx < zDim; ++zIdx) {
          // mazeLog << "xIdx = " << xIdx << ", yIdx = " << yIdx << ", zIdx = " << zIdx << "\n" << std::flush;
          if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::N)) {
            if (yIdx + 1 >= yDim) {
              cout <<"Error: no edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", N) " <<yDim <<endl;
              continue;
            } 
            mazeLog << "Edge: " << getPoint(p, xIdx, yIdx).x()   << " " << getPoint(p, xIdx, yIdx).y()   << " " << zIdx
                    << " "      << getPoint(p, xIdx, yIdx+1).x() << " " << getPoint(p, xIdx, yIdx+1).y() << " " << zIdx << "\n";
          }
          if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::E)) {
            if (xIdx + 1 >= xDim) {
              cout <<"Error: no edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", E) " << xDim <<endl;
              continue;
            }
            mazeLog << "Edge: " << getPoint(p, xIdx, yIdx).x()   << " " << getPoint(p, xIdx, yIdx).y()   << " " << zIdx
                    << " "      << getPoint(p, xIdx+1, yIdx).x() << " " << getPoint(p, xIdx+1, yIdx).y() << " " << zIdx << "\n";
          }
          if (hasEdge(xIdx, yIdx, zIdx, frDirEnum::U)) {
            if (zIdx + 1 >= zDim) {
              cout <<"Error: no edge (" <<xIdx <<", " <<yIdx <<", " <<zIdx <<", U) " <<zDim <<endl;
              continue;
            }
            mazeLog << "Edge: " << getPoint(p, xIdx, yIdx).x() << " " << getPoint(p, xIdx, yIdx).y() << " " << zIdx
                    << " "      << getPoint(p, xIdx, yIdx).x() << " " << getPoint(p, xIdx, yIdx).y() << " " << zIdx + 1 << "\n";
          }
        }
      }
    }
  } else {
    cout << "Error: Fail to open maze log\n";
  }
}

