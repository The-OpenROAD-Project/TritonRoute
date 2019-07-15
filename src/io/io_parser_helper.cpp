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
#include <iostream>
#include <boost/graph/connected_components.hpp>
//#include <boost/property_map/property_map.hpp>
#include "global.h"
#include "io/io.h"
#include "frBaseTypes.h"
#include <fstream>

//#include "frGuidePrep.h"
using namespace std;
using namespace fr;
using namespace boost::polygon::operators;

// void io::Parser::initDefaultVias() {
//   for (int layerNum = 1; layerNum < (int)tech->getLayers().size(); layerNum += 2) {
//     map<frViaDef*, bool>    isWithinLayer1;
//     map<frViaDef*, bool>    isWithinLayer2;
//     map<frViaDef*, frCoord> layer1Area;
//     map<frViaDef*, frCoord> layer2Area;
//     map<frViaDef*, int>     numCuts;
//     for (auto &uViaDef: tech->getVias()) {
//       auto viaDef = uViaDef.get();
//       if (viaDef->getCutLayerNum() == layerNum && viaDef->getDefault()) {
//         std::cout << "add " << viaDef->getName() << " to layer " << layerNum << "\n";
//         tech->getLayer(layerNum)->addViaDef(viaDef);
//         numCuts[viaDef] = viaDef->getCutFigs().size();
//         isWithinLayer1[viaDef] = true;
//         isWithinLayer2[viaDef] = true;
//         layer1Area[viaDef]     = 0;
//         layer2Area[viaDef]     = 0;
//         for (auto &pinFig: viaDef->getLayer1Figs()) {
//           if (pinFig->typeId() == frcRect) {
//             auto routeLayerNum = viaDef->getLayer1Num();
//             frBox box;
//             pinFig->getBBox(box);
//             layer1Area[viaDef] += (box.right() - box.left()) * (box.top() - box.bottom());
//             if (tech->getLayers().at(routeLayerNum)->getDir() == frcHorzPrefRoutingDir) {
//               if (frCoord(tech->getLayers().at(routeLayerNum)->getWidth()) < box.top() - box.bottom()) {
//                 isWithinLayer1[viaDef] = false;
//               }
//             } else {
//               if (frCoord(tech->getLayers().at(routeLayerNum)->getWidth()) < box.right() - box.left()) {
//                 isWithinLayer1[viaDef] = false;
//               }
//             }
//           } else if (pinFig->typeId() == frcPolygon) {
//             cout <<"Error: initDefaultVias does not support polygons" <<endl;
//             exit(0);
//           } else {
//             cout <<"Error: initDefaultVias cannot find rect or poly shapes" <<endl;
//             exit(0);
//           }
//         }
//         for (auto &pinFig: viaDef->getLayer2Figs()) {
//           if (pinFig->typeId() == frcRect) {
//             auto routeLayerNum = viaDef->getLayer2Num();
//             frBox box;
//             pinFig->getBBox(box);
//             layer2Area[viaDef] += (box.right() - box.left()) * (box.top() - box.bottom());
//             if (tech->getLayers().at(routeLayerNum)->getDir() == frcHorzPrefRoutingDir) {
//               if (frCoord(tech->getLayers().at(routeLayerNum)->getWidth()) < box.top() - box.bottom()) {
//                 isWithinLayer2[viaDef] = false;
//               }
//             } else {
//               if (frCoord(tech->getLayers().at(routeLayerNum)->getWidth()) < box.right() - box.left()) {
//                 isWithinLayer2[viaDef] = false;
//               }
//             }
//           } else if (pinFig->typeId() == frcPolygon) {
//             cout <<"Error: initDefaultVias does not support polygons" <<endl;
//             exit(0);
//           } else {
//             cout <<"Error: initDefaultVias cannot find rect or poly shapes" <<endl;
//             exit(0);
//           }
//         }
//       } else {
//         continue;
//       }
//     }
      
//     // single-cut only
//     frViaDef* defaultSingleCutVia = nullptr;
//     for (auto &it: numCuts) {
//       auto viaDef = it.first;
//       if (numCuts[viaDef] == 1) {
//         if (isWithinLayer1[viaDef] && isWithinLayer2[viaDef]) {
//           if (defaultSingleCutVia) {
//             auto defaultViaArea = layer1Area[defaultSingleCutVia] + layer2Area[defaultSingleCutVia];
//             auto currentViaArea = layer1Area[viaDef] + layer2Area[viaDef];
//             if (currentViaArea < defaultViaArea) {
//               defaultSingleCutVia = viaDef;
//             }
//           } else {
//             defaultSingleCutVia = viaDef;
//           }
//         }
//       }
//     }
//     if (!defaultSingleCutVia) {
//       for (auto &it: numCuts) {
//         auto viaDef = it.first;
//         if (numCuts[viaDef] == 1) {
//           if (isWithinLayer1[viaDef] || isWithinLayer2[viaDef]) {
//             if (defaultSingleCutVia) {
//               auto defaultViaArea = layer1Area[defaultSingleCutVia] + layer2Area[defaultSingleCutVia];
//               auto currentViaArea = layer1Area[viaDef] + layer2Area[viaDef];
//               if (currentViaArea < defaultViaArea) {
//                 defaultSingleCutVia = viaDef;
//               }
//             } else {
//               defaultSingleCutVia = viaDef;
//             }
//           }
//         }
//       }
//     }
//     if (!defaultSingleCutVia) {
//       for (auto &it: numCuts) {
//         auto viaDef = it.first;
//         if (numCuts[viaDef] == 1) {
//             if (defaultSingleCutVia) {
//               auto defaultViaArea = layer1Area[defaultSingleCutVia] + layer2Area[defaultSingleCutVia];
//               auto currentViaArea = layer1Area[viaDef] + layer2Area[viaDef];
//               if (currentViaArea < defaultViaArea) {
//                 defaultSingleCutVia = viaDef;
//               }
//             } else {
//               defaultSingleCutVia = viaDef;
//             }
//         }
//       }
//     }

//     if (defaultSingleCutVia) {
//       tech->getLayer(layerNum)->setDefaultViaDef(defaultSingleCutVia);
//     } else {
//       cout <<"Error: no default single-cut via available!" <<endl;
//       exit(0);
//     }
//   }
// }

void io::Parser::initDefaultVias() {
  for (auto &uViaDef: tech->getVias()) {
    auto viaDef = uViaDef.get();
    tech->getLayer(viaDef->getCutLayerNum())->addViaDef(viaDef);
  }



  std::map<frLayerNum, std::map<int, std::map<viaRawPriorityTuple, frViaDef*> > > layerNum2ViaDefs;
  for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
    if (design->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::CUT) {
      continue;
    }
    for (auto &viaDef: design->getTech()->getLayer(layerNum)->getViaDefs()) {
      int cutNum = int(viaDef->getCutFigs().size());
      viaRawPriorityTuple priority;
      getViaRawPriority(viaDef, priority);
      layerNum2ViaDefs[layerNum][cutNum][priority] = viaDef;
    }
    if (!layerNum2ViaDefs[layerNum][1].empty()) {
      auto defaultSingleCutVia = (layerNum2ViaDefs[layerNum][1].begin())->second;
      tech->getLayer(layerNum)->setDefaultViaDef(defaultSingleCutVia);
    } else {
      std::cout << "Error: " << tech->getLayer(layerNum)->getName() << " does not have single-cut via\n";
      exit(1);
    }
  }
}

void io::Parser::getViaRawPriority(frViaDef* viaDef, viaRawPriorityTuple &priority) {
  bool isNotDefaultVia = !(viaDef->getDefault());
  bool isNotUpperAlign = false;
  bool isNotLowerAlign = false;
  PolygonSet viaLayerPS1;

  for (auto &fig: viaDef->getLayer1Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    viaLayerPS1 += bboxRect;
  }
  Rectangle layer1Rect;
  extents(layer1Rect, viaLayerPS1);
  bool isLayer1Horz = (xh(layer1Rect) - xl(layer1Rect)) > (yh(layer1Rect) - yl(layer1Rect));
  frCoord layer1Width = std::min((xh(layer1Rect) - xl(layer1Rect)), (yh(layer1Rect) - yl(layer1Rect)));
  isNotLowerAlign = (isLayer1Horz && (tech->getLayer(viaDef->getLayer1Num())->getDir() == frcVertPrefRoutingDir)) ||
                    (!isLayer1Horz && (tech->getLayer(viaDef->getLayer1Num())->getDir() == frcHorzPrefRoutingDir));

  PolygonSet viaLayerPS2;
  for (auto &fig: viaDef->getLayer2Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    viaLayerPS2 += bboxRect;
  }
  Rectangle layer2Rect;
  extents(layer2Rect, viaLayerPS2);
  bool isLayer2Horz = (xh(layer2Rect) - xl(layer2Rect)) > (yh(layer2Rect) - yl(layer2Rect));
  frCoord layer2Width = std::min((xh(layer2Rect) - xl(layer2Rect)), (yh(layer2Rect) - yl(layer2Rect)));
  isNotUpperAlign = (isLayer2Horz && (tech->getLayer(viaDef->getLayer2Num())->getDir() == frcVertPrefRoutingDir)) ||
                    (!isLayer2Horz && (tech->getLayer(viaDef->getLayer2Num())->getDir() == frcHorzPrefRoutingDir));

  frCoord layer1Area = area(viaLayerPS1);
  frCoord layer2Area = area(viaLayerPS2);

  priority = std::make_tuple(isNotDefaultVia, layer1Width, layer2Width, isNotUpperAlign, layer2Area, layer1Area, isNotLowerAlign);
}

// 11m_2xa1xd3xe2y2r
void io::Parser::initDefaultVias_N16(const string &node) {
  for (int layerNum = 1; layerNum < (int)tech->getLayers().size(); layerNum += 2) {
    for (auto &uViaDef: tech->getVias()) {
      auto viaDef = uViaDef.get();
      if (viaDef->getCutLayerNum() == layerNum && node == "N16_11m_2xa1xd3xe2y2r_utrdl") {
        switch(layerNum) {
          case 1 : // VIA1
                   if (viaDef->getName() == "NR_VIA1_PH") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 3 : // VIA2
                   if (viaDef->getName() == "VIA2_0_25_3_32_HV_Vxa") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 5 : // VIA3
                   if (viaDef->getName() == "VIA3_3_34_4_35_VH_Vxd") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 7 : // VIA4
                   if (viaDef->getName() == "VIA4_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 9 : // VIA5
                   if (viaDef->getName() == "VIA5_0_50_0_50_VH_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 11: // VIA6
                   if (viaDef->getName() == "VIA6_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 13: // VIA7
                   if (viaDef->getName() == "NR_VIA7_VH") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 15: // VIA8
                   if (viaDef->getName() == "VIA8_0_27_0_27_HV_Vy") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 17: // VIA9
                   if (viaDef->getName() == "VIA9_18_72_18_72_VH_Vr") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 19: // VIA10
                   if (viaDef->getName() == "VIA10_18_72_18_72_HV_Vr") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          default: ;
        }
      } else if (viaDef->getCutLayerNum() == layerNum && node == "N16_9m_2xa1xd4xe1z_utrdl") {
        switch(layerNum) {
          case 1 : // VIA1
                   if (viaDef->getName() == "NR_VIA1_PH") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 3 : // VIA2
                   if (viaDef->getName() == "VIA2_0_25_3_32_HV_Vxa") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 5 : // VIA3
                   if (viaDef->getName() == "VIA3_3_34_4_35_VH_Vxd") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 7 : // VIA4
                   if (viaDef->getName() == "VIA4_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 9 : // VIA5
                   if (viaDef->getName() == "VIA5_0_50_0_50_VH_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 11: // VIA6
                   if (viaDef->getName() == "VIA6_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 13: // VIA7
                   if (viaDef->getName() == "VIA7_0_50_0_50_VH_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 15: // VIA8
                   if (viaDef->getName() == "VIA8_18_72_18_72_HV_Vz") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 17: // VIA9
                   if (viaDef->getName() == "RV_450_450_450_450_XX_RV") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          default: ;
        }
      } else if (viaDef->getCutLayerNum() == layerNum && node == "N16_9m_2xa1xd3xe2z_utrdl") {
        switch(layerNum) {
          case 1 : // VIA1
                   if (viaDef->getName() == "NR_VIA1_PH") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 3 : // VIA2
                   if (viaDef->getName() == "VIA2_0_25_3_32_HV_Vxa") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 5 : // VIA3
                   if (viaDef->getName() == "VIA3_3_34_4_35_VH_Vxd") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 7 : // VIA4
                   if (viaDef->getName() == "VIA4_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 9 : // VIA5
                   if (viaDef->getName() == "VIA5_0_50_0_50_VH_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 11: // VIA6
                   if (viaDef->getName() == "VIA6_0_50_0_50_HV_Vxe") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 13: // VIA7
                   if (viaDef->getName() == "VIA7_18_72_18_72_VH_Vz") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 15: // VIA8
                   if (viaDef->getName() == "VIA8_18_72_18_72_HV_Vz") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          case 17: // VIA9
                   if (viaDef->getName() == "RV_450_450_450_450_XX_RV") {
                     tech->getLayer(layerNum)->setDefaultViaDef(viaDef);
                   }
                   break;
          default: ;
        }
      }
    }
  }
}

void io::Parser::postProcess() {
  if (DBPROCESSNODE == "N16_11m_2xa1xd3xe2y2r_utrdl") {
    initDefaultVias_N16(DBPROCESSNODE);
  } else {
    initDefaultVias();
  }
  tech->printDefaultVias();


  instAnalysis();

  // init region query
  cout <<endl <<"init region query ..." <<endl;
  design->getRegionQuery()->init(design->getTech()->getLayers().size());
  design->getRegionQuery()->print();
  // test
  /*
  frBox box(372500, 1765500, 375500, 1768500);
  for (frLayerNum i = 0; i < (int)design->getTech()->getLayers().size(); i++) {
    vector<rq_rptr_value_t<frBlockObject> > result;
    design->getRegionQuery()->query(box, i, result);
    for (auto &[bBox, rptr]: result) {
      cout <<"bBox (" <<bg::get<bg::min_corner, 0>(bBox) * 1.0 / design->getTopBlock()->getDBUPerUU() <<", "
                      <<bg::get<bg::min_corner, 1>(bBox) * 1.0 / design->getTopBlock()->getDBUPerUU() <<") ("
                      <<bg::get<bg::max_corner, 0>(bBox) * 1.0 / design->getTopBlock()->getDBUPerUU() <<", "
                      <<bg::get<bg::max_corner, 1>(bBox) * 1.0 / design->getTopBlock()->getDBUPerUU() <<") "
                      <<design->getTech()->getLayer(i)->getName() <<endl;
      if (rptr->typeId() == frcInstTerm) {
        cout <<" instTerm " <<static_cast<frInstTerm*>(rptr)->getInst()->getName() <<"/"
             <<static_cast<frInstTerm*>(rptr)->getTerm()->getName() <<endl;
      } else if (rptr->typeId() == frcTerm) {
        cout <<"     term PIN/" <<static_cast<frTerm*>(rptr)->getName() <<endl;
      } else if (rptr->typeId() == frcLayerBlockage) {
        cout <<"     lblk" <<endl;
      } else if (rptr->typeId() == frcPathSeg) {
        cout <<"     pseg ";
        if (static_cast<frPathSeg*>(rptr)->hasNet()) {
          cout <<static_cast<frPathSeg*>(rptr)->getNet()->getName();
        }
        cout <<endl;
      } else if (rptr->typeId() == frcVia) {
        cout <<"      via ";
        if (static_cast<frVia*>(rptr)->hasNet()) {
          cout <<static_cast<frVia*>(rptr)->getNet()->getName();
        }
        cout <<endl;
      } else {
        cout <<"Error: unknown type in rq test" <<endl;
      }
    }
  }*/
}

void io::Parser::postProcessGuide() {
  if (VERBOSE > 0) {
    cout <<endl <<"post process guides ..." <<endl;
  }
  buildGCellPatterns();
  
  design->getRegionQuery()->initOrigGuide(design->getTech()->getLayers().size(), tmpGuides);
  int cnt = 0;
  //for (auto &[netName, rects]:tmpGuides) {
  //  if (design->getTopBlock()->name2net.find(netName) == design->getTopBlock()->name2net.end()) {
  //    cout <<"Error: postProcessGuide cannot find net" <<endl;
  //    exit(1);
  //  }
  //  auto net = design->getTopBlock()->name2net[netName];
  for (auto &[net, rects]:tmpGuides) {
    genGuides(net, rects);
    cnt++;
    if (VERBOSE > 0) {
      if (cnt < 100000) {
        if (cnt % 10000 == 0) {
          cout <<"  complete " <<cnt <<" nets" <<endl;
        }
      } else {
        if (cnt % 100000 == 0) {
          cout <<"  complete " <<cnt <<" nets" <<endl;
        }
      }
    }
  }

  // global unique id for guides
  int currId = 0;
  for (auto &net: design->getTopBlock()->getNets()) {
    for (auto &guide: net->getGuides()) {
      guide->setId(currId);
      currId++;
    }
  }

  buildCMap();
  
  cout <<endl <<"init guide query ..." <<endl;
  design->getRegionQuery()->initGuide(design->getTech()->getLayers().size());
  design->getRegionQuery()->printGuide();
  cout <<endl <<"init gr pin query ..." <<endl;
  design->getRegionQuery()->initGRPin(tmpGRPins);

  if (OUTGUIDE_FILE == string("")) {
    ;
  } else {
    writeGuideFile();
  }

}

void io::Parser::buildGCellPatterns_helper(frCoord &GCELLGRIDX, frCoord &GCELLGRIDY,
                                           frCoord &GCELLOFFSETX, frCoord &GCELLOFFSETY) {
  buildGCellPatterns_getWidth(GCELLGRIDX, GCELLGRIDY);
  buildGCellPatterns_getOffset(GCELLGRIDX, GCELLGRIDY, GCELLOFFSETX, GCELLOFFSETY);
}

void io::Parser::buildGCellPatterns_getWidth(frCoord &GCELLGRIDX, frCoord &GCELLGRIDY) {
  map<frCoord, int> guideGridXMap, guideGridYMap;
  // get GCell size information loop
  for (auto &[netName, rects]: tmpGuides) {
    for (auto &rect: rects) {
      frLayerNum layerNum = rect.getLayerNum();
      frBox guideBBox;
      rect.getBBox(guideBBox);
      frCoord guideWidth = (tech->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) ?
                           (guideBBox.top() - guideBBox.bottom()):
                           (guideBBox.right() - guideBBox.left());
      if (tech->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
        if (guideGridYMap.find(guideWidth) == guideGridYMap.end()) {
          guideGridYMap[guideWidth] = 0;
        }
        guideGridYMap[guideWidth]++;
      } else if (tech->getLayer(layerNum)->getDir() == frcVertPrefRoutingDir) {
        if (guideGridXMap.find(guideWidth) == guideGridXMap.end()) {
          guideGridXMap[guideWidth] = 0;
        }
        guideGridXMap[guideWidth]++;
      }
    }
  }
  frCoord tmpGCELLGRIDX = -1, tmpGCELLGRIDY = -1;
  int tmpGCELLGRIDXCnt = -1, tmpGCELLGRIDYCnt = -1;
  for (auto mapIt = guideGridXMap.begin(); mapIt != guideGridXMap.end(); ++mapIt) {
    auto cnt = mapIt->second;
    if (cnt > tmpGCELLGRIDXCnt) {
      tmpGCELLGRIDXCnt = cnt;
      tmpGCELLGRIDX = mapIt->first;
    }
    //cout <<"X width=" <<mapIt->first <<"/" <<mapIt->second <<endl;
  }
  for (auto mapIt = guideGridYMap.begin(); mapIt != guideGridYMap.end(); ++mapIt) {
    auto cnt = mapIt->second;
    if (cnt > tmpGCELLGRIDYCnt) {
      tmpGCELLGRIDYCnt = cnt;
      tmpGCELLGRIDY = mapIt->first;
    }
    //cout <<"Y width=" <<mapIt->first <<"/" <<mapIt->second <<endl;
  }
  if (tmpGCELLGRIDX != -1) {
    GCELLGRIDX = tmpGCELLGRIDX;
  } else {
    cout <<"Error: no GCELLGRIDX" <<endl;
    exit(1);
  }
  if (tmpGCELLGRIDY != -1) {
    GCELLGRIDY = tmpGCELLGRIDY;
  } else {
    cout <<"Error: no GCELLGRIDY" <<endl;
    exit(1);
  }
}

void io::Parser::buildGCellPatterns_getOffset(frCoord GCELLGRIDX, frCoord GCELLGRIDY,
                                              frCoord &GCELLOFFSETX, frCoord &GCELLOFFSETY) {
  std::map<frCoord, int> guideOffsetXMap, guideOffsetYMap;
  // get GCell offset information loop
  for (auto &[netName, rects]: tmpGuides) {
    for (auto &rect: rects) {
      //frLayerNum layerNum = rect.getLayerNum();
      frBox guideBBox;
      rect.getBBox(guideBBox);
      frCoord guideXOffset = guideBBox.left() % GCELLGRIDX;
      frCoord guideYOffset = guideBBox.bottom() % GCELLGRIDY;
      if (guideXOffset < 0) {
        guideXOffset = GCELLGRIDX - guideXOffset;
      }
      if (guideYOffset < 0) {
        guideYOffset = GCELLGRIDY - guideYOffset;
      }
      if (guideOffsetXMap.find(guideXOffset) == guideOffsetXMap.end()) {
        guideOffsetXMap[guideXOffset] = 0;
      }
      guideOffsetXMap[guideXOffset]++;
      if (guideOffsetYMap.find(guideYOffset) == guideOffsetYMap.end()) {
        guideOffsetYMap[guideYOffset] = 0;
      }
      guideOffsetYMap[guideYOffset]++;
    }
  }
  frCoord tmpGCELLOFFSETX = -1, tmpGCELLOFFSETY = -1;
  int tmpGCELLOFFSETXCnt = -1, tmpGCELLOFFSETYCnt = -1;
  for (auto mapIt = guideOffsetXMap.begin(); mapIt != guideOffsetXMap.end(); ++mapIt) {
    auto cnt = mapIt->second;
    if (cnt > tmpGCELLOFFSETXCnt) {
      tmpGCELLOFFSETXCnt = cnt;
      tmpGCELLOFFSETX = mapIt->first;
    }
  }
  for (auto mapIt = guideOffsetYMap.begin(); mapIt != guideOffsetYMap.end(); ++mapIt) {
    auto cnt = mapIt->second;
    if (cnt > tmpGCELLOFFSETYCnt) {
      tmpGCELLOFFSETYCnt = cnt;
      tmpGCELLOFFSETY = mapIt->first;
    }
  }
  if (tmpGCELLOFFSETX != -1) {
    GCELLOFFSETX = tmpGCELLOFFSETX;
  } else {
    cout <<"Error: no GCELLGRIDX" <<endl;
    exit(1);
  } 
  if (tmpGCELLOFFSETY != -1) {
    GCELLOFFSETY = tmpGCELLOFFSETY;
  } else {
    cout <<"Error: no GCELLGRIDX" <<endl;
    exit(1);
  }
}

void io::Parser::buildGCellPatterns() {
  // horizontal = false is gcell lines along y direction (x-grid)
  frBox dieBox;
  design->getTopBlock()->getBoundaryBBox(dieBox);

  frCoord GCELLOFFSETX, GCELLOFFSETY, GCELLGRIDX, GCELLGRIDY;
  buildGCellPatterns_helper(GCELLGRIDX, GCELLGRIDY, GCELLOFFSETX, GCELLOFFSETY);

  frGCellPattern xgp;
  xgp.setHorizontal(false);
  // find first coord >= dieBox.left()
  frCoord startCoordX = dieBox.left() / (frCoord)GCELLGRIDX * (frCoord)GCELLGRIDX;
  //cout <<"here " <<dieBox.left() <<" " <<GCELLGRIDX <<endl;
  if (startCoordX < dieBox.left()) {
    startCoordX += (frCoord)GCELLGRIDX;
  }
  xgp.setStartCoord(startCoordX);
  //xgp.setStartCoord(GCELLOFFSETX);
  xgp.setSpacing(GCELLGRIDX);
  if ((dieBox.right() - (frCoord)GCELLOFFSETX) / (frCoord)GCELLGRIDX < 1) {
    cout <<"Error: gcell cnt < 1" <<endl;
    exit(1);
  }
  //xgp.setCount((dieBox.right() - GCELLOFFSETX) / GCELLGRIDX + 1);
  xgp.setCount((dieBox.right() - (frCoord)startCoordX) / (frCoord)GCELLGRIDX);
  
  frGCellPattern ygp;
  ygp.setHorizontal(true);
  // find first coord >= dieBox.bottom()
  frCoord startCoordY = dieBox.bottom() / (frCoord)GCELLGRIDY * (frCoord)GCELLGRIDY;
  if (startCoordY < dieBox.bottom()) {
    startCoordY += (frCoord)GCELLGRIDY;
  }
  ygp.setStartCoord(startCoordY);
  //ygp.setStartCoord(GCELLOFFSETY);
  ygp.setSpacing(GCELLGRIDY);
  if ((dieBox.top() - (frCoord)GCELLOFFSETY) / (frCoord)GCELLGRIDY < 1) {
    cout <<"Error: gcell cnt < 1" <<endl;
    exit(1);
  }
  //ygp.setCount((dieBox.top() - GCELLOFFSETY) / GCELLGRIDY + 1);
  ygp.setCount((dieBox.top() - startCoordY) / (frCoord)GCELLGRIDY);

  if (VERBOSE > 0) {
    cout <<"GCELLGRID X " <<ygp.getStartCoord() <<" DO " <<ygp.getCount() <<" STEP " <<ygp.getSpacing() <<" ;" <<endl;
    cout <<"GCELLGRID Y " <<xgp.getStartCoord() <<" DO " <<xgp.getCount() <<" STEP " <<xgp.getSpacing() <<" ;" <<endl;
  }

  design->getTopBlock()->setGCellPatterns({xgp, ygp});

  auto &cmap = design->getTopBlock()->getCMap();
  cmap.init(xgp.getCount(), ygp.getCount(), tech->layers.size());

  for (int layerNum = 0; layerNum <= (int)tech->getLayers().size(); layerNum += 2) {
    for (int i = 0; i < (int)xgp.getCount(); i++) {
      for (int j = 0; j < (int)ygp.getCount(); j++) {
        frBox gcellBox;
        design->getTopBlock()->getGCellBox(frPoint(i, j), gcellBox);
        bool isH = (tech->getLayers().at(layerNum)->getDir() == frcHorzPrefRoutingDir);
        frCoord gcLow  = isH ? gcellBox.bottom() : gcellBox.right();
        frCoord gcHigh = isH ? gcellBox.top()    : gcellBox.left();
        //if (isH) {
        //  if (j == (int)ygp.getCount() - 1) {
        //    gcHigh = dieBox.top();
        //  }
        //} else {
        //  if (i == (int)xgp.getCount() - 1) {
        //    gcHigh = dieBox.right();
        //  }
        //}
        int trackCnt = 0;
        for (auto &tp: design->getTopBlock()->getTrackPatterns(layerNum)) {
          if ((tech->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir &&
               tp->isHorizontal() == false) ||
              (tech->getLayer(layerNum)->getDir() == frcVertPrefRoutingDir &&
               tp->isHorizontal() == true)) {
            int trackNum = (gcLow - tp->getStartCoord()) / (int)tp->getTrackSpacing();
            if (trackNum < 0) {
              trackNum = 0;
            }
            if (trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < gcLow) {
              trackNum++;
            }
            for (; trackNum < (int)tp->getNumTracks() && trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < gcHigh; trackNum++) {
              trackCnt++;
            }

          }
        }
        cmap.setSupply(i,j,layerNum, trackCnt);
      }
    }
  }
  //design->getTopBlock()->setCMap(cmap);
}

void io::Parser::buildCMap() {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (VERBOSE > 0) {
    cout <<endl <<"building cmap ... " <<endl;
  }

  auto guides = design->getTopBlock()->getGuides();
  if (enableOutput) {
    cout <<"all guides = " <<guides.size() <<endl;
  }
  //auto &gCellPatterns = design->getTopBlock()->getGCellPatterns();
  //auto &xgp = gCellPatterns.at(0);
  //auto &ygp = gCellPatterns.at(1);
  auto &cmap = design->getTopBlock()->getCMap();

  frPoint begin, end;
  frUInt4 xIndex1   = 0;
  frUInt4 yIndex1   = 0;
  frUInt4 layerNum1 = 0;
  frUInt4 xIndex2   = 0;
  frUInt4 yIndex2   = 0;
  frUInt4 layerNum2 = 0;
  for (auto &guide: guides) {
    guide->getPoints(begin, end);
    //xIndex1 = (begin.x() - xgp.getStartCoord()) / xgp.getSpacing();
    //yIndex1 = (begin.y() - ygp.getStartCoord()) / ygp.getSpacing();
    //xIndex2 = (end.x()   - xgp.getStartCoord()) / xgp.getSpacing();
    //yIndex2 = (end.y()   - ygp.getStartCoord()) / ygp.getSpacing();
    
    //xIndex1 = (xIndex1 >= xgp.getCount()) ? xgp.getCount() - 1 : xIndex1;
    //xIndex2 = (xIndex2 >= xgp.getCount()) ? xgp.getCount() - 1 : xIndex2;
    //yIndex1 = (yIndex1 >= ygp.getCount()) ? ygp.getCount() - 1 : yIndex1;
    //yIndex2 = (yIndex2 >= ygp.getCount()) ? ygp.getCount() - 1 : yIndex2;
    frPoint idx;
    design->getTopBlock()->getGCellIdx(begin, idx);
    xIndex1 = idx.x();
    yIndex1 = idx.y();
    design->getTopBlock()->getGCellIdx(end, idx);
    xIndex2 = idx.x();
    yIndex2 = idx.y();

    layerNum1 = guide->getBeginLayerNum();
    layerNum2 = guide->getEndLayerNum();
    bool isHLine = (xIndex1 == xIndex2) ? false : true;
    bool isVia   = (layerNum1 == layerNum2) ? false : true;
    bool isLocal = (xIndex1 == xIndex2 && yIndex1 == yIndex2) ? true : false;

    if (isVia) {
      cmap.setUpDemand(xIndex1, yIndex1, layerNum1, cmap.getUpDemand(xIndex1, yIndex1, layerNum1) + 1);
    } else {
      if (isLocal) {
        cmap.setLocalDemand(xIndex1, yIndex1, layerNum1, cmap.getLocalDemand(xIndex1, yIndex1, layerNum1) + 1);
      } else {
        cmap.setEdge2Demand(xIndex1, yIndex1, layerNum1, cmap.getEdge2Demand(xIndex1, yIndex1, layerNum1) + 1);
        cmap.setEdge1Demand(xIndex2, yIndex2, layerNum2, cmap.getEdge1Demand(xIndex2, yIndex2, layerNum2) + 1);
        if (isHLine) {
          for (auto i = xIndex1 + 1; i < xIndex2; i++) {
            cmap.setThroughDemand(i, yIndex1, layerNum1, cmap.getThroughDemand(i, yIndex1, layerNum1) + 1);
          }
        } else {
          for (auto i = yIndex1 + 1; i < yIndex2; i++) {
            cmap.setThroughDemand(xIndex1, i, layerNum1, cmap.getThroughDemand(xIndex1, i, layerNum1) + 1);
          }
        }
      }
    }
  }

  
  //if (VERBOSE > 0) {
  //  cout <<endl <<"end building cmap" <<endl;
  //}

  if (enableOutput) {
    design->printCMap();
  }

}

void io::Parser::writeGuideFile() {
  //auto &gp = design->getTopBlock()->getGCellPatterns();
  //auto &xgp = gp[0];
  //auto &ygp = gp[1];
  // use frCoord to prevent negative problems
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  ofstream outputGuide(OUTGUIDE_FILE.c_str());
  if (outputGuide.is_open()) {
    for (auto &net: design->topBlock->getNets()) {
      auto netName = net->getName();
      outputGuide << netName << endl;
      outputGuide << "(\n"; 
      for (auto &guide: net->getGuides()) {
        frPoint bp, ep;
        guide->getPoints(bp, ep);
        frPoint bpIdx, epIdx;
        design->getTopBlock()->getGCellIdx(bp, bpIdx);
        design->getTopBlock()->getGCellIdx(ep, epIdx);
        frBox bbox, ebox;
        design->getTopBlock()->getGCellBox(bpIdx, bbox);
        design->getTopBlock()->getGCellBox(epIdx, ebox);
        frLayerNum bNum = guide->getBeginLayerNum();
        frLayerNum eNum = guide->getEndLayerNum();
        // append unit guide in case of stacked via
        if (bNum != eNum) {
          auto startLayerName = tech->getLayer(bNum)->getName();
          //outputGuide << bp.x() - GCELLGRIDX / 2 << " " << bp.y() - GCELLGRIDY / 2 << " "
          //            << bp.x() + GCELLGRIDX / 2 << " " << bp.y() + GCELLGRIDY / 2 << " "
          //            << startLayerName <<".5" << endl;
          outputGuide << bbox.left()  << " " << bbox.bottom() << " "
                      << bbox.right() << " " << bbox.top()    << " "
                      << startLayerName <<".5" << endl;
        } else {
          auto layerName = tech->getLayer(bNum)->getName();
          //outputGuide << bp.x() - GCELLGRIDX / 2 << " " << bp.y() - GCELLGRIDY / 2 << " "
          //            << ep.x() + GCELLGRIDX / 2 << " " << ep.y() + GCELLGRIDY / 2 << " "
          //            << layerName << endl;
          outputGuide << bbox.left()  << " " << bbox.bottom() << " "
                      << ebox.right() << " " << ebox.top()    << " "
                      << layerName << endl;
        }
      }
      outputGuide << ")\n";
    }
  }
}
