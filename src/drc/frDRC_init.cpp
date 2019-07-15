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

void fr::DRCWorker::init() {
  bool enableOutput = false;
  // populateLayerConstraints();
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  sortObj();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  initNetRects();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  // general setup is done
  if (enableOutput) {
    duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);
    duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
    std::stringstream ss;
    ss   <<"time (SORTOBJ/INITNETRECTS) " <<time_span0.count() <<" " 
                                     <<time_span1.count() <<" "
                                     << std::endl;
    std::cout <<ss.str() << std::flush;
  }
}

// void fr::DRCWorker::populateLayerConstraints() {
//   for (auto &layer: design->getTech()->getLayers()) {
//     frLayerNum currLayerNum = layer->getLayerNum();
//     for (auto &constraint: layer->getConstraints()) {
//       layer2Constraints[currLayerNum].push_back(constraint.get());
//     }
//   }
// }

void fr::DRCWorker::initNetRects() {
  initSpecialNets();
  initNetRects_terms();
  initNetRects_connFigs();
  initNetRects_drConnFigs();
  initNetRects_blockages();
}

void fr::DRCWorker::initNetRects_drConnFigs(const std::vector<drConnFig*> &drConnFigs) {
  for (auto obj: drConnFigs) {
    DRCNet* drcNet = nullptr;
    switch (obj->typeId()) {
      case drcPathSeg:
      {
        int netId = -1;
        drPathSeg* pathSeg = static_cast<drPathSeg*>(obj);
        // get netId
        if (pathSeg->hasNet()) {
          if (net2Id.find(pathSeg->getNet()->getFrNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(pathSeg->getNet()->getFrNet())) {
            //   targetNetId = netId;
            // }
            (drcNets.back())->setSrc(pathSeg->getNet()->getFrNet());
            net2Id[pathSeg->getNet()->getFrNet()] = netId;
          } else {
            netId = net2Id[pathSeg->getNet()->getFrNet()];
          }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();
        frLayerNum layerNum = pathSeg->getLayerNum();
        frBox bbox;
        pathSeg->getBBox(bbox);
        Rectangle pathSegRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
        drcNet->addRouteLayerRect(layerNum, pathSegRect);
        break;
      }
      case drcVia:
      {
        int netId = -1;
        drVia* via = static_cast<drVia*>(obj);
        // get netId
        if (via->hasNet()) {
          if (net2Id.find(via->getNet()->getFrNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(via->getNet()->getFrNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(via->getNet());
            (drcNets.back())->setSrc(via->getNet()->getFrNet());
            net2Id[via->getNet()->getFrNet()] = netId;
          } else {
            netId = net2Id[via->getNet()->getFrNet()];
          }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape. TODO: handle cut layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();
        // push layer1 rect
        // double dbu = design->getTech()->getDBUPerUU();
        frLayerNum layerNum1 = via->getViaDef()->getLayer1Num();
        frTransform xform;
        via->getTransform(xform);
        for (auto &fig: via->getViaDef()->getLayer1Figs()) {
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          drcNet->addRouteLayerRect(layerNum1, bboxRect);
        }
        // push cut layer rect
        frLayerNum cutLayerNum = via->getViaDef()->getCutLayerNum();
        for (auto &fig: via->getViaDef()->getCutFigs()) {
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          drcNet->addRouteLayerRect(cutLayerNum, bboxRect);
          drcNet->addRouteLayerCutRect(cutLayerNum, bboxRect);
        }
        // push layer2 rect
        frLayerNum layerNum2 = via->getViaDef()->getLayer2Num();
        for (auto &fig: via->getViaDef()->getLayer2Figs()) {
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          drcNet->addRouteLayerRect(layerNum2, bboxRect);
        }
        break;
      }
      case drcPatchWire:
      {
        int netId = -1;
        drPatchWire* pwire = static_cast<drPatchWire*>(obj);
        // get netId
        if (pwire->hasNet()) {
          if (net2Id.find(pwire->getNet()->getFrNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(pwire->getNet()->getFrNet())) {
            //   targetNetId = netId;
            // }
            (drcNets.back())->setSrc(pwire->getNet()->getFrNet());
            net2Id[pwire->getNet()->getFrNet()] = netId;
          } else {
            netId = net2Id[pwire->getNet()->getFrNet()];
          }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();
        frLayerNum layerNum = pwire->getLayerNum();
        frBox bbox;
        pwire->getBBox(bbox);
        Rectangle patchWireRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
        drcNet->addRouteLayerRect(layerNum, patchWireRect);
        break;
      }
      default:
        std::cout << "Warning: unsupported connFig type in initNetRects_connFigs, skipped...\n";
    }
  }
}

void fr::DRCWorker::initNetRects_drConnFigs() {
  // double dbu = design->getTech()->getDBUPerUU();
  for (auto drNetIt = drNet2DrConnFigs.begin(); drNetIt != drNet2DrConnFigs.end(); ++drNetIt) {
    auto &drConnFigs = drNetIt->second;
    initNetRects_drConnFigs(drConnFigs);
  }
}

// void fr::DRCWorker::initNetRects_Incre_drConnFigs(drNet *net) {
//   for (auto obj: drConnFigs) {
//     DRCNet* drcNet = nullptr;
//     switch (obj->typeId()) {
//       case drcPathSeg:
//       {
//         drPathSeg* pathSeg = static_cast<drPathSeg*>(obj);
//         if (pathSeg->getNet() != net) {
//           continue;
//         }
//         frLayerNum layerNum = pathSeg->getLayerNum();
//         frBox bbox;
//         pathSeg->getBBox(bbox);
//         Rectangle pathSegRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
//         drNet2Layer2PolySet[net][layerNum] += pathSegRect;
//         break;
//       }
//       case drcVia:
//       {
//         drVia* via = static_cast<drVia*>(obj);
//         if (via->getNet() != net) {
//           continue;
//         }
//         // push layer1 rect
//         frLayerNum layerNum1 = via->getViaDef()->getLayer1Num();
//         frTransform xform;
//         via->getTransform(xform);
//         for (auto &fig: via->getViaDef()->getLayer1Figs()) {
//           frBox bbox;
//           fig->getBBox(bbox);
//           bbox.transform(xform);
//           Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
//           drNet2Layer2PolySet[net][layerNum1] += bboxRect;
//         }

//         // push cut layer rect
//         frLayerNum cutLayerNum = via->getViaDef()->getCutLayerNum();
//         for (auto &fig: via->getViaDef()->getCutFigs()) {
//           frBox bbox;
//           fig->getBBox(bbox);
//           bbox.transform(xform);
//           Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
//           drNet2Layer2PolySet[net][cutLayerNum] += bboxRect;
//         }

//         // push layer2 rect
//         frLayerNum layerNum2 = via->getViaDef()->getLayer2Num();
//         for (auto &fig: via->getViaDef()->getLayer2Figs()) {
//           frBox bbox;
//           fig->getBBox(bbox);
//           bbox.transform(xform);
//           Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
//           drNet2Layer2PolySet[net][layerNum2] += bboxRect;
//         }
//         break;
//       }
//       case drcPatchWire:
//       {
//         drPatchWire* pwire = static_cast<drPatchWire*>(obj);
//         if (pwire->getNet() != net) {
//           continue;
//         }       
//         frLayerNum layerNum = pwire->getLayerNum();
//         frBox bbox;
//         pwire->getBBox(bbox);
//         Rectangle patchWireRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
//         drNet2Layer2PolySet[net][layerNum] += patchWireRect;
//         break;
//       }

//       default:
//         std::cout << "Warning: unsupported connFig type in initIncre_drConnFigs, skipped...\n";
//     }
//   }
// }


void fr::DRCWorker::initSpecialNets() {
  for (int i = 0; i < netCnt; ++i) {
    drcNets.push_back(std::make_unique<DRCNet>(i));
  }
}

void fr::DRCWorker::initNetRects_terms() {
  for (auto obj: terms) {
    DRCNet* drcNet = nullptr;
    switch (obj->typeId()) {
      case frcInstTerm:
      {
        int netId = -1;
        frInstTerm *instTerm = static_cast<frInstTerm*>(obj);
        frInst* inst = instTerm->getInst();
        frBox mbox;
        frTransform xform;
        inst->getUpdatedXform(xform);
        auto term = instTerm->getTerm();
        // get netId
        if (instTerm->hasNet()) {
          if (net2Id.find(instTerm->getNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(instTerm->getNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(instTerm->getNet());
            (drcNets.back())->setSrc(instTerm->getNet());
            net2Id[instTerm->getNet()] = netId;
          } else {
            netId = net2Id[instTerm->getNet()];
          }
        } else {
          if (term->getType() == frTermEnum::frcNormalTerm || 
              term->getType() == frTermEnum::frcClockTerm) {
            // netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(instTerm)) {
            //   targetNetId = netId;
            // }
            (drcNets.back())->setSrc(instTerm);
            term2Id[term] = netId;
          } else if (term->getType() == frTermEnum::frcPowerTerm) {
            netId = static_cast<int>(drcSpecialNetIdEnum::FLOATPOWER);
          } else if (term->getType() == frTermEnum::frcGroundTerm) {
            netId = static_cast<int>(drcSpecialNetIdEnum::FLOATGROUND);
          }
        }

        // push layer shapes
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_terms\n";
          continue;
        }
        drcNet = drcNets[netId].get();

        for (auto &pin: term->getPins()) {
          auto pinLayer2PolySet = pin->getLayer2PolySet();
          for (auto layerIt = pinLayer2PolySet.begin(); layerIt != pinLayer2PolySet.end(); ++layerIt) {
            auto &layerNum = layerIt->first;
            auto &polySet = layerIt->second;
            for (auto &poly: polySet) {
              std::vector<Point> transformedPoints;
              for (auto it = begin_points(poly); it != end_points(poly); ++it) {
                frPoint tmpPt((*it).x(), (*it).y());
                tmpPt.transform(xform);
                transformedPoints.push_back(Point(tmpPt.x(), tmpPt.y()));
              }
              Polygon tmpPoly;
              set_points(tmpPoly, transformedPoints.begin(), transformedPoints.end());
              drcNet->addFixedLayerPoly(layerNum, tmpPoly);
            }
          }
        }
        break;
      }
      case frcTerm:
      {
        int netId = -1;
        frTerm* term = static_cast<frTerm*>(obj);
        // get netId
        if (term->hasNet()) {
          if (net2Id.find(term->getNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(term->getNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(term->getNet());
            (drcNets.back())->setSrc(term->getNet());
            net2Id[term->getNet()] = netId;
          } else {
            netId = net2Id[term->getNet()];
          }
        } else {
          if (term->getType() == frTermEnum::frcNormalTerm || 
              term->getType() == frTermEnum::frcClockTerm) {
            // netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(term)) {
            //   targetNetId = netId;
            // }
            (drcNets.back())->setSrc(term);
            term2Id[term] = netId;
          } else if (term->getType() == frTermEnum::frcPowerTerm) {
            netId = static_cast<int>(drcSpecialNetIdEnum::FLOATPOWER);
          } else if (term->getType() == frTermEnum::frcGroundTerm) {
            netId = static_cast<int>(drcSpecialNetIdEnum::FLOATGROUND);
          }
        }

        // push layer shapes
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_terms\n";
          continue;
        }
        drcNet = drcNets[netId].get();

        for (auto &pin: term->getPins()) {
          auto pinLayer2PolySet = pin->getLayer2PolySet();
          for (auto layerIt = pinLayer2PolySet.begin(); layerIt != pinLayer2PolySet.end(); ++layerIt) {
            auto &layerNum = layerIt->first;
            auto &polySet = layerIt->second;
            for (auto &poly: polySet) {
              drcNet->addFixedLayerPoly(layerNum, poly);
            }
          }
        }
        break;
      }
      default:
        std::cout << "Error: unexpected type in initNetRects_terms\n";
    }
  }
}

void fr::DRCWorker::initNetRects_connFigs() {
  // double dbu = design->getTech()->getDBUPerUU();
  for (auto obj: connFigs) {
    DRCNet* drcNet = nullptr;
    switch (obj->typeId()) {
      case frcPathSeg:
      {
        int netId = -1;
        frPathSeg* pathSeg = static_cast<frPathSeg*>(obj);
        // get netId
        if (pathSeg->hasNet()) {
          if (net2Id.find(pathSeg->getNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(pathSeg->getNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(pathSeg->getNet());
            (drcNets.back())->setSrc(pathSeg->getNet());
            net2Id[pathSeg->getNet()] = netId;
          } else {
            netId = net2Id[pathSeg->getNet()];
          }
        } else if (pathSeg->hasPin()) {
          for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
            if (netId != -1) {
              break;
            }
            auto term = mapIt->first;
            for (auto &pin: term->getPins()) {
              auto pinPtr = pin.get();
              if (pinPtr == pathSeg->getPin()) {
                netId = mapIt->second;
                break;
              }
            }
          }
          // if (netId == -1) {
          //   netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
          // }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();
        
        frLayerNum layerNum = pathSeg->getLayerNum();
        frBox bbox;
        pathSeg->getBBox(bbox);
        Rectangle pathSegRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
        // std::cout << "  add pathseg bbox (" << bbox.left() / dbu<< ", " << bbox.bottom() / dbu
        //           << ") -- (" << bbox.right() / dbu << ", " << bbox.top() / dbu << ") from net " << netId << "\n";
        drcNet->addRouteLayerRect(layerNum, pathSegRect);
        // std::cout << "  added pathSeg\n" << std::flush;

        break;
      }
      case frcVia:
      {
        int netId = -1;
        frVia* via = static_cast<frVia*>(obj);
        // get netId
        if (via->hasNet()) {
          if (net2Id.find(via->getNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(via->getNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(via->getNet());
            (drcNets.back())->setSrc(via->getNet());
            net2Id[via->getNet()] = netId;
          } else {
            netId = net2Id[via->getNet()];
          }
        } else if (via->hasPin()) {
          for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
            if (netId != -1) {
              break;
            }
            auto term = mapIt->first;
            for (auto &pin: term->getPins()) {
              auto pinPtr = pin.get();
              if (pinPtr == via->getPin()) {
                netId = mapIt->second;
                break;
              }
            }
          }
          if (netId == -1) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            auto term = via->getPin()->getTerm();
            term2Id[term] = netId;
          }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape. TODO: handle cut layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();

        // push layer1 rect
        // double dbu = design->getTech()->getDBUPerUU();
        frLayerNum layerNum1 = via->getViaDef()->getLayer1Num();
        frTransform xform;
        via->getTransform(xform);
        for (auto &fig: via->getViaDef()->getLayer1Figs()) {
          
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          // std::cout << "  add to net" << netId << " layer " << layerNum1 << " (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu
          //           << ") - (" << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
          drcNet->addRouteLayerRect(layerNum1, bboxRect);
        }

        // push cut layer rect
        frLayerNum cutLayerNum = via->getViaDef()->getCutLayerNum();
        for (auto &fig: via->getViaDef()->getCutFigs()) {
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          drcNet->addRouteLayerRect(cutLayerNum, bboxRect);
          drcNet->addRouteLayerCutRect(cutLayerNum, bboxRect);
        }

        // push layer2 rect
        frLayerNum layerNum2 = via->getViaDef()->getLayer2Num();
        for (auto &fig: via->getViaDef()->getLayer2Figs()) {
          frBox bbox;
          fig->getBBox(bbox);
          bbox.transform(xform);
          Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
          // std::cout << "  add to net" << netId << " layer " << layerNum2 << " (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu
          //           << ") - (" << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
          drcNet->addRouteLayerRect(layerNum2, bboxRect);
        }
        break;
      }
      case frcPatchWire:
      {
        int netId = -1;
        frPatchWire* pwire = static_cast<frPatchWire*>(obj);
        // get netId
        if (pwire->hasNet()) {
          if (net2Id.find(pwire->getNet()) == net2Id.end()) {
            netId = drcNets.size();
            drcNets.push_back(std::make_unique<DRCNet>(netId));
            // if (target && target == static_cast<frBlockObject*>(pwire->getNet())) {
            //   targetNetId = netId;
            // }
            // (drcNets.back())->setNet(pwire->getNet());
            (drcNets.back())->setSrc(pwire->getNet());
            net2Id[pwire->getNet()] = netId;
          } else {
            netId = net2Id[pwire->getNet()];
          }
        } else if (pwire->hasPin()) {
          for (auto mapIt = term2Id.begin(); mapIt != term2Id.end(); ++mapIt) {
            if (netId != -1) {
              break;
            }
            auto term = mapIt->first;
            for (auto &pin: term->getPins()) {
              auto pinPtr = pin.get();
              if (pinPtr == pwire->getPin()) {
                netId = mapIt->second;
                break;
              }
            }
          }
          // if (netId == -1) {
          //   netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
          // }
        } else {
          netId = static_cast<int>(drcSpecialNetIdEnum::FLOATSIGNAL);
        }
        // push layer shape
        if (netId == -1) {
          std::cout << "Error: unexpected netId in initNetRects_connFigs\n";
          continue;
        }
        drcNet = drcNets[netId].get();
        
        frLayerNum layerNum = pwire->getLayerNum();
        frBox bbox;
        pwire->getBBox(bbox);
        Rectangle patchWireRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
        // std::cout << "  add pathseg bbox (" << bbox.left() / dbu<< ", " << bbox.bottom() / dbu
        //           << ") -- (" << bbox.right() / dbu << ", " << bbox.top() / dbu << ") from net " << netId << "\n";
        drcNet->addRouteLayerRect(layerNum, patchWireRect);
        // std::cout << "  added pwire\n" << std::flush;

        break;
      }
      default:
        std::cout << "Warning: unsupported connFig type in initNetRects_connFigs, skipped...\n";
    }
  }
}

void fr::DRCWorker::initNetRects_blockages() {
  for (auto obj: blockages) {
    DRCNet *drcNet = nullptr;
    switch (obj->typeId()) {
      case frcLayerBlockage:
      {
        frLayerBlockage* layerBlockage = static_cast<frLayerBlockage*>(obj);
        int netId = static_cast<int>(drcSpecialNetIdEnum::OBS);
        // push layer shapes
        drcNet = drcNets[netId].get();
        frLayerNum layerNum = layerBlockage->getLayerNum();
        auto blockageFrPoints = layerBlockage->getPoints();
        std::vector<Point> blockagePoints;
        for (auto pt: blockageFrPoints) {
          blockagePoints.push_back(Point(pt.x(), pt.y()));
        }
        Polygon blockagePoly;
        set_points(blockagePoly, blockagePoints.begin(), blockagePoints.end());
        drcNet->addFixedLayerPoly(layerNum, blockagePoly);
        // add cut obs to cut layer rect
        if (design->getTech()->getLayer(layerNum)->getType() == frLayerTypeEnum::CUT) {
          std::vector<Rectangle> maxRects;
          PolygonSet tempPS;
          tempPS += blockagePoly;
          get_max_rectangles(maxRects, tempPS);
          for (auto &maxRect: maxRects) {
            drcNet->addRouteLayerCutRect(layerNum, maxRect);
          }
        }
        break;
      }
      case frcInstBlockage:
      {
        // std::cout << "adding frInstBlockage\n";
        frBlockage* blockage = (static_cast<frInstBlockage*>(obj))->getBlockage();
        if (blockage->typeId() != frcLayerBlockage) {
          std::cout << "Error: unexpected blockage type in initNetRects_blockages\n";
          break;
        }
        frInst* inst = (static_cast<frInstBlockage*>(obj))->getInst();
        frTransform xform;
        inst->getUpdatedXform(xform);
        frLayerBlockage* layerBlockage = static_cast<frLayerBlockage*>(blockage);
        int netId = static_cast<int>(drcSpecialNetIdEnum::OBS);
        // push layer shapes
        drcNet = drcNets[netId].get();
        frLayerNum layerNum = layerBlockage->getLayerNum();
        auto blockageFrPoints = layerBlockage->getPoints();
        std::vector<Point> blockagePoints;
        for (auto pt: blockageFrPoints) {
          frPoint tmpPt = pt;
          tmpPt.transform(xform);
          blockagePoints.push_back(Point(tmpPt.x(), tmpPt.y()));
        }
        Polygon blockagePoly;
        set_points(blockagePoly, blockagePoints.begin(), blockagePoints.end());
        drcNet->addFixedLayerPoly(layerNum, blockagePoly);
        // add cut obs to cut layer rect
        if (design->getTech()->getLayer(layerNum)->getType() == frLayerTypeEnum::CUT) {
          std::vector<Rectangle> maxRects;
          PolygonSet tempPS;
          tempPS += blockagePoly;
          get_max_rectangles(maxRects, tempPS);
          for (auto &maxRect: maxRects) {
            drcNet->addRouteLayerCutRect(layerNum, maxRect);
          }
        }
        break;
      }
      default:
        std::cout << "Warning: unsupported blockage type in initNetRects_blockages, skipped...\n";
    }
  }
}

void fr::DRCWorker::sortObj() {
  for (auto obj: objs) {
    switch (obj->typeId()) {
      case frcTerm:
        terms.push_back(obj);
        break;
      case frcInstTerm:
        terms.push_back(obj);
        break;
      case frcVia:
        connFigs.push_back(static_cast<frConnFig*>(obj));
        break;
      case frcPathSeg:
        connFigs.push_back(static_cast<frConnFig*>(obj));
        break;
      case frcPatchWire:
        connFigs.push_back(static_cast<frConnFig*>(obj));
        break;
      case frcLayerBlockage:
        blockages.push_back(static_cast<frBlockage*>(obj));
      case frcInstBlockage:
        blockages.push_back(static_cast<frInstBlockage*>(obj));
      default:
        continue;
    }
  }
}

void fr::DRCWorker::addDRNets(const std::vector<std::unique_ptr<drNet> > &nets) {
  for (auto &net: nets) {
    addDRNet(net.get());
  }
}

void fr::DRCWorker::addDRNet(drNet* net) {
  for (auto &conn: net->getExtConnFigs()) {
    addDRConnFig(conn.get());
  }
  for (auto &conn: net->getRouteConnFigs()) {
    addDRConnFig(conn.get());
  }
}

// assuming the route objects of the net have changed
// need to update DRC for the drNet
bool fr::DRCWorker::updateDRNet(drNet *net) {
  int netId = getNetId(net);
  frNet *ownerNet = net->getFrNet();
  if (netId < 0 || netId >= (int)drcNets.size()) {
    return false;
  }
  auto drcNet = drcNets[netId].get();
  setupCutRectRTree_net(drcNet, false);
  setupMergedMaxRectRTree_net(drcNet, false);
  setupEdgeRTree_net(drcNet, false);
  drcNet->resetDRCEdges();
  // remove old drConnFigs
  if (drNet2DrConnFigs.find(net) != drNet2DrConnFigs.end()) {
    drNet2DrConnFigs[net].clear();
  }
  // add new drConnFigs
  addDRNet(net);
  // re-initialize for the given net (need to do this for all drNets belonging to the same frNet)
  for (auto netIt = drNet2DrConnFigs.begin(); netIt != drNet2DrConnFigs.end(); ++netIt) {
    auto currDrNet = netIt->first;
    if (currDrNet->getFrNet() != ownerNet) {
      continue;
    } else {
      initNetRects_drConnFigs(drNet2DrConnFigs[currDrNet]);
    }
  }
  // init done
  setupEdges_net(drcNet);
  setupEdgeRTree_net(drcNet, true);
  setupMergedMaxRectRTree_net(drcNet, true);
  setupCutRectRTree_net(drcNet, true);
  return true;
}

void fr::DRCWorker::addDRConnFig(drConnFig* in) {
  switch (in->typeId()) {
    case drcVia:
    {
      auto via = static_cast<drVia*>(in);
      auto net = via->getNet();
      // drConnFigs.push_back(via);
      drNet2DrConnFigs[net].push_back(via);
      break;
    }
    case drcPathSeg:
    {
      auto pathSeg = static_cast<drPathSeg*>(in);
      auto net = pathSeg->getNet();
      // drConnFigs.push_back(pathSeg);
      drNet2DrConnFigs[net].push_back(pathSeg);
      break;
    }
    case drcPatchWire:
    {
      auto pwire = static_cast<drPatchWire*>(in);
      auto net = pwire->getNet();
      // drConnFigs.push_back(static_cast<drPatchWire*>(in));
      drNet2DrConnFigs[net].push_back(pwire);
      break;
    }
    default:
      std::cout << "Error: unexpected in addDRConnFig\n";
  }
}

