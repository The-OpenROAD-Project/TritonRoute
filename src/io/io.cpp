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
#include <fstream>
#include <sstream>

#include "global.h"
#include "io/io.h"
#include "db/tech/frConstraint.h"

//#include <boost/polygon/polygon.hpp>

using namespace std;
using namespace fr;
//using namespace boost::polygon::operators;

int io::Parser::getDefVias(defrCallbackType_e type, defiVia* via, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != defrViaCbkType)) {
    cout <<"Type is not defrViaCbkType!" <<endl;
    exit(1);
  }

  if (enableOutput) {
    cout <<"- " <<via->name() <<endl;
    //cout <<"  numLayers = " <<via->numLayers() <<endl;
  }
  // viaRule defined via
  if (via->hasViaRule()) {
    char* viaRuleName;
    char* botLayer;
    char* cutLayer;
    char* topLayer;
    int xSize, ySize, xCutSpacing, yCutSpacing, xBotEnc, yBotEnc, xTopEnc, yTopEnc;
    via->viaRule(&viaRuleName, &xSize, &ySize, &botLayer, &cutLayer, &topLayer,
                 &xCutSpacing, &yCutSpacing, &xBotEnc, &yBotEnc, &xTopEnc, &yTopEnc);
    int xOffset = 0;
    int yOffset = 0;
    if (via->hasOrigin()) {
      via->origin(&xOffset, &yOffset);
    }
    int xBotOffset = 0;
    int yBotOffset = 0;
    int xTopOffset = 0;
    int yTopOffset = 0;
    if (via->hasOffset()) {
      via->offset(&xBotOffset, &yBotOffset, &xTopOffset, &yTopOffset);
    }
    int numCutRows = 1;
    int numCutCols = 1;
    if (via->hasRowCol()) {
      via->rowCol(&numCutRows, &numCutCols);
    }
    if (enableOutput) {
      cout <<" + VIARULE "    <<viaRuleName <<endl;
      //cout <<" + CUTSIZE " <<xSize * 1.0 / ((io::Parser*)data)->tmpBlock->getDBUPerUU() <<" " 
      //                     <<ySize * 1.0 / ((io::Parser*)data)->tmpBlock->getDBUPerUU() <<endl;
      cout <<" + CUTSIZE "    <<xSize       <<" " <<ySize       <<endl;
      cout <<" + LAYERS "     <<botLayer    <<" " <<cutLayer    <<" "   <<topLayer <<endl;
      cout <<" + CUTSPACING " <<xCutSpacing <<" " <<yCutSpacing <<endl;
      cout <<" + ENCLOSURE "  <<xBotEnc     <<" " <<yBotEnc     <<" "
                              <<xTopEnc     <<" " <<yTopEnc     <<endl;
      if (via->hasRowCol()) {
        cout <<" + ROWCOL " <<numCutRows <<" " <<numCutCols <<endl;
      }
      if (via->hasOrigin()) {
        cout <<" + ORIGIN " <<xOffset <<" " <<yOffset <<endl;
      }
      if (via->hasOffset()) {
        cout <<" + OFFSET " <<xBotOffset <<" " <<yBotOffset <<" " <<xTopOffset <<" " <<yTopOffset <<endl;
      }
      cout <<" ;" <<endl;

    }

    // create cut figs
    frLayerNum cutLayerNum = 0;
    if (((io::Parser*)data)->tech->name2layer.find(cutLayer) == ((io::Parser*)data)->tech->name2layer.end()) {
      cout <<"Error: cannot find cut layer" <<endl;
      exit(1);
    } else {
      cutLayerNum = ((io::Parser*)data)->tech->name2layer.find(cutLayer)->second->getLayerNum();
    }
    frLayerNum botLayerNum = 0;
    if (((io::Parser*)data)->tech->name2layer.find(botLayer) == ((io::Parser*)data)->tech->name2layer.end()) {
      cout <<"Error: cannot find bot layer" <<endl;
      exit(1);
    } else {
      botLayerNum = ((io::Parser*)data)->tech->name2layer.find(botLayer)->second->getLayerNum();
    }
    // create cut figs
    frLayerNum topLayerNum = 0;
    if (((io::Parser*)data)->tech->name2layer.find(topLayer) == ((io::Parser*)data)->tech->name2layer.end()) {
      cout <<"Error: cannot find top layer" <<endl;
      exit(1);
    } else {
      topLayerNum = ((io::Parser*)data)->tech->name2layer.find(topLayer)->second->getLayerNum();
    }

    frCoord currX = 0;
    frCoord currY = 0;
    vector<unique_ptr<frShape> > cutFigs;
    for (int i = 0; i < numCutRows; i++) {
      currX = 0;
      for (int j = 0; j < numCutCols; j++) {
        auto rect = make_unique<frRect>();
        frBox tmpBox(currX, currY, currX + xSize, currY + ySize);
        rect->setBBox(tmpBox);
        rect->setLayerNum(cutLayerNum);
        //cout <<"cutFig (" <<currX <<", " <<currY <<") (" <<currX+xSize <<"," <<currY+ySize <<") " <<cutLayer <<endl;

        unique_ptr<frShape> tmp(std::move(rect));
        cutFigs.push_back(std::move(tmp));

        currX += xSize + xCutSpacing;
      }
      currY += ySize + yCutSpacing;
    }
    currX -= xCutSpacing; // max cut X
    currY -= yCutSpacing; // max cut Y
    //cout <<"max x/y " <<currX <<" " <<currY <<endl;

    frTransform cutXform(-currX / 2 + xOffset, -currY / 2 + yOffset);
    for (auto &uShape: cutFigs) {
      auto rect = static_cast<frRect*>(uShape.get());
      //cout <<"orig " <<*rect <<endl;
      rect->move(cutXform);
      //cout <<"move " <<*rect <<endl;
    }

    unique_ptr<frShape> uBotFig = make_unique<frRect>();
    auto botFig = static_cast<frRect*>(uBotFig.get());
    unique_ptr<frShape> uTopFig = make_unique<frRect>();
    auto topFig = static_cast<frRect*>(uTopFig.get());

    frBox botBox(0 - xBotEnc, 0 - yBotEnc, currX + xBotEnc, currY + yBotEnc);
    frBox topBox(0 - xTopEnc, 0 - yTopEnc, currX + xTopEnc, currY + yTopEnc);

    frTransform botXform(-currX / 2 + xOffset + xBotOffset, -currY / 2 + yOffset + yBotOffset);
    frTransform topXform(-currX / 2 + xOffset + xTopOffset, -currY / 2 + yOffset + yTopOffset);
    botBox.transform(botXform);
    topBox.transform(topXform);

    botFig->setBBox(botBox);
    topFig->setBBox(topBox);
    botFig->setLayerNum(botLayerNum);
    topFig->setLayerNum(topLayerNum);

    // create via
    auto viaDef = make_unique<frViaDef>(via->name());
    viaDef->addLayer1Fig(uBotFig);
    viaDef->addLayer2Fig(uTopFig);
    for (auto &uShape: cutFigs) {
      viaDef->addCutFig(uShape);
    }
    ((io::Parser*)data)->tech->addVia(viaDef);
  // RECT defined via
  } else {
    if (via->numPolygons()) {
      cout <<"Error: unsupport polygon in def via" <<endl;
      exit(1);
    }
    char* layerName;
    int xl;
    int yl;
    int xh;
    int yh;
    map<frLayerNum, set<int> > lNum2Int;
    for (int i = 0; i < via->numLayers(); ++i) {
      via->layer(i, &layerName, &xl, &yl, &xh, &yh);
      if (((io::Parser*)data)->tech->name2layer.find(layerName) == ((io::Parser*)data)->tech->name2layer.end()) {
        if (VERBOSE > -1) {
          cout <<"Warning: layer " <<layerName <<" is skipiped for " <<via->name() <<endl;
        }
        return 0;
      }
      auto layerNum = ((io::Parser*)data)->tech->name2layer.at(layerName)->getLayerNum();
      lNum2Int[layerNum].insert(i);
      //cout <<"layerNum " <<layerNum <<" i " <<i <<endl;
    }
    if ((int)lNum2Int.size() != 3) {
      if (VERBOSE > -1) {
        cout <<"Error: unsupported via" <<endl;
      }
      exit(1);
    }
    if (lNum2Int.begin()->first + 2 != (--lNum2Int.end())->first) {
      if (VERBOSE > -1) {
        cout <<"Error: non-consecutive layers" <<endl;
      }
      exit(1);
    }
    auto viaDef = make_unique<frViaDef>(via->name());
    int cnt = 0;
    for (auto &[layerNum, intS]: lNum2Int) {
      for (auto i: intS) {
        via->layer(i, &layerName, &xl, &yl, &xh, &yh);
        if (enableOutput) {
          cout <<" + RECT " <<layerName <<" ( " <<xl <<" " <<yl <<" ) ( " <<xh <<" " <<yh <<" )" <<endl;
        }
        unique_ptr<frRect> pinFig = make_unique<frRect>();
        pinFig->setBBox(frBox(xl, yl, xh, yh));
        pinFig->setLayerNum(layerNum);
        unique_ptr<frShape> tmp(std::move(pinFig));
        switch(cnt) {
          case 0 :
            viaDef->addLayer1Fig(tmp);
            break;
          case 1 :
            viaDef->addCutFig(tmp);
            break;
          default:
            viaDef->addLayer2Fig(tmp);
            break;
        }
      }
      //for (int j = 0; j < via->numPolygons(i); ++j) {
      //  if (enableOutput) {
      //    cout <<"    POLYGON"; 
      //  }
      //  vector<frPoint> tmpPoints;
      //  for (int k = 0; k < via->getPolygon(i, j).numPoints; k++) {
      //    frCoord x = round(via->getPolygon(i, j).x[k] * ((io::Parser*)data)->tech->getDBUPerUU());
      //    frCoord y = round(via->getPolygon(i, j).y[k] * ((io::Parser*)data)->tech->getDBUPerUU());
      //    tmpPoints.push_back(frPoint(x, y));
      //    if (enableOutput) {
      //       cout <<" " <<x * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
      //                  <<y * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
      //    }
      //  }
      //  unique_ptr<frPolygon> pinFig = make_unique<frPolygon>();
      //  pinFig->setPoints(tmpPoints);
      //  pinFig->setLayerNum(layerNum);
      //  unique_ptr<frShape> tmp(std::move(pinFig));
      //  if (enableOutput) {
      //    cout <<" ;" <<endl;
      //  }
      //  switch(cnt) {
      //    case 0 :
      //      viaDef->addLayer1Fig(tmp);
      //      break;
      //    case 1 :
      //      viaDef->addCutFig(tmp);
      //      break;
      //    default:
      //      viaDef->addLayer2Fig(tmp);
      //      break;
      //  }
      //}
      cnt++;
    }
    if (enableOutput) {
      cout <<" ;" <<endl;
    }
    ((io::Parser*)data)->tech->addVia(viaDef);
  }

  return 0;
}

int io::Parser::getDefComponents(defrCallbackType_e type, defiComponent* comp, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != defrComponentCbkType)) {
    cout <<"Type is not defrComponentCbkType!" <<endl;
    exit(1);
  }

  if (((io::Parser*)data)->design->name2refBlock.find(comp->name()) == ((io::Parser*)data)->design->name2refBlock.end()) {
    if (VERBOSE > -1) {
      cout <<"Error: library cell not found!" <<endl;
    }
    exit(1);
  }
  
  if (enableOutput) {
    cout <<"- " <<comp->id() <<" " <<comp->name() <<" + STATUS ( " <<comp->placementX()
         <<" " <<comp->placementY() <<" ) " <<comp->placementOrient() <<endl;
  }

  
  auto uInst = make_unique<frInst>();
  auto tmpInst = uInst.get();
  tmpInst->setId(((io::Parser*)data)->numInsts);
  ((io::Parser*)data)->numInsts++;
  tmpInst->setName(comp->id());
  tmpInst->setOrigin(frPoint(comp->placementX(), comp->placementY()));
  tmpInst->setOrient(frOrientEnum(comp->placementOrient()));
  tmpInst->setRefBlock(((io::Parser*)data)->design->name2refBlock.at(comp->name()));
  for (auto &uTerm: tmpInst->getRefBlock()->getTerms()) {
    auto term = uTerm.get();
    unique_ptr<frInstTerm> instTerm = make_unique<frInstTerm>();
    instTerm->setId(((io::Parser*)data)->numTerms);
    ((io::Parser*)data)->numTerms++;
    instTerm->addToInst(tmpInst);
    instTerm->addTerm(term);
    tmpInst->addInstTerm(instTerm);
  }
  for (auto &uBlk: tmpInst->getRefBlock()->getBlockages()) {
    auto blk = uBlk.get();
    unique_ptr<frInstBlockage> instBlk = make_unique<frInstBlockage>();
    instBlk->setId(((io::Parser*)data)->numBlockages);
    ((io::Parser*)data)->numBlockages++;
    instBlk->addToInst(tmpInst);
    instBlk->addBlockage(blk);
    tmpInst->addInstBlockage(instBlk);
  }

  if (((io::Parser*)data)->tmpBlock->name2inst.find(comp->id()) != ((io::Parser*)data)->tmpBlock->name2inst.end()) {
    if (VERBOSE > -1) {
      cout <<"Error: same cell name!" <<endl;
    }
    exit(1);
  }
  ((io::Parser*)data)->tmpBlock->addInst(uInst);
  if (((io::Parser*)data)->tmpBlock->insts.size() < 100000) {
    if (((io::Parser*)data)->tmpBlock->insts.size() % 10000 == 0) {
      cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->insts.size() <<" components" <<endl;
    }
  } else {
    if (((io::Parser*)data)->tmpBlock->insts.size() % 100000 == 0) {
      cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->insts.size() <<" components" <<endl;
    }
  }

  return 0;
}

int io::Parser::getDefString(defrCallbackType_e type, const char* str, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type == defrDesignStartCbkType)) {
    auto &tmpBlock = ((io::Parser*)data)->tmpBlock;
    tmpBlock = make_unique<frBlock>();
    tmpBlock->setName(string(str));
    tmpBlock->trackPatterns.clear();
    tmpBlock->trackPatterns.resize(((io::Parser*)data)->tech->layers.size());
    if (enableOutput) {
      cout <<"DESIGN " <<tmpBlock->getName() <<" ;" <<endl;
    }
  }
  return 0;
}

int io::Parser::getDefVoid(defrCallbackType_e type, void* variable, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type == defrDesignEndCbkType)) {
    ((io::Parser*)data)->tmpBlock->setId(0);
    ((io::Parser*)data)->design->setTopBlock(((io::Parser*)data)->tmpBlock);
    if (enableOutput) {
      cout <<"END DESIGN" <<endl;
    }
  }
  return 0;
}

int io::Parser::getDefDieArea(defrCallbackType_e type, defiBox* box, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != defrDieAreaCbkType)) {
    cout <<"Type is not defrDieAreaCbkType!" <<endl;
    exit(1);
  }
  //((io::Parser*)data)->tmpBlock->setBBox(frBox(box->xl(), box->yl(), box->xh(), box->yh()));
  vector<frBoundary> bounds;
  frBoundary bound;
  vector<frPoint> points;
  points.push_back(frPoint(box->xl(), box->yl()));
  points.push_back(frPoint(box->xh(), box->yl()));
  points.push_back(frPoint(box->xh(), box->yh()));
  points.push_back(frPoint(box->xl(), box->yh()));
  bound.setPoints(points);
  bounds.push_back(bound);
  ((io::Parser*)data)->tmpBlock->setBoundaries(bounds);
  if (enableOutput) {
    cout <<"DIEAREA ( " <<box->xl() <<" " <<box->yl() <<" ) ( " <<box->xh() <<" " <<box->yh() <<" ) ;" <<endl;
  }
  return 0;
}

//int FlexRoute::getDefInteger(defrCallbackType_e type, int number, defiUserData data) {
//  if (type == defrComponentStartCbkType) {
//    ((FlexRoute*)data)->comps = vector<Component>(((FlexRoute*)data)->numComps);
//  }
//  if (type == defrNetStartCbkType) {
//    ((FlexRoute*)data)->numNets = number;
//    ((FlexRoute*)data)->nets = vector<Net>(((FlexRoute*)data)->numNets);
//  }
//  if (type == defrSNetStartCbkType) {
//    cout << "special net count = " << number << endl;
//    ((FlexRoute*)data)->numSNets = number;
//    // ((FlexRoute*)data)->snets = vector<SNet>(number);
//  }
//  return 0;
//}

int io::Parser::getDefNets(defrCallbackType_e type, defiNet* net, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  bool isSNet = false;

  if (type != defrNetCbkType && type != defrSNetCbkType) {
    cout <<"Type is not defr(S)NetCbkType!" <<endl;
    exit(1);
  }

  if (type == defrSNetCbkType) {
    //cout <<"Type is Special Net: " <<net->name() <<endl;
    //exit(1);
    isSNet = true;
  }

  unique_ptr<frNet> uNetIn = make_unique<frNet>(net->name());
  auto netIn = uNetIn.get();
  netIn->setId(((io::Parser*)data)->numNets);
  ((io::Parser*)data)->numNets++;
  if (enableOutput) {
    cout <<"- " <<net->name();
  }
  for (int i = 0; i < net->numConnections(); i++) {
    if (enableOutput) {
      if (i % 4 == 0) {
        cout <<endl <<" ";
      }
      cout <<" ( " <<net->instance(i) <<" " <<net->pin(i) <<" )";
    }
    
    if (!strcmp(net->instance(i), "PIN")) {
      // IOs
      if (((io::Parser*)data)->tmpBlock->name2term.find(net->pin(i)) == ((io::Parser*)data)->tmpBlock->name2term.end()) {
        if (VERBOSE > -1) {
          cout <<"Error: term not found!" <<endl;
        }
        exit(1);
      }
      auto term = ((io::Parser*)data)->tmpBlock->name2term[net->pin(i)]; // frTerm*
      term->addToNet(netIn);
      netIn->addTerm(term);
      // add physical, absolute coordinate pins
      //for (auto &pin: term->getPins()) {
      //  shared_ptr<frPin> newPin = make_shared<frPin>(*pin);
      //  for (auto &pinFig: newPin->getFigs()) {
      //    if (pinFig->typeId() == frcRect || pinFig->typeId() == frcPolygon) {
      //      pinFig->addToNet(netIn);
      //    } else {
      //      cout <<"Error: currently only support rect and polygon with net" <<endl;
      //    }
      //  }
      //  //newPin->addToNet(netIn); // currently only support rect and polygon
      //  netIn->addPin(newPin);
      //}
    } else {
      // Instances
      if (!strcmp(net->instance(i), "*")) {
        for (auto &inst: ((io::Parser*)data)->tmpBlock->getInsts()) {
          for (auto &uInstTerm: inst->getInstTerms()) {
            auto instTerm = uInstTerm.get();
            auto name = instTerm->getTerm()->getName();
            if (name == frString(net->pin(i))) {
              instTerm->addToNet(netIn);
              netIn->addInstTerm(instTerm);
              break;
            }
          }
        }
      } else {
        if (((io::Parser*)data)->tmpBlock->name2inst.find(net->instance(i)) == ((io::Parser*)data)->tmpBlock->name2inst.end()) {
          if (VERBOSE > -1) {
            cout <<"Error: component not found!" <<endl;
          }
          exit(1);
        }
        auto inst = ((io::Parser*)data)->tmpBlock->name2inst[net->instance(i)]; //frInst*
        bool flag =   false;
        for (auto &uInstTerm: inst->getInstTerms()) {
          auto instTerm = uInstTerm.get();
          auto name = instTerm->getTerm()->getName();
          if (name == frString(net->pin(i))) {
            flag = true;
            instTerm->addToNet(netIn);
            netIn->addInstTerm(instTerm);
            // add physical, absolute coordinate pins
            //frBox mbox;
            //inst->getRefBlock()->getBoundaryBBox(mbox);
            //frTransform xform;
            //inst->getTransform(xform);
            //frPoint size(mbox.right(), mbox.top());
            //xform.updateXform(size);
            //for (auto &pin: instTerm->getTerm()->getPins()) {
            //  shared_ptr<frPin> newPin = make_shared<frPin>(*pin, xform);
            //  for (auto &pinFig: newPin->getFigs()) {
            //    if (pinFig->typeId() == frcRect || pinFig->typeId() == frcPolygon) {
            //      pinFig->addToNet(netIn);
            //    } else {
            //      cout <<"Error: currently only support rect and polygon with net" <<endl;
            //    }
            //  }
            //  //newPin->addToNet(netIn); // currently only support rect and polygon
            //  netIn->addPin(newPin);
            //}

            break;
          }
        }
        if (!flag) {
          if (VERBOSE > -1) {
            cout <<"Error: component pin not found!" <<endl;
          }
          exit(1);
        }
      }
    }
  }
  // read pre-route
  //cout << "Net " << net->name() << " has " << net->numWires() << " wires\n";
  //cout << "Net " << net->name() << " has " << net->numPaths() << " paths\n"; // no paths
  //cout << "Net " << net->name() << " has " << net->numVpins() << " vpins\n"; // no vpins
  
  // initialize
  string layerName   = "";
  string viaName     = "";
  string shape       = "";
  bool hasBeginPoint = false;
  bool hasEndPoint   = false;
  frCoord beginX     = -1;
  frCoord beginY     = -1;
  frCoord beginExt   = -1;
  frCoord endX       = -1;
  frCoord endY       = -1;
  frCoord endExt     = -1;
  bool hasRect       = false;
  frCoord left       = -1;
  frCoord bottom     = -1;
  frCoord right      = -1;
  frCoord top        = -1;
  frCoord width      = 0;
  for (int i = 0; i < (int)net->numWires(); i++) {
    defiWire* tmpWire = net->wire(i);
    //cout << "Wire " << i << "\n";
    //cout << "  Type: " << tmpWire->wireType() << endl;
    //cout << "  has " << tmpWire->numPaths() << " paths\n";
    
    if (enableOutput) {
       cout <<endl <<"  + " <<tmpWire->wireType();
    }
    // each path is a def line
    for (int j = 0; j < (int)tmpWire->numPaths(); j++) {
      defiPath* path     = tmpWire->path(j);
      path->initTraverse();
      // initialize
      layerName     = "";
      viaName       = "";
      shape         = "";
      hasBeginPoint = false;
      hasEndPoint   = false;
      beginX        = -1;
      beginY        = -1;
      beginExt      = -1;
      endX          = -1;
      endY          = -1;
      endExt        = -1;
      hasRect       = false;
      left          = -1;
      bottom        = -1;
      right         = -1;
      top           = -1;
      width         = 0;
      //cout <<"path here" <<endl;
      
      int pathId;
      while ((pathId = path->next()) != DEFIPATH_DONE) {
        //cout << "  pathId = " << pathId << endl;
        switch(pathId) {
          case DEFIPATH_LAYER:
            layerName = string(path->getLayer());
            if (((io::Parser*)data)->tech->name2layer.find(layerName) == ((io::Parser*)data)->tech->name2layer.end()) {
              if (VERBOSE > -1) {
                cout <<"Error: unsupported layer: " <<layerName <<endl;
              }
              //continue;
              exit(1);
            }
            if (enableOutput) {
              if (!j) {
                cout <<" " <<layerName;
              } else {
                cout <<endl <<"    NEW " <<layerName;
              }
            }
            break;
          case DEFIPATH_VIA:
            viaName = string(path->getVia());
            if (enableOutput) {
              cout <<" " <<viaName;
            }
            break;
          case DEFIPATH_WIDTH:
            width = path->getWidth();
            if (enableOutput) {
              cout <<" " <<width;
            }
            break;
          case DEFIPATH_POINT:
            if (!hasBeginPoint) {
              path->getPoint(&beginX, &beginY);
              if (enableOutput) {
                cout <<" ( " <<beginX <<" " <<beginY <<" )";
              }
              hasBeginPoint = true;
            } else {
              path->getPoint(&endX, &endY);
              if (enableOutput) {
                cout <<" ( " <<endX <<" " <<endY <<" )";
              }
              hasEndPoint = true;
            }
            break;
          case DEFIPATH_FLUSHPOINT:
            if (!hasBeginPoint) {
              path->getFlushPoint(&beginX, &beginY, &beginExt);
              if (enableOutput) {
                cout <<" ( " <<beginX <<" " <<beginY <<" " <<beginExt <<" )";
              }
              hasBeginPoint = true;
            } else {
              path->getFlushPoint(&endX, &endY, &endExt);
              if (enableOutput) {
                cout <<" ( " <<endX <<" " <<endY <<" " <<endExt <<" )";
              }
              hasEndPoint = true;
            }
            break;
          case DEFIPATH_SHAPE:
            shape = path->getShape();
            if (enableOutput) {
              cout <<" + SHAPE " <<shape;
            }
            break;
          case DEFIPATH_RECT:
            path->getViaRect(&left, &bottom, &right, &top);
            if (enableOutput) {
              cout <<" RECT ( " <<left <<" " <<bottom <<" " <<right <<" " <<top <<" )";
            }
            hasRect = true;
            break;
          case DEFIPATH_VIRTUALPOINT:
            if (!hasBeginPoint) {
              path->getVirtualPoint(&beginX, &beginY);
              if (enableOutput) {
                cout <<" ( " <<beginX <<" " <<beginY <<" )";
              }
              hasBeginPoint = true;
            } else {
              path->getVirtualPoint(&endX, &endY);
              if (enableOutput) {
                cout <<" ( " <<endX <<" " <<endY <<" )";
              }
              hasEndPoint = true;
            }
            break;
          default : cout <<" net " <<net->name() <<" unknown pathId " <<pathId <<endl; break;
        }
      }


      auto layerNum = ((io::Parser*)data)->tech->name2layer[layerName]->getLayerNum();
      // add rect
      if (hasRect) {
        //shared_ptr<frBlockObject> rect = make_shared<frRect>(); // incomplete
        // end steiner = frRect
        continue;
      }

      // add wire, currently do not consider extension
      if (hasEndPoint) {
        // route
        auto tmpP = make_unique<frPathSeg>();

        // avoid begin > end case
        if (beginX > endX || beginY > endY) {
          tmpP->setPoints(frPoint(endX, endY), frPoint(beginX, beginY));
          swap(beginExt, endExt);
        } else {
          tmpP->setPoints(frPoint(beginX, beginY), frPoint(endX, endY));
        }
        tmpP->addToNet(netIn);
        tmpP->setLayerNum(layerNum);

        width = (width) ? width : ((io::Parser*)data)->tech->name2layer[layerName]->getWidth();
        auto defaultBeginExt = width / 2;
        auto defaultEndExt   = width / 2;

        frEndStyleEnum tmpBeginEnum;
        if (beginExt == -1) {
          tmpBeginEnum = frcExtendEndStyle;
        } else if (beginExt == 0) {
          tmpBeginEnum = frcTruncateEndStyle;
        } else {
          tmpBeginEnum = frcVariableEndStyle;
        }
        frEndStyle tmpBeginStyle(tmpBeginEnum);

        frEndStyleEnum tmpEndEnum;
        if (endExt == -1) {
          tmpEndEnum = frcExtendEndStyle;
        } else if (endExt == 0) {
          tmpEndEnum = frcTruncateEndStyle;
        } else {
          tmpEndEnum = frcVariableEndStyle;
        }
        frEndStyle tmpEndStyle(tmpEndEnum);

        frSegStyle tmpSegStyle;
        tmpSegStyle.setWidth(width);
        tmpSegStyle.setBeginStyle(tmpBeginStyle, tmpBeginEnum == frcExtendEndStyle ? defaultBeginExt : beginExt);
        tmpSegStyle.setEndStyle(tmpEndStyle, tmpEndEnum == frcExtendEndStyle ? defaultEndExt : endExt);
        tmpP->setStyle(tmpSegStyle);
        unique_ptr<frShape> tmpS(std::move(tmpP));
        netIn->addShape(tmpS);
      }

      // add via
      if (viaName != "") {
        if (((io::Parser*)data)->tech->name2via.find(viaName) == ((io::Parser*)data)->tech->name2via.end()) {
          if (VERBOSE > -1) {
            cout <<"Error: unsupported via: " <<viaName <<endl;
          }
        } else {
          frPoint p;
          if (hasEndPoint) {
            p.set(endX, endY);
          } else {
            p.set(beginX, beginY);
          }
          auto viaDef = ((io::Parser*)data)->tech->name2via[viaName];
          auto tmpP = make_unique<frVia>(viaDef);
          tmpP->setOrigin(p);
          tmpP->addToNet(netIn);
          netIn->addVia(tmpP);
        }
      }
    } // end path
  } // end wire
 
  frNetEnum netType = frNetEnum::frcNormalNet;
  if (net->hasUse()) {
    string str(net->use());
    if (str == "SIGNAL") {
      ;
    } else if (str == "CLOCK") {
      netType = frNetEnum::frcClockNet;
    } else if (str == "POWER") {
      netType = frNetEnum::frcPowerNet;
    } else if (str == "GROUND") {
      netType = frNetEnum::frcGroundNet;
    } else {
      cout <<"Error: unsupported NET USE in def" <<endl;
      exit(1);
    }
    if (enableOutput) {
      cout <<" + USE " <<str <<endl;
    }
  }
  netIn->setType(netType);

  if (enableOutput) {
    cout <<endl <<" ;" <<endl;
  }

  //if (enableOutput) {
  //  cout <<net->name() <<endl;
  ////cout <<"numRect " <<net->numRectangles() <<endl; // rect always 0
  //}


  if (isSNet) {
    ((io::Parser*)data)->tmpBlock->addSNet(uNetIn); 
    if (((io::Parser*)data)->tmpBlock->snets.size() < 100000) {
      if (((io::Parser*)data)->tmpBlock->snets.size() % 10000 == 0) {
        cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->snets.size() <<" snets" <<endl;
      }
    } else {
      if (((io::Parser*)data)->tmpBlock->snets.size() % 10000 == 0) {
        cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->snets.size() <<" snets" <<endl;
      }
    }
  } else {
    ((io::Parser*)data)->tmpBlock->addNet(uNetIn); 
    if (((io::Parser*)data)->tmpBlock->nets.size() < 100000) {
      if (((io::Parser*)data)->tmpBlock->nets.size() % 10000 == 0) {
        cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->nets.size() <<" nets" <<endl;
      }
    } else {
      if (((io::Parser*)data)->tmpBlock->nets.size() % 100000 == 0) {
        cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->nets.size() <<" nets" <<endl;
      }
    }
  }

  return 0;
}

int io::Parser::getDefTerminals(defrCallbackType_e type, defiPin* term, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != defrPinCbkType) {
    cout <<"Type is not defrPinCbkType!" <<endl;
    exit(1);
  }

  frTermEnum termType = frTermEnum::frcNormalTerm;

  if (term->hasUse()) {
    string str(term->use());
    if (str == "SIGNAL") {
      ;
    } else if (str == "CLOCK") {
      termType = frTermEnum::frcClockTerm;
    } else if (str == "POWER") {
      termType = frTermEnum::frcPowerTerm;
    } else if (str == "GROUND") {
      termType = frTermEnum::frcGroundTerm;
    } else {
      cout <<"Error: unsupported PIN USE in lef" <<endl;
      exit(1);
    }
    if (enableOutput) {
      cout <<"    USE " <<str <<" ;" <<endl;
    }
  }

  if (term->hasPort()) {
    cout <<"Error: multiple pin ports existing in DEF" <<endl;
    exit(1);
  } else {
    //cout <<"  pinName  = " <<term->pinName() <<endl;
    //cout <<"  hasLayer = " <<term->hasLayer() <<endl;
    //cout <<"  numLayer = " <<term->numLayer() <<endl;
    //cout <<"  numPolys = " <<term->numPolygons() <<endl;
    //cout <<"  numPorts = " <<term->numPorts() <<endl;

    // term
    auto uTermIn = make_unique<frTerm>(term->pinName());
    auto termIn = uTermIn.get();
    termIn->setId(((io::Parser*)data)->numTerms);
    ((io::Parser*)data)->numTerms++;
    termIn->setType(termType);
    // term should add pin
    // pin
    auto pinIn  = make_unique<frPin>();
    pinIn->setId(0);
    //pinIn->setTerm(termIn);
    // pin should add pinFigs
    // term
    //frOrientEnum orient = frOrientEnum(term->orient());
    for (int i = 0; i < term->numLayer(); ++i) {
      //cout <<"  layerName= " <<term->layer(i) <<endl;
      string layer = term->layer(i);
      if (((io::Parser*)data)->tech->name2layer.find(layer) == ((io::Parser*)data)->tech->name2layer.end()) {
        if (VERBOSE > -1) {
          cout <<"Error: unsupported layer: " <<layer <<endl;
        }
        //continue;
        exit(1);
      }

      frLayerNum layerNum = ((io::Parser*)data)->tech->name2layer[layer]->getLayerNum();
      frCoord xl = 0;
      frCoord yl = 0;
      frCoord xh = 0;
      frCoord yh = 0;
      term->bounds(i, &xl, &yl, &xh, &yh);
      // pinFig
      unique_ptr<frRect> pinFig = make_unique<frRect>();
      pinFig->setBBox(frBox(xl, yl, xh, yh));
      pinFig->addToPin(pinIn.get());
      pinFig->setLayerNum(layerNum);
      pinFig->move(frTransform(term->placementX(), term->placementY(), frOrientEnum(term->orient())));
      //cout <<"move" <<endl;
      frBox transformedBBox;
      pinFig->getBBox(transformedBBox);
      // pinFig completed
      // pin
      unique_ptr<frPinFig> uptr(std::move(pinFig));
      pinIn->addPinFig(uptr);
      // Rectangle pinFigRect(xl, yl, xh, yh);
      // std::cout << "Rect" << transformedBBox.left() << ", " << transformedBBox.bottom() << ", " << transformedBBox.right() << ", " << transformedBBox.top() << "\n";
      Rectangle pinFigRect(transformedBBox.left(), transformedBBox.bottom(), transformedBBox.right(), transformedBBox.top());
      pinIn->addLayerShape(layerNum, pinFigRect);
      // pin completed
    }
    // polygon
    for (int i = 0; i < term->numPolygons(); ++i) {
      //cout <<"  polyName= " <<term->polygonName(i) <<endl;
      string layer = term->polygonName(i);
      if (((io::Parser*)data)->tech->name2layer.find(layer) == ((io::Parser*)data)->tech->name2layer.end()) {
        if (VERBOSE > -1) {
          cout <<"Error: unsupported layer: " <<layer <<endl;
        }
        //continue;
        exit(1);
      }

      frLayerNum layerNum = ((io::Parser*)data)->tech->name2layer[layer]->getLayerNum();
      auto polyPoints = term->getPolygon(i);
      frCollection<frPoint> tmpPoints;
      for (int j = 0; j < polyPoints.numPoints; j++) {
        tmpPoints.push_back(frPoint((polyPoints.x)[j], (polyPoints.y)[j]));
      }

      // pinFig
      unique_ptr<frPolygon> pinFig = make_unique<frPolygon>();
      pinFig->setPoints(tmpPoints);
      pinFig->addToPin(pinIn.get());
      pinFig->setLayerNum(layerNum);
      pinFig->move(frTransform(term->placementX(), term->placementY(), frOrientEnum(term->orient())));
      // pinFig completed
      // pin
      unique_ptr<frPinFig> uptr(std::move(pinFig));
      pinIn->addPinFig(uptr);
      Polygon pinFigPoly;
      frVector<Point> boostPolyPoints;
      frTransform tmpXform(term->placementX(), term->placementY(), frOrientEnum(term->orient()));
      for (auto &pt: tmpPoints) {
        pt.transform(tmpXform);
        boostPolyPoints.push_back(Point(pt.x(), pt.y()));
        // std::cout << pt.x() << " " << pt.y() << "\n";
      }
      boost::polygon::set_points(pinFigPoly, boostPolyPoints.begin(), boostPolyPoints.end());
      pinIn->addLayerShape(layerNum, pinFigPoly);
      // pin completed
    }
    termIn->addPin(pinIn);
    //cout <<"  placeXY  = (" <<term->placementX() <<"," <<term->placementY() <<")" <<endl;
    ((io::Parser*)data)->tmpBlock->addTerm(uTermIn);
  }

  if (((io::Parser*)data)->tmpBlock->terms.size() % 1000 == 0) {
    cout <<"defIn read " <<((io::Parser*)data)->tmpBlock->terms.size() <<" pins" <<endl;
  }

  return 0;

}

int io::Parser::getDefTracks(defrCallbackType_e type, defiTrack* track, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if ((type != defrTrackCbkType)) {
    cout <<"Type is not defrTrackCbkType!" <<endl;
    exit(1);
  }

  if (enableOutput) {
    cout <<"TRACKS " <<track->macro() <<" " <<track->x() 
         <<" DO " <<track->xNum() <<" STEP " <<track->xStep() 
         <<" LAYER " <<track->layer(0) <<endl;
  }

  unique_ptr<frTrackPattern> tmpTrackPattern = make_unique<frTrackPattern>();
  if (((io::Parser*)data)->tech->name2layer.find(track->layer(0)) == ((io::Parser*)data)->tech->name2layer.end()) {
    cout <<"Error: cannot find layer: " <<track->layer(0) <<endl;
    exit(2);
  }
  tmpTrackPattern->setLayerNum(((io::Parser*)data)->tech->name2layer.at(track->layer(0))->getLayerNum());
  if (!strcmp(track->macro(), "X")) {
    tmpTrackPattern->setHorizontal(true);
  } else if (!strcmp(track->macro(), "Y")) {
    tmpTrackPattern->setHorizontal(false);
  } else {
    cout <<"Error: unsupporterd direction: " <<track->macro() <<endl;
    exit(2);
  }
  tmpTrackPattern->setStartCoord(track->x());
  tmpTrackPattern->setNumTracks(track->xNum());
  tmpTrackPattern->setTrackSpacing(track->xStep());
  ((io::Parser*)data)->tmpBlock->trackPatterns.at(tmpTrackPattern->getLayerNum()).push_back(std::move(tmpTrackPattern));
  //cout <<"here" <<endl;

  return 0;
}

int io::Parser::getDefUnits(defrCallbackType_e type, double number, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  ((io::Parser*)data)->tmpBlock->setDBUPerUU(static_cast<frUInt4>(number));
  if (enableOutput) {
    cout <<"UNITS DISTANCE MICRONS " <<((io::Parser*)data)->tmpBlock->getDBUPerUU() <<" ;" <<endl;
  }
  return 0;
}

void io::Parser::readDef() {
  FILE* f;
  int res;
  
  defrInit();
  defrReset();

  defrInitSession(1);
  
  defrSetUserData((defiUserData)this);

  defrSetDesignCbk(getDefString);
  defrSetDesignEndCbk(getDefVoid);
  defrSetDieAreaCbk(getDefDieArea);
  defrSetUnitsCbk(getDefUnits);
  defrSetTrackCbk(getDefTracks);
  defrSetComponentCbk(getDefComponents);
  defrSetPinCbk(getDefTerminals);
  //defrSetSNetStartCbk(getDefInteger);
  defrSetSNetCbk(getDefNets);
  defrSetNetCbk(getDefNets);
  defrSetAddPathToNet();
  defrSetViaCbk(getDefVias);

  // debug
  // exit(1);

  if ((f = fopen(DEF_FILE.c_str(),"r")) == 0) {
    cout <<"Couldn't open def file" <<endl;
    exit(2);
  }

  res = defrRead(f, DEF_FILE.c_str(), (defiUserData)this, 1);
  if (res != 0) {
    cout <<"DEF parser returns an error!" <<endl;
    exit(2);
  }
  fclose(f);

  //numPins = readPinCnt;

  defrClear();
}

int io::Parser::getLef58SpacingTable_parallelRunLength(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  SPACINGTABLE" <<endl;
    cout <<"  PARALLELRUNLENGTH";
  }

  bool    isWrongDirection    = false;
  bool    isSameMask          = false;

  bool    exceptEol           = false;
  frCoord eolWidth            = 0;

  //bool    hasExcludeSpacing   = false; // per WIDTH
  frCoord lowExcludeSpacing   = 0;
  frCoord highExcludeSpacing  = 0;

  // 2d spacing table
  frCollection<frCoord> rowVals, colVals;
  frCollection<frCollection<frCoord> > tblVals;
  frCollection<frCoord> tblRowVals;
  
  // except within
  map<frCoord, pair<frCoord, frCoord> > ewVals;

  int stage = 0;

  istringstream istr(sIn);
  string word;
  while (istr >> word) {
    if (word == string("WRONGDIRECTION")) {
      //isWrongDirection = true;
      if (enableOutput) {
        cout <<" WRONGDIRECTION";
      }
    } else if (word == string("SAMEMASK")) {
      isSameMask = true;
      if (enableOutput) {
        cout <<" SAMEMASK";
      }
    } else if (word == string("EXCEPTEOL")) {
      //exceptEol = true;
      double tmp;
      if (istr >> tmp) {
        //eolWidth = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" EXCEPTEOL " <<tmp;
        }
      } else {
        cout <<"Error: getLef58SpacingTable_parallelRunLength" <<endl;
      }
    } else if (word == string("EXCEPTWITHIN")) {
      //hasExcludeSpacing = true;
      if (enableOutput) {
        cout <<" EXCEPTWITHIN";
      }
      double tmp;
      if (istr >> tmp) {
        lowExcludeSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" " <<tmp;
        }
      } else {
        cout <<"Error: getLef58SpacingTable_parallelRunLength" <<endl;
      }
      if (istr >> tmp) {
        highExcludeSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" " <<tmp;
        }
      } else {
        cout <<"Error: getLef58SpacingTable_parallelRunLength" <<endl;
      }
      ewVals[rowVals.size()-1] = make_pair(lowExcludeSpacing, highExcludeSpacing);
    } else if (word == string("WIDTH")) {
      if (tblRowVals.size()) {
        tblVals.push_back(tblRowVals);
        tblRowVals.clear();
      }
      //hasExcludeSpacing   = false;
      stage = 1;
      double tmp;
      if (istr >> tmp) {
        rowVals.push_back(frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU())));
        if (enableOutput) {
          cout <<endl <<"  WIDTH " <<tmp;
        }
      } else {
        cout <<"Error: getLef58SpacingTable_parallelRunLength" <<endl;
      }
    } else if (word == string(";")) {
      if (stage == 1 && tblRowVals.size()) {
        tblVals.push_back(tblRowVals);
        tblRowVals.clear();
      }
      if (enableOutput) {
        cout <<" ;";
      }
    } else {
      // get length
      if (stage == 0) {
        colVals.push_back(frCoord(round(stod(word) * ((io::Parser*)data)->tech->getDBUPerUU())));
        if (enableOutput) {
          cout <<" " <<word;
        }
      }
      if (stage == 1) {
        tblRowVals.push_back(frCoord(round(stod(word) * ((io::Parser*)data)->tech->getDBUPerUU())));
        if (enableOutput) {
          cout <<" " <<word;
        }
      }
      //if (stage == 0) {
      //  rowVals.push_back(frCoord(round(stod(word) * ((io::Parser*)data)->tech->getDBUPerUU())));
      //  cout <<"pushed back row value " <<word <<endl;
      //}
    }
  }

  string rowName("WIDTH");
  string colName("PARALLELRUNLENGTH");
  shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > prlTbl = make_shared<
    fr2DLookupTbl<frCoord, frCoord, frCoord> >(rowName, rowVals, colName, colVals, tblVals);
  shared_ptr<frLef58SpacingTableConstraint> spacingTableConstraint = make_shared<frLef58SpacingTableConstraint>(prlTbl, ewVals);
  spacingTableConstraint->setWrongDirection(isWrongDirection);
  spacingTableConstraint->setSameMask(isSameMask);
  if (exceptEol) {
    spacingTableConstraint->setEolWidth(eolWidth);
  }
  
  ((io::Parser*)data)->tech->addConstraint(spacingTableConstraint);
  tmpLayer->addConstraint(spacingTableConstraint);

  //if (enableOutput) {
  //  cout <<endl;
  //}


  //cout <<"tblVals " <<endl;
  //for (auto &v1: tblVals) {
  //  for (auto &v2: v1) {
  //    cout <<" " <<v2;
  //  }
  //  cout <<endl;
  //}
  //cout <<"ewVals " <<endl;
  //for (auto &it: ewVals) {
  //  cout <<"idx = " <<it.first <<", " <<"l/h = " <<it.second.first <<"/" <<it.second.second <<endl;
  //}


  //cout <<sIn <<endl;
  return 0;
}

int io::Parser::getLef58SpacingTable(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  PROPERTY LEF58_SPACINGTABLE \"";
  }
  istringstream istr(sIn);
  string word;
  stringstream ss;

  string keyword;
  while (istr >> word) {
    if (word == string("SPACINGTABLE")) {
      ss.str("");
    } else if (word == string("PARALLELRUNLENGTH")) {
      //cout <<"found PARALLELRUNLENGTH" <<endl;
      keyword = "PARALLELRUNLENGTH";
    } else if (word == string(";")) {
      ss <<" " <<word;
      if (keyword == string("PARALLELRUNLENGTH")) {
        getLef58SpacingTable_parallelRunLength(data, tmpLayer, ss.str());
      }
      //cout <<"found ;" <<endl;
    } else {
      //cout <<"found " <<word <<endl;
      ss <<" " <<word;
    }
  }
  //cout <<ss.str() <<endl;
  if (enableOutput) {
    cout <<"\" ;" <<endl;
  }

  return 0;
}


int io::Parser::getLef58Spacing_endOfLineWithin(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //cout <<endl <<"xxx " <<sIn <<endl;
  //if (enableOutput) {
  //  cout <<endl <<"  SPACING";
  //}
  //cout <<"test: " <<sIn <<endl;
  // minspacing
  frCoord eolSpace           = 0;
  frCoord eolWidth           = 0;

  bool    hasExactWidth      = false;

  bool    hasWrongDirSpacing = false;
  frCoord wrongDirSpace      = 0;

  bool    hasOppositeWidth   = false;
  frCoord oppositeWidth      = 0;

  frCoord eolWithin          = 0;

  bool    hasWrongDirWithin  = false;
  frCoord wrongDirWithin     = 0;

  bool    hasSameMask        = false;

  // ENDTOEND
  bool    hasEndToEnd          = false;
  frCoord endToEndSpace        = 0;
  bool    hasCutSpace          = false;
  frCoord oneCutSpace          = 0;
  frCoord twoCutSpace          = 0;
  bool    hasExtension         = false;
  frCoord extension            = 0;
  bool    hasWrongDirExtension = false;
  frCoord wrongDirExtension    = 0;
  bool    hasOtherEndWidth     = false;
  frCoord otherEndWidth        = 0;
  
  // MINLENGTH/MAXLENGTH
  bool    hasLength          = false;
  bool    isMax              = false;
  frCoord length             = 0;
  bool    hasTwoSides        = false;

  // PARALLELEDGE
  bool    hasParallelEdge          = false;
  bool    hasSubtractEolWidth      = false;
  frCoord parSpace                 = 0;
  frCoord parWithin                = 0;
  bool    hasPrl                   = false;
  frCoord prl                      = 0;
  bool    hasParallelEdgeMinLength = false;
  frCoord parallelEdgeMinLength    = 0;
  bool    hasTwoEdges              = false;
  bool    hasSameMetal             = false;
  bool    hasNonEolCornerOnly      = false;
  bool    hasParallelSameMask      = false;

  bool    skip = false;
  //string stage = "";

  istringstream istr(sIn);
  string word;
  int stage = 0;
  while (istr >> word) {
    if (word == string("SPACING")) {
      double tmp;
      if (istr >> tmp) {
        eolSpace = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<endl <<"  SPACING " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (word == string("ENDOFLINE")) {
      double tmp;
      if (istr >> tmp) {
        eolWidth = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" ENDOFLINE " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (word == string("EXACTWIDTH")) {
      hasExactWidth = true;
      if (enableOutput) {
        cout <<" EXACTWIDTH";
      }
      stage = 0;
    } else if (word == string("WRONGDIRSPACING")) {
      hasWrongDirSpacing = true;
      double tmp;
      if (istr >> tmp) {
        wrongDirSpace = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" WRONGDIRSPACING " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (word == string("OPPOSITEWIDTH")) {
      hasOppositeWidth = true;
      double tmp;
      if (istr >> tmp) {
        oppositeWidth = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" OPPOSITEWIDTH " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (word == string("WITHIN")) {
      double tmp;
      if (istr >> tmp) {
        eolWithin = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" WITHIN " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 1;
    } else if (word == string("SAMEMASK")) {
      hasSameMask = true;
      if (enableOutput) {
        cout <<" SAMEMASK";
      }
      stage = 0;
    } else if (word == string("EXCEPTEXACTWIDTH")) {
      if (enableOutput) {
        cout <<" EXCEPTEXACTWIDTH(SKIP)";
      }
      stage = 0;
      skip = true;
    } else if (word == string("FILLCONCAVECORNER")) {
      if (enableOutput) {
        cout <<" FILLCONCAVECORNER(SKIP)";
      }
      stage = 0;
      skip = true;
    } else if (word == string("WITHCUT")) {
      if (enableOutput) {
        cout <<" WITHCUT(SKIP)";
      }
      stage = 0;
    } else if (word == string("ENDPRLSPACING")) {
      if (enableOutput) {
        cout <<" ENDPRLSPACING(SKIP)";
      }
      stage = 0;
      skip = true;
    } else if (word == string("ENDTOEND")) {
      hasEndToEnd = true;
      double tmp;
      if (istr >> tmp) {
        endToEndSpace = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" ENDTOEND " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 2;
    } else if (word == string("MAXLENGTH")) {
      hasLength = true;
      isMax     = true;
      double tmp;
      if (istr >> tmp) {
        length = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" MAXLENGTH " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (stage != 3 && word == string("MINLENGTH")) {
      hasLength = true;
      isMax     = false;
      double tmp;
      if (istr >> tmp) {
        length = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" MINLENGTH " <<tmp;
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      stage = 0;
    } else if (word == string("TWOSIDES")) {
      hasTwoSides = true;
      if (enableOutput) {
        cout <<" TWOSIDES";
      }
      stage = 0;
    } else if (word == string("EQUALRECTWIDTH")) {
      if (enableOutput) {
        cout <<" EQUALRECTWIDTH(SKIP)";
      }
      stage = 0;
      skip = true;
    } else if (word == string("PARALLELEDGE")) {
      hasParallelEdge = true;
      string tmp;
      // read to parSpace
      if (istr >> tmp) {
        if (tmp == string("SUBTRACTEOLWIDTH")) {
          hasSubtractEolWidth = true;
          double tmp2;
          if (istr >> tmp2) {
            parSpace = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" PARALLELEDGE SUBTRACTEOLWIDTH " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        } else {
          parSpace = frCoord(round(stod(tmp) * ((io::Parser*)data)->tech->getDBUPerUU()));
          if (enableOutput) {
            cout <<" PARALLELEDGE " <<tmp;
          }
        }
      } else {
        cout <<"Error: getLef58Spacing_eolSpace" <<endl;
      }
      // read to parWithin
      if (istr >> tmp) {
        if (tmp == string("WITHIN")) {
          double tmp2;
          if (istr >> tmp2) {
            parWithin = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" (PE)WITHIN " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        } else {
          cout <<"Error: getLef58Spacing_eolSpace" <<endl;
        }
      }
      stage = 3;
    } else if (word == string("ENCLOSECUT")) {
      if (enableOutput) {
        cout <<" ENCLOSECUT(SKIP)";
      }
      stage = 0;
      skip = true;
    } else if (word == string(";")) {
      if (enableOutput) {
        cout <<" ;";
      }
      stage = 0;
    } else {
      // stage = 1, read wrongDirWithin
      if (stage == 1) {
        hasWrongDirWithin = true;
        cout <<flush;
        wrongDirWithin = frCoord(round(stod(word) * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" " <<word;
        }
        stage = 0;
      // stage = 2, read end to end from onecutspace
      } else if (stage == 2) {
        auto tmp = word;
        if (tmp == string("EXTENSION")) {
          hasExtension = true;
          double tmp2;
          if (istr >> tmp2) {
            extension = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" EXTENSION " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
          stage = 20;
        } else if (tmp == string("OTHERENDWIDTH")) {
          hasOtherEndWidth = true;
          double tmp2;
          if (istr >> tmp2) {
            otherEndWidth = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" OTHERENDWIDTH " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        } else {
          hasCutSpace = true;
          oneCutSpace = frCoord(round(stod(tmp) * ((io::Parser*)data)->tech->getDBUPerUU()));
          if (enableOutput) {
            cout <<" " <<tmp;
          }
          double tmp2;
          if (istr >> tmp2) {
            twoCutSpace = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        }
      // stage = 20, read end to end wrongDirExtension
      } else if (stage == 20) {
        hasWrongDirExtension = true;
        wrongDirExtension = frCoord(round(stod(word) * ((io::Parser*)data)->tech->getDBUPerUU()));
        if (enableOutput) {
          cout <<" " <<word;
        }
        stage = 2;
      // stage = 3, read paralleledge from prl
      } else if (stage == 3) {
        if (word == string("PRL")) {
          hasPrl = true;
          double tmp2;
          if (istr >> tmp2) {
            prl = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" PRL " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        } else if (word == string("MINLENGTH")) {
          hasParallelEdgeMinLength = true;
          double tmp2;
          if (istr >> tmp2) {
            parallelEdgeMinLength = frCoord(round(tmp2 * ((io::Parser*)data)->tech->getDBUPerUU()));
            if (enableOutput) {
              cout <<" (PE)MINLENGTH " <<tmp2;
            }
          } else {
            cout <<"Error: getLef58Spacing_eolSpace" <<endl;
          }
        } else if (word == string("TWOEDGES")) {
          hasTwoEdges = true;
        } else if (word == string("SAMEMETAL")) {
          hasSameMetal = true;
        } else if (word == string("NONEOLCORNERONLY")) {
          hasNonEolCornerOnly = true;
        } else if (word == string("PARALLELSAMEMASK")) {
          hasParallelSameMask = true;
        } else {
          ;
        }
      } else {
        ;
      }
    }
  }

  if (skip) {
    ;
  } else {
    auto con = make_shared<frLef58SpacingEndOfLineConstraint>();
    con->setEol(eolSpace, eolWidth, hasExactWidth);
    if (hasWrongDirSpacing) {
      con->setWrongDirSpace(wrongDirSpace);
    }

    auto within = make_shared<frLef58SpacingEndOfLineWithinConstraint>();
    con->setWithinConstraint(within);
    if (hasOppositeWidth) {
      within->setOppositeWidth(oppositeWidth);
    }
    within->setEolWithin(eolWithin);
    if (hasWrongDirWithin) {
      within->setWrongDirWithin(wrongDirWithin);
    }
    if (hasSameMask) {
      within->setSameMask(hasSameMask);
    }
    if (hasEndToEnd) {
      auto endToEnd = make_shared<frLef58SpacingEndOfLineWithinEndToEndConstraint>();
      within->setEndToEndConstraint(endToEnd);
      endToEnd->setEndToEndSpace(endToEndSpace);
      if (hasCutSpace) {
        endToEnd->setCutSpace(oneCutSpace, twoCutSpace);
      }
      if (hasExtension) {
        if (hasWrongDirExtension) {
          endToEnd->setExtension(extension, wrongDirExtension);
        } else {
          endToEnd->setExtension(extension);
        }
      }
      if (hasOtherEndWidth) {
        endToEnd->setOtherEndWidth(otherEndWidth);
      }
    }
    if (hasParallelEdge) {
      auto parallelEdge = make_shared<frLef58SpacingEndOfLineWithinParallelEdgeConstraint>();
      within->setParallelEdgeConstraint(parallelEdge);
      if (hasSubtractEolWidth) {
        parallelEdge->setSubtractEolWidth(hasSubtractEolWidth);
      }
      parallelEdge->setPar(parSpace, parWithin);
      if (hasPrl) {
        parallelEdge->setPrl(prl);
      }
      if (hasParallelEdgeMinLength) {
        parallelEdge->setMinLength(parallelEdgeMinLength);
      }
      if (hasTwoEdges) {
        parallelEdge->setTwoEdges(hasTwoEdges);
      }
      if (hasSameMetal) {
        parallelEdge->setSameMetal(hasSameMetal);
      }
      if (hasNonEolCornerOnly) {
        parallelEdge->setNonEolCornerOnly(hasNonEolCornerOnly);
      }
      if (hasParallelSameMask) {
        parallelEdge->setParallelSameMask(hasParallelSameMask);
      }
    }
    if (hasLength) {
      auto len = make_shared<frLef58SpacingEndOfLineWithinMaxMinLengthConstraint>();
      within->setMaxMinLengthConstraint(len);
      len->setLength(isMax, length, hasTwoSides);
    }

    ((io::Parser*)data)->tech->addConstraint(con);
    //tmpLayer->addConstraint(con);
    tmpLayer->lef58SpacingEndOfLineConstraints.push_back(con);
  }

  //if (enableOutput) {
  //  cout <<endl;
  //}

  return 0;
}


int io::Parser::getLef58Spacing(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  PROPERTY LEF58_SPACING \"";
  }
  //cout <<sIn <<endl;
  istringstream istr(sIn);
  string word;
  stringstream ss;

  string keyword;
  string keyword2;
  while (istr >> word) {
    if (word == string("SPACING")) {
      ss.str("");
      //ss.str(string("SPACING"));
      //ss.clear();
      //ss.str("");
      ss <<word;
      //cout <<"xxx " <<ss.str() <<endl;
      keyword = "";
    } else if (word == string("EOLPERPENDICULAR")) {
      keyword = "EOLPERPENDICULAR";
      ss <<" " <<word;
    } else if (word == string("AREA")) {
      keyword = "AREA";
      ss <<" " <<word;
    } else if (word == string("LAYER")) {
      keyword = "LAYER";
      ss <<" " <<word;
    } else if (word == string("NOTCHLENGTH")) {
      keyword = "NOTCHLENGTH";
      ss <<" " <<word;
    } else if (word == string("NOTCHSPAN")) {
      keyword = "NOTCHSPAN";
      ss <<" " <<word;
    } else if (word == string("ENDOFLINE")) {
      keyword = "ENDOFLINE";
      ss <<" " <<word;
    } else if (word == string("CONVEXCORNERS")) {
      keyword = "CONVEXCORNERS";
      ss <<" " <<word;
    } else if (word == string("TOCONCAVECORNER")) {
      keyword2 = "TOCONCAVECORNER";
      ss <<" " <<word;
    } else if (word == string("TONOTCHLENGTH")) {
      keyword2 = "TONOTCHLENGTH";
      ss <<" " <<word;
    } else if (word == string(";")) {
      ss <<" " <<word;
      if (keyword == string("ENDOFLINE")) {
        if (keyword2 == string("")) {
          getLef58Spacing_endOfLineWithin(data, tmpLayer, ss.str());
        } else if (keyword2 == string("TOCONCAVECORNER")) {
          ;
        } else if (keyword2 == string("TONOTCHLENGTH")) {
          ;
        } else {
          ;
        }
      } else {
        ; // has keyword, or SAMEMASK, or WRONGDIRECTION, or nothing
      }
    } else {
      ss <<" " <<word;
    }
  }
  if (enableOutput) {
    cout <<"\" ;" <<endl;
  }
  return 0;
}

int io::Parser::getLef58CutClass(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  PROPERTY LEF58_CUTCLASS \"";
  }
  istringstream istr(sIn);
  string word;

  string  name       = "";
  frCoord viaWidth   = 0;
  bool    hViaLength = false;
  frCoord viaLength  = 0;
  bool    hNumCut    = false;
  frUInt4 numCut     = 0;

  auto lef58CutClassCon = make_shared<frLef58CutClassConstraint>();
  if (tmpLayer->lef58CutClassConstraint.lock()) {
    cout <<"Warning: PROPERTY LEF58_CUTCLASS already exists" <<endl;
  }
  tmpLayer->lef58CutClassConstraint = lef58CutClassCon;
  ((io::Parser*)data)->tech->addConstraint(lef58CutClassCon);

  while (istr >> word) {
    if (word == string("CUTCLASS")) {
      // initialization
      name       = "";
      viaWidth   = 0;
      hViaLength = false;
      viaLength  = 0;
      hNumCut    = false;
      numCut     = 0;
      if (istr >> name) {
        ;
      } else {
        cout <<"Error: getLef58CutClass" <<endl;
      }
      if (enableOutput) {
        cout <<endl <<"  CUTCLASS " <<name;
      }
    } else if (word == "WIDTH") {
      double tmp;
      if (istr >> tmp) {
        viaWidth = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutClass" <<endl;
      }
      if (enableOutput) {
        cout <<" WIDTH " <<tmp;
      }
    } else if (word == "LENGTH") {
      double tmp;
      if (istr >> tmp) {
        hViaLength = true;
        viaLength = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutClass" <<endl;
      }
      if (enableOutput) {
        cout <<" LENGTH " <<tmp;
      }
    } else if (word == "CUTS") {
      if (istr >> numCut) {
        hNumCut = true;
      } else {
        cout <<"Error: getLef58CutClass" <<endl;
      }
      if (enableOutput) {
        cout <<" CUTS " <<numCut;
      }
    } else if (word == ";") {
      if (enableOutput) {
        cout <<" ;";
      }
      // push rule here;
      auto con = make_shared<frLef58CutClass>();
      con->setName(name);
      con->setViaWidth(viaWidth);
      if (hViaLength) {
        con->setViaLength(viaLength);
      } else {
        con->setViaLength(viaWidth);
      }
      if (hNumCut) {
        con->setNumCut(numCut);
      } else {
        con->setNumCut(1);
      }
      lef58CutClassCon->addToCutClass(con);
    }
  }
  if (enableOutput) {
    cout <<"\" ;" <<endl;
  }
  return 0;
}

int io::Parser::getLef58CutSpacing_helper(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  //bool enableOutput = false;

  string keyword = "";

  istringstream istr(sIn);
  string word;
  stringstream ss;
  while (istr >> word) {
    if (word == string("SPACING")) {
      keyword = "";
      ss.str("");
      ss <<word;
    } else if (keyword == "" && word == string("SAMEMASK")) {
      keyword = "SAMEMASK";
      ss <<" " <<word;
    } else if (word == string("MAXXY")) {
      keyword = "MAXXY";
      ss <<" " <<word;
    } else if (word == string("LAYER")) {
      keyword = "LAYER";
      ss <<" " <<word;
    } else if (word == string("ADJACENTCUTS")) {
      keyword = "ADJACENTCUTS";
      ss <<" " <<word;
    } else if (word == string("PARALLELOVERLAP")) {
      keyword = "PARALLELOVERLAP";
      ss <<" " <<word;
    } else if (word == string("PARALLELWITHIN")) {
      keyword = "PARALLELWITHIN";
      ss <<" " <<word;
    } else if (word == string("SAMEMETALSHAREDEDGE")) {
      keyword = "SAMEMETALSHAREDEDGE";
      ss <<" " <<word;
    } else if (word == string("AREA")) {
      keyword = "AREA";
      ss <<" " <<word;
    } else {
      ss <<" " <<word;
    }
  }

  if (keyword == "LAYER") {
    getLef58CutSpacing_layer(data, tmpLayer, ss.str());
  } else if (keyword == "ADJACENTCUTS") {
    getLef58CutSpacing_adjacentCuts(data, tmpLayer, ss.str());
  } else if (keyword == "PARALLELWITHIN") {
    getLef58CutSpacing_parallelWithin(data, tmpLayer, ss.str());
  } else {
    ; //skip unsupported rules
  }
  
  return 0;

}

int io::Parser::getLef58CutSpacing(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  PROPERTY LEF58_SPACING \"";
  }
  istringstream istr(sIn);
  string word;
  stringstream ss;
  while (istr >> word) {
    if (word == string("SPACING")) {
      ss.str("");
      ss <<word;
    } else if (word == ";") {
      ss <<" " <<word;
      getLef58CutSpacing_helper(data, tmpLayer, ss.str());
    } else {
      ss <<" " <<word;
    }
  }
  if (enableOutput) {
    cout <<"\" ;" <<endl;
  }
  return 0;
}

int io::Parser::getLef58CutSpacing_layer(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;

  frCoord  cutSpacing        = 0;

  bool     skip              = false;
  //frLayerNum secondLayerNum  = 0;
  frString secondLayerName   = "";
  // cutclass
  frString className         = "";
  bool     hasAboveWidth     = false;
  frCoord  aboveWidth        = 0;
  bool     hasEnc            = false;
  frCoord  enclosure         = 0;

  istringstream istr(sIn);
  string word;

  string keyword = "";
  //int stage = 0;
  while (istr >> word) {
    if (word == string("SPACING")) {
      double tmp;
      if (istr >> tmp) {
        cutSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<endl <<"  SPACING " <<tmp;
      }
    } else if (word == string("LAYER")) {
      if (istr >> secondLayerName) {
        //if (((io::Parser*)data)->tech->layers.find(layerName) == ((io::Parser*)data)->tech->layers.end()) {
        //  skip = true; // skip for via0
        //} else {
        //  secondLayerNum = ((io::Parser*)data)->tech->layers.at(layerName)->getLayerNum();
        //}
        
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" LAYER " <<secondLayerName;
      }
    } else if (word == string("CUTCLASS")) {
      if (istr >> className) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" CUTCLASS " <<className;
      }
    } else if (word == string("SHORTEDGEONLY")) {
      skip = true;
      if (enableOutput) {
        cout <<" SHORTEDGEONLY(SKIP)";
      }
    } else if (word == string("PRL")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" PRL(SKIP) " <<tmp;
      }
    } else if (word == string("CONCAVECORNER")) {
      skip = true;
      if (enableOutput) {
        cout <<" CONCAVECORNER(SKIP)";
      }
    } else if (word == string("WIDTH")) {
      skip = true;
      string tmpS;
      double tmpD;
      if (istr >> tmpD) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" WIDTH(SKIP) " <<tmpD;
      }
      if (istr >> tmpS) {
        ; // ENCLOSURE
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (istr >> tmpD) {
        ; // enclosure
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE " <<tmpD;
      }
      if (istr >> tmpS) {
        ; // EDGELENGTH
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (istr >> tmpD) {
        ; // edgeLength
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" EDGELENGTH " <<tmpD;
      }
    } else if (word == string("PARALLEL")) {
      skip = true;
      string tmpS;
      double tmpD;
      if (istr >> tmpD) {
        ; // parLength
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" PARALLEL(SKIP) " <<tmpD;
      }
      if (istr >> tmpS) {
        ; // WITHIN
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (istr >> tmpD) {
        ; // parWithin
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" WITHIN " <<tmpD;
      }
      if (istr >> tmpS) {
        ; // ENCLOSURE
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (istr >> tmpD) {
        ; // enclosure
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE " <<tmpD;
      }
    } else if (word == string("EDGELENGTH")) {
      skip = true;
      string tmpS;
      double tmpD;
      if (istr >> tmpD) {
        ; // edgeLength
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" EDGELENGTH(SKIP) " <<tmpD;
      }
      if (istr >> tmpS) {
        ; // ENCLOSURE
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (istr >> tmpD) {
        ; // edgeEnclosure
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE " <<tmpD;
      }
      if (istr >> tmpD) {
        ; // adjEnclosure
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" " <<tmpD;
      }
    } else if (word == string("EXTENSION")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" EXTENSION(SKIP) " <<tmp;
      }
    } else if (word == string("NONEOLCONVEXCORNER")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" NONEOLCONVEXCORNER " <<tmp;
      }
    } else if (word == string("MINLENGTH")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" MINLENGTH(SKIP) " <<tmp;
      }
    } else if (word == string("ABOVEWIDTH")) {
      hasAboveWidth = true;
      double tmp;
      if (istr >> tmp) {
        aboveWidth = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" ABOVEWIDTH " <<tmp;
      }
    } else if (word == string("ENCLOSURE")) {
      hasEnc = true;
      double tmp;
      if (istr >> tmp) {
        enclosure = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE " <<tmp;
      }
    } else if (word == string("MASKOVERLAP")) {
      skip = true;
      if (enableOutput) {
        cout <<" MASKOVERLAP(SKIP)";
      }
    } else if (word == string("WRONGDIRECTION")) {
      skip = true;
      if (enableOutput) {
        cout <<" WRONGDIRECTION(SKIP)";
      }
    } else if (word == string(";")) {
      if (enableOutput) {
        cout <<" ;";
      }
    } else {
      ;
    }
  }

  if (skip) {
    ;
  } else {
    auto lr = make_shared<frLef58CutSpacingLayerConstraint>();
    //lr->setSecondLayerNum(secondLayerNum);
    lr->setSecondLayerName(secondLayerName);
    if (!(className == "")) {
      lr->setCutClass(className);
      if (hasAboveWidth) {
        lr->setAboveWidth(aboveWidth);
        if (hasEnc) {
          lr->setEnclosure(enclosure);
        }
      }
    }

    auto con = make_shared<frLef58CutSpacingConstraint>();
    con->setCutSpacing(cutSpacing);
    con->setLayerConstraint(lr);
    tmpLayer->lef58CutSpacingConstraints.push_back(con);
    ((io::Parser*)data)->tech->addConstraint(con);
  }
  return 0;
}
// lefdef ref spacing
int io::Parser::getLef58CutSpacing_adjacentCuts(void *data, frLayer* tmpLayer, const string &sIn) {
  bool enableOutput = false;
  //bool enableOutput = true;

  bool     skip              = false;

  frCoord  cutSpacing        = 0;
  frUInt4  numAdjCuts        = 0;
  // two cuts
  bool     hasTwoCuts        = false;
  frUInt4  twoCuts           = 0;
  bool     hasTwoCutsSpacing = false;
  frCoord  twoCutsSpacing    = 0;
  bool     hasSameCut        = false;
  // within
  frCoord  cutWithin1        = 0;
  frCoord  cutWithin2        = 0;
  // cutclass
  frString className         = "";
  bool     hasToAll          = false;

  istringstream istr(sIn);
  string word;

  string keyword = "";
  int stage = 0;
  while (istr >> word) {
    if (word == string("SPACING")) {
      double tmp;
      if (istr >> tmp) {
        cutSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<endl <<"  SPACING " <<tmp;
      }
    } else if (word == string("ADJACENTCUTS")) {
      if (istr >> numAdjCuts) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" ADJACENTCUTS " <<numAdjCuts;
      }
    } else if (word == string("EXACTALIGNED")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        //cutSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" EXACTALIGNED(SKIP)";
      }
    } else if (word == string("TWOCUTS")) {
      hasTwoCuts = true;
      if (istr >> twoCuts) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" TWOCUTS " <<twoCuts;
      }
    } else if (word == string("TWOCUTSSPACING")) {
      hasTwoCutsSpacing = true;
      double tmp;
      if (istr >> tmp) {
        twoCutsSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" TWOCUTSSPACING " <<tmp;
      }
    } else if (word == string("SAMECUT")) {
      hasSameCut = true;
      if (enableOutput) {
        cout <<" SAMECUT";
      }
    } else if (word == string("WITHIN")) {
      stage = 1;
      double tmp;
      if (istr >> tmp) {
        cutWithin1 = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        cutWithin2 = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" WITHIN " <<tmp;
      }
    } else if (word == string("EXCEPTSAMEPGNET")) {
      skip = true;
      if (enableOutput) {
        cout <<" EXCEPTSAMEPGNET(SKIP)";
      }
    } else if (word == string("EXCEPTALLWITHIN")) {
      skip = true;
      double tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" EXCEPTALLWITHIN(SKIP) " <<tmp;
      }
    } else if (word == string("ENCLOSURE")) {
      skip = true;
      string tmp;
      if (istr >> tmp) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE(SKIP) " <<tmp;
      }
      double tmpD;
      if (istr >> tmpD) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" " <<tmpD;
      }
    } else if (word == string("CUTCLASS")) {
      if (istr >> className) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" CUTCLASS " <<className;
      }
    } else if (word == string("TO")) {
      hasToAll = true;
      string tmp;
      if (istr >> tmp) {
        if (tmp == "ALL") {
          ;
        } else {
          cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
        }
      } else {
        cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
      }
      if (enableOutput) {
        cout <<" TO ALL";
      }
    } else if (word == string("NOPRL")) {
      skip = true;
      if (enableOutput) {
        cout <<" NOPRL(SKIP)";
      }
    } else if (word == string("SIDEPARALLELOVERLAP")) {
      skip = true;
      if (enableOutput) {
        cout <<" SIDEPARALLELOVERLAP(SKIP)";
      }
    } else if (word == string("SAMEMASK")) {
      skip = true;
      if (enableOutput) {
        cout <<" SAMEMASK(SKIP)";
      }
    } else if (word == string(";")) {
      if (enableOutput) {
        cout <<" ;";
      }
    } else {
      if (stage == 1) {
        double tmp;
        if (istr >> tmp) {
          cutWithin2 = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
        } else {
          cout <<"Error: getLef58CutSpacing_adjacentCuts" <<endl;
        }
        if (enableOutput) {
          cout <<" " <<tmp;
        }
        stage = 0;
      } else {
        if (enableOutput) {
          cout <<" " <<word;
        }
      }
    }
  }

  if (skip) {
    ;
  } else{
    auto ac = make_shared<frLef58CutSpacingAdjacentCutsConstraint>();
    ac->setNumAdjCuts(numAdjCuts);
    if (hasTwoCuts) {
      if (hasTwoCutsSpacing) {
        ac->setTwoCuts(twoCuts, twoCutsSpacing, hasSameCut);
      } else {
        ac->setTwoCuts(twoCuts, hasSameCut);
      }
    }
    ac->setWithin(cutWithin1, cutWithin2);
    if (!(className == "")) {
      ac->setCutClass(className, hasToAll);
    }

    auto con = make_shared<frLef58CutSpacingConstraint>();
    con->setCutSpacing(cutSpacing);
    con->setAdjacentCutsConstraint(ac);
    tmpLayer->lef58CutSpacingConstraints.push_back(con);
    ((io::Parser*)data)->tech->addConstraint(con);
  }
  return 0;
}

int io::Parser::getLef58CutSpacing_parallelWithin(void *data, frLayer* tmpLayer, const string &sIn) {
  bool enableOutput = false;
  //bool enableOutput = true;

  bool     skip             = false;
  frCoord  cutSpacing       = 0;
  frCoord  within           = 0;
  bool     hasExceptSameNet = false;
  bool     hasCutClass      = false;
  frString className        = "";
  bool     hasLongEdgeOnly  = false;
  //bool     hasEnc           = false;
  frCoord  enclosure        = 0;
  bool     isAbove          = false;
  //bool     hasParallel      = false;
  frCoord  parLength        = 0;
  frCoord  parWithin        = 0;

  istringstream istr(sIn);
  string word;

  string keyword = "";
  int stage = 0;
  while (istr >> word) {
    if (word == string("SPACING")) {
      double tmp;
      if (istr >> tmp) {
        cutSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<endl <<"  SPACING " <<tmp;
      }
    } else if (word == string("PARALLELWITHIN")) {
      double tmp;
      if (istr >> tmp) {
        within = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<" PARALLELWITHIN " <<tmp;
      }
    } else if (word == string("EXCEPTSAMENET")) {
      hasExceptSameNet = true;
      if (enableOutput) {
        cout <<" EXCEPTSAMENET";
      }
    } else if (word == string("CUTCLASS")) {
      hasCutClass = true;
      if (istr >> className) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<" CUTCLASS " <<className;
      }
    } else if (word == string("LONGEDGEONLY")) {
      hasLongEdgeOnly = true;
      if (enableOutput) {
        cout <<" LONGEDGEONLY";
      }
    } else if (word == string("ENCLOSURE")) {
      //hasEnc = true;
      double tmp;
      if (istr >> tmp) {
        enclosure = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<" ENCLOSURE " <<tmp;
      }
    } else if (word == string("ABOVE")) {
      isAbove = true;
      if (enableOutput) {
        cout <<" ABOVE";
      }
    } else if (word == string("BELOW")) {
      isAbove = false;
      if (enableOutput) {
        cout <<" BELOW";
      }
    } else if (word == string("PARALLEL")) {
      //hasParallel = true;
      double tmp;
      if (istr >> tmp) {
        parLength = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<" PARALLEL " <<tmp;
      }
      stage = 1;
    } else if (stage == 1 && word == string("WITHIN")) {
      double tmp;
      if (istr >> tmp) {
        parWithin = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacing_parallelWithin" <<endl;
      }
      if (enableOutput) {
        cout <<" WITHIN " <<tmp;
      }
    } else {
      if (enableOutput) {
        cout <<" " <<word;
      }
    }
  }

  if (skip) {
    ;
  } else {
    auto pw = make_shared<frLef58CutSpacingParallelWithinConstraint>();
    pw->setParallelWithin(within, hasExceptSameNet);
    if (hasCutClass) {
      pw->setClassName(className);
      pw->setLongEdgeOnly(hasLongEdgeOnly);
      pw->setEnclosure(enclosure, isAbove);
      pw->setParallel(parLength, parWithin);
    }

    auto con = make_shared<frLef58CutSpacingConstraint>();
    con->setCutSpacing(cutSpacing);
    con->setParallelWithinConstraint(pw);
    tmpLayer->lef58CutSpacingConstraints.push_back(con);
    ((io::Parser*)data)->tech->addConstraint(con);
  }
  return 0;
}

int io::Parser::getLef58CutSpacingTable(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"  PROPERTY LEF58_SPACINGTABLE \"";
  }
  istringstream istr(sIn);
  string word;
  stringstream ss;
  while (istr >> word) {
    if (word == string("SPACINGTABLE")) {
      ss.str("");
      ss <<word;
    } else if (word == ";") {
      ss <<" " <<word;
      getLef58CutSpacingTable_helper(data, tmpLayer, ss.str());
    } else {
      ss <<" " <<word;
    }
  }
  if (enableOutput) {
    cout <<"\" ;" <<endl;
  }
  return 0;
}

int io::Parser::getLef58CutSpacingTable_helper(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  //bool enableOutput = false;
  //if (enableOutput) {
  //  cout <<endl <<"  PROPERTY LEF58_SPACINGTABLE \"" <<endl;
  //}

  string keyword = "";

  istringstream istr(sIn);
  string word;
  stringstream ss;
  while (istr >> word) {
    if (word == string("SPACINGTABLE")) {
      keyword = "";
      ss.str("");
      ss <<word;
      //if (enableOutput) {
      //  cout <<"  SPACINGTABLE";
      //}
    } else if (word == string("CENTERSPACING")) {
      //if (enableOutput) {
      //  cout <<" CENTERSPACING";
      //}
      keyword = "CENTERSPACING";
      ss <<" " <<word;
    } else if (word == string("ORTHOGONAL")) {
      //if (enableOutput) {
      //  cout <<" CENTERSPACING";
      //}
      keyword = "ORTHOGONAL";
      ss <<" " <<word;
    } else {
      ss <<" " <<word;
    }
  }

  if (keyword == "CENTERSPACING") {
    ; //skip center spacing rules
  } else if (keyword == "ORTHOGONAL") {
    ; //skip orthogonal rules
  } else {
    getLef58CutSpacingTable_others(data, tmpLayer, ss.str());
  }
  
  //if (enableOutput) {
  //  cout <<"\" ;" <<endl;
  //}

  return 0;

}

int io::Parser::getLef58CutSpacingTable_default(void *data, frLayer* tmpLayer, const string &sIn, 
    const shared_ptr<frLef58CutSpacingTableConstraint> &con) {
  //cout <<sIn <<endl;
  //bool enableOutput = true;
  bool enableOutput = false;
  
  frCoord defaultCutSpacing = 0;

  istringstream istr(sIn);
  string word;

  while (istr >> word) {
    if (word == string("DEFAULT")) {
      double tmp;
      if (istr >> tmp) {
        defaultCutSpacing = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacingTable_default" <<endl;
      }
      if (enableOutput) {
        cout <<endl <<"  DEFAULT " <<tmp;
      }
    } else {
      ; // skip unknown rules
    }
  }

  con->setDefaultCutSpacing(defaultCutSpacing);
  return 0;
}

int io::Parser::getLef58CutSpacingTable_prl(void *data, frLayer* tmpLayer, const string &sIn, 
    const shared_ptr<frLef58CutSpacingTableConstraint> &con) {
  //cout <<sIn <<endl;
  //bool enableOutput = true;
  bool enableOutput = false;
  
  frCoord prl          = 0;
  bool    isHorizontal = false;
  bool    isVertical   = false;
  bool    isMaxXY      = false;

  istringstream istr(sIn);
  string word;

  while (istr >> word) {
    if (word == string("PRL")) {
      double tmp;
      if (istr >> tmp) {
        prl = frCoord(round(tmp * ((io::Parser*)data)->tech->getDBUPerUU()));
      } else {
        cout <<"Error: getLef58CutSpacingTable_prl" <<endl;
      }
      if (enableOutput) {
        cout <<" PRL " <<tmp;
      }
    } else if (word == string("HORIZONTAL")) {
      isHorizontal = true;
      if (enableOutput) {
        cout <<" HORIZONTAL";
      }
    } else if (word == string("VERTICAL")) {
      isVertical = true;
      if (enableOutput) {
        cout <<" VERTICAL";
      }
    } else if (word == string("MAXXY")) {
      isMaxXY = true;
      if (enableOutput) {
        cout <<" MAXXY";
      }
    } else {
      ; // skip unknown rules
    }
  }

  auto ptr = make_shared<frLef58CutSpacingTablePrlConstraint>();
  ptr->setPrl(prl);
  ptr->setHorizontal(isHorizontal);
  ptr->setVertical(isVertical);
  ptr->setMaxXY(isMaxXY);
  con->setPrlConstraint(ptr);

  return 0;
}

int io::Parser::getLef58CutSpacingTable_layer(void *data, frLayer* tmpLayer, const string &sIn, 
    const shared_ptr<frLef58CutSpacingTableConstraint> &con, frLayerNum &secondLayerNum) {

  //bool enableOutput = true;
  bool enableOutput = false;

  frString secondLayerName    = "";
  bool     isNonZeroEnclosure = false;

  istringstream istr(sIn);
  string word;

  while (istr >> word) {
    if (word == string("LAYER")) {
      if (istr >> secondLayerName) {
        ;
      } else {
        cout <<"Error: getLef58CutSpacingTable_layer" <<endl;
      }
      if (enableOutput) {
        cout <<" LAYER " <<secondLayerName;
      }
    } else if (word == string("NONZEROENCLOSURE")) {
      isNonZeroEnclosure = true;
      if (enableOutput) {
        cout <<endl <<"  NONZEROENCLOSURE";
      }
    } else {
      ; // skip unknown rules
    }
  }

  auto ptr = make_shared<frLef58CutSpacingTableLayerConstraint>();
  //cout <<secondLayerName <<endl <<flush;
  secondLayerNum = ((io::Parser*)data)->tech->name2layer.at(secondLayerName)->getLayerNum();
  ptr->setSecondLayerNum(secondLayerNum);
  ptr->setNonZeroEnc(isNonZeroEnclosure);
  con->setLayerConstraint(ptr);

  return 0;
}


int io::Parser::getLef58CutSpacingTable_cutClass(void *data, frLayer* tmpLayer, const string &sIn, 
    const shared_ptr<frLef58CutSpacingTableConstraint> &con, bool hasSecondLayer, frLayerNum secondLayerNum) {
  //bool enableOutput = true;
  bool enableOutput = false;

  auto defaultCutSpacing = con->getDefaultCutSpacing();
  // 2d spacing table
  //frCollection<frString> rowVals, colVals;
  frCollection<frCollection<pair<frCoord, frCoord> > > tblVals;
  //frCollection<pair<frCoord, frCoord> > tblRowVals;

  //cout <<endl <<sIn <<endl;
  // check numRows and numCols
  istringstream istr1(sIn);
  string word;
  int numCols = 0;
  int numRows = 0;
  bool isPrevNum   = false;
  while (istr1 >> word) {
    // "-" is treated as number 0
    if (word == "-") {
      word = "0";
    }
    stringstream tmpss(word);
    double tmpd;
    // is a number
    if (tmpss >> tmpd) {
      isPrevNum = true;
      numCols++;
    // is a string
    } else {
      if (word == ";") {
        numRows++;
      } else if (isPrevNum) {
        numRows++;
        numCols = 0;
      }
      isPrevNum = false;
    }
  }

  numCols /= 2;
  //cout <<endl <<"#rows/cols = " <<numRows <<"/" <<numCols <<endl;
  
  vector<frString> colNames;
  vector<int>      dupColNames; //duplicate side and all
  
  vector<frString> rowNames;
  vector<int>      dupRowNames; //duplicate side and all
  
  vector<vector<pair<frCoord, frCoord> > > tmpTbl;
  vector<pair<frCoord, frCoord> > tmpTblRow;
  
  istringstream istr2(sIn);
  word = "";
  int stage = 0; // 0 = read columns; 1 = read rows
  int readNum = 0; // numbers read in a row
  while (istr2 >> word) {
    // "-" is treated as number 0
    //if (word == ";") {
    //  cout <<"found ;" <<endl;
    //}
    if (word == "-") {
      word = to_string(defaultCutSpacing * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU());
    }
    if (word == "CUTCLASS") {
      if (enableOutput) {
        cout <<endl <<"  CUTCLASS";
      }
    // read numCols times
    } else if ((int)colNames.size() < numCols) {
      if (word == "SIDE" || word == "END") {
        *(--colNames.end()) = *(--colNames.end()) + word;
        *(--dupColNames.end()) = 1;
      } else {
        colNames.push_back(word);
        dupColNames.push_back(2);
      }
      if (enableOutput) {
        cout <<" " <<word;
      }
      //cout <<"testX" <<endl;
    } else if (stage == 0 && (int)colNames.size() == numCols) {
      // last word of column
      if (word == "SIDE" || word == "END") {
        *(--colNames.end()) = *(--colNames.end()) + word;
        *(--dupColNames.end()) = 1;
        if (enableOutput) {
          cout <<" " <<word;
        }
      // first word of row
      } else {
        rowNames.push_back(word);
        dupRowNames.push_back(2);
        if (enableOutput) {
          cout <<endl <<"  " <<word;
        }
      }
      stage = 1;
      //cout <<"testXX" <<endl;
    } else if (word == ";") {
      if (enableOutput) {
        cout <<" ;";
      }
      tmpTbl.push_back(tmpTblRow);
      //cout <<"testXXX" <<endl;
    } else if (stage == 1) {
      if (word == "SIDE" || word == "END") {
        *(--rowNames.end()) = *(--rowNames.end()) + word;
        *(--dupRowNames.end()) = 1;
        if (enableOutput) {
          cout <<" " <<word;
        }
      } else {
        stringstream ss(word);
        double firstNum;
        //cout <<"test: " <<word <<endl;
        // number
        if (ss >> firstNum) {
          frCoord val1 = frCoord(round(firstNum * ((io::Parser*)data)->tech->getDBUPerUU()));
          string tmpS;
          if (istr2 >> tmpS) {
            ;
          } else {
            cout <<"Error: getLef58CutSpacingTable_cutClass" <<endl;
          }
          stringstream tmpSS(tmpS);
          double secondNum;
          if (tmpSS >> secondNum) {
            frCoord val2 = frCoord(round(secondNum * ((io::Parser*)data)->tech->getDBUPerUU()));
            tmpTblRow.push_back(make_pair(val1, val2));
            if (enableOutput) {
              cout <<" " <<firstNum <<" " <<secondNum;
            }
          } else {
            // the number is "-", use default spacing
            frCoord val2 = defaultCutSpacing;
            tmpTblRow.push_back(make_pair(val1, val2));
            if (enableOutput) {
              cout <<" " <<firstNum <<" " <<defaultCutSpacing * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
            }
          }
          readNum += 1;
        // first word
        } else {
          rowNames.push_back(word);
          dupRowNames.push_back(2);
          if (enableOutput) {
            cout <<endl <<"  " <<word;
          }
          if (readNum) {
            tmpTbl.push_back(tmpTblRow);
            tmpTblRow.clear();
          }
          readNum = 0;
        }
      }
      //cout <<"testXXXX" <<endl;
    }
  }

  //cout <<endl <<"column:";
  //for (auto &str: colNames) {
  //  cout <<" " <<str;
  //}
  //
  //cout <<endl <<"row:";
  //for (auto &str: rowNames) {
  //  cout <<" " <<str;
  //}

  //cout <<endl <<"table: ";
  //for (auto &v1: tmpTbl) {
  //  cout <<"test here";
  //  cout <<endl <<"    ";
  //  for (auto &v2: v1) {
  //    cout <<"test here";
  //    cout <<" " <<v2.first <<" " <<v2.second;
  //  }
  //}
  //cout <<flush;

  vector<frString> expColNames;
  //cout <<endl <<"new expand column:";
  for (int i = 0; i < (int)colNames.size(); i++) {
    if (dupColNames.at(i) == 2) {
      string name1 = colNames.at(i) + "SIDE";
      string name2 = colNames.at(i) + "END";
      expColNames.push_back(name1);
      expColNames.push_back(name2);
      //cout <<" " <<name1 <<" " <<name2;
    } else {
      string name = colNames.at(i);
      expColNames.push_back(name);
      //cout <<" " <<name;
    };
  }
  
  vector<frString> expRowNames;
  //cout <<endl <<"new expand rows:";
  for (int i = 0; i < (int)rowNames.size(); i++) {
    if (dupRowNames.at(i) == 2) {
      string name1 = rowNames.at(i) + "SIDE";
      string name2 = rowNames.at(i) + "END";
      expRowNames.push_back(name1);
      expRowNames.push_back(name2);
      //cout <<" " <<name1 <<" " <<name2;
    } else {
      string name = rowNames.at(i);
      expRowNames.push_back(name);
      //cout <<" " <<name;
    };
  }

  vector<vector<pair<frCoord, frCoord> > > expTmpTbl;
  for (int i = 0; i < (int)rowNames.size(); i++) {
    vector<pair<frCoord, frCoord> > expTmpTblRow;
    for (int j = 0; j < (int)colNames.size(); j++) {
      expTmpTblRow.push_back(tmpTbl.at(i).at(j));
      if (dupColNames.at(j) == 2) {
        expTmpTblRow.push_back(tmpTbl.at(i).at(j));
      }
    }
    expTmpTbl.push_back(expTmpTblRow);
    if (dupRowNames.at(i) == 2) {
      expTmpTbl.push_back(expTmpTblRow);
    }
  }
  //cout <<"new expand table: ";
  //for (auto &v1: expTmpTbl) {
  //  cout <<endl <<"    ";
  //  for (auto &v2: v1) {
  //    cout <<" " <<v2.first <<" " <<v2.second;
  //  }
  //}

  vector<pair<frString, int> > expColNames_helper;
  for (int i = 0; i < (int)expColNames.size(); i++) {
    expColNames_helper.push_back(make_pair(expColNames.at(i), i));
  }
  sort(expColNames_helper.begin(), expColNames_helper.end(), 
       [](const pair<frString, int> &a, const pair<frString, int> &b) 
       {return a.first < b.first;});
  sort(expColNames.begin(), expColNames.end());

  //cout <<endl <<"sorted expand column:";
  //for (auto &it: expColNames_helper) {
  //  cout <<" " <<it.first;
  //}

  vector<pair<frString, int> > expRowNames_helper;
  for (int i = 0; i < (int)expRowNames.size(); i++) {
    expRowNames_helper.push_back(make_pair(expRowNames.at(i), i));
  }
  sort(expRowNames_helper.begin(), expRowNames_helper.end(), 
       [](const pair<frString, int> &a, const pair<frString, int> &b) 
       {return a.first < b.first;});
  sort(expRowNames.begin(), expRowNames.end());

  //cout <<endl <<"sorted expand row:";
  //for (auto &it: expRowNames_helper) {
  //  cout <<" " <<it.first;
  //}

  tblVals = expTmpTbl;
  //cout <<endl <<"sorted tbl:";
  for (int i = 0; i < (int)expRowNames_helper.size(); i++) {
    //cout <<endl;
    for (int j = 0; j < (int)expColNames_helper.size(); j++) {
      int orig_i = expRowNames_helper.at(i).second;
      int orig_j = expColNames_helper.at(j).second;
      tblVals.at(i).at(j) = expTmpTbl.at(orig_i).at(orig_j);
      //cout <<" " <<tblVals.at(i).at(j).first <<" " <<tblVals.at(i).at(j).second;
    }
  }

  string rowName("CUTCLASS");
  string colName("CUTCLASS");
  auto ptr = make_shared<fr2DLookupTbl<frString, frString, pair<frCoord, frCoord> > >(rowName, 
      expRowNames, colName, expColNames, tblVals);
  con->setCutClassTbl(ptr);

  return 0;
}

// lefdef ref 2nd spacing table, no orthogonal case
int io::Parser::getLef58CutSpacingTable_others(void *data, frLayer* tmpLayer, const string &sIn) {
  //bool enableOutput = true;
  bool enableOutput = false;

  istringstream istr(sIn);
  string word;

  stringstream ssDefault;
  stringstream ssLayer;    
  stringstream ssPrl;
  stringstream ssCutClass; 
  
  string keyword = "";
  while (istr >> word) {
    if (word == string("SPACINGTABLE")) {
      if (enableOutput) {
        cout <<endl <<"  SPACINGTABLE";
      }
    } else if (word == string("DEFAULT")) {
      keyword = "DEFAULT";
      ssDefault <<word;
    } else if (word == string("SAMEMASK")) {
      keyword = "SAMEMASK";
    } else if (word == string("SAMENET") || word == string("SAMEMETAL") || word == string("SAMEVIA")) {
      keyword = "SAMENETMETALVIA";
    } else if (word == string("LAYER")) {
      keyword = "LAYER";
      ssLayer <<word;
    } else if (word == string("CENTERTOCENTER")) {
      keyword = "CENTERTOCENTER";
    } else if (word == string("CENTERANDEDGE")) {
      keyword = "CENTERANDEDGE";
    } else if (word == string("PRL")) {
      keyword = "PRL";
      ssPrl <<"PRL";
    } else if (word == string("PRLTWOSIDES")) {
      keyword = "PRLTWOSIDES";
    } else if (word == string("ENDEXTENSION")) {
      keyword = "ENDEXTENSION";
    } else if (word == string("EXACTALIGNEDSPACING")) {
      keyword = "EXACTALIGNEDSPACING";
    } else if (word == string("NONOPPOSITEENCLOSURESPACING")) {
      keyword = "NONOPPOSITEENCLOSURESPACING";
    } else if (word == string("OPPOSITEENCLOSURERESIZESPACING")) {
      keyword = "OPPOSITEENCLOSURERESIZESPACING";
    } else if (word == string("CUTCLASS")) {
      keyword = "CUTCLASS";
      ssCutClass <<word;
    } else {
      if (keyword == "DEFAULT") {
        ssDefault <<" " <<word;
      } else if (keyword == "CUTCLASS") {
        ssCutClass <<" " <<word;
      } else if (keyword == "PRL") {
        ssPrl <<" " <<word;
      } else if (keyword == "LAYER") {
        ssLayer <<" " <<word;
      } else {
        ;
      }
    }
  }

  auto con = make_shared<frLef58CutSpacingTableConstraint>();

  bool hasSecondLayer       = false;
  frLayerNum secondLayerNum = 0;

  bool isFirstViaLayerHavingSecondLayerNum = false;

  if (ssDefault.str() != "") {
    getLef58CutSpacingTable_default(data, tmpLayer, ssDefault.str(), con);
  }
  if (ssPrl.str() != "") {
    getLef58CutSpacingTable_prl(data, tmpLayer, ssPrl.str(), con);
  }
  if (ssLayer.str() != "") {
    if (tmpLayer->getLayerNum() == 1) {
      isFirstViaLayerHavingSecondLayerNum = true;
    } else {
      getLef58CutSpacingTable_layer(data, tmpLayer, ssLayer.str(), con, secondLayerNum);
    }
  }
  if (ssCutClass.str() != "" && !isFirstViaLayerHavingSecondLayerNum) {
    getLef58CutSpacingTable_cutClass(data, tmpLayer, ssCutClass.str(), con, hasSecondLayer, secondLayerNum);
  }

  if (isFirstViaLayerHavingSecondLayerNum) {
    ;
  } else {
    tmpLayer->lef58CutSpacingTableConstraints.push_back(con);
    ((io::Parser*)data)->tech->addConstraint(con);

    //if (ssLayer.str() == "") {
    //  if (tmpLayer->getLayerNum() == 3) {
    //    auto result = con->getCutClassTbl()->find(string("VxaSIDE"), string("VxaEND"));
    //    cout <<"VxaSIDE/VxaEND " <<result.first <<" " <<result.second <<endl;
    //    result = con->getCutClassTbl()->find(string("VxaLRGEND"), string("VxaRECTSIDE"));
    //    cout <<"VxaLRGEND/VxaRECTSIDE " <<result.first <<" " <<result.second <<endl;
    //  }
    //}

  }

  return 0;
}

////int FlexRoute::getLef58CornerSpacing(void *data, const string &stringIn) {
////  istringstream istr(stringIn);
////  string word;
////  bool hasCornerSpacing = false;
////
////  bool hasConvexCorner  = false;
////  bool hasSameMask      = false;
////  bool hasCornerOnly    = false;
////  frUInt4 within        = 0;
////  bool hasExceptEol     = false;
////  frUInt4 eolWidth      = 0;
////
////  bool hasConcaveCorner = false;
////  bool hasMinLength     = false;
////  frUInt4 minLength     = 0;
////  bool hasExceptNotch   = false;
////  frUInt4 notchLength   = 0;
////
////  bool exceptSameNet   = false;
////  bool exceptSameMetal = false;
////
////  bool hasWidthSpacing = false;
////  frCollection< frCollection<frUInt4> > widthSpacing;
////  frCollection<frUInt4> tmpWidthSpacing;
////
////  bool doCornerSpacing = false;
////  bool doConvexCorner  = false;
////  bool doConcaveCorner = false;
////  bool doWidthSpacing  = false;
////
////  while (istr >> word) {
////    //cout <<" " <<word;
////
////    if (word == ";") {
////      doCornerSpacing = false;
////      doConvexCorner  = false;
////      doConcaveCorner = false;
////      doWidthSpacing  = false;
////      continue;
////    }
////
////    if (!doCornerSpacing && word == "CORNERSPACING") {
////      doCornerSpacing = true;
////      hasCornerSpacing = true;
////      cout <<"CORNERSPACING";
////      continue;
////    }
////    if (doCornerSpacing &&  word == "CONVEXCORNER") {
////      doConvexCorner  = true;
////      hasConvexCorner = true;
////      doConcaveCorner = false;
////      doWidthSpacing  = false;
////      cout <<endl <<"  CONVEXCORNER";
////      continue;
////    }
////    if (doCornerSpacing &&  word == "CONCAVECORNER") {
////      doConvexCorner  = false;
////      doConcaveCorner = true;
////      hasConcaveCorner = true;
////      doWidthSpacing  = false;
////      cout <<endl <<"  CONCAVECORNER";
////      continue;
////    }
////
////    if (doCornerSpacing &&  word == "EXCEPTSAMENET") {
////      doConvexCorner  = false;
////      doConcaveCorner = false;
////      doWidthSpacing  = false;
////      exceptSameNet   = true;
////      cout <<endl <<"  EXCEPTSAMENET";
////      continue;
////    }
////    
////    if (doCornerSpacing &&  word == "EXCEPTSAMEMETAL") {
////      doConvexCorner  = false;
////      doConcaveCorner = false;
////      doWidthSpacing  = false;
////      exceptSameMetal = true;
////      cout <<endl <<"  EXCEPTSAMEMETAL";
////      continue;
////    }
////
////    if (doCornerSpacing &&  word == "WIDTH") {
////      doConvexCorner  = false;
////      doConcaveCorner = false;
////      doWidthSpacing  = true;
////      hasWidthSpacing = true;
////      if (!tmpWidthSpacing.empty()) {
////        widthSpacing.push_back(tmpWidthSpacing);
////      }
////      tmpWidthSpacing.clear();
////      cout <<endl <<"  WIDTH";
////      continue;
////    }
////    
////    if (doConvexCorner && word == "SAMEMASK") {
////      hasSameMask = true;
////      cout <<" SAMEMASK";
////    }
////    if (doConvexCorner && word == "CORNERONLY") {
////      hasCornerOnly = true;
////      double tmpWithin;
////      istr >> tmpWithin;
////      within = round(tmpWithin * ((FlexRoute*)data)->units);
////      cout <<endl <<"    CORNERONLY " <<within * 1.0 / ((FlexRoute*)data)->units;
////    }
////    // currently do not support EXCEPTJOGLENGTH
////    if (doConvexCorner && word == "EXCEPTEOL") {
////      hasExceptEol = true;
////      double tmpEolWidth;
////      istr >> tmpEolWidth;
////      eolWidth = round(tmpEolWidth * ((FlexRoute*)data)->units);
////      cout <<endl <<"    EXCEPTEOL " <<eolWidth * 1.0 / ((FlexRoute*)data)->units;
////    }
////    
////    if (doConcaveCorner && word == "MINLENGTH") {
////      hasMinLength = true;
////      double tmpMinLength;
////      istr >> tmpMinLength;
////      minLength = round(tmpMinLength * ((FlexRoute*)data)->units);
////      cout <<endl <<"    MINLENGTH " <<minLength * 1.0 / ((FlexRoute*)data)->units;
////    }
////    if (doConcaveCorner && word == "EXCEPTNOTCH") {
////      hasExceptNotch = true;
////      double tmpNotchLength;
////      istr >> tmpNotchLength;
////      notchLength = round(tmpNotchLength * ((FlexRoute*)data)->units);
////      cout <<endl <<"    EXCEPTNOTCH " <<notchLength * 1.0 / ((FlexRoute*)data)->units;
////    }
////
////    if (doWidthSpacing &&  word == "SPACING") {
////      cout <<" SPACING";
////      continue;
////    }
////
////    if (doWidthSpacing &&  word != "SPACING") {
////      frUInt4 tmp = round(stod(word) * ((FlexRoute*)data)->units);
////      tmpWidthSpacing.push_back(tmp);
////      cout <<" " <<tmp * 1.0 / ((FlexRoute*)data)->units;
////      continue;
////    }
////
////  }
////
////  if (!tmpWidthSpacing.empty()) {
////    widthSpacing.push_back(tmpWidthSpacing);
////  }
////
////  cout <<endl;
////  return 0;
////}

int io::Parser::getLefLayers(lefrCallbackType_e type, lefiLayer* layer, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //bool enableDoubleCheck = true;
  //bool enableDoubleCheck = false;
  if (type != lefrLayerCbkType) {
    cout <<"Type is not lefrLayerCbkType!" <<endl;
    exit(1);
  }
  unique_ptr<frLayer> uLayer = make_unique<frLayer>();
  auto tmpLayer = uLayer.get();

  if (!strcmp(layer->type(), "ROUTING")) {
    if (enableOutput) {
      cout <<"LAYER "       <<layer->name() <<endl;
      cout <<"  TYPE      " <<layer->type() <<endl;
      cout <<"  DIRECTION " <<layer->direction() <<endl;
      //cout <<"  MINWIDTH  " <<layer->minwidth() <<endl;
      cout <<"  AREA      " <<layer->area() <<endl;
      cout <<"  WIDTH     " <<layer->width() <<endl;
    }
    tmpLayer->setLayerNum(((io::Parser*)data)->readLayerCnt++);
    tmpLayer->setName(layer->name());
    tmpLayer->setWidth(round(layer->width() * ((io::Parser*)data)->tech->getDBUPerUU()));
    if (layer->hasMinwidth()) {
      tmpLayer->setMinWidth(round(layer->minwidth() * ((io::Parser*)data)->tech->getDBUPerUU()));
    } else {
      tmpLayer->setMinWidth(tmpLayer->getWidth());
    }
    // add minWidth constraint
    auto minWidthConstraint = make_unique<frMinWidthConstraint>(tmpLayer->getMinWidth());
    tmpLayer->setMinWidthConstraint(minWidthConstraint.get());
    unique_ptr<frConstraint> minWidthTempPtr = std::move(minWidthConstraint);
    ((io::Parser*)data)->tech->addUConstraint(minWidthTempPtr);
    //tmpLayer->addConstraint(minWidthConstraint);

    tmpLayer->setType(frLayerTypeEnum::ROUTING);
    //tmpLayer.idx        = ((FlexRoute*)data)->readLayerCnt;
    //tmpLayer.layerName  = layer->name();
    //tmpLayer.type       = layer->type();
    if (!strcmp(layer->direction(), "HORIZONTAL")) {
      tmpLayer->setDir(frcHorzPrefRoutingDir);
    } else if (!strcmp(layer->direction(), "VERTICAL")) {
      tmpLayer->setDir(frcVertPrefRoutingDir);
    }
    tmpLayer->setPitch(round(layer->pitch() * ((io::Parser*)data)->tech->getDBUPerUU()));

    // Add short rule for every layer
    auto shortConstraint = make_unique<frShortConstraint>();
    tmpLayer->setShortConstraint(shortConstraint.get());
    unique_ptr<frConstraint> shortTempPtr = std::move(shortConstraint);
    //std::cout << "add shortConstraint to layer " <<tmpLayer->getName() << "\n";
    ((io::Parser*)data)->tech->addUConstraint(shortTempPtr);
    //tmpLayer->addConstraint(shortConstraint);


    //cout <<"number of props " <<layer->numProps() <<endl;
    for (int i = 0; i < layer->numProps(); i++) {
      if (string(layer->propName(i)) == string("LEF58_PROTRUSIONWIDTH") ||
          string(layer->propName(i)) == string("LEF58_MINSTEP") ||
          string(layer->propName(i)) == string("LEF58_ENCLOSURESPACING") ||
          string(layer->propName(i)) == string("LEF58_VOLTAGESPACING") ||
          string(layer->propName(i)) == string("LEF58_ANTENNAGATEPLUSDIFF") ||
          string(layer->propName(i)) == string("LEF58_ANTENNAGATEPWL") ||
          string(layer->propName(i)) == string("LEF58_ANTENNAGATEPWL") ||
          string(layer->propName(i)) == string("LEF58_FORBIDDENSPACING") 
         ) {
        ;
      } else {
        //cout <<"name:     " <<layer->propName(i) <<endl;
        //cout <<"value:    " <<layer->propValue(i) <<endl;
        //cout <<"number:   " <<layer->propNumber(i) <<endl;
        //cout <<"type:     " <<layer->propType(i) <<endl;
        //cout <<"isNumber: " <<layer->propIsNumber(i) <<endl;
        //cout <<"isString: " <<layer->propIsString(i) <<endl;
        //if (!strcmp(layer->propName(i), "LEF58_CORNERSPACING") && layer->propIsString(i)) {
        //  //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
        //  getLef58CornerSpacing(data, layer->propValue(i));
        //}
        if (!strcmp(layer->propName(i), "LEF58_SPACING") && layer->propIsString(i)) {
          //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
          //cout <<"name:     " <<layer->propName(i) <<endl;
          //cout <<"value:    " <<layer->propValue(i) <<endl;
          getLef58Spacing(data, tmpLayer, layer->propValue(i));
        } else if (!strcmp(layer->propName(i), "LEF57_SPACING") && layer->propIsString(i)) {
          getLef58Spacing(data, tmpLayer, layer->propValue(i));
        } else if (!strcmp(layer->propName(i), "LEF58_SPACINGTABLE") && layer->propIsString(i)) {
          //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
          //cout <<"name:     " <<layer->propName(i) <<endl;
          //cout <<"value:    " <<layer->propValue(i) <<endl;
          getLef58SpacingTable(data, tmpLayer, layer->propValue(i));
        } else {
          //cout <<"  name:     " <<layer->propName(i) <<endl;
        }
      }
    }

    // read minArea rule
    if (layer->hasArea()) {
      frCoord minArea = frCoord(round(layer->area() * ((io::Parser*)data)->tech->getDBUPerUU() * ((io::Parser*)data)->tech->getDBUPerUU()));
      unique_ptr<frConstraint> uCon = make_unique<frAreaConstraint>(minArea);
      auto rptr = static_cast<frAreaConstraint*>(uCon.get());
      ((io::Parser*)data)->tech->addUConstraint(uCon);
      //std::cout << "Add minArea constraint to " << tmpLayer->getName() << "\n";
      tmpLayer->setAreaConstraint(rptr);
    }

    if (layer->hasMinstep()) {
      if (layer->numMinstep() > 1) {
        std::cout << "ERROR: only one minStep rule should be defined for a given layer. Only the last one is checked\n";
      }
      for (int i = 0; i < layer->numMinstep(); ++i) {
        unique_ptr<frConstraint> uCon = make_unique<frMinStepConstraint>();
        auto rptr = static_cast<frMinStepConstraint*>(uCon.get());
        if (layer->hasMinstepType(i)) {
          if (strcmp(layer->minstepType(i), "INSIDECORNER") == 0) {
            rptr->setInsideCorner(true);
            rptr->setOutsideCorner(false);
            rptr->setStep(false);
          } else if (strcmp(layer->minstepType(i), "OUTSIDECORNER") == 0) {
            rptr->setInsideCorner(false);
            rptr->setOutsideCorner(true);
            rptr->setStep(false);
          } else if (strcmp(layer->minstepType(i), "STEP") == 0) {
            rptr->setInsideCorner(false);
            rptr->setOutsideCorner(false);
            rptr->setStep(true);
          }
        }
        if (layer->hasMinstepLengthsum(i)) {
          rptr->setMaxLength(frCoord(layer->minstepLengthsum(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
        }
        if (layer->hasMinstepMaxedges(i)) {
          rptr->setMaxEdges(layer->minstepMaxedges(i));
          rptr->setInsideCorner(true);
          rptr->setOutsideCorner(true);
          rptr->setStep(true);
        }
        rptr->setMinStepLength(layer->minstep(i) * ((io::Parser*)data)->tech->getDBUPerUU());
        ((io::Parser*)data)->tech->addUConstraint(uCon);
        //std::cout << "Add minStep constraint to " << tmpLayer->getName() << "\n";
        tmpLayer->setMinStepConstraint(rptr);
      }
    }

    // read spacing rule
    for (int i = 0; i < layer->numSpacing(); ++i) {
      //std::shared_ptr<frSpacingConstraint> minSpacingCosntraint;
      frCoord minSpacing = frCoord(round(layer->spacing(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
      // minSpacingCosntraint = make_shared<frMinSpacingConstraint>(minSpacing);
      if (layer->hasSpacingRange(i)) {
        cout <<" WARNING: hasSpacing Range unsupported" <<endl;
      } else if (layer->hasSpacingLengthThreshold(i)) {
        cout <<" WARNING: hasSpacingLengthThreshold unsupported" <<endl;
      } else if (layer->hasSpacingEndOfLine(i)) {
        // new
        unique_ptr<frConstraint> uCon = make_unique<frSpacingEndOfLineConstraint>();
        auto rptr = static_cast<frSpacingEndOfLineConstraint*>(uCon.get());
        if (enableOutput) {
          cout <<"  SPACING " <<layer->spacing(i) <<" ENDOFLINE " <<layer->spacingEolWidth(i)
               <<" WITHIN " <<layer->spacingEolWithin(i);
        }
        frCoord eolWidth = frCoord(round(layer->spacingEolWidth(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
        frCoord eolWithin = frCoord(round(layer->spacingEolWithin(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
        rptr->setMinSpacing(minSpacing);
        rptr->setEolWidth(eolWidth);
        rptr->setEolWithin(eolWithin);
        if (layer->hasSpacingParellelEdge(i)) {
          if (enableOutput) {
            cout <<" PARALLELEDGE " <<layer->spacingParSpace(i) <<" WITHIN " <<layer->spacingParWithin(i);
            if (layer->hasSpacingTwoEdges(i)) {
              cout <<" TWOEDGES";
            }
          }
          frCoord parSpace = frCoord(round(layer->spacingParSpace(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
          frCoord parWithin = frCoord(round(layer->spacingParWithin(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
          rptr->setParSpace(parSpace);
          rptr->setParWithin(parWithin);
          rptr->setTwoEdges(layer->hasSpacingTwoEdges(i));
        }
        if (enableOutput) {
          cout <<" ;" <<endl;
        }
        ((io::Parser*)data)->tech->addUConstraint(uCon);
        tmpLayer->addEolSpacing(rptr);
        
        // double check
        //if (enableDoubleCheck) {
        //  cout <<"@ SPACING " <<minSpacing <<" ENDOFLINE " <<eolWidth <<" WITHIN " <<eolWithin;
        //  if (rptr->hasParallelEdge()) {
        //    cout <<" PARALLELEDGE " <<rptr->getParSpace();
        //    cout <<" WITHIN " <<rptr->getParSpace();
        //    if (rptr->hasTwoEdges()) {
        //      cout <<" TWOEDGES";
        //    }
        //  }
        //  cout <<" ;" <<endl;
        //  cout <<"eol size = " <<tmpLayer->getEolSpacing().size() <<endl;
        //}
      } else if (layer->hasSpacingSamenet(i)) {
        cout <<" WARNING: hasSpacingSamenet unsupported" <<endl;
      } else if (layer->hasSpacingNotchLength(i)) {
        cout <<" WARNING: hasSpacingNotchLength unsupported" <<endl;
      } else if (layer->hasSpacingEndOfNotchWidth(i)) {
        cout <<" WARNING: hasSpacingEndOfNotchWidth unsupported" <<endl;
      } else { // min spacing
        // old
        if (enableOutput) {
          cout <<"  SPACING " <<layer->spacing(i) <<" ;" <<endl;
        }
        //auto spacingConstraint = make_shared<frSpacingConstraint>(minSpacing);
        //((io::Parser*)data)->tech->addConstraint(spacingConstraint);
        //tmpLayer->addConstraint(spacingConstraint);
        // new
        unique_ptr<frConstraint> uCon = make_unique<frSpacingConstraint>(minSpacing);
        auto rptr = uCon.get();
        ((io::Parser*)data)->tech->addUConstraint(uCon);
        tmpLayer->setMinSpacing(rptr);
      }
    }

    // read spacingTable
    for (int i = 0; i < layer->numSpacingTable(); ++i) {
      // old
      std::shared_ptr<frSpacingTableConstraint> spacingTableConstraint;
      auto spTable = layer->spacingTable(i);
      if (spTable->isInfluence()) {
        cout <<" WARNING: SPACINGTABLE INFLUENCE unsupported" <<endl;
      } else if (spTable->isParallel()) {
        // old
        shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > prlTbl;
        auto parallel = spTable->parallel();
        frCollection<frCoord> rowVals, colVals;
        frCollection<frCollection<frCoord> > tblVals;
        frCollection<frCoord> tblRowVals;
        frString rowName("WIDTH"), colName("PARALLELRUNLENGTH");
        if (enableOutput) {
          cout <<"  SPACINGTABLE" <<endl;
          cout <<"  PARALLELRUNLENGTH";
        }
        for (int j = 0; j < parallel->numLength(); ++j) {
          frCoord prl = frCoord(round(parallel->length(j) * ((io::Parser*)data)->tech->getDBUPerUU()));
          if (enableOutput) {
            cout <<" " <<prl * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
          }
          colVals.push_back(prl);
        }
        for (int j = 0; j < parallel->numWidth(); ++j) {
          frCoord width = frCoord(round(parallel->width(j) * ((io::Parser*)data)->tech->getDBUPerUU()));
          rowVals.push_back(width);
          if (enableOutput) {
            cout <<endl <<"  WIDTH " <<width * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
          }
          tblRowVals.clear();
          for (int k = 0; k < parallel->numLength(); ++k) {
            //frCoord prl = frCoord(round(parallel->length(k) * ((io::Parser*)data)->tech->getDBUPerUU()));
            frCoord spacing = frCoord(round(parallel->widthSpacing(j, k) * ((io::Parser*)data)->tech->getDBUPerUU()));
            tblRowVals.push_back(spacing);
            if (enableOutput) {
              cout <<" " <<spacing * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
            }
          }
          tblVals.push_back(tblRowVals);
        }
        if (enableOutput) {
          cout <<" ;" <<endl;
        }
        // old
        prlTbl = make_shared<fr2DLookupTbl<frCoord, frCoord, frCoord> >(rowName, rowVals, colName, colVals, tblVals);
        spacingTableConstraint = make_shared<frSpacingTableConstraint>(prlTbl);
        ((io::Parser*)data)->tech->addConstraint(spacingTableConstraint);
        tmpLayer->addConstraint(spacingTableConstraint);
        //if (tmpLayer->getLayerNum() == 2) {
        //  prlTbl->printTbl();
        //  cout <<"should crash here" <<flush <<prlTbl->find(140, 160) <<flush <<endl;
        //  exit(0);
        //}

        // new
        unique_ptr<frConstraint> uCon = make_unique<frSpacingTablePrlConstraint>(fr2DLookupTbl(rowName, rowVals, colName, colVals, tblVals));
        auto rptr = static_cast<frSpacingTablePrlConstraint*>(uCon.get());
        //cout <<"@test " <<rptr->find(0.47*2000, 0.47*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.47*2000, 0.48*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.46*2000, 0.48*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.47*2000, 0.46*2000) / 2000.0 <<endl;
        ((io::Parser*)data)->tech->addUConstraint(uCon);
        tmpLayer->setMinSpacing(rptr);
      } else { // two width spacing rule
        auto tw = spTable->twoWidths();
        //bool hasPrl = false;
        frCoord defaultPrl = -abs(frCoord(round(tw->widthSpacing(0,0) * ((io::Parser*)data)->tech->getDBUPerUU())));
        //cout <<"default prl: " <<defaultPrl <<endl;
        frCollection<frSpacingTableTwRowType> rowVals, colVals;
        frCollection<frCollection<frCoord> > tblVals;
        frCollection<frCoord> tblRowVals;
        frString rowName("WIDTH1PRL"), colName("WIDTH2PRL");
        if (enableOutput) {
          cout <<"  SPACINGTABLE TWOWIDTHS";
        }
        for (int j = 0; j < tw->numWidth(); ++j) {
          frCoord width = frCoord(round(tw->width(j) * ((io::Parser*)data)->tech->getDBUPerUU()));
          frCoord prl   = defaultPrl;
          if (enableOutput) {
            cout <<endl <<"    WIDTH " <<tw->width(j);
          }
          if (tw->hasWidthPRL(j)) {
            if (enableOutput) {
              cout <<" PRL " <<tw->widthPRL(j);
            }
            prl = frCoord(round(tw->widthPRL(j) * ((io::Parser*)data)->tech->getDBUPerUU()));
            defaultPrl = prl;
            //hasPrl = true;
          } else {
            //if (!hasPrl) {
            //  defaultPrl = -abs(frCoord(round(tw->widthSpacing(j,0) * ((io::Parser*)data)->tech->getDBUPerUU())));
            //  prl = defaultPrl;
            //}
          }
          colVals.push_back(frSpacingTableTwRowType(width, prl));
          rowVals.push_back(frSpacingTableTwRowType(width, prl));
          tblRowVals.clear();
          for (int k = 0; k < tw->numWidthSpacing(j); k++) {
            if (enableOutput) {
              cout <<" " <<tw->widthSpacing(j, k);
            }
            frCoord spacing = frCoord(round(tw->widthSpacing(j, k) * ((io::Parser*)data)->tech->getDBUPerUU()));
            tblRowVals.push_back(spacing);
          }
          tblVals.push_back(tblRowVals);
        }
        if (enableOutput) {
          cout <<" ;" <<endl;
        }
        unique_ptr<frConstraint> uCon = make_unique<frSpacingTableTwConstraint>(fr2DLookupTbl(rowName, rowVals, colName, colVals, tblVals));
        auto rptr = static_cast<frSpacingTableTwConstraint*>(uCon.get());
        //cout <<"@test " <<rptr->find(0.156*2000, 0.072*2000, 0.000*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.156*2000, 0.156*2000, 0.000*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.209*2000, 0.072*2000, 0.300*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.209*2000, 0.156*2000, 0.300*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.000*2000, 0.000*2000, -0.001*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.000*2000, 0.000*2000, 0.000*2000) / 2000.0 <<endl;
        //cout <<"@test " <<rptr->find(0.000*2000, 0.000*2000, 0.300*2000) / 2000.0 <<endl;
        ((io::Parser*)data)->tech->addUConstraint(uCon);
        tmpLayer->setMinSpacing(rptr);
        //vector<int> tttttest = {0,1,2,3,4,5,6,7};
        //auto it1 = upper_bound(tttttest.begin(), tttttest.end(), -1);
        //--it1;
        //auto it2 = upper_bound(tttttest.begin(), tttttest.end(), 3);
        //--it2;
        //cout <<"dist1: " <<distance(tttttest.begin(), it1) <<endl;
        //cout <<"dist2: " <<distance(tttttest.begin(), it2) <<endl;

      }
    }

    for (int i = 0; i < layer->numMinimumcut(); i++) {
      double dbu = ((io::Parser*)data)->tech->getDBUPerUU();
      unique_ptr<frConstraint> uCon = make_unique<frMinimumcutConstraint>();
      auto rptr = static_cast<frMinimumcutConstraint*>(uCon.get());
      
      //if (enableOutput) {
      //  cout <<"  MINIMUMCUT " <<layer->minimumcut(i) <<" WIDTH " <<layer->minimumcutWidth(i);
      //}
      rptr->setNumCuts(layer->minimumcut(i));
      rptr->setWidth(frCoord(round(layer->minimumcutWidth(i) * dbu)));

      if (layer->hasMinimumcutWithin(i)) {
        //if (enableOutput) {
        //  cout <<" WITHIN " <<layer->minimumcutWithin(i);
        //}
        rptr->setWithin(frCoord(round(layer->minimumcutWithin(i) * dbu)));
      }

      if (layer->hasMinimumcutConnection(i)) {
        //if (enableOutput) {
        //  cout <<" " <<layer->minimumcutConnection(i);
        //}
        if (strcmp(layer->minimumcutConnection(i), "FROMABOVE") == 0) {
          rptr->setConnection(frMinimumcutConnectionEnum::FROMABOVE);
        } else if (strcmp(layer->minimumcutConnection(i), "FROMBELOW") == 0) {
          rptr->setConnection(frMinimumcutConnectionEnum::FROMBELOW);
        } else {
          cout <<"Error: layer minimumcut unsupporterd connection type" <<endl;
          exit(1);
        }
      }
      
      // hasMinimumcutNumCuts actually outputs whether there is [LENGTH length WITHIN distance]
      if (layer->hasMinimumcutNumCuts(i)) {
        //if (enableOutput) {
        //  cout <<" LENGTH " <<layer->minimumcutLength(i) <<" WITHIN " <<layer->minimumcutDistance(i);
        //}
        rptr->setLength(frCoord(round(layer->minimumcutLength(i) * dbu)), frCoord(round(layer->minimumcutDistance(i) * dbu)));
      }

      ((io::Parser*)data)->tech->addUConstraint(uCon);
      tmpLayer->addMinimumcutConstraint(rptr);
      //if (enableOutput) {
      //  cout <<" ;" <<endl;
      //}
      if (enableOutput) {
        cout <<"  MINIMUMCUT " <<rptr->getNumCuts() <<" WIDTH " <<rptr->getWidth() / dbu;
        if (rptr->hasWithin()) {
          cout <<" WITHIN " <<rptr->getCutDistance() / dbu;
        }
        if (rptr->hasConnection()) {
          switch(rptr->getConnection()) {
            case frMinimumcutConnectionEnum::FROMABOVE: 
              cout <<" FROMABOVE"; 
              break;
            case frMinimumcutConnectionEnum::FROMBELOW: 
              cout <<" FROMBELOW";
              break;
            default: 
              cout <<" UNKNOWN";
          }
        }
        if (rptr->hasLength()) {
          cout <<" LENGTH " <<rptr->getLength() / dbu <<" WITHIN " <<rptr->getDistance() / dbu;
        }
        cout <<" ;" <<endl;
      }
    }

    

    ((io::Parser*)data)->tech->addLayer(uLayer);
  } else if (strcmp(layer->type(), "CUT") == 0 && ((io::Parser*)data)->readLayerCnt > 0) {
    //cout <<"LAYER " <<layer->name() <<endl <<flush;
    if (enableOutput) {
      cout <<"LAYER "       <<layer->name() <<endl;
      cout <<"  TYPE      " <<layer->type() <<endl;
    }
    tmpLayer->setLayerNum(((io::Parser*)data)->readLayerCnt++);
    tmpLayer->setName(layer->name());
    tmpLayer->setType(frLayerTypeEnum::CUT);
    //tmpLayer.idx        = ((FlexRoute*)data)->readLayerCnt;
    //tmpLayer.layerName  = layer->name();
    //tmpLayer.type       = layer->type();
    //if (layer->numSpacing() > 1) {
    //  cout << "NumSpacing = " << layer->numSpacing() << endl;
    //  tmpLayer.minSpacing = 0;
    //  // cout <<"Unsupported CUT spacing!!!" <<endl;
    //  // exit(2);
    //}
    //// TODO: need to handle multiple spacing rules
    //for (int i = 0; i < layer->numSpacing(); ++i) {
    //  if (layer->hasSpacingAdjacent(i)) {
    //    cout <<"  SPACING " <<layer->spacing(i) <<" ADJACENTCUTS " <<layer->spacingAdjacentCuts(i)
    //         <<" WITHIN " <<layer->spacingAdjacentWithin(i) <<endl;
    //  } else {
    //    tmpLayer.minSpacing = max(tmpLayer.minSpacing, loc_t(round(layer->spacing(i)* ((FlexRoute*)data)->units)));
    //    cout <<"  SPACING " <<layer->spacing(i) <<endl;
    //  }
    //}
    //((FlexRoute*)data)->layers.push_back(tmpLayer);
    //
    //((FlexRoute*)data)->layer2Idx[tmpLayer.layerName] = tmpLayer.idx;
    //
    //++(((FlexRoute*)data)->readLayerCnt);

    auto shortConstraint = make_shared<frShortConstraint>();
    std::cout << "add shortConstraint to layer " <<tmpLayer->getName() << "\n";
    ((io::Parser*)data)->tech->addConstraint(shortConstraint);
    tmpLayer->addConstraint(shortConstraint);
    tmpLayer->setShortConstraint(shortConstraint.get());

    // read spacing constraint
    for (int i = 0; i < layer->numSpacing(); ++i) {
      std::shared_ptr<frCutSpacingConstraint> cutSpacingConstraint;
      frCoord cutArea = frCoord(round(layer->spacingArea(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
      frCoord cutSpacing = frCoord(round(layer->spacing(i) * ((io::Parser*)data)->tech->getDBUPerUU()));
      bool centerToCenter = layer->hasSpacingCenterToCenter(i);
      bool sameNet = layer->hasSpacingSamenet(i);
      bool stack = layer->hasSpacingLayerStack(i);
      bool exceptSamePGNet = layer->hasSpacingSamenetPGonly(i);
      bool parallelOverlap = layer->hasSpacingParallelOverlap(i);
      frString secondLayerName;
      int adjacentCuts = layer->spacingAdjacentCuts(i);
      frCoord cutWithin = frCoord(round(layer->spacingAdjacentWithin(i) * ((io::Parser*)data)->tech->getDBUPerUU()));

      // std::cout << cutSpacing << " " << centerToCenter << " " << sameNet << " " << stack << " " << exceptSamePGNet 
      //           << " " << parallelOverlap << " " << secondLayerName << " " << adjacentCuts << " " << cutWithin << "\n";

      // std::cout << "raw cutArea = " << layer->spacingArea(i) << "\n";
      // std::cout << "cutArea = " << cutArea << "\n";
      // initialize for invalid variables
      cutArea = (cutArea == 0) ? -1 : cutArea;
      cutWithin = (cutWithin == 0) ? -1 : cutWithin;
      adjacentCuts = (adjacentCuts == 0) ? -1 : adjacentCuts;
      // std::cout << "cutArea = " << cutArea << "\n";


      // if (layer->hasSpacingAdjacent(i)) {

      // } else {

      // }

      cutSpacingConstraint = make_shared<frCutSpacingConstraint>(cutSpacing,
                                                              centerToCenter, 
                                                              sameNet, 
                                                              secondLayerName, 
                                                              stack,
                                                              adjacentCuts,
                                                              cutWithin,
                                                              exceptSamePGNet,
                                                              parallelOverlap,
                                                              cutArea);

      ((io::Parser*)data)->tech->addConstraint(cutSpacingConstraint);
      tmpLayer->addConstraint(cutSpacingConstraint);
      tmpLayer->addCutConstraint(cutSpacingConstraint.get());

    }

    // lef58
    //cout <<"number of props " <<layer->numProps() <<endl;
    for (int i = 0; i < layer->numProps(); i++) {
      if (string(layer->propName(i)) == string("LEF58_ENCLOSUREEDGE") ||
          string(layer->propName(i)) == string("LEF58_ENCLOSURE") ||
          string(layer->propName(i)) == string("LEF58_ENCLOSURETABLE")
         ) {
        ;
      } else {
        //cout <<"name:     " <<layer->propName(i) <<endl;
        //cout <<"value:    " <<layer->propValue(i) <<endl;
        //cout <<"number:   " <<layer->propNumber(i) <<endl;
        //cout <<"type:     " <<layer->propType(i) <<endl;
        //cout <<"isNumber: " <<layer->propIsNumber(i) <<endl;
        //cout <<"isString: " <<layer->propIsString(i) <<endl;
        //if (!strcmp(layer->propName(i), "LEF58_CORNERSPACING") && layer->propIsString(i)) {
        //  //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
        //  getLef58CornerSpacing(data, layer->propValue(i));
        //}
        if (!strcmp(layer->propName(i), "LEF58_CUTCLASS") && layer->propIsString(i)) {
          //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
          //cout <<"name:     " <<layer->propName(i) <<endl;
          //cout <<"value:    " <<layer->propValue(i) <<endl;
          getLef58CutClass(data, tmpLayer, layer->propValue(i));
        } else if (!strcmp(layer->propName(i), "LEF58_SPACING") && layer->propIsString(i)) {
          //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
          //cout <<"name:     " <<layer->propName(i) <<endl;
          //cout <<"value:    " <<layer->propValue(i) <<endl;
          getLef58CutSpacing(data, tmpLayer, layer->propValue(i));
        } else if (!strcmp(layer->propName(i), "LEF58_SPACINGTABLE") && layer->propIsString(i)) {
          //cout <<"start parsing LEF58_CORNERSPACING" <<endl;
          //cout <<"name:     " <<layer->propName(i) <<endl;
          //cout <<"value:    " <<layer->propValue(i) <<endl;
          getLef58CutSpacingTable(data, tmpLayer, layer->propValue(i));
        } else {
          cout <<"  name:     " <<layer->propName(i) <<endl;
        }
      }
    }
    ((io::Parser*)data)->tech->addLayer(uLayer);
  } else {
    ;
  }

  return 0;
}

int io::Parser::getLefMacros(lefrCallbackType_e type, lefiMacro* macro, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != lefrMacroCbkType)) {
    cout <<"Type is not lefrMacroCbkType!" <<endl;
    exit(2);
  }

  frCoord originX = round(macro->originX() * ((io::Parser*)data)->tech->getDBUPerUU()); 
  frCoord originY = round(macro->originY() * ((io::Parser*)data)->tech->getDBUPerUU());
  frCoord sizeX   = round(macro->sizeX()   * ((io::Parser*)data)->tech->getDBUPerUU());
  frCoord sizeY   = round(macro->sizeY()   * ((io::Parser*)data)->tech->getDBUPerUU());
  if (enableOutput) {
    cout <<"  ORIGIN " <<originX * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                       <<originY * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" ;" <<endl;
    cout <<"  SIZE   " <<sizeX   * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                       <<sizeY   * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" ;" <<endl;
  }
  vector<frBoundary> bounds;
  frBoundary bound;
  vector<frPoint> points;
  points.push_back(frPoint(originX, originY));
  points.push_back(frPoint(sizeX,   originY));
  points.push_back(frPoint(sizeX,   sizeY));
  points.push_back(frPoint(originX, sizeY));
  bound.setPoints(points);
  bounds.push_back(bound);
  //((io::Parser*)data)->tmpBlock->setBBox(frBox(originX, originY, sizeX, sizeY));
  ((io::Parser*)data)->tmpBlock->setBoundaries(bounds);

  if (enableOutput) {
    if (macro->hasClass()) {
      std::cout << macro->macroClass() << "\n";
    }
  }
  if (macro->hasClass()) {
    if (strcmp(macro->macroClass(), "CORE") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE);
    } else if (strcmp(macro->macroClass(), "CORE TIEHIGH") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE_TIEHIGH);
    } else if (strcmp(macro->macroClass(), "CORE TIELOW") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE_TIELOW);
    } else if (strcmp(macro->macroClass(), "CORE WELLTAP") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE_WELLTAP);
    } else if (strcmp(macro->macroClass(), "CORE SPACER") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE_SPACER);
    } else if (strcmp(macro->macroClass(), "CORE ANTENNACELL") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::CORE_ANTENNACELL);
    } else if (strcmp(macro->macroClass(), "ENDCAP PRE") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::ENDCAP_PRE);
    } else if (strcmp(macro->macroClass(), "BLOCK") == 0) {
      ((io::Parser*)data)->tmpBlock->setMacroClass(MacroClassEnum::BLOCK);
    } else {
      cout << "Warning: unknown macroClass " << macro->macroClass() << ", skipped macroClass property\n"; 
    }
  }


  return 0;
}

int io::Parser::getLefPins(lefrCallbackType_e type, lefiPin* pin, lefiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != lefrPinCbkType) {
    cout <<"Type is not lefrPinCbkType!" <<endl;
    exit(1);
  }

  // term
  unique_ptr<frTerm>         uTerm = make_unique<frTerm>(pin->name());
  auto term = uTerm.get();
  term->setId(((io::Parser*)data)->numTerms);
  ((io::Parser*)data)->numTerms++;
  // term should add pin
  // instTerm
  //shared_ptr<frInstTerm> instTerm = make_shared<frInstTerm>();
  //instTerm->addToInst(((FlexRoute*)data)->tmpMacro);
  //instTerm->addTerm(term);
  // instTerm creation completed

  // inst 
  ((io::Parser*)data)->tmpBlock->addTerm(uTerm);
  // inst completed

  
  if (enableOutput) {
    cout <<"  PIN " <<pin->name() <<endl;
  }
  
  frTermEnum termType = frTermEnum::frcNormalTerm;
  if (pin->hasUse()) {
    string str(pin->use());
    if (str == "SIGNAL") {
      ;
    } else if (str == "CLOCK") {
      termType = frTermEnum::frcClockTerm;
    } else if (str == "POWER") {
      termType = frTermEnum::frcPowerTerm;
    } else if (str == "GROUND") {
      termType = frTermEnum::frcGroundTerm;
    } else {
      cout <<"Error: unsupported PIN USE in lef" <<endl;
      exit(1);
    }
    if (enableOutput) {
      cout <<"    USE " <<str <<" ;" <<endl;
    }
  }
  term->setType(termType);

  int numPorts = pin->numPorts();
  int numItems = 0;
  int itemType = 0;
  // cout <<"pin->numPorts: " <<numPorts <<endl;
  for (int i = 0; i < numPorts; ++i) {
    numItems = pin->port(i)->numItems();
    // cout <<"pin->ports(" <<i <<")->numItems: " <<numItems <<endl;
    if (enableOutput) {
      cout <<"    PORT" <<endl;
    }

    // pin
    auto pinIn = make_unique<frPin>();
    pinIn->setId(i);
    //pinIn->setTerm(term);
    // pin should add pinFigs

    // term

    frLayerNum layerNum = -1;
    for (int j = 0; j < numItems; ++j) {
      itemType = pin->port(i)->itemType(j);
      if (itemType == 1) {
        string layer = pin->port(i)->getLayer(j);
        if (((io::Parser*)data)->tech->name2layer.find(layer) == ((io::Parser*)data)->tech->name2layer.end()) {
          if (VERBOSE > -1) {
            cout <<"Warning: layer " <<layer <<" is skipped for " <<((io::Parser*)data)->tmpBlock->getName() <<"/" <<pin->name() <<endl;
          }
          layerNum = -1;
          continue;
        }

        layerNum = ((io::Parser*)data)->tech->name2layer.at(layer)->getLayerNum();
        //cout <<"  layer: " <<pin->port(i)->getLayer(j) <<endl;
        if (enableOutput) {
          cout <<"    LAYER " <<layer <<" ;" <<endl;
        }
        //cout <<"    LAYERNUM " <<layerNum <<" ;" <<endl;
      } else if (itemType == 8) {
        if (layerNum == -1) {
          continue;
        }
        frCoord xl = round(pin->port(i)->getRect(j)->xl * ((io::Parser*)data)->tech->getDBUPerUU());
        frCoord yl = round(pin->port(i)->getRect(j)->yl * ((io::Parser*)data)->tech->getDBUPerUU());
        frCoord xh = round(pin->port(i)->getRect(j)->xh * ((io::Parser*)data)->tech->getDBUPerUU());
        frCoord yh = round(pin->port(i)->getRect(j)->yh * ((io::Parser*)data)->tech->getDBUPerUU());

        // pinFig
        //shared_ptr<frPinFig> pinFig = make_shared<frRect>();
        unique_ptr<frRect> pinFig = make_unique<frRect>();
        pinFig->setBBox(frBox(xl, yl, xh, yh));
        pinFig->addToPin(pinIn.get());
        pinFig->setLayerNum(layerNum);
        // pinFig completed
        // pin
        unique_ptr<frPinFig> uptr(std::move(pinFig));
        pinIn->addPinFig(uptr);
        Rectangle pinFigRect(xl, yl, xh, yh);
        // std::cout << "(" << xl << ", " << yl << ") -- (" << xh << ", " << yh << ")\n";
        pinIn->addLayerShape(layerNum, pinFigRect);
        // pin completed

        if (enableOutput) {
          cout <<"      RECT " <<xl * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                               <<yl * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                               <<xh * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                               <<yh * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" ;" <<endl;
        }
             
      } else if (itemType == 10) {
        if (layerNum == -1) {
          continue;
        }
        //Polygon polygon;
        if (enableOutput) {
          cout <<"      POLYGON"; 
        }
        frCollection<frPoint> tmpPoints;
        for (int k = 0; k < pin->port(i)->getPolygon(j)->numPoints; k++) {
          frCoord x = round(pin->port(i)->getPolygon(j)->x[k] * ((io::Parser*)data)->tech->getDBUPerUU());
          frCoord y = round(pin->port(i)->getPolygon(j)->y[k] * ((io::Parser*)data)->tech->getDBUPerUU());
          tmpPoints.push_back(frPoint(x, y));
          if (enableOutput) {
             cout <<" " <<x * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                        <<y * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
          }
        }
        // pinFig
        unique_ptr<frPolygon> pinFig = make_unique<frPolygon>();
        pinFig->setPoints(tmpPoints);
        pinFig->addToPin(pinIn.get());
        pinFig->setLayerNum(layerNum);
        // pinFig completed
        // pin
        unique_ptr<frPinFig> uptr(std::move(pinFig));
        pinIn->addPinFig(uptr);
        Polygon pinFigPoly;
        frVector<Point> boostPolyPoints;
        for (auto &pt: tmpPoints) {
          boostPolyPoints.push_back(Point(pt.x(), pt.y()));
          // std::cout << pt.x() << " " << pt.y() << "\n";
        }
        boost::polygon::set_points(pinFigPoly, boostPolyPoints.begin(), boostPolyPoints.end());
        pinIn->addLayerShape(layerNum, pinFigPoly);
        // pin completed

        if (enableOutput) {
          cout <<" ;" <<endl;
        }
      } else {
        if (VERBOSE > -1) {
          cout <<"unsupported lefiGeometries!" <<endl;
        }
        continue;
        // exit(2);
      }
      //cout <<"  enum: " <<pin->port(i)->itemType(j) <<endl;
    }
    term->addPin(pinIn);
    // term completed
    if (enableOutput) {
      cout <<"    END" <<endl;
    }
  }
  if (enableOutput) {
    cout <<"  END " <<pin->name() <<endl;
  }

  return 0;
}


int io::Parser::getLefObs(lefrCallbackType_e type, lefiObstruction* obs, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;

  if (type != lefrObstructionCbkType) {
    cout <<"Type is not lefrObstructionCbkType!" <<endl;
    exit(1);
  }

  vector<unique_ptr<frBlockage> > blks;

  if (enableOutput) {
    cout <<"  OBS" <<endl;
  }
  
  auto geometry = obs->geometries();
  int numItems  = geometry->numItems();
  
  string layer = "";
  frLayerNum layerNum = -1;
  for (int i = 0; i < numItems; ++i) {
    if (geometry->itemType(i) == lefiGeomLayerE) {
      layer = geometry->getLayer(i);
      if (((io::Parser*)data)->tech->name2layer.find(layer) != ((io::Parser*)data)->tech->name2layer.end()) {
        layerNum = ((io::Parser*)data)->tech->name2layer[layer]->getLayerNum();
      } else {
        if (VERBOSE > 2) {
          cout <<"Warning: layer " <<geometry->getLayer(i) <<" is skipped for " <<((io::Parser*)data)->tmpBlock->getName() <<"/OBS" <<endl; 
        }
        layerNum = -1;
        continue;
      }
      if (enableOutput) {
        cout <<"    LAYER " <<layer <<" ;" <<endl;
      }
    } else if (geometry->itemType(i) == lefiGeomRectE) {
      if (layerNum == -1) {
        // cout <<"Warning: OBS on undefined layer " <<" is skipped... " <<endl; 
        continue;
      }
      auto rect = geometry->getRect(i);
      frCoord xl = round(rect->xl * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yl = round(rect->yl * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord xh = round(rect->xh * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yh = round(rect->yh * ((io::Parser*)data)->tech->getDBUPerUU());
      frBox box(xl, yl, xh, yh);
      xl = box.left();
      yl = box.bottom();
      xh = box.right();
      yh = box.top();
      vector<frPoint> points;
      points.push_back(frPoint(xl,yl));
      points.push_back(frPoint(xh,yl));
      points.push_back(frPoint(xh,yh));
      points.push_back(frPoint(xl,yh));
      auto blk = make_unique<frLayerBlockage>();
      blk->setId(((io::Parser*)data)->numBlockages);
      ((io::Parser*)data)->numBlockages++;
      blk->setLayerNum(layerNum);
      blk->setPoints(points);
      blks.push_back(std::move(blk));
      if (enableOutput) {
        cout <<"      RECT " <<rect->xl <<" " <<rect->yl <<" " <<rect->xh <<" " <<rect->yh <<" ;" <<endl;
      }
    } else {
      if (VERBOSE > 2) {
        cout <<"Warning: unsupported OBS" <<endl;
      }
      continue;
    }
  }
  ((io::Parser*)data)->tmpBlock->setBlockages(blks);
  return 0;
}

int io::Parser::getLefString(lefrCallbackType_e type, const char* str, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (type == lefrMacroBeginCbkType) {
    auto &tmpBlock = ((io::Parser*)data)->tmpBlock;
    tmpBlock = make_unique<frBlock>();
    tmpBlock->setName(string(str));
    if (enableOutput) {
      cout <<"MACRO " <<tmpBlock->getName() <<endl;
    }
  } else if (type == lefrMacroEndCbkType) {
    auto &tmpBlock = ((io::Parser*)data)->tmpBlock;
    tmpBlock->setId(((io::Parser*)data)->numRefBlocks + 1);
    if (enableOutput) {
      cout <<"END " <<tmpBlock->getName() <<" " <<((io::Parser*)data)->numRefBlocks + 1 <<endl;
    }
    ((io::Parser*)data)->design->addRefBlock(((io::Parser*)data)->tmpBlock);
    ((io::Parser*)data)->numRefBlocks++;
    ((io::Parser*)data)->numTerms     = 0;
    ((io::Parser*)data)->numBlockages = 0;
  } else {
    cout <<"Type is not supported!" <<endl;
    // exit(2);
  }
  return 0;
}

int io::Parser::getLefUnits(lefrCallbackType_e type, lefiUnits* units, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  ((io::Parser*)data)->tech->setDBUPerUU(frUInt4(units->databaseNumber()));
  if (enableOutput) {
    cout <<"DATABASE MICRONS " <<((io::Parser*)data)->tech->getDBUPerUU() <<endl;
  }
  return 0;
}

int io::Parser::getLefManufacturingGrid(lefrCallbackType_e type, double number, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  ((io::Parser*)data)->tech->setManufacturingGrid(frUInt4(round(number * ((io::Parser*)data)->tech->getDBUPerUU())));
  if (enableOutput) {
    cout <<"MANUFACTURINGGRID " <<number <<endl;
  }
  return 0;
}

int io::Parser::getLefVias(lefrCallbackType_e type, lefiVia* via, lefiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != lefrViaCbkType) {
    cout <<"Type is not lefrViaCbkType!" <<endl;
    // exit(1);
  }
  if (enableOutput) {
    cout <<"VIA " <<via->name();
    if (via->hasDefault()) {
      cout <<" DEFAULT";
    }
    cout <<endl;
  }
  if (via->numLayers() != 3) {
    if (VERBOSE > -1) {
      cout <<"Error: unsupported via" <<endl;
    }
    exit(1);
  }
  map<frLayerNum, int> lNum2Int;
  for (int i = 0; i < via->numLayers(); ++i) {
    if (((io::Parser*)data)->tech->name2layer.find(via->layerName(i)) == ((io::Parser*)data)->tech->name2layer.end()) {
      if (VERBOSE > -1) {
        cout <<"Warning: layer " <<via->layerName(i) <<" is skipiped for " <<via->name() <<endl;
      }
      return 0;
    }
    lNum2Int[((io::Parser*)data)->tech->name2layer.at(via->layerName(i))->getLayerNum()] = i;
  }
  //for (auto &m: lNum2Int) {
  //  cout <<"print " <<m.first <<" " <<m.second <<endl;
  //}
  if (lNum2Int.begin()->first + 2 != (--lNum2Int.end())->first) {
    if (VERBOSE > -1) {
      cout <<"Error: non-consecutive layers" <<endl;
    }
    exit(1);
  }

  auto viaDef = make_unique<frViaDef>(via->name());
  if (via->hasDefault()) {
    viaDef->setDefault(1);
  }
  int cnt = 0;
  for (auto &m: lNum2Int) {
    int i = m.second;
    if (enableOutput) {
      cout <<"  LAYER " <<via->layerName(i) <<" ;" <<endl;
    }
    //auto layerNum = ((FlexRoute*)data)->layers[via->layerName(i)]->getLayerNum();
    auto layerNum = m.first;
    for (int j = 0; j < via->numRects(i); ++j) {
      frCoord xl = round(via->xl(i, j) * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yl = round(via->yl(i, j) * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord xh = round(via->xh(i, j) * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yh = round(via->yh(i, j) * ((io::Parser*)data)->tech->getDBUPerUU());
      unique_ptr<frRect> pinFig = make_unique<frRect>();
      pinFig->setBBox(frBox(xl, yl, xh, yh));
      pinFig->setLayerNum(layerNum);
      unique_ptr<frShape> tmp(std::move(pinFig));
      if (enableOutput) {
        cout <<"    RECT "   <<xl * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                             <<yl * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                             <<xh * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                             <<yh * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" ;" <<endl;
      }
      switch(cnt) {
        case 0 :
          viaDef->addLayer1Fig(tmp);
          break;
        case 1 :
          viaDef->addCutFig(tmp);
          break;
        default:
          viaDef->addLayer2Fig(tmp);
          break;
      }
    }
    for (int j = 0; j < via->numPolygons(i); ++j) {
      if (enableOutput) {
        cout <<"    POLYGON"; 
      }
      vector<frPoint> tmpPoints;
      for (int k = 0; k < via->getPolygon(i, j).numPoints; k++) {
        frCoord x = round(via->getPolygon(i, j).x[k] * ((io::Parser*)data)->tech->getDBUPerUU());
        frCoord y = round(via->getPolygon(i, j).y[k] * ((io::Parser*)data)->tech->getDBUPerUU());
        tmpPoints.push_back(frPoint(x, y));
        if (enableOutput) {
           cout <<" " <<x * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU() <<" " 
                      <<y * 1.0 / ((io::Parser*)data)->tech->getDBUPerUU();
        }
      }
      unique_ptr<frPolygon> pinFig = make_unique<frPolygon>();
      pinFig->setPoints(tmpPoints);
      pinFig->setLayerNum(layerNum);
      unique_ptr<frShape> tmp(std::move(pinFig));
      if (enableOutput) {
        cout <<" ;" <<endl;
      }
      switch(cnt) {
        case 0 :
          viaDef->addLayer1Fig(tmp);
          break;
        case 1 :
          viaDef->addCutFig(tmp);
          break;
        default:
          viaDef->addLayer2Fig(tmp);
          break;
      }
    }
    cnt++;
  }
  if (enableOutput) {
    cout <<"END " <<via->name() <<endl;
  }

  // add via class information
  auto cutLayerNum = viaDef->getCutLayerNum();
  auto cutLayer    = ((io::Parser*)data)->tech->getLayer(cutLayerNum);
  if (cutLayer->hasLef58CutClassConstraint()) {
    bool getCutClass = false;
    for (auto &cutClass: cutLayer->getLef58CutClassConstraint()->getCutClasses()) {
      //int  numCut = 0;
      bool flag   = true;
      for (auto &cutFig: viaDef->getCutFigs()) {
        if (cutFig->typeId() == frcRect) {
          frBox box;
          cutFig->getBBox(box);
          auto width  = box.right() - box.left();
          auto height = box.top() - box.bottom();
          if (min(width, height) != cutClass->getViaWidth()) {
            flag = false;
            //break;
          } else if (max(width, height) != cutClass->getViaLength()) {
            flag = false;
            //break;
          }
        } else {
          flag = false;
          cout <<"Warning: cut shape is not a rectable / does not have a cut class" <<endl;
          //break;
        }
        //numCut++;
      }
      //if (flag && numCut == (int)cutClass->getNumCut()) {
      if (flag) {
        viaDef->setCutClass(cutClass);
        //cout <<"CUTCLASS " <<cutClass->getName() <<endl;
        getCutClass = true;
      }
    }
    if (!getCutClass) {
      cout <<"Warning: no cut class available for via " <<viaDef->getName() <<endl;
    }
  }

  ((io::Parser*)data)->tech->addVia(viaDef);
  return 0;
}

int io::Parser::getLefViaRules(lefrCallbackType_e type, lefiViaRule* viaRule, lefiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != lefrViaRuleCbkType) {
    cout <<"Type is not lefrViaRuleCbkType!" <<endl;
    // exit(1);
  }
  if (enableOutput) {
    cout <<"VIARULE " <<viaRule->name();
    if (viaRule->hasGenerate()) {
      cout <<" GENERATE";
    } else {
      cout <<"Error: getLefViaRules does not support non-generate rules" <<endl;
      exit(1);
    }
    if (viaRule->hasDefault()) {
      cout <<" DEFAULT";
    }
    cout <<endl;
  }
  if (viaRule->numLayers() != 3) {
    if (VERBOSE > -1) {
      cout <<"Error: unsupported via" <<endl;
    }
    exit(1);
  }
  map<frLayerNum, int> lNum2Int;
  for (int i = 0; i < viaRule->numLayers(); ++i) {
    auto viaRuleLayer = viaRule->layer(i);
    if (((io::Parser*)data)->tech->name2layer.find(viaRuleLayer->name()) == ((io::Parser*)data)->tech->name2layer.end()) {
      if (VERBOSE > -1) {
        cout <<"Warning: layer " <<viaRuleLayer->name() <<" is skipiped for " <<viaRule->name() <<endl;
      }
      return 0;
    }
    lNum2Int[((io::Parser*)data)->tech->name2layer.at(viaRuleLayer->name())->getLayerNum()] = i;
  }
  if (lNum2Int.begin()->first + 2!= (--lNum2Int.end())->first) {
    if (VERBOSE > -1) {
      cout <<"Error: non-consecutive layers" <<endl;
    }
    exit(1);
  }

  if (!viaRule->hasGenerate()) {
    cout <<"Error: getLefViaRules does not support non-generate rules" <<endl;
    exit(1);
  }
  auto viaRuleGen = make_unique<frViaRuleGenerate>(viaRule->name());
  if (viaRule->hasDefault()) {
    viaRuleGen->setDefault(1);
  }
  int cnt = 0;
  for (auto &[lNum, i]: lNum2Int) {
    auto viaRuleLayer = viaRule->layer(i);
    if (enableOutput) {
      cout <<"  LAYER " <<viaRuleLayer->name() <<" ;" <<endl;
    }
    if (viaRuleLayer->hasEnclosure()) {
      frCoord x = round(viaRuleLayer->enclosureOverhang1() * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord y = round(viaRuleLayer->enclosureOverhang2() * ((io::Parser*)data)->tech->getDBUPerUU());
      frPoint enc(x, y);
      switch(cnt) {
        case 0:
          viaRuleGen->setLayer1Enc(enc);
          break;
        case 1:
          cout <<"Error: getViaRuleGenerates cutLayer cannot have overhands" <<endl;
          break;
        default:
          viaRuleGen->setLayer2Enc(enc);
          break;
      }
      if (enableOutput) {
        cout <<"    ENCLOSURE " <<viaRuleLayer->enclosureOverhang1() <<" " <<viaRuleLayer->enclosureOverhang1() <<" ;" <<endl;
      }
    }
    if (viaRuleLayer->hasRect()) {
      frCoord xl = round(viaRuleLayer->xl() * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yl = round(viaRuleLayer->yl() * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord xh = round(viaRuleLayer->xh() * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord yh = round(viaRuleLayer->yh() * ((io::Parser*)data)->tech->getDBUPerUU());
      frBox box(xl, yl, xh, yh);
      switch(cnt) {
        case 0:
          cout <<"Error: getViaRuleGenerates botLayer cannot have rect" <<endl;
          break;
        case 1:
          viaRuleGen->setCutRect(box);
          break;
        default:
          cout <<"Error: getViaRuleGenerates topLayer cannot have rect" <<endl;
          break;
      }
      if (enableOutput) {
        cout <<"    RECT " <<viaRuleLayer->xl() <<" " 
                           <<viaRuleLayer->yl() <<" " 
                           <<viaRuleLayer->xh() <<" " 
                           <<viaRuleLayer->yh() 
                           <<" ;" <<endl;
      }
    }
    if (viaRuleLayer->hasSpacing()) {
      frCoord x = round(viaRuleLayer->spacingStepX() * ((io::Parser*)data)->tech->getDBUPerUU());
      frCoord y = round(viaRuleLayer->spacingStepY() * ((io::Parser*)data)->tech->getDBUPerUU());
      frPoint pt(x, y);
      switch(cnt) {
        case 0:
          cout <<"Error: getViaRuleGenerates botLayer cannot have spacing" <<endl;
          break;
        case 1:
          viaRuleGen->setCutSpacing(pt);
          break;
        default:
          cout <<"Error: getViaRuleGenerates topLayer cannot have spacing" <<endl;
          break;
      }
      if (enableOutput) {
        cout <<"    SPACING " <<viaRuleLayer->spacingStepX() <<" BY " <<viaRuleLayer->spacingStepY() <<" ;" <<endl;
      }
    }
    cnt++;
  }

  ((io::Parser*)data)->tech->addViaRuleGenerate(viaRuleGen);
  return 0;
}

void io::Parser::readLef() {
  FILE* f;
  int res;

  lefrInitSession(1);

  lefrSetUserData ((lefiUserData)this);

  lefrSetMacroCbk(getLefMacros);
  lefrSetMacroBeginCbk(getLefString);
  lefrSetMacroEndCbk(getLefString);
  lefrSetUnitsCbk(getLefUnits);
  lefrSetManufacturingCbk(getLefManufacturingGrid);
  lefrSetPinCbk(getLefPins);
  lefrSetObstructionCbk(getLefObs);
  lefrSetLayerCbk(getLefLayers);
  lefrSetViaCbk(getLefVias);
  lefrSetViaRuleCbk(getLefViaRules);

  if ((f = fopen(LEF_FILE.c_str(),"r")) == 0) {
    cout <<"Couldn't open lef file" <<endl;
    exit(2);
  }

  res = lefrRead(f, LEF_FILE.c_str(), (lefiUserData)this);
  if (res != 0) {
    cout <<"LEF parser returns an error!" <<endl;
    exit(2);
  }
  fclose(f);

  lefrClear();
}

void io::Parser::readLefDef() {
  //bool enableOutput = false;
  bool enableOutput = true;

  if (VERBOSE > 0) {
    cout <<endl <<"reading lef ..." <<endl;
  }

  readLef();

  if (VERBOSE > 0) {
    cout <<endl;
    cout <<"units:       " <<tech->getDBUPerUU()      <<endl;
    cout <<"#layers:     " <<tech->layers.size()      <<endl;
    cout <<"#macros:     " <<design->refBlocks.size() <<endl;
    cout <<"#vias:       " <<tech->vias.size()        <<endl;
    cout <<"#viarulegen: " <<tech->viaRuleGenerates.size() <<endl;
  }
  //exit(1);
  auto numLefVia = tech->vias.size();

  //tech->printAllConstraints();
  
  if (enableOutput) {
    //design->printAllMacros();
    // printAllLayers();
    //tech->printAllVias();
    //printLayerMaps();
  }

  if (VERBOSE > 0) {
    cout <<endl <<"reading def ..." <<endl;
  }

  readDef();

  if (VERBOSE > 0) {
    cout <<endl;
    frBox dieBox;
    design->getTopBlock()->getBoundaryBBox(dieBox);
    cout <<"design:      " <<design->getTopBlock()->getName()    <<endl;
    cout <<"die area:    " <<dieBox                              <<endl;
    cout <<"trackPts:    " <<design->getTopBlock()->getTrackPatterns().size() <<endl;
    cout <<"defvias:     " <<tech->vias.size() - numLefVia       <<endl;
    cout <<"#components: " <<design->getTopBlock()->insts.size() <<endl;
    cout <<"#terminals:  " <<design->getTopBlock()->terms.size() <<endl;
    cout <<"#snets:      " <<design->getTopBlock()->snets.size() <<endl;
    cout <<"#nets:       " <<design->getTopBlock()->nets.size()  <<endl;
    //cout <<"#pins:       " <<numPins <<endl;
  }
  //cout <<flush;

  if (enableOutput) {
    //tech->printAllVias();
    //design->printAllComps();
    //printCompMaps();
    //design->printAllTerms();
    //printTermMaps();
    //printAllNets();
    //printAllTrackGens();
    //printAllTrackPatterns();
  }
  //exit(1);

}

void io::Parser::readGuide() {

  if (VERBOSE > 0) {
    cout <<endl <<"reading guide ..." <<endl;
  }

  int numGuides = 0;

  string netName = "";
  frNet* net = nullptr;

  ifstream fin(GUIDE_FILE.c_str());
  string line;
  frBox  box;
  frLayerNum layerNum;

  if (fin.is_open()){
    while (fin.good()) {
      getline(fin, line);
      //cout <<line <<endl <<line.size() <<endl;
      if (line == "(" || line == "") continue;
      if (line == ")") {
        continue;
      }

      stringstream ss(line);
      string word = "";
      vector<string> vLine;
      while (!ss.eof()) {
        ss >>word;
        vLine.push_back(word);
        //cout <<word <<" ";
      }
      //cout <<endl;

      if (vLine.size() == 0) {
        cout <<"Error: reading guide file!" <<endl;
        exit(2);
      } else if (vLine.size() == 1) {
        netName = vLine[0];
        if (design->topBlock->name2net.find(vLine[0]) == design->topBlock->name2net.end()) {
          cout <<"Error: cannot find net: " <<vLine[0] <<endl;
          exit(2);
        }
        net = design->topBlock->name2net[netName]; 
      } else if (vLine.size() == 5) {
        if (tech->name2layer.find(vLine[4]) == tech->name2layer.end()) {
          cout <<"Error: cannot find layer: " <<vLine[4] <<endl;
          exit(2);
        }
        layerNum = tech->name2layer[vLine[4]]->getLayerNum();
        box.set(stoi(vLine[0]), stoi(vLine[1]), stoi(vLine[2]), stoi(vLine[3]));
        frRect rect;
        rect.setBBox(box);
        rect.setLayerNum(layerNum);
        //tmpGuides[netName].push_back(rect);
        tmpGuides[net].push_back(rect);
        ++numGuides;
        if (numGuides < 1000000) {
          if (numGuides % 100000 == 0) {
            cout <<"guideIn read " <<numGuides <<" guides" <<endl;
          }
        } else {
          if (numGuides % 1000000 == 0) {
            cout <<"guideIn read " <<numGuides <<" guides" <<endl;
          }
        }

      } else {
        cout <<"Error: reading guide file!" <<endl;
        exit(2);
      }
    }
    fin.close();
  } else {
    cout <<"Error: failed to open guide file" <<endl;
    exit(2);
  }


  if (VERBOSE > 0) {
    cout <<endl;
    cout <<"#guides:     " <<numGuides <<endl;
  }

}

void io::Writer::fillConnFigs_net(frNet* net, bool isTA) {
  //bool enableOutput = true;
  bool enableOutput = false;
  auto netName = net->getName();
  if (isTA) {
    for (auto &uGuide: net->getGuides()) {
      //cout <<"find guide" <<endl;
      for (auto &uConnFig: uGuide->getRoutes()) {
        auto connFig = uConnFig.get();
        if (connFig->typeId() == frcPathSeg) {
          connFigs[netName].push_back(make_shared<frPathSeg>(*static_cast<frPathSeg*>(connFig)));
        } else if (connFig->typeId() == frcVia) {
          connFigs[netName].push_back(make_shared<frVia>(*static_cast<frVia*>(connFig)));
          //frPoint bp, ep;
          //static_pointer_cast<frGuide>(objPtr)->getPoints(bp, ep);
          //cout <<"fillConnFigs_net found via in guide " <<bp <<" " <<ep <<" "
          //     <<static_pointer_cast<frGuide>(objPtr)->getBeginLayerNum() <<" " 
          //     <<static_pointer_cast<frGuide>(objPtr)->getEndLayerNum() <<endl;
        } else {
          cout <<"Error: io::Writer::filliConnFigs does not support this type" <<endl;
        }
      }
    }
  } else {
    if (enableOutput) {
      cout << netName << ":\n";
    }
    for (auto &shape: net->getShapes()) {
      if (shape->typeId() == frcPathSeg) {
        auto pathSeg = *static_cast<frPathSeg*>(shape.get());
        frPoint start, end;
        pathSeg.getPoints(start, end);

        if (enableOutput) {
          frLayerNum currLayerNum = pathSeg.getLayerNum();
          cout << "  connfig pathseg (" << start.x() / 2000.0<< ", " << start.y() / 2000.0 
               << ") - (" << end.x() / 2000.0 << ", " << end.y() / 2000.0 << ") " << currLayerNum  <<"\n"; 
        }
        connFigs[netName].push_back(make_shared<frPathSeg>(pathSeg));
      }
    }
    for (auto &via: net->getVias()) {
      connFigs[netName].push_back(make_shared<frVia>(*via));
    }
    for (auto &shape: net->getPatchWires()) {
      auto pwire = static_cast<frPatchWire*>(shape.get());
      connFigs[netName].push_back(make_shared<frPatchWire>(*pwire));
    }
  }
}

void io::Writer::splitVia_helper(frLayerNum layerNum, int isH, frCoord trackLoc, frCoord x, frCoord y, 
  vector< vector< map<frCoord, vector<shared_ptr<frPathSeg> > > > > &mergedPathSegs) {
  if (layerNum >= 0 && layerNum < (int)(getTech()->getLayers().size()) &&
      mergedPathSegs.at(layerNum).at(isH).find(trackLoc) != mergedPathSegs.at(layerNum).at(isH).end()) {
    for (auto &pathSeg: mergedPathSegs.at(layerNum).at(isH).at(trackLoc)) {
      frPoint begin, end;
      pathSeg->getPoints(begin, end);
      if ((isH == 0 && (begin.x() < x) && (end.x() > x)) ||
          (isH == 1 && (begin.y() < y) && (end.y() > y))) {
        frSegStyle style1, style2, style_default;
        pathSeg->getStyle(style1);
        pathSeg->getStyle(style2);
        style_default = getTech()->getLayer(layerNum)->getDefaultSegStyle();
        shared_ptr<frPathSeg> newPathSeg = make_shared<frPathSeg>(*pathSeg);
        pathSeg->setPoints(begin, frPoint(x,y));
        style1.setEndStyle(style_default.getEndStyle(), style_default.getEndExt());
        pathSeg->setStyle(style1);
        newPathSeg->setPoints(frPoint(x,y), end);
        style2.setBeginStyle(style_default.getBeginStyle(), style_default.getBeginExt());
        newPathSeg->setStyle(style2);
        mergedPathSegs.at(layerNum).at(isH).at(trackLoc).push_back(newPathSeg);
        // via can only intersect at most one merged pathseg on one track
        break;
      }
    }
  }
}




// merge pathseg, delete redundant via
void io::Writer::mergeSplitConnFigs(list<shared_ptr<frConnFig> > &connFigs) {
  //if (VERBOSE > 0) {
  //  cout <<endl <<"merge and split ..." <<endl;
  //}
  // initialzie pathseg and via map
  //map< tuple<layerNum, isHorizontal, trackLoc>,
  //     map<frCoord, vector< tuple<shared_ptr<frPathSeg>, isBegin> > > 
  //   > map;
  map < tuple<frLayerNum, bool, frCoord>,
        map<frCoord, vector< tuple<shared_ptr<frPathSeg>, bool> > 
           >
      > pathSegMergeMap;
  map < tuple<frCoord, frCoord, frLayerNum>, shared_ptr<frVia> > viaMergeMap;
  for (auto &connFig: connFigs) {
    if (connFig->typeId() == frcPathSeg) {
      auto pathSeg = dynamic_pointer_cast<frPathSeg>(connFig);
      frPoint begin, end;
      pathSeg->getPoints(begin, end);
      frLayerNum layerNum = pathSeg->getLayerNum();
      if (begin == end) {
        // std::cout << "Warning: 0 length connfig\n";
        continue; // if segment length = 0, ignore
      } else {
        // std::cout << "xxx\n";
        bool isH = (begin.x() == end.x()) ? false : true;
        frCoord trackLoc   = isH ? begin.y() : begin.x();
        frCoord beginCoord = isH ? begin.x() : begin.y();
        frCoord endCoord   = isH ? end.x()   : end.y();
        pathSegMergeMap[make_tuple(layerNum, isH, trackLoc)][beginCoord].push_back(make_tuple(pathSeg, true));
        pathSegMergeMap[make_tuple(layerNum, isH, trackLoc)][endCoord].push_back(make_tuple(pathSeg, false));
      }
    } else if (connFig->typeId() == frcVia) {
      auto via = dynamic_pointer_cast<frVia>(connFig);
      auto cutLayerNum = via->getViaDef()->getCutLayerNum();
      //auto layer1Num = via->getLayer1Num();
      //auto layer2Num = via->getLayer2Num();
      frPoint viaPoint;
      via->getOrigin(viaPoint);
      viaMergeMap[make_tuple(viaPoint.x(), viaPoint.y(), cutLayerNum)] = via;
      //cout <<"found via" <<endl;
    } else {
      ;
    }
  }

  // merge pathSeg
  map<frCoord, vector<shared_ptr<frPathSeg> > > tmp1;
  vector< map<frCoord, vector<shared_ptr<frPathSeg> > > > tmp2(2, tmp1);
  // mergedPathSegs[layerNum][isHorizontal] is a map<trackLoc, vector<shared_ptr<frPathSeg> > >
  vector< vector< map<frCoord, vector<shared_ptr<frPathSeg> > > > > mergedPathSegs(getTech()->getLayers().size(), tmp2);

  for (auto &it1: pathSegMergeMap) {
    auto layerNum = get<0>(it1.first);
    int  isH = get<1>(it1.first);
    auto trackLoc = get<2>(it1.first);
    bool hasSeg = false;
    int cnt = 0;
    shared_ptr<frPathSeg> newPathSeg;
    frSegStyle style;
    frPoint begin, end;
    for (auto &it2: it1.second) {
      //auto coord = it2.first;
      //cout <<"coord " <<coord <<endl;
      for (auto &pathSegTuple: it2.second) {
        cnt += get<1>(pathSegTuple)? 1 : -1;
      }
      // newPathSeg begin
      if (!hasSeg && cnt > 0) {
        style.setBeginStyle(frcTruncateEndStyle, 0);
        style.setEndStyle(frcTruncateEndStyle, 0);
        newPathSeg = make_shared<frPathSeg>(*(get<0>(*(it2.second.begin()))));
        for (auto &pathSegTuple: it2.second) {
          auto pathSeg = get<0>(pathSegTuple);
          auto isBegin = get<1>(pathSegTuple);
          if (isBegin) {
            pathSeg->getPoints(begin, end);
            frSegStyle tmpStyle;
            pathSeg->getStyle(tmpStyle);
            if (tmpStyle.getBeginExt() > style.getBeginExt()) {
              style.setBeginStyle(tmpStyle.getBeginStyle(), tmpStyle.getBeginExt());
            }
          }
        }
        newPathSeg->setStyle(style);
        hasSeg = true;
      // newPathSeg end
      } else if (hasSeg && cnt == 0) {
        newPathSeg->getPoints(begin, end);
        for (auto &pathSegTuple: it2.second) {
          auto pathSeg = get<0>(pathSegTuple);
          auto isBegin = get<1>(pathSegTuple);
          if (!isBegin) {
            frPoint tmp;
            pathSeg->getPoints(tmp, end);
            frSegStyle tmpStyle;
            pathSeg->getStyle(tmpStyle);
            if (tmpStyle.getEndExt() > style.getEndExt()) {
              style.setEndStyle(tmpStyle.getEndStyle(), tmpStyle.getEndExt());
            }
          }
        }
        newPathSeg->setPoints(begin, end);
        newPathSeg->setStyle(style);
        hasSeg = false;
        (mergedPathSegs.at(layerNum).at(isH))[trackLoc].push_back(newPathSeg);
      }
    }
  }

  // split pathseg from via
  // mergedPathSegs[layerNum][isHorizontal] is a map<frCoord, vector<shared_ptr<frPathSeg> > >
  //map < tuple<frCoord, frCoord, frLayerNum>, shared_ptr<frVia> > viaMergeMap;
  for (auto &it1: viaMergeMap) {
    auto x           = get<0>(it1.first);
    auto y           = get<1>(it1.first);
    auto cutLayerNum = get<2>(it1.first);
    frCoord trackLoc;

    auto layerNum = cutLayerNum - 1;
    int  isH      = 1;
    trackLoc = (isH == 1) ? y : x;
    splitVia_helper(layerNum, isH, trackLoc, x, y, mergedPathSegs);

    layerNum = cutLayerNum - 1;
    isH = 0;
    trackLoc = (isH == 1) ? y : x;
    splitVia_helper(layerNum, isH, trackLoc, x, y, mergedPathSegs);

    layerNum = cutLayerNum + 1;
    trackLoc = (isH == 1) ? y : x;
    splitVia_helper(layerNum, isH, trackLoc, x, y, mergedPathSegs);

    layerNum = cutLayerNum + 1;
    isH = 0;
    trackLoc = (isH == 1) ? y : x;
    splitVia_helper(layerNum, isH, trackLoc, x, y, mergedPathSegs);
  }

  // split intersecting pathSegs
  for (auto &it1: mergedPathSegs) {
    // vertical for mapIt1
    for (auto &mapIt1: it1.at(0)) {
      // horizontal for mapIt2
      for (auto &mapIt2: it1.at(1)) {
        // at most split once
        // seg1 is vertical
        for (auto &seg1: mapIt1.second) {
          bool skip = false;
          // seg2 is horizontal
          frPoint seg1Begin, seg1End;
          seg1->getPoints(seg1Begin, seg1End);
          for (auto &seg2: mapIt2.second) {
            frPoint seg2Begin, seg2End;
            seg2->getPoints(seg2Begin, seg2End);
            bool pushNewSeg1 = false;
            bool pushNewSeg2 = false;
            shared_ptr<frPathSeg> newSeg1;
            shared_ptr<frPathSeg> newSeg2;
            // check whether seg1 needs to be split, break seg1
            if (seg2Begin.y() > seg1Begin.y() && seg2Begin.y() < seg1End.y()) {
              pushNewSeg1 = true;
              newSeg1 = make_shared<frPathSeg>(*seg1);
              // modify seg1
              seg1->setPoints(seg1Begin, frPoint(seg1End.x(), seg2End.y()));
              // modify newSeg1
              newSeg1->setPoints(frPoint(seg1End.x(), seg2Begin.y()), seg1End);
              // modify endstyle
              auto layerNum = seg1->getLayerNum();
              frSegStyle tmpStyle1;
              frSegStyle tmpStyle2;
              frSegStyle style_default;
              seg1->getStyle(tmpStyle1);
              seg1->getStyle(tmpStyle2);
              style_default = getTech()->getLayer(layerNum)->getDefaultSegStyle();
              tmpStyle1.setEndStyle(frcExtendEndStyle, style_default.getEndExt());
              seg1->setStyle(tmpStyle1);
              tmpStyle2.setBeginStyle(frcExtendEndStyle, style_default.getBeginExt());
              newSeg1->setStyle(tmpStyle2);
            }
            // check whether seg2 needs to be split, break seg2
            if (seg1Begin.x() > seg2Begin.x() && seg1Begin.x() < seg2End.x()) {
              pushNewSeg2 = true;
              newSeg2 = make_shared<frPathSeg>(*seg1);
              // modify seg2
              seg2->setPoints(seg2Begin, frPoint(seg1End.x(), seg2End.y()));
              // modify newSeg2
              newSeg2->setPoints(frPoint(seg1End.x(), seg2Begin.y()), seg2End);
              // modify endstyle
              auto layerNum = seg2->getLayerNum();
              frSegStyle tmpStyle1;
              frSegStyle tmpStyle2;
              frSegStyle style_default;
              seg2->getStyle(tmpStyle1);
              seg2->getStyle(tmpStyle2);
              style_default = getTech()->getLayer(layerNum)->getDefaultSegStyle();
              tmpStyle1.setEndStyle(frcExtendEndStyle, style_default.getEndExt());
              seg2->setStyle(tmpStyle1);
              tmpStyle2.setBeginStyle(frcExtendEndStyle, style_default.getBeginExt());
              newSeg2->setStyle(tmpStyle2);
            }
            if (pushNewSeg1) {
              mapIt1.second.push_back(newSeg1);
            }
            if (pushNewSeg2) {
              mapIt2.second.push_back(newSeg2);
            }
            if (pushNewSeg1 || pushNewSeg2) {
              skip = true;
              break;
            }
            //cout <<"found" <<endl;
          }
          if (skip) break;
        }
      }
    }
  }


  // write back pathseg
  connFigs.clear();
  for (auto &it1: mergedPathSegs) {
    for (auto &it2: it1) {
      for (auto &it3: it2) {
        for (auto &it4: it3.second) {
          connFigs.push_back(it4);
        }
      }
    }
  }

  // write back via
  //map < tuple<frCoord, frCoord, frLayerNum>, shared_ptr<frVia> > viaMergeMap;
  for (auto &it: viaMergeMap) {
    connFigs.push_back(it.second);
  }

}

void io::Writer::fillConnFigs(bool isTA) {
  connFigs.clear();
  if (VERBOSE > 0) {
    cout <<endl <<"post processing ..." <<endl;
  }
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    fillConnFigs_net(net.get(), isTA);
  }
  if (isTA) {
    for (auto &it: connFigs) {
      mergeSplitConnFigs(it.second);
    }
  }
}

void io::Writer::writeFromTA() {
  fillConnFigs(true);
  writeDef(true);
}

void io::Writer::writeFromDR(const string &str) {
  fillConnFigs(false);
  writeDef(false, str);
}
