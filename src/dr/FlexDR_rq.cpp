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

using namespace std;
using namespace fr;

frDesign* FlexDRWorkerRegionQuery::getDesign() const {
  return drWorker->getDesign();
}

void FlexDRWorkerRegionQuery::add(drConnFig* connFig) {
  frBox frb;
  box_t boostb;
  if (connFig->typeId() == drcPathSeg || connFig->typeId() == frcRect || connFig->typeId() == drcPatchWire) {
    auto obj = static_cast<drShape*>(connFig);
    obj->getBBox(frb);
    boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
    shapes.at(obj->getLayerNum()).insert(make_pair(boostb, obj));
  } else if (connFig->typeId() == drcVia) {
    auto via = static_cast<drVia*>(connFig);
    frTransform xform;
    frPoint origin;
    via->getOrigin(origin);
    xform.set(origin);
    for (auto &uShape: via->getViaDef()->getLayer1Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getLayer1Num()).insert(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getLayer2Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getLayer2Num()).insert(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getCutFigs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getCutLayerNum()).insert(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
  } else {
    cout <<"Error: unsupported region query add" <<endl;
  }
}



void FlexDRWorkerRegionQuery::add(drConnFig* connFig, vector<vector<rq_rptr_value_t<drConnFig> > > &allShapes) {
  frBox frb;
  box_t boostb;
  if (connFig->typeId() == drcPathSeg || connFig->typeId() == frcRect || connFig->typeId() == drcPatchWire) {
    auto obj = static_cast<drShape*>(connFig);
    obj->getBBox(frb);
    boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
    allShapes.at(obj->getLayerNum()).push_back(make_pair(boostb, obj));
  } else if (connFig->typeId() == drcVia) {
    auto via = static_cast<drVia*>(connFig);
    frTransform xform;
    frPoint origin;
    via->getOrigin(origin);
    xform.set(origin);
    for (auto &uShape: via->getViaDef()->getLayer1Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        allShapes.at(via->getViaDef()->getLayer1Num()).push_back(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getLayer2Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        allShapes.at(via->getViaDef()->getLayer2Num()).push_back(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getCutFigs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        allShapes.at(via->getViaDef()->getCutLayerNum()).push_back(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query add" <<endl;
      }
    }
  } else {
    cout <<"Error: unsupported region query add" <<endl;
  }
}

void FlexDRWorkerRegionQuery::remove(drConnFig* connFig) {
  frBox frb;
  box_t boostb;
  if (connFig->typeId() == drcPathSeg || connFig->typeId() == frcRect || connFig->typeId() == drcPatchWire) {
    auto obj = static_cast<drShape*>(connFig);
    obj->getBBox(frb);
    boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
    shapes.at(obj->getLayerNum()).remove(make_pair(boostb, obj));
  } else if (connFig->typeId() == drcVia) {
    auto via = static_cast<drVia*>(connFig);
    frTransform xform;
    frPoint origin;
    via->getOrigin(origin);
    xform.set(origin);
    for (auto &uShape: via->getViaDef()->getLayer1Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getLayer1Num()).remove(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query remove" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getLayer2Figs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getLayer2Num()).remove(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query remove" <<endl;
      }
    }
    for (auto &uShape: via->getViaDef()->getCutFigs()) {
      auto shape = uShape.get();
      if (shape->typeId() == frcRect) {
        shape->getBBox(frb);
        frb.transform(xform);
        boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
        shapes.at(via->getViaDef()->getCutLayerNum()).remove(make_pair(boostb, via));
      } else {
        cout <<"Error: unsupported region query remove" <<endl;
      }
    }
  } else {
    cout <<"Error: unsupported region query remove" <<endl;
  }
}

void FlexDRWorkerRegionQuery::query(const frBox &box, frLayerNum layerNum, vector<drConnFig*> &result) {
  vector<rq_rptr_value_t<drConnFig> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  shapes.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexDRWorkerRegionQuery::query(const frBox &box, frLayerNum layerNum, vector<rq_rptr_value_t<drConnFig> > &result) {
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  shapes.at(layerNum).query(bgi::intersects(boostb), back_inserter(result));
}

void FlexDRWorkerRegionQuery::init() {
  int numLayers = getDesign()->getTech()->getLayers().size();
  shapes.clear();
  shapes.resize(numLayers);
  vector<vector<rq_rptr_value_t<drConnFig> > > allShapes(numLayers);
  for (auto &net: getDRWorker()->getNets()) {
    for (auto &connFig: net->getRouteConnFigs()) {
      add(connFig.get(), allShapes);
    }
    for (auto &connFig: net->getExtConnFigs()) {
      add(connFig.get(), allShapes);
    }
  }
  for (auto i = 0; i < numLayers; i++) {
    shapes.at(i) = boost::move(bgi::rtree<rq_rptr_value_t<drConnFig>, bgi::quadratic<16> >(allShapes.at(i)));
    allShapes.at(i).clear();
    allShapes.at(i).shrink_to_fit();
    //if (VERBOSE > 0) {
    //  cout <<"  complete " <<design->getTech()->getLayer(i)->getName() <<endl;
    //}
  }
}
