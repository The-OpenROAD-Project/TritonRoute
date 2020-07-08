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

#include "ta/FlexTA.h"

using namespace std;
using namespace fr;

frDesign* FlexTAWorkerRegionQuery::getDesign() const {
  return taWorker->getDesign();
}

void FlexTAWorkerRegionQuery::add(taPinFig* fig) {
  box_t boostb;
  frPoint bp, ep;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    obj->getPoints(bp, ep);
    boostb = box_t(point_t(bp.x(), bp.y()), point_t(ep.x(), ep.y()));
    shapes.at(obj->getLayerNum()).insert(make_pair(boostb, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    obj->getOrigin(bp);
    boostb = box_t(point_t(bp.x(), bp.y()), point_t(bp.x(), bp.y()));
    shapes.at(obj->getViaDef()->getCutLayerNum()).insert(make_pair(boostb, obj));
  } else {
    cout <<"Error: unsupported region query add" <<endl;
  }
}

void FlexTAWorkerRegionQuery::remove(taPinFig* fig) {
  box_t boostb;
  frPoint bp, ep;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    obj->getPoints(bp, ep);
    boostb = box_t(point_t(bp.x(), bp.y()), point_t(ep.x(), ep.y()));
    shapes.at(obj->getLayerNum()).remove(make_pair(boostb, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    obj->getOrigin(bp);
    boostb = box_t(point_t(bp.x(), bp.y()), point_t(bp.x(), bp.y()));
    shapes.at(obj->getViaDef()->getCutLayerNum()).remove(make_pair(boostb, obj));
  } else {
    cout <<"Error: unsupported region query add" <<endl;
  }
}

void FlexTAWorkerRegionQuery::query(const frBox &box, frLayerNum layerNum, set<taPin*, frBlockObjectComp> &result) {
  vector<rq_rptr_value_t<taPinFig> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  shapes.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  //transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second->getPin();});
  for (auto &[boostb, rptr]: temp) {
    result.insert(rptr->getPin());
  }
}

void FlexTAWorkerRegionQuery::init() {
  int numLayers = getDesign()->getTech()->getLayers().size();
  shapes.clear();
  shapes.resize(numLayers);
  costs.clear();
  costs.resize(numLayers);
  //aps.clear();
  //aps.resize(numLayers);
  //vector<vector<rq_rptr_value_t<drConnFig> > > allShapes(numLayers);
  //for (auto &net: getDRWorker()->getNets()) {
  //  for (auto &connFig: net->getRouteConnFigs()) {
  //    add(connFig.get(), allShapes);
  //  }
  //  for (auto &connFig: net->getExtConnFigs()) {
  //    add(connFig.get(), allShapes);
  //  }
  //}
  //for (auto i = 0; i < numLayers; i++) {
  //  shapes.at(i) = boost::move(bgi::rtree<rq_rptr_value_t<drConnFig>, bgi::quadratic<16> >(allShapes.at(i)));
  //  allShapes.at(i).clear();
  //  allShapes.at(i).shrink_to_fit();
  //  //if (VERBOSE > 0) {
  //  //  cout <<"  complete " <<design->getTech()->getLayer(i)->getName() <<endl;
  //  //}
  //}
}

void FlexTAWorkerRegionQuery::addCost(const frBox &box, frLayerNum layerNum, frBlockObject* obj, frConstraint* con) {
  costs.at(layerNum).insert(make_pair(box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top())), make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::removeCost(const frBox &box, frLayerNum layerNum, frBlockObject* obj, frConstraint* con) {
  costs.at(layerNum).remove(make_pair(box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top())), make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::queryCost(const frBox &box, frLayerNum layerNum, vector<rq_generic_value_t<pair<frBlockObject*, frConstraint*> > > &result) {
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  costs.at(layerNum).query(bgi::intersects(boostb), back_inserter(result));
}

//void FlexTAWorkerRegionQuery::addAP(const frBox &box, frLayerNum layerNum, frBlockObject* obj, frConstraint* con) {
//  aps.at(layerNum).insert(make_pair(box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top())), obj));
//}
//
//void FlexTAWorkerRegionQuery::removeAP(const frBox &box, frLayerNum layerNum, frBlockObject* obj, frConstraint* con) {
//  aps.at(layerNum).remove(make_pair(box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top())), obj));
//}
//
//void FlexTAWorkerRegionQuery::queryAP(const frBox &box, frLayerNum layerNum, vector<rq_rptr_value_t<frBlockObject> > &result) {
//  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
//  aps.at(layerNum).query(bgi::intersects(boostb), back_inserter(result));
//}
