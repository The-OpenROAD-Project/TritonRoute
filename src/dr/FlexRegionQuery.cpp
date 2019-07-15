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

#include "dr/FlexRegionQuery.h"
using namespace std;
using namespace fr;

void FlexRegionQuery::add(frListIter<frPathSeg> &pathSeg) {
  frBox frb;
  pathSeg->getBBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //frListIter<int> test;
  //int* ptr = new int(3);
  //shared_ptr<frConnFig> ptr = nullptr;
  //pathsegs.at(pathSeg->getLayerNum()).insert(make_pair(boostb, ptr));
  //unique_lock lock(mutex_pathsegs.at(pathSeg->getLayerNum()));
  //unique_lock lock(m);
  pathsegs.at(pathSeg->getLayerNum()).insert(make_pair(boostb, pathSeg));
}

void FlexRegionQuery::add(frListIter<frVia> &via) {
  frBox frb;
  via->getLayer1BBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock1(mutex_vias.at(vias->getViaDef()->getLayer1Num()));
  vias.at(via->getViaDef()->getLayer1Num()).insert(make_pair(boostb, via));
  //lock1.unlock();

  via->getLayer2BBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock2(mutex_vias.at(vias->getViaDef()->getLayer2Num()));
  vias.at(via->getViaDef()->getLayer2Num()).insert(make_pair(boostb, via));
  //lock2.unlock();

  via->getCutBBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock3(mutex_vias.at(vias->getViaDef()->getCutLayerNum()));
  vias.at(via->getViaDef()->getCutLayerNum()).insert(make_pair(boostb, via));
  //lock3.unlock();
}

void FlexRegionQuery::add(frListIter<FlexAccessPattern> &ap) {
  frPoint beginPoint, endPoint;
  ap->getPoints(beginPoint, endPoint);
  box_t boostb = box_t(point_t(endPoint.x(), endPoint.y()), point_t(endPoint.x(), endPoint.y()));
  aps.at(ap->getEndLayerNum()).insert(make_pair(boostb, ap));
}

void FlexRegionQuery::add(frShape* shape) {
  frBox frb;
  shape->getBBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(shape->getLayerNum()).insert(make_pair(boostb, shape));
}

void FlexRegionQuery::add(frVia *via) {
  frBox frb;
  via->getLayer1BBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getLayer1Num()).insert(make_pair(boostb, via));

  via->getLayer2BBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getLayer2Num()).insert(make_pair(boostb, via));

  via->getCutBBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getCutLayerNum()).insert(make_pair(boostb, via));
}

void FlexRegionQuery::remove(frListIter<frPathSeg> &pathSeg) {
  frBox frb;
  pathSeg->getBBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock(mutex_pathsegs.at(pathSeg->getLayerNum()));
  pathsegs.at(pathSeg->getLayerNum()).remove(make_pair(boostb, pathSeg));
}

void FlexRegionQuery::remove(frListIter<frVia> &via) {
  frBox frb;
  via->getLayer1BBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock1(mutex_vias.at(vias->getViaDef()->getLayer1Num()));
  vias.at(via->getViaDef()->getLayer1Num()).remove(make_pair(boostb, via));
  //lock1.unlock();

  via->getLayer2BBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock2(mutex_vias.at(vias->getViaDef()->getLayer2Num()));
  vias.at(via->getViaDef()->getLayer2Num()).remove(make_pair(boostb, via));
  //lock2.unlock();

  via->getCutBBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  //unique_lock lock3(mutex_vias.at(vias->getViaDef()->getCutLayerNum()));
  vias.at(via->getViaDef()->getCutLayerNum()).remove(make_pair(boostb, via));
  //lock3.unlock();
}

void FlexRegionQuery::remove(frListIter<FlexAccessPattern> &ap) {
  frPoint beginPoint, endPoint;
  ap->getPoints(beginPoint, endPoint);
  box_t boostb = box_t(point_t(endPoint.x(), endPoint.y()), point_t(endPoint.x(), endPoint.y()));
  aps.at(ap->getEndLayerNum()).remove(make_pair(boostb, ap));
}

void FlexRegionQuery::remove(frShape* shape) {
  frBox frb;
  shape->getBBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(shape->getLayerNum()).remove(make_pair(boostb, shape));
}

void FlexRegionQuery::remove(frVia *via) {
  frBox frb;
  via->getLayer1BBox(frb);
  box_t boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getLayer1Num()).remove(make_pair(boostb, via));

  via->getLayer2BBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getLayer2Num()).remove(make_pair(boostb, via));

  via->getCutBBox(frb);
  boostb = box_t(point_t(frb.left(), frb.bottom()), point_t(frb.right(), frb.top()));
  shapes.at(via->getViaDef()->getCutLayerNum()).remove(make_pair(boostb, via));
}

void FlexRegionQuery::query(const frBox &box, frLayerNum layerNum, frVector<frListIter<frPathSeg> > &result) {
  vector<rq_iter_value_t<frPathSeg> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  pathsegs.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frVector<frListIter<frPathSeg> > &result) {
  vector<rq_iter_value_t<frPathSeg> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  for (auto &m: pathsegs) {
    m.query(bgi::intersects(boostb), back_inserter(temp));
  }
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frLayerNum layerNum, frVector<frListIter<frVia> > &result) {
  vector<rq_iter_value_t<frVia> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  vias.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frVector<frListIter<frVia> > &result) {
  vector<rq_iter_value_t<frVia> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  for (auto &m: vias) {
    m.query(bgi::intersects(boostb), back_inserter(temp));
  }
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frLayerNum layerNum, frVector<frListIter<FlexAccessPattern> > &result) {
  vector<rq_iter_value_t<FlexAccessPattern> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  aps.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frVector<frListIter<FlexAccessPattern> > &result) {
  vector<rq_iter_value_t<FlexAccessPattern> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  for (auto &m: aps) {
    m.query(bgi::intersects(boostb), back_inserter(temp));
  }
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frLayerNum layerNum, frVector<frPinFig*> &result) {
  vector<rq_rptr_value_t<frPinFig> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  shapes.at(layerNum).query(bgi::intersects(boostb), back_inserter(temp));
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::query(const frBox &box, frVector<frPinFig*> &result) {
  vector<rq_rptr_value_t<frPinFig> > temp;
  box_t boostb = box_t(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  for (auto &m: shapes) {
    m.query(bgi::intersects(boostb), back_inserter(temp));
  }
  transform(temp.begin(), temp.end(), back_inserter(result), [](auto &kv) {return kv.second;});
}

void FlexRegionQuery::init(frLayerNum numLayers) {
  pathsegs.clear();
  vias.clear();
  aps.clear();
  shapes.clear();
  pathsegs.resize(numLayers);
  vias.resize(numLayers);
  aps.resize(numLayers);
  shapes.resize(numLayers);
}

void FlexRegionQuery::print() {
  cout <<endl;
  for (int i = 0; i < (int)getTech()->getLayers().size(); i++) {
    frString layerName;
    getTech()->getLayers().at(i)->getName(layerName);
    cout <<layerName <<" pathseg region query size = " <<pathsegs.at(i).size() <<endl <<flush;
  }
  cout <<endl;
  for (int i = 0; i < (int)getTech()->getLayers().size(); i++) {
    frString layerName;
    getTech()->getLayers().at(i)->getName(layerName);
    cout <<layerName <<" via region query size = " <<vias.at(i).size() <<endl <<flush;
  }
  cout <<endl;
  for (int i = 0; i < (int)getTech()->getLayers().size(); i++) {
    frString layerName;
    getTech()->getLayers().at(i)->getName(layerName);
    cout <<layerName <<" ap region query size = " <<aps.at(i).size() <<endl <<flush;
  }
  cout <<endl;
  for (int i = 0; i < (int)getTech()->getLayers().size(); i++) {
    frString layerName;
    getTech()->getLayers().at(i)->getName(layerName);
    cout <<layerName <<" shape region query size = " <<shapes.at(i).size() <<endl <<flush;
  }
}
