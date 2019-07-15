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
#include "global.h"

using namespace std;
using namespace fr;

string DEF_FILE;
string GUIDE_FILE;
string OUTGUIDE_FILE;
string LEF_FILE;
string OUTTA_FILE;
string OUT_FILE;
string OUT_MAZE_FILE;
string DRC_RPT_FILE;
//string DBPROCESSNODE = "N16_11m_2xa1xd3xe2y2r_utrdl";
string DBPROCESSNODE = "";
int    MAX_THREADS   = 8;
int    VERBOSE       = 0;
int    BOTTOM_ROUTING_LAYER = 0;
bool   ALLOW_PIN_AS_FEEDTHROUGH = false;
bool   USENONPREFTRACKS = true;
bool   USEMINSPACING_OBS = true;
bool   RESERVE_VIA_ACCESS = true;
bool   ENABLE_BOUNDARY_MAR_FIX = true;

int END_ITERATION = 1;

frUInt4 TAVIACOST       = 1;
frUInt4 TAPINCOST       = 4;
frUInt4 TAALIGNCOST     = 4;
frUInt4 TADRCCOST       = 32;
float   TASHAPEBLOATWIDTH = 1.5;

frUInt4 VIACOST         = 4;
// new cost used
frUInt4 GRIDCOST        = 2;
frUInt4 SHAPECOST       = 8;
frUInt4 DRCCOST         = 8;
frUInt4 MARKERCOST      = 64;
frUInt4 MARKERBLOATWIDTH= 1;
frUInt4 BLOCKCOST       = 32;
frUInt4 GUIDECOST       = 1; // disabled change getNextPathCost to enable
float   MARKERDECAY     = 0.8;
float   SHAPEBLOATWIDTH = 3;

ostream& operator<< (ostream& os, const frPoint &pIn) {
  os <<"( " <<pIn.x() <<" " <<pIn.y() <<" )";
  return os;
}

ostream& operator<< (ostream& os, const frRect &pinFigIn) {
  if (pinFigIn.getPin()) {
    os <<"PINFIG (PINNAME/LAYER) " <<pinFigIn.getPin()->getTerm()->getName() <<" " <<pinFigIn.getLayerNum() <<endl;
  }
  frBox tmpBox;
  pinFigIn.getBBox(tmpBox);
  os <<"  RECT " <<tmpBox.left() <<" " <<tmpBox.bottom() <<" " <<tmpBox.right() <<" " <<tmpBox.top();
  return os;
}

ostream& operator<< (ostream& os, const frPolygon &pinFigIn) {
  if (pinFigIn.getPin()) {
    os <<"PINFIG (NAME/LAYER) " <<pinFigIn.getPin()->getTerm()->getName() <<" " <<pinFigIn.getLayerNum() <<endl;
  }
  os <<"  POLYGON";
  for (auto &m: pinFigIn.getPoints()) {
    os <<" ( " <<m.x() <<" " <<m.y() <<" )";
  }
  return os;
}

ostream& operator<< (ostream& os, const frPin &pinIn) {
  os <<"PIN (NAME) " <<pinIn.getTerm()->getName();
  for (auto &m: pinIn.getFigs()) {
    if (m->typeId() == frcRect) {
      os <<endl <<*(static_cast<frRect*>(m.get()));
    } else if (m->typeId() == frcPolygon) {
      os <<endl <<*(static_cast<frPolygon*>(m.get()));
    } else {
      os <<endl <<"Unsupported pinFig object!";
    }
  }
  return os;
}

ostream& operator<< (ostream& os, const frTerm &termIn) {
  frString name;
  frString netName;
  name = termIn.getName();
  if (termIn.getNet()) {
    netName = termIn.getNet()->getName();
  }
  os <<"TERM (NAME/NET) " <<name <<" " <<netName;
  for (auto &m: termIn.getPins()){
    os <<endl <<*m;
  }
  return os;
}

ostream& operator<< (ostream& os, const frInstTerm &instTermIn) {
  frString name;
  frString cellName;
  frString termName;
  frString netName;
  name = instTermIn.getInst()->getName();
  cellName = instTermIn.getInst()->getRefBlock()->getName();
  termName = instTermIn.getTerm()->getName();
  if (instTermIn.getNet()) {
    netName = instTermIn.getNet()->getName();
  } 
  os <<"INSTTERM (NAME/CELL/TERM/NET) " <<name <<" " <<cellName <<" " <<termName <<" " <<netName <<endl;
  os <<*instTermIn.getTerm();
  return os;
}

ostream& operator<< (ostream& os, const frViaDef &viaDefIn) {
  frString name;
  name = viaDefIn.getName();
  os <<"VIA " <<name;
  if (viaDefIn.getDefault()) {
    os <<" DEFAULT";
  }
  for (auto &m: viaDefIn.getLayer1Figs()) {
    if (m->typeId() == frcRect) {
      os <<endl <<*(static_cast<frRect*>(m.get()));
    } else if (m->typeId() == frcPolygon) {
      os <<endl <<*(static_cast<frPolygon*>(m.get()));
    } else {
      os <<endl <<"Unsupported pinFig object!";
    }
  }
  for (auto &m: viaDefIn.getCutFigs()) {
    if (m->typeId() == frcRect) {
      os <<endl <<*(static_cast<frRect*>(m.get()));
    } else if (m->typeId() == frcPolygon) {
      os <<endl <<*(static_cast<frPolygon*>(m.get()));
    } else {
      os <<endl <<"Unsupported pinFig object!";
    }
  }
  for (auto &m: viaDefIn.getLayer2Figs()) {
    if (m->typeId() == frcRect) {
      os <<endl <<*(static_cast<frRect*>(m.get()));
    } else if (m->typeId() == frcPolygon) {
      os <<endl <<*(static_cast<frPolygon*>(m.get()));
    } else {
      os <<endl <<"Unsupported pinFig object!";
    }
  }
  return os;
}

ostream& operator<< (ostream& os, const frLayerBlockage &blkIn) {
  os <<"BLK (LAYER) " <<blkIn.getLayerNum();
  for (auto &point: blkIn.getPoints()) {
    os <<endl <<"  " <<point.x() <<" " <<point.y();
  }
  return os;
}

ostream& operator<< (ostream& os, const frBlock &blockIn) {
  frBox box;
  blockIn.getBBox(box);
  os <<"MACRO "    <<blockIn.getName() <<endl
       <<"  ORIGIN " <<box.left()  <<" " <<box.bottom() <<endl
       <<"  SIZE "   <<box.right() <<" " <<box.top();
  for (auto &m: blockIn.getTerms()) {
    os <<endl <<*m;
  }
  for (auto &m: blockIn.getBlockages()) {
    if (m->typeId() == frcLayerBlockage) {
      os <<endl <<*(static_cast<frLayerBlockage*>(m.get()));
    } else {
      os <<endl <<"Unsupported macro blockage!";
    }
  }
  return os;
}

ostream& operator<< (ostream& os, const frInst &instIn) {
  frPoint tmpPoint;
  frString tmpString;
  frString tmpName;
  instIn.getOrigin(tmpPoint);
  auto tmpOrient = instIn.getOrient();
  tmpName = instIn.getName();
  tmpString = instIn.getRefBlock()->getName();
  os <<"- " <<tmpName <<" " <<tmpString <<" + STATUS + ( "
       <<tmpPoint.x() <<" " <<tmpPoint.y() <<" ) " <<tmpOrient.getName() <<endl;
  for (auto &m: instIn.getInstTerms()) {
    os <<endl <<*m;
  }
  return os;
}

ostream& operator<< (ostream& os, const frBox &box) {
  os <<"( " <<box.left() <<" " <<box.bottom() <<" ) ( " <<box.right() <<" " <<box.top() <<" )";
  return os;
}

