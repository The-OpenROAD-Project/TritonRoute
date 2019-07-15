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

#ifndef _FR_IO_H_
#define _FR_IO_H_

#include <memory>
#include <list>
#include <boost/icl/interval_set.hpp>
#include "frDesign.h"

#include "defrReader.hpp"
#include "defwWriter.hpp"
#include "lefrReader.hpp"

namespace fr {
  namespace io {
    // not default via, upperWidth, lowerWidth, not align upper, upperArea, lowerArea, not align lower
    typedef std::tuple<bool, frCoord, frCoord, bool, frCoord, frCoord, bool> viaRawPriorityTuple;
    
    class Parser {
    public:
      // constructors
      Parser(frDesign* designIn): design(designIn), tech(design->getTech()), tmpBlock(nullptr), readLayerCnt(0),
                                  tmpGuides(), tmpGRPins(), trackOffsetMap(), prefTrackPatterns(), numRefBlocks(0),
                                  numInsts(0), numTerms(0), numNets(0), numBlockages(0) {}
      // others
      void readLefDef();
      void readGuide();
      void postProcess();
      void postProcessGuide();
      // rtree init
      //void buildRtree4Routes();
      //void buildRtree4Insts();
      std::map<frBlock*, std::map<frOrient, std::map<std::vector<frCoord>, std::set<frInst*, frBlockObjectComp> > >, frBlockObjectComp> &getTrackOffsetMap() {
        return trackOffsetMap;
      }
      std::vector<frTrackPattern*> &getPrefTrackPatterns() {
        return prefTrackPatterns;
      }

    protected:
      void readLef();
      void readDef();

      frDesign*       design;
      frTechObject*   tech;

      std::unique_ptr<frBlock>        tmpBlock;
      // temporary variables
      int                             readLayerCnt;
      std::map<frNet*, std::vector<frRect>, frBlockObjectComp> tmpGuides;
      std::vector<std::pair<frBlockObject*, frPoint> > tmpGRPins;
      std::map<frBlock*, 
               std::map<frOrient, std::map<std::vector<frCoord>, std::set<frInst*, frBlockObjectComp> > >,
               frBlockObjectComp> trackOffsetMap;
      std::vector<frTrackPattern*> prefTrackPatterns;
      int numRefBlocks;
      int numInsts;
      int numTerms;     // including instterm and term
      int numNets;      // including snet and net
      int numBlockages; // including instBlockage and blockage

      // LEF/DEF parser helper
      static int getDefDieArea(defrCallbackType_e type, defiBox* box, defiUserData data);
      static int getDefTracks(defrCallbackType_e type, defiTrack* track, defiUserData data);
      static int getDefVias(defrCallbackType_e type, defiVia* comp, defiUserData data);
      static int getDefComponents(defrCallbackType_e type, defiComponent* comp, defiUserData data);
      static int getDefTerminals(defrCallbackType_e type, defiPin* term, defiUserData data);
      static int getDefNets(defrCallbackType_e type, defiNet* net, defiUserData data);
      static int getDefInteger(defrCallbackType_e type, int number, defiUserData data);
      static int getDefString(defrCallbackType_e type, const char* str, defiUserData data);
      static int getDefVoid(defrCallbackType_e type, void* variable, defiUserData data);
      static int getDefUnits(defrCallbackType_e type, double number, defiUserData data);
      static int getLefMacros(lefrCallbackType_e type, lefiMacro* macro, lefiUserData data);
      static int getLefPins(lefrCallbackType_e type, lefiPin* pin, lefiUserData data);
      static int getLefObs(lefrCallbackType_e type, lefiObstruction* obs, lefiUserData data);
      static int getLefString(lefrCallbackType_e type, const char* string, lefiUserData data);
      static int getLefUnits(lefrCallbackType_e type, lefiUnits* units, lefiUserData data);
      static int getLefManufacturingGrid(lefrCallbackType_e type, double number, lefiUserData data);
      static int getLefLayers(lefrCallbackType_e type, lefiLayer* layer, lefiUserData data);
      static int getLefVias(lefrCallbackType_e type, lefiVia* via, lefiUserData data);
      static int getLefViaRules(lefrCallbackType_e type, lefiViaRule* via, lefiUserData data);
      
      static int getLef58CornerSpacing(void *data, const std::string &sIn);
      static int getLef58SpacingTable(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58SpacingTable_parallelRunLength(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58Spacing(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58Spacing_endOfLineWithin(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutClass(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacing(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacing_helper(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacing_parallelWithin(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacing_adjacentCuts(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacing_layer(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacingTable(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacingTable_helper(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacingTable_others(void *data, frLayer* tmpLayer, const std::string &sIn);
      static int getLef58CutSpacingTable_prl(void *data, frLayer* tmpLayer, const std::string &sIn, 
                                             const std::shared_ptr<frLef58CutSpacingTableConstraint> &con);
      static int getLef58CutSpacingTable_default(void *data, frLayer* tmpLayer, const std::string &sIn, 
                                             const std::shared_ptr<frLef58CutSpacingTableConstraint> &con);
      static int getLef58CutSpacingTable_layer(void *data, frLayer* tmpLayer, const std::string &sIn, 
                                               const std::shared_ptr<frLef58CutSpacingTableConstraint> &con,
                                               frLayerNum &secondLayerNum);
      static int getLef58CutSpacingTable_cutClass(void *data, frLayer* tmpLayer, const std::string &sIn, 
                                                  const std::shared_ptr<frLef58CutSpacingTableConstraint> &con,
                                                  bool hasSecondLayer, frLayerNum secondLayerNum);

      // postProcess functions
      void buildCMap();
      void buildGCellPatterns();
      void buildGCellPatterns_helper(frCoord &GCELLGRIDX, frCoord &GCELLGRIDY, frCoord &GCELLOFFSETX, frCoord &GCELLOFFSETY);
      void buildGCellPatterns_getWidth(frCoord &GCELLGRIDX, frCoord &GCELLGRIDY);
      void buildGCellPatterns_getOffset(frCoord GCELLGRIDX, frCoord GCELLGRIDY, frCoord &GCELLOFFSETX, frCoord &GCELLOFFSETY);
      void initDefaultVias();
      void getViaRawPriority(frViaDef* viaDef, viaRawPriorityTuple &priority);
      void initDefaultVias_N16(const std::string &in);

      // instance analysis
      void instAnalysis();

      // postProcessGuide functions
      void genGuides(frNet* net, std::vector<frRect> &rects);
      void genGuides_merge(std::vector<frRect> &rects, std::vector<std::map<frCoord, boost::icl::interval_set<frCoord> > > &intvs);
      void genGuides_split(std::vector<frRect> &rects, std::vector<std::map<frCoord, boost::icl::interval_set<frCoord> > > &intvs,
                           std::map<std::pair<frPoint, frLayerNum>, std::set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap,
                           std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap,
                           bool isRetry);
      void genGuides_gCell2PinMap(frNet* net, std::map<std::pair<frPoint, frLayerNum>, std::set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap,
                                  std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap);
      void genGuides_gCell2TermMap(std::map<std::pair<frPoint, frLayerNum>, std::set<frBlockObject*, frBlockObjectComp> > &gCell2PinMap, 
                                   std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap,
                                   frTerm* term, frBlockObject* origTerm);
      void genGuides_initPin2GCellMap(frNet* net, std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap);
      void genGuides_buildNodeMap(std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap, int &gCnt, int &nCnt,
                                  std::vector<frRect> &rects, std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap);
      bool genGuides_astar(frNet *net,
                           std::vector<bool> &adjVisited, std::vector<int> &adjPrevIdx, 
                           std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap, int &gCnt, int &nCnt, bool forceFeedThrough, bool retry);
      void genGuides_final(frNet *net, std::vector<frRect> &rects, std::vector<bool> &adjVisited, std::vector<int> &adjPrevIdx, int gCnt, int nCnt,
                           std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2GCellMap);

      // write guide
      void writeGuideFile();
    };
    class Writer {
    public:
      // constructors
      //Writer(): tech(std::make_shared<frTechObject>()), design(std::make_shared<frDesign>()) {};
      Writer(frDesign* designIn): tech(designIn->getTech()), design(designIn) {}
      // getters
      frTechObject* getTech() const {
        return tech;
      }
      frDesign* getDesign() const {
        return design;
      }
      // others
      void writeFromTA();
      void writeFromDR(const std::string &str = "");
      std::map< frString, std::list<std::shared_ptr<frConnFig> > > connFigs; // all connFigs ready to def
    protected:
      frTechObject*                                  tech;
      frDesign*                                      design;
      //std::list< // temp for merge and split
      

      void fillConnFigs(bool isTA);
      void fillConnFigs_net(frNet* net, bool isTA);
      void mergeSplitConnFigs(std::list<std::shared_ptr<frConnFig> > &connFigs);
      void splitVia_helper(frLayerNum layerNum, int isH, frCoord trackLoc, frCoord x, frCoord y, 
                           std::vector< std::vector< std::map<frCoord, std::vector<std::shared_ptr<frPathSeg> > > > > &mergedPathSegs);
      int writeDef(bool isTA, const std::string &str = "");
    };
  }
}


#endif
