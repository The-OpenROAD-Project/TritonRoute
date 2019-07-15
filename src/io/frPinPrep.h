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

#ifndef _FR_PIN_PREP_H
#define _FR_PIN_PREP_H

#include <memory>
#include "frDesign.h"
#include "frBaseTypes.h"
#include <map>
#include <set>
#include <memory>
#include <tuple>
#include <chrono>

namespace fr {
  class frNet;
  class FlexTrack;
  // not default via, not align upper layer, non-enclosed area, not align lower layer,
  // upper layer area, lower layer area
  typedef std::tuple<bool, bool, frCoord, bool, frCoord, frCoord> viaPriorityTuple;
  // not default via, not align upper layer,
  // upper layer area, lower layer area
  // typedef std::tuple<bool, bool, frCoord, frCoord> viaRawPriorityTuple;
  // not default via, upperWidth, lowerWidth, not align upper, upperArea, lowerArea, not align lower
  typedef std::tuple<bool, frCoord, frCoord, bool, frCoord, frCoord, bool> viaRawPriorityTuple;

  class FlexPinPrep {
  public:
    // constructor
    FlexPinPrep(frTechObject* techIn,
                frDesign* designIn,
                std::vector<frTrackPattern*> &prefTrackPatternsIn,
                std::map<frBlock*, std::map<frOrient, std::map<std::vector<frCoord>, std::set<frInst*, frBlockObjectComp> > >, frBlockObjectComp> &trackOffsetMapIn):
                tech(techIn), design(designIn), prefTrackPatterns(prefTrackPatternsIn), trackOffsetMap(trackOffsetMapIn) {};

    // getters
    // std::shared_ptr<frTechObject> getTech() const {
    //   return tech.lock();
    // }
    // std::shared_ptr<frDesign> getDesign() const {
    //   return design.lock();
    // }
    frTechObject* getTech() const {
      return tech;
    }
    frDesign* getDesign() const {
      return design;
    }
    // others
    void init();
    int main();

    void pinPrep();
    void netPinPrep(const std::shared_ptr<frNet> &net);
    //void mergePinShapes(const std::shared_ptr<frPin> &pin, std::map<frLayerNum, PolygonSet> &layerNum2PS);
    void mergePinShapes(frPin* pin, frTransform &xform, std::map<frLayerNum, PolygonSet> &layerNum2PS);
    void mergePinShapes(frPin* pin, std::map<frLayerNum, PolygonSet> &layerNum2PS);
    void getPinLayerBBox(const std::map<frLayerNum, PolygonSet> &layerNum2PS, 
                         std::map<frLayerNum, Rectangle> &layerNum2BBox);
    void getPinTrackPatterns(const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                             std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &xLoc2TrackPatterns,
                             std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &yLoc2TrackPatterns,
                             std::map<frLayerNum, frPrefRoutingDirEnum> &layerNum2PrefRouteDir);
    void getLayerOnGridPoints(const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                              const std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &xLoc2TrackPatterns,
                              const std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &yLoc2TrackPatterns,
                              std::map<frLayerNum, frPrefRoutingDirEnum> layerNum2PrefRouteDir,
                              std::map<frLayerNum, std::set<frPoint> > &layerNum2OnGridPoints);
    void filterOnGridAccessPoint(const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                                 std::map<frLayerNum, std::set<frPoint> > &layerNum2OnGridPoints,
                                 std::map<frLayerNum, std::set<frPoint> > &layerNum2OnGridAccessPoints);

    void getPinGCellSpan(const std::shared_ptr<frPin> &pin,
                         const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                         std::map<frPoint, std::set<frLayerNum> > &ovlpGCell2Layer);
    void genAndPopulateLayer2APRtree(const std::shared_ptr<frNet> &net,
                                     const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                                     const std::map<frLayerNum, std::set<frPoint> > &layerNum2OnGridPoints,
                                     std::map<frLayerNum, bgi::rtree< rq_iter_value_t<FlexAccessPattern>, bgi::quadratic<16> > > &layer2APs,
                                     std::shared_ptr<frPin> &pin);
    void addOffGridPinAccessPattern(const std::shared_ptr<frNet> &net,
                                    const std::map<frLayerNum, PolygonSet> &layerNum2PS,
                                    const std::map<frPoint, std::set<frLayerNum> > &ovlpGCell2Layer,
                                    std::map<frLayerNum, bgi::rtree< rq_iter_value_t<FlexAccessPattern>, bgi::quadratic<16> > > &layer2APs,
                                    std::shared_ptr<frPin> &pin);
    // bool instTermPinPrep(frInstTerm *instTerm,
    //                      std::map<frPin*, std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > > > &pin2APEPs);
    void instTermPinPrep(frInstTerm* instTerm, const std::set<frInst*, frBlockObjectComp> &instsIn);
    // void termPinPrep(frTerm* term,
    //                  std::map<frPin*, std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > > > &pin2APEPs);
    void termPinPrep(frTerm *term);
    void getPinBBoxTrackPatterns(const std::map<frLayerNum, Rectangle> &layerNum2BBox,
                                 std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &xLoc2TrackPatterns,
                                 std::map<frCoord, std::map<frLayerNum, std::shared_ptr<frTrackPattern> > > &yLoc2TrackPatterns,
                                 std::map<frLayerNum, frPrefRoutingDirEnum> &layerNum2PrefRouteDir);
    void getPinLayerBBoxTrackPatterns(const frLayerNum &currLayerNum,
                                      const PolygonSet &currLayerPS,
                                      std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                      std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns);
    void getPinLayerAPStartPoints(const frLayerNum &currLayerNum,
                                  const PolygonSet &currLayerPS,
                                  std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                  std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                  std::map<frPoint, int> &startPoints);
    void getPinLayerAPStartPoints_Macro(const frLayerNum &currLayerNum,
                                        const PolygonSet &currLayerPS,
                                        const frBox &instBBox,
                                        std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                        std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                        std::map<frPoint, int> &startPoints);
    void getPinLayerAPStartPoints_shapeCenter(const frLayerNum &currLayerNum,
                                              const PolygonSet &currLayerPS,
                                              std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                              std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                              std::map<frPoint, int> &startPoints);
    void getPinLayerAPEndPoints(const frLayerNum &currLayerNum,
                                const PolygonSet &currLayerPS,
                                std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                std::set<std::pair<frLayerNum, frPoint> > &endPoints);
    void getAccessPattern(const frPoint &startPoint, 
                          const frLayerNum &startLayerNum,
                          const frPoint &endPoint,
                          const frLayerNum &endLayerNum,
                          FlexAccessPattern &ap);
    
    void getPlanarEP(const frLayerNum &layerNum, const frPoint &startPt, const frDirEnum &dir, frPoint &endPt);

    bool isValidViaAccess(frPin *pinPtr,
                          frInstTerm *instTerm,
                          const std::vector<frBlockObject*> instObjs,
                          const frLayerNum &layerNum,
                          const frPoint &startPt,
                          frViaDef* viaDef,
                          viaPriorityTuple &priority);
    bool isValidViaAccess(frPin *pinPtr,
                          frTerm *term,
                          const std::vector<frBlockObject*> instObjs,
                          const frLayerNum &layerNum,
                          const frPoint &startPt,
                          frViaDef* viaDef,
                          viaPriorityTuple &priority);
    bool isValidPlanarAccess(frPin* pinPtr,
                             frInstTerm *instTerm,
                             const std::vector<frBlockObject*> instObjs,
                             const frLayerNum &layerNum,
                             const frPoint &startPt,
                             const frPoint &endPt,
                             const PolygonSet &layerPS);
    bool isValidPlanarAccess(frPin* pinPtr,
                             frTerm *term,
                             const std::vector<frBlockObject*> instObjs,
                             const frLayerNum &layerNum,
                             const frPoint &startPt,
                             const frPoint &endPt,
                             const PolygonSet &layerPS);
    void updateAPCost(frInst *inst);
    void getViaRawPriority(frViaDef* viaDef, viaRawPriorityTuple &priority);
  protected:
    frTechObject *tech;
    frDesign *design;
    int APCnt = 0;
    std::vector<frTrackPattern*> prefTrackPatterns;
    std::map<frBlock*, std::map<frOrient, std::map<std::vector<frCoord>, std::set<frInst*, frBlockObjectComp> > >, frBlockObjectComp> trackOffsetMap;
    std::map<frLayerNum, std::map<int, std::map<viaRawPriorityTuple, frViaDef*> > > layerNum2ViaDefs;
    int maxViaAttemptLimit = 3;
    std::chrono::duration<double> accuSpan0, accuSpan1, accuSpan2;
    std::chrono::duration<double> drcSpan0, drcSpan1, drcSpan2;
  };
}



#endif
