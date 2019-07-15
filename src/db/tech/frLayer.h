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

#ifndef _FR_LAYER_H_
#define _FR_LAYER_H_

#include "frBaseTypes.h"
#include "db/infra/frPrefRoutingDir.h"
#include "db/infra/frSegStyle.h"
#include "db/obj/frVia.h"
#include "db/tech/frConstraint.h"

namespace fr {
  namespace io {
    class Parser;
  }
  class frLayer {
  public:
    friend class io::Parser;
    // constructor
    frLayer(): pitch(0), width(0), defaultViaDef(nullptr), minSpc(nullptr), shortConstraint(nullptr), areaConstraint(nullptr), minStepConstraint(nullptr) {}
    frLayer(frLayerNum layerNumIn, const frString &nameIn): layerNum(layerNumIn), name(nameIn), pitch(0), width(0), minWidth(-1), defaultViaDef(nullptr),
                                                            minSpc(nullptr), shortConstraint(nullptr), areaConstraint(nullptr), minStepConstraint(nullptr),
                                                            minWidthConstraint(nullptr), minimumcutConstraints() {}
    // setters
    void setLayerNum(frLayerNum layerNumIn) {
      layerNum = layerNumIn;
    }
    void setName(const frString &nameIn) {
      name = nameIn;
    }
    void setPitch(frUInt4 in) {
      pitch = in;
    }
    void setWidth(frUInt4 widthIn) {
      width = widthIn;
    }
    void setMinWidth(frUInt4 minWidthIn) {
      minWidth = minWidthIn;
    }
    void setDir(frPrefRoutingDirEnum dirIn) {
      dir.set(dirIn);
    }
    void setDefaultViaDef(frViaDef* in) {
      defaultViaDef = in;
    }
    void addConstraint(const std::shared_ptr<frConstraint> &consIn) {
      constraints.push_back(consIn);
    }
    void setType(frLayerTypeEnum typeIn) {
      type = typeIn;
    }
    void addViaDef(frViaDef* viaDefIn) {
      viaDefs.insert(viaDefIn);
    }

    // getters
    frLayerNum getLayerNum() const {
      return layerNum;
    }
    void getName(frString &nameIn) const {
      nameIn = name;
    }
    frString getName() const {
      return name;
    }
    frUInt4 getPitch() const {
      return pitch;
    }
    frUInt4 getWidth() const {
      return width;
    }
    frUInt4 getMinWidth() const {
      return minWidth;
    }
    frPrefRoutingDir getDir() const {
      return dir;
    }
    frSegStyle getDefaultSegStyle() const {
      frSegStyle style;
      style.setWidth(width);
      style.setBeginStyle(frcExtendEndStyle, width/2);
      style.setEndStyle(frcExtendEndStyle, width/2);
      return style;
    }
    frViaDef* getDefaultViaDef() const {
      return defaultViaDef;
    }
    std::set<frViaDef*> getViaDefs() const {
      return viaDefs;
    }
    frCollection<std::shared_ptr<frConstraint> > getConstraints() const {
      frCollection<std::shared_ptr<frConstraint> > constraintsOut;
      for (auto constraint: constraints) {
        constraintsOut.push_back(constraint.lock());
      }
      return constraintsOut;
    }
    frLayerTypeEnum getType() const {
      return type;
    }

    // cut classes
    bool hasLef58CutClassConstraint() const {
      return (lef58CutClassConstraint.lock()) ? true : false;
    }
    std::shared_ptr<frLef58CutClassConstraint> getLef58CutClassConstraint() const {
      return lef58CutClassConstraint.lock();
    }
    // cut spacing table
    bool hasLef58CutSpacingTableConstraints() const {
      return (lef58CutSpacingTableConstraints.size()) ? true : false;
    }
    frCollection<std::shared_ptr<frLef58CutSpacingTableConstraint> > getLef58CutSpacingTableConstraints() const {
      frCollection<std::shared_ptr<frLef58CutSpacingTableConstraint> > sol;
      std::transform(lef58CutSpacingTableConstraints.begin(), lef58CutSpacingTableConstraints.end(), 
                     std::back_inserter(sol), 
                     [](auto &kv) {return kv.lock();});
      return sol;
    }
    // cut spacing
    bool hasLef58CutSpacingConstraints() const {
      return (lef58CutSpacingConstraints.size()) ? true : false;
    }
    frCollection<std::shared_ptr<frLef58CutSpacingConstraint> > getLef58CutSpacingConstraints() const {
      frCollection<std::shared_ptr<frLef58CutSpacingConstraint> > sol;
      std::transform(lef58CutSpacingConstraints.begin(), lef58CutSpacingConstraints.end(), 
                     std::back_inserter(sol), 
                     [](auto &kv) {return kv.lock();});
      return sol;
    }
    // spacing end of line
    bool hasLef58SpacingEndOfLineConstraints() const {
      return (lef58SpacingEndOfLineConstraints.size()) ? true : false;
    }
    frCollection<std::shared_ptr<frLef58SpacingEndOfLineConstraint> > getLef58SpacingEndOfLineConstraints() const {
      frCollection<std::shared_ptr<frLef58SpacingEndOfLineConstraint> > sol;
      std::transform(lef58SpacingEndOfLineConstraints.begin(), lef58SpacingEndOfLineConstraints.end(), 
                     std::back_inserter(sol), 
                     [](auto &kv) {return kv.lock();});
      return sol;
    }

    // new functions
    bool hasMinSpacing() const {
      return (minSpc);
    }
    frConstraint* getMinSpacing() const {
      return minSpc;
    }
    void setMinSpacing(frConstraint* in) {
      if (minSpc != nullptr) {
        std::cout <<"Warning: override minspacing rule, ";
        if (minSpc->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
          std::cout <<"original type is SPACING, ";
        } else if (minSpc->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
          std::cout <<"original type is SPACINGTABLE PARALLELRUNLENGTH, ";
        } else if (minSpc->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
          std::cout <<"original type is SPACINGTABLE TWOWIDTHS, ";
        } else {
          std::cout <<"original type is UNKNWON, ";
        }
        if (in->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
          std::cout <<"new type is SPACING";
        } else if (in->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
          std::cout <<"new type is SPACINGTABLE PARALLELRUNLENGTH";
        } else if (in->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
          std::cout <<"new type is SPACINGTABLE TWOWIDTHS";
        } else {
          std::cout <<"new type is UNKNWON";
        }
        std::cout <<std::endl;
      }
      minSpc = in;
    }
    bool hasEolSpacing() const {
      return (eols.empty() ? false : true);
    }
    void addEolSpacing(frSpacingEndOfLineConstraint* in) {
      eols.push_back(in);
    }
    const std::vector<frSpacingEndOfLineConstraint*>& getEolSpacing() const {
      return eols;
    }
    std::vector<frSpacingEndOfLineConstraint*>& getEolSpacing() {
      return eols;
    }
    void addCutConstraint(frCutSpacingConstraint* in) {
      cutConstraints.push_back(in);
    }
    const std::vector<frCutSpacingConstraint*>& getCutConstraint() const {
      return cutConstraints;
    }
    const std::vector<frCutSpacingConstraint*>& getCutSpacing() const {
      return cutConstraints;
    }
    std::vector<frCutSpacingConstraint*>& getCutSpacing() {
      return cutConstraints;
    }
    bool hasCutSpacing() const {
      return (!cutConstraints.empty());
    }
    void setShortConstraint(frShortConstraint* in) {
      shortConstraint = in;
    }
    frShortConstraint* getShortConstraint() {
      return shortConstraint;
    }
    void setAreaConstraint(frAreaConstraint* in) {
      areaConstraint = in;
    }
    frAreaConstraint* getAreaConstraint() {
      return areaConstraint;
    }
    void setMinStepConstraint(frMinStepConstraint* in) {
      minStepConstraint = in;
    }
    frMinStepConstraint* getMinStepConstraint() {
      return minStepConstraint;
    }
    void setMinWidthConstraint(frMinWidthConstraint* in) {
      minWidthConstraint = in;
    }
    frMinWidthConstraint* getMinWidthConstraint() {
      return minWidthConstraint;
    }

    void addMinimumcutConstraint(frMinimumcutConstraint* in) {
      minimumcutConstraints.push_back(in);
    }
    const std::vector<frMinimumcutConstraint*>& getMinimumcutConstraints() const {
      return minimumcutConstraints;
    }
    bool hasMinimumcut() const {
      return (!minimumcutConstraints.empty());
    }

  protected:
    frLayerTypeEnum                                                 type;
    frLayerNum                                                      layerNum;
    frString                                                        name;
    frUInt4                                                         pitch;
    frUInt4                                                         width;
    frUInt4                                                         minWidth;
    frPrefRoutingDir                                                dir;
    frViaDef*                                                       defaultViaDef;
    std::set<frViaDef*>                                             viaDefs;
    frCollection<std::weak_ptr<frConstraint>  >                     constraints;
    std::weak_ptr<frLef58CutClassConstraint>                        lef58CutClassConstraint;     
    frCollection<std::weak_ptr<frLef58CutSpacingTableConstraint> >  lef58CutSpacingTableConstraints;
    frCollection<std::weak_ptr<frLef58CutSpacingConstraint> >       lef58CutSpacingConstraints;
    frCollection<std::weak_ptr<frLef58SpacingEndOfLineConstraint> > lef58SpacingEndOfLineConstraints;
    frConstraint*                                                   minSpc;
    std::vector<frSpacingEndOfLineConstraint*>                      eols;
    std::vector<frCutSpacingConstraint*>                            cutConstraints;
    frShortConstraint*                                              shortConstraint;
    frAreaConstraint*                                               areaConstraint;
    frMinStepConstraint*                                            minStepConstraint;
    frMinWidthConstraint*                                           minWidthConstraint;
    std::vector<frMinimumcutConstraint*>                            minimumcutConstraints;
  };
}

#endif
