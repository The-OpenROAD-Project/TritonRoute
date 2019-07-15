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

#ifndef _FR_TECHOBJECT_H_
#define _FR_TECHOBJECT_H_

#include <map>
#include <iostream>
#include <vector>
#include <memory>
#include "frBaseTypes.h"
#include "db/tech/frLayer.h"
#include "db/obj/frVia.h"
#include "db/tech/frViaRuleGenerate.h"

namespace fr {
  namespace io {
    class Parser;
  }
  class frTechObject {
  public:
    // constructors
    frTechObject() {}
    // getters
    frUInt4 getDBUPerUU() const {
      return dbUnit;
    }
    frUInt4 getManufacturingGrid() const {
      return manufacturingGrid;
    }
    frLayer* getLayer(const frString &name) const {
      if (name2layer.find(name) == name2layer.end()) {
        // std::cout <<"Error: cannot find layer" <<std::endl;
        // exit(1);
        return nullptr;
      } else {
        return name2layer.at(name);
      }
    }
    frLayer* getLayer(frLayerNum in) const {
      if ((int)in < 0 || in >= (int)layers.size()) {
        std::cout <<"Error: cannot find layer" <<std::endl;
        exit(1);
      } else {
        return layers.at(in).get();
      }
    }
    frLayerNum getBottomLayerNum() const {
      return 0;
    }
    frLayerNum getTopLayerNum() const {
      return (frLayerNum)((int)layers.size() - 1);
    }
    std::vector<std::unique_ptr<frLayer> >& getLayers() {
      return layers;
    }
    const std::vector<std::unique_ptr<frLayer> >& getLayers() const {
      return layers;
    }
    std::vector<std::unique_ptr<frViaDef> >& getVias() {
      return vias;
    }
    const std::vector<std::unique_ptr<frViaDef> >& getVias() const {
      return vias;
    }
    std::vector<std::unique_ptr<frViaRuleGenerate> >& getViaRuleGenerates() {
      return viaRuleGenerates;
    }
    const std::vector<std::unique_ptr<frViaRuleGenerate> >& getViaRuleGenerates() const {
      return viaRuleGenerates;
    }

    // setters
    void setDBUPerUU(frUInt4 uIn) {
      dbUnit = uIn;
    }
    void setManufacturingGrid(frUInt4 in) {
      manufacturingGrid = in;
    }
    void addLayer(std::unique_ptr<frLayer> &in) {
      name2layer[in->getName()] = in.get();
      layers.push_back(std::move(in));
    }
    void addVia(std::unique_ptr<frViaDef> &in) {
      name2via[in->getName()] = in.get();
      vias.push_back(std::move(in));
    }
    void addViaRuleGenerate(std::unique_ptr<frViaRuleGenerate> &in) {
      name2viaRuleGenerate[in->getName()] = in.get();
      viaRuleGenerates.push_back(std::move(in));
    }
    void addConstraint(const std::shared_ptr<frConstraint> &constraintIn) {
      constraints.push_back(constraintIn);
    }
    void addUConstraint(std::unique_ptr<frConstraint> &in) {
      uConstraints.push_back(std::move(in));
    }

    // debug
    void printAllConstraints() {
      std::cout << "List of Constraints:\n";
      for (auto &layer: layers) {
        std::cout << "  Layer " << layer->getName() << "\n";
        for (auto &constraint: layer->getConstraints()) {
          if (std::dynamic_pointer_cast<frCutSpacingConstraint>(constraint)) {
            std::cout << "    CUT SPACING " << std::dynamic_pointer_cast<frCutSpacingConstraint>(constraint)->getCutSpacing() * 1.0 / dbUnit << "\n";
          }
          if (std::dynamic_pointer_cast<frSpacingConstraint>(constraint)) {
            std::cout << "    ROUTING SPACING " << std::dynamic_pointer_cast<frSpacingConstraint>(constraint)->getMinSpacing() * 1.0 / dbUnit << "\n"; 
          }
        }
      }
    }

    void printDefaultVias() {
      std::cout << "List of default vias:\n";
      for (auto &layer: layers) {
        if (layer->getType() == frLayerTypeEnum::CUT) {
          std::cout << "  Layer " << layer->getName() << "\n" << std::flush;
          std::cout << "    default via: " << layer->getDefaultViaDef()->getName() << "\n";
        }
      }
    }     

    // others
    void printAllVias();
    friend class io::Parser;
  protected:
    frUInt4                                          dbUnit;
    frUInt4                                          manufacturingGrid;

    std::map<frString, frLayer*>                     name2layer;
    std::vector<std::unique_ptr<frLayer> >           layers;

    std::map<frString, frViaDef*>                    name2via;
    std::vector<std::unique_ptr<frViaDef> >          vias;

    std::map<frString, frViaRuleGenerate*>           name2viaRuleGenerate;
    std::vector<std::unique_ptr<frViaRuleGenerate> > viaRuleGenerates;

    frCollection<std::shared_ptr<frConstraint> >     constraints;
    std::vector<std::unique_ptr<frConstraint> >      uConstraints;
  };
}

#endif
