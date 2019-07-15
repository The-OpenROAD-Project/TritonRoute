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

#ifndef _FR_NET_H_
#define _FR_NET_H_

//#include <boost/graph/adjacency_list.hpp>
#include "frBaseTypes.h"
#include "db/obj/frBlockObject.h"
#include "db/obj/frGuide.h"
#include "db/obj/frShape.h"
#include "db/obj/frVia.h"

namespace fr {
  class frInstTerm;
  class frTerm;

  class frNet: public frBlockObject {
  public:
    // constructors
    //frNet(): frBlockObject() {}
    frNet(const frString &in): frBlockObject(), name(in), instTerms(), terms(), shapes(), vias(), pwires(), guides(), type(frNetEnum::frcNormalNet) {}
    // getters
    const frString& getName() const {
      return name;
    }
    const std::vector<frInstTerm*>& getInstTerms() const {
      return instTerms;
    }
    std::vector<frInstTerm*>& getInstTerms() {
      return instTerms;
    }
    const std::vector<frTerm*>& getTerms() const {
      return terms;
    }
    std::vector<frTerm*>& getTerms() {
      return terms;
    }
    std::list<std::unique_ptr<frShape> >& getShapes() {
      return shapes;
    }
    const std::list<std::unique_ptr<frShape> >& getShapes() const {
      return shapes;
    }
    std::list<std::unique_ptr<frVia> >& getVias() {
      return vias;
    }
    const std::list<std::unique_ptr<frVia> >& getVias() const {
      return vias;
    }
    std::list<std::unique_ptr<frShape> >& getPatchWires() {
      return pwires;
    }
    const std::list<std::unique_ptr<frShape> >& getPatchWires() const {
      return pwires;
    }
    std::vector<std::unique_ptr<frGuide> >& getGuides() {
      return guides;
    }
    const std::vector<std::unique_ptr<frGuide> >& getGuides() const {
      return guides;
    }
    // setters
    void addInstTerm(frInstTerm* in) {
      instTerms.push_back(in);
    }
    void addTerm(frTerm* in) {
      terms.push_back(in);
    }
    void setName(const frString &stringIn) {
      name = stringIn;
    }
    void addShape(std::unique_ptr<frShape> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      shapes.push_back(std::move(in));
      rptr->setIter(--shapes.end());
    }
    void addVia(std::unique_ptr<frVia> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      vias.push_back(std::move(in));
      rptr->setIter(--vias.end());
    }
    void addPatchWire(std::unique_ptr<frShape> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      pwires.push_back(std::move(in));
      rptr->setIter(--pwires.end());
    }
    void addGuide(std::unique_ptr<frGuide> &in) {
      auto rptr = in.get();
      rptr->addToNet(this);
      guides.push_back(std::move(in));
      //rptr->setIter(--vias.end());
    }
    void clearGuides() {
      guides.clear();
    }
    void removeShape(frShape* in) {
      shapes.erase(in->getIter());
    }
    void removeVia(frVia* in) {
      vias.erase(in->getIter());
    }
    void removePatchWire(frShape* in) {
      pwires.erase(in->getIter());
    }
    // others
    frNetEnum getType() const {
      return type;
    }
    void setType(frNetEnum in) {
      type = in;
    }
    virtual frBlockObjectEnum typeId() const override {
      return frcNet;
    }
  protected:
    frString                                  name;
    std::vector<frInstTerm*>                  instTerms;
    std::vector<frTerm*>                      terms;     // terms is IO
    std::list<std::unique_ptr<frShape> >      shapes;
    std::list<std::unique_ptr<frVia> >        vias;
    std::list<std::unique_ptr<frShape> >      pwires;
    //std::list<std::unique_ptr<frGuide> >    guides;
    std::vector<std::unique_ptr<frGuide> >    guides;
    frNetEnum                                 type;
  };
}

#endif
