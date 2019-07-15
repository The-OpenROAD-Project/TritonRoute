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

#ifndef _TA_PIN_H_
#define _TA_PIN_H_

#include "db/taObj/taBlockObject.h"
//#include "db/taObj/taFig.h"
#include "db/taObj/taShape.h"
#include "db/taObj/taVia.h"

namespace fr{
  //class taTrack;
  class frGuide;
  //class frNet;
  class taPin: public taBlockObject {
  public:
    // constructors
    //taPin(): begin(0), end(0), trackIdx(0), wlen_helper(0), nbrBegin(0), nbrEnd(0),
    //         downViaCoords(), upViaCoords(), guide(nullptr), cost(0) {}
    //taPin(const taIroute &in): taBlockObject(in), begin(in.begin), end(in.end), trackIdx(in.trackIdx), 
    //                           wlen_helper(in.wlen_helper), nbrBegin(in.nbrBegin), nbrEnd(in.nbrEnd),
    //                           downViaCoords(in.downViaCoords), upViaCoords(in.upViaCoords), guide(in.guide),
    //                           cost(0) {}
    taPin(): taBlockObject(), guide(nullptr), pinFigs(), wlen_helper(0), pin(false), wlen_helper2(0), cost(0), numAssigned(0)/*, drcCost(0)*/ {}
    // getters
    //void getCoords(frCoord &bp, frCoord &ep) const {
    //  bp = begin;
    //  ep = end;
    //}
    //void getNbrCoords(frCoord &bp, frCoord &ep) const {
    //  bp = nbrBegin;
    //  ep = nbrEnd;
    //}
    //taTrack* getTrack() const {
    //  return track;
    //}
    //int getTrackIdx() const {
    //  return trackIdx;
    //}
    int getWlenHelper() const {
      return wlen_helper;
    }
    bool hasWlenHelper2() const {
      return pin;
    }
    int getWlenHelper2() const {
      return wlen_helper2;
    }
    //const std::vector<frCoord>& getViaCoords(bool isUpVia) const {
    //  return isUpVia ? upViaCoords : downViaCoords;
    //}
    //std::vector<frCoord>& getViaCoords(bool isUpVia) {
    //  return isUpVia ? upViaCoords : downViaCoords;
    //}
    frGuide* getGuide() const {
      return guide;
    }
    const std::vector<std::unique_ptr<taPinFig> >& getFigs() const {
      return pinFigs;
    }
    std::vector<std::unique_ptr<taPinFig> >& getFigs() {
      return pinFigs;
    }
    frCost getCost() const {
      return cost;
    }
    int getNumAssigned() const {
      return numAssigned;
    }
    //frCost getDrcCost() const {
    //  return drcCost;
    //}
    // setters
    //void setCoords(frCoord bp, frCoord ep) {
    //  begin = bp;
    //  end = ep;
    //}
    //void setNbrCoords(frCoord bp, frCoord ep) {
    //  nbrBegin = bp;
    //  nbrEnd = ep;
    //}
    //void setTrack(taTrack *in) {
    //  track = in;
    //}
    //void setTrackIdx(int in) {
    //  trackIdx = in;
    //}
    void setWlenHelper(int in) {
      wlen_helper = in;
    }
    void setWlenHelper2(frCoord in) {
      pin = true;
      wlen_helper2 = in;
    }
    //void addViaCoords(frCoord in, bool isUpVia) {
    //  if (isUpVia) {
    //    upViaCoords.push_back(in);
    //  } else {
    //    downViaCoords.push_back(in);
    //  }
    //}
    void setGuide(frGuide* in) {
      guide = in;
    }
    void addPinFig(std::unique_ptr<taPinFig> &in) {
      in->addToPin(this);
      pinFigs.push_back(std::move(in));
    }
    void setCost(frCost in) {
      cost = in;
    }
    void addCost(frCost in) {
      cost += in;
    }
    void addNumAssigned() {
      numAssigned++;
    }
    //void setDrcCost(frCost in) {
    //  drcCost = in;
    //}
    // others
    frBlockObjectEnum typeId() const override {
      return tacPin;
    }
    bool operator<(const taPin &b) const {
      if (this->cost != b.cost) {
        return this->getCost() > b.getCost();
      } else {
        return this->getId() < b.getId();
      }
    }
  protected:
    //frCoord                begin, end;
    //int                    trackIdx;
    //frCoord                nbrBegin, nbrEnd; // for nbr local guides
    //std::vector<frCoord>   downViaCoords;
    //std::vector<frCoord>   upViaCoords;
    frGuide*                                guide;
    std::vector<std::unique_ptr<taPinFig> > pinFigs;
    int                                     wlen_helper; // for nbr global guides
    bool                                    pin;
    frCoord                                 wlen_helper2; // for local guides and pin guides
    frCost                                  cost;
    int                                     numAssigned;
    //frCost                                  drcCost;
  };
  struct taPinComp {
    bool operator()(const taPin* lhs, const taPin* rhs) const {
      return *lhs < *rhs;
    }
  };
}
#endif
