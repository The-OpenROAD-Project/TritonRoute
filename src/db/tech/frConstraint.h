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

#ifndef _FR_CONSTRAINT_H_
#define _FR_CONSTRAINT_H_

#include "frBaseTypes.h"
#include <map>
#include <iterator>
#include <algorithm>
#include <memory>
#include <utility>
#include "db/tech/frLookupTbl.h"

namespace fr {
  // enum class frConstraintTypeEnum {
  //   SPACING,
  //   SPACINGTABLE
  // };

  //enum class frSpacingConstraintTypeEnum {
  //  RANGE,
  //  LENGTHTHRESHOLD,
  //  ENDOFLINE,
  //  SAMENET,
  //  NOTCHLENGTH,
  //  ENDOFNOTCHWIDTH
  //};

  //enum class frSpacingTableConstraintTypeEnum {
  //  PARALLELRUNLENGTH,
  //  TWOWIDTH
  //};

  enum class frLef58CornerSpacingExceptionEnum {
    NONE,
    EXCEPTSAMENET,
    EXCEPTSAMEMETAL
  };


  // base type for design rule
  class frConstraint {
  public:
    frConstraint() {}
    virtual ~frConstraint() {}
    virtual frConstraintTypeEnum typeId() const = 0;    
  protected:
  };

  namespace io {
    class Parser;
  }
  class frLef58CutClassConstraint : public frConstraint {
  public:
    friend class io::Parser;
    // constructors;
    frLef58CutClassConstraint() {}
    // getters
    frCollection< std::shared_ptr<frLef58CutClass> > getCutClasses() const {
      frCollection<std::shared_ptr<frLef58CutClass> > sol;
      std::transform(cutClasses.begin(), cutClasses.end(), 
                     std::back_inserter(sol), 
                     [](auto &kv) {return kv.second;});
      return sol;
    }
    // setters
    void addToCutClass(const std::shared_ptr<frLef58CutClass> &in) {
      cutClasses[in->getName()] = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutClassConstraint;
    }
  protected:
    std::map<frString, std::shared_ptr<frLef58CutClass> > cutClasses;
  };

  // short

  class frShortConstraint : public frConstraint {
  public:
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcShortConstraint;
    }
  };

  // minStep
  class frMinStepConstraint : public frConstraint {
  public:
    // constructor
    frMinStepConstraint() : minStepLength(-1), 
                            maxLength(-1), 
                            insideCorner(false), 
                            outsideCorner(true), 
                            step(false),
                            maxEdges(-1) {}
    // getter
    frCoord getMinStepLength() {
      return minStepLength;
    }
    bool hasMaxLength() {
      return (maxLength != -1);
    }
    frCoord getMaxLength() {
      return maxLength;
    }
    bool hasInsideCorner() {
      return insideCorner;
    }
    bool hasOutsideCorner() {
      return outsideCorner;
    }
    bool hasStep() {
      return step;
    }
    bool hasMaxEdges() {
      return (maxEdges != -1);
    }
    int getMaxEdges() {
      return maxEdges;
    }
    // setter
    void setInsideCorner(bool in) {
      insideCorner = in;
    }
    void setOutsideCorner(bool in) {
      outsideCorner = in;
    }
    void setStep(bool in) {
      step = in;
    }
    void setMinStepLength(frCoord in) {
      minStepLength = in;
    }
    void setMaxLength(frCoord in) {
      maxLength = in;
    }
    void setMaxEdges(int in) {
      maxEdges = in;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcMinStepConstraint;
    }

  protected:
    frCoord minStepLength;
    frCoord maxLength;
    bool insideCorner;
    bool outsideCorner;
    bool step;
    int maxEdges;
  };

  // minimumcut
  class frMinimumcutConstraint: public frConstraint {
  public: 
    frMinimumcutConstraint(): numCuts(-1), width(-1), cutDistance(-1), 
                              connection(frMinimumcutConnectionEnum::UNKNOWN),
                              length(-1), distance(-1) {}
    // getters
    int getNumCuts() const {
      return numCuts;
    }
    frCoord getWidth() const {
      return width;
    }
    bool hasWithin() const {
      return !(cutDistance == -1);
    }
    frCoord getCutDistance() const {
      return cutDistance;
    }
    bool hasConnection() const {
      return !(connection == frMinimumcutConnectionEnum::UNKNOWN);
    }
    frMinimumcutConnectionEnum getConnection() const {
      return connection;
    }
    bool hasLength() const {
      return !(length == -1);
    }
    frCoord getLength() const {
      return length;
    }
    frCoord getDistance() const {
      return distance;
    }
    // setters
    void setNumCuts(int in) {
      numCuts = in;
    }
    void setWidth(frCoord in) {
      width = in;
    }
    void setWithin(frCoord in) {
      cutDistance = in;
    }
    void setConnection(frMinimumcutConnectionEnum in) {
      connection = in;
    }
    void setLength(frCoord in1, frCoord in2) {
      length = in1;
      distance = in2;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcMinimumcutConstraint;
    }
  protected:
    int                        numCuts;
    frCoord                    width;
    frCoord                    cutDistance;
    frMinimumcutConnectionEnum connection;
    frCoord                    length;
    frCoord                    distance;
  };

  // minArea

  class frAreaConstraint : public frConstraint {
  public:
    // constructor
    frAreaConstraint(frCoord minAreaIn) {
      minArea = minAreaIn;
    }
    // getter
    frCoord getMinArea() {
      return minArea;
    }
    // setter
    void setMinArea(frCoord &minAreaIn) {
      minArea = minAreaIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcAreaConstraint;
    }
  protected:
    frCoord minArea;
  };

  // minWidth
  class frMinWidthConstraint: public frConstraint {
  public:
    // constructor
    frMinWidthConstraint(frCoord minWidthIn) {
      minWidth = minWidthIn;
    }
    // getter
    frCoord getMinWidth() {
      return minWidth;
    }
    // setter
    void set(frCoord &minWidthIn) {
      minWidth = minWidthIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcMinWidthConstraint;
    }
  protected:
    frCoord minWidth;
  };

  class frLef58SpacingEndOfLineWithinEndToEndConstraint : public frConstraint {
  public:
    // constructors
    frLef58SpacingEndOfLineWithinEndToEndConstraint(): endToEndSpace(0), cutSpace(false), oneCutSpace(0),
                                                       twoCutSpace(0), hExtension(false), extension(0),
                                                       wrongDirExtension(false), hOtherEndWidth(false),
                                                       otherEndWidth(0) {}
    // getters
    frCoord getEndToEndSpace() const {
      return endToEndSpace;
    }
    frCoord getOneCutSpace() const {
      return oneCutSpace;
    }
    frCoord getTwoCutSpace() const {
      return twoCutSpace;
    }
    bool hasExtension() const {
      return hExtension;
    }
    frCoord getExtension() const {
      return extension;
    }
    frCoord getWrongDirExtension() const {
      return wrongDirExtension;
    }
    bool hasOtherEndWidth() const {
      return hOtherEndWidth;
    }
    frCoord getOtherEndWidth() const {
      return otherEndWidth;
    }
    
    // setters
    void setEndToEndSpace(frCoord in) {
      endToEndSpace = in;
    }
    void setCutSpace(frCoord one, frCoord two) {
      oneCutSpace = one;
      twoCutSpace = two;
    }
    void setExtension(frCoord extensionIn) {
      hExtension = true;
      extension = extensionIn;
      wrongDirExtension = extensionIn;
    }
    void setExtension(frCoord extensionIn, frCoord wrongDirExtensionIn) {
      hExtension = true;
      extension = extensionIn;
      wrongDirExtension = wrongDirExtensionIn;
    }
    void setOtherEndWidth(frCoord in) {
      hOtherEndWidth = true;
      otherEndWidth = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingEndOfLineWithinEndToEndConstraint;
    }
  protected:
    frCoord    endToEndSpace;
    bool       cutSpace;
    frCoord    oneCutSpace;
    frCoord    twoCutSpace;
    bool       hExtension;
    frCoord    extension;
    frCoord    wrongDirExtension;
    bool       hOtherEndWidth;
    frCoord    otherEndWidth;
  };

  class frLef58SpacingEndOfLineWithinParallelEdgeConstraint : public frConstraint {
  public:
    // constructors
    frLef58SpacingEndOfLineWithinParallelEdgeConstraint(): subtractEolWidth(false), parSpace(0), parWithin(0),
                                                           hPrl(false), prl(0), hMinLength(false), minLength(0),
                                                           twoEdges(false), sameMetal(false), nonEolCornerOnly(false),
                                                           parallelSameMask(false) {}
    // getters
    bool hasSubtractEolWidth() const {
      return subtractEolWidth;
    }
    frCoord getParSpace() const {
      return parSpace;
    }
    frCoord getParWithin() const {
      return parWithin;
    }
    bool hasPrl() const {
      return hPrl;
    }
    frCoord getPrl() const {
      return prl;
    }
    bool hasMinLength() const {
      return hMinLength;
    }
    frCoord getMinLength() const {
      return minLength;
    }
    bool hasTwoEdges() const {
      return twoEdges;
    }
    bool hasSameMetal() const {
      return sameMetal;
    }
    bool hasNonEolCornerOnly() const {
      return nonEolCornerOnly;
    }
    bool hasParallelSameMask() const {
      return parallelSameMask;
    }
    // setters
    void setSubtractEolWidth(bool in) {
      subtractEolWidth = in;
    }
    void setPar(frCoord parSpaceIn, frCoord parWithinIn) {
      parSpace = parSpaceIn;
      parWithin = parWithinIn;
    }
    void setPrl(frCoord in) {
      hPrl = true;
      prl = in;
    }
    void setMinLength(frCoord in) {
      hMinLength = true;
      minLength = in;
    }
    void setTwoEdges(bool in) {
      twoEdges = in;
    }
    void setSameMetal(bool in) {
      sameMetal = in;
    }
    void setNonEolCornerOnly(bool in) {
      nonEolCornerOnly = in;
    }
    void setParallelSameMask(bool in) {
      parallelSameMask = in;
    }
    
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingEndOfLineWithinParallelEdgeConstraint;
    }
  protected:
    bool    subtractEolWidth;
    frCoord parSpace;
    frCoord parWithin;
    bool    hPrl;
    frCoord prl;
    bool    hMinLength;
    frCoord minLength;
    bool    twoEdges;
    bool    sameMetal;
    bool    nonEolCornerOnly;
    bool    parallelSameMask;
  };

  class frLef58SpacingEndOfLineWithinMaxMinLengthConstraint : public frConstraint {
  public:
    // constructors
    frLef58SpacingEndOfLineWithinMaxMinLengthConstraint(): maxLength(false), length(0), twoSides(false) {}
    // getters
    frCoord getLength() const {
      return length;
    }
    bool isMaxLength() const {
      return maxLength;
    }
    // setters
    void setLength(bool maxLengthIn, frCoord lengthIn, bool twoSidesIn = false) {
      maxLength = maxLengthIn;
      length    = lengthIn;
      twoSides  = twoSidesIn;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingEndOfLineWithinMaxMinLengthConstraint;
    }
  protected:
    bool    maxLength;
    frCoord length;
    bool    twoSides;
  };


  class frLef58SpacingEndOfLineWithinConstraint : public frConstraint {
  public:
    // constructors
    frLef58SpacingEndOfLineWithinConstraint(): hOppositeWidth(false), oppositeWidth(0), eolWithin(0),
                                               wrongDirWithin(false), sameMask(false), 
                                               endToEndConstraint(nullptr), parallelEdgeConstraint(nullptr) {}
    // getters
    bool hasOppositeWidth() const {
      return hOppositeWidth;
    }
    frCoord getOppositeWidth() const {
      return oppositeWidth;
    }
    frCoord getEolWithin() const {
      return eolWithin;
    }
    frCoord getWrongDirWithin() const {
      return wrongDirWithin;
    }
    bool hasSameMask() const {
      return sameMask;
    }
    bool hasExceptExactWidth() const {
      return false; // skip for now
    }
    bool hasFillConcaveCorner() const {
      return false; // skip for now
    }
    bool hasWithCut() const {
      return false; // skip for now
    }
    bool hasEndPrlSpacing() const {
      return false; // skip for now
    }
    bool hasEndToEndConstraint() const {
      return (endToEndConstraint) ? true : false;
    }
    std::shared_ptr<frLef58SpacingEndOfLineWithinEndToEndConstraint> getEndToEndConstraint() const {
      return endToEndConstraint;
    }
    bool hasMinMaxLength() const {
      return false; // skip for now
    }
    bool hasEqualRectWidth() const {
      return false; // skip for now
    }
    bool hasParallelEdgeConstraint() const {
      return (parallelEdgeConstraint) ? true : false;
    }
    std::shared_ptr<frLef58SpacingEndOfLineWithinParallelEdgeConstraint> getParallelEdgeConstraint() const {
      return parallelEdgeConstraint;
    }
    bool hasMaxMinLengthConstraint() const {
      return (maxMinLengthConstraint) ? true : false;
    }
    std::shared_ptr<frLef58SpacingEndOfLineWithinMaxMinLengthConstraint> getMaxMinLengthConstraint() const {
      return maxMinLengthConstraint;
    }
    bool hasEncloseCut() const {
      return false; // skip for now
    }
    // setters
    void setOppositeWidth(frCoord in) {
      hOppositeWidth = true;
      oppositeWidth = in;
    }
    void setEolWithin(frCoord in) {
      eolWithin = in;
      wrongDirWithin = in;
    }
    void setWrongDirWithin(frCoord in) {
      wrongDirWithin = in;
    }
    void setSameMask(bool in) {
      sameMask = in;
    }
    void setEndToEndConstraint(const std::shared_ptr<frLef58SpacingEndOfLineWithinEndToEndConstraint> &in) {
      endToEndConstraint = in;
    }
    void setParallelEdgeConstraint(const std::shared_ptr<frLef58SpacingEndOfLineWithinParallelEdgeConstraint> &in) {
      parallelEdgeConstraint = in;
    }
    void setMaxMinLengthConstraint(const std::shared_ptr<frLef58SpacingEndOfLineWithinMaxMinLengthConstraint> &in) {
      maxMinLengthConstraint = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingEndOfLineWithinConstraint;
    }
  protected:
    bool                                                                 hOppositeWidth;
    frCoord                                                              oppositeWidth;
    frCoord                                                              eolWithin;
    frCoord                                                              wrongDirWithin;
    bool                                                                 sameMask;
    std::shared_ptr<frLef58SpacingEndOfLineWithinEndToEndConstraint>     endToEndConstraint;
    std::shared_ptr<frLef58SpacingEndOfLineWithinParallelEdgeConstraint> parallelEdgeConstraint;
    std::shared_ptr<frLef58SpacingEndOfLineWithinMaxMinLengthConstraint> maxMinLengthConstraint;
  };

  class frLef58SpacingEndOfLineConstraint : public frConstraint {
  public:
    // constructors
    frLef58SpacingEndOfLineConstraint(): eolSpace(0), eolWidth(0), exactWidth(false),
                                         wrongDirSpacing(false), wrongDirSpace(0),
                                         withinConstraint(nullptr) {}
    // getters
    frCoord getEolSpace() const {
      return eolSpace;
    }
    frCoord getEolWidth() const {
      return eolWidth;
    }
    bool hasExactWidth() const {
      return exactWidth;
    }
    bool hasWrongDirSpacing() const {
      return wrongDirSpacing;
    }
    frCoord getWrongDirSpace() const {
      return wrongDirSpace;
    }
    bool hasWithinConstraint() const {
      return (withinConstraint) ? true : false;
    }
    std::shared_ptr<frLef58SpacingEndOfLineWithinConstraint> getWithinConstraint() const {
      return withinConstraint;
    }
    bool hasToConcaveCornerConstraint() const {
      return false;
    }
    bool hasToNotchLengthConstraint() const {
      return false;
    }
    // setters
    void setEol(frCoord eolSpaceIn, frCoord eolWidthIn, bool exactWidthIn = false) {
      eolSpace = eolSpaceIn;
      eolWidth = eolWidthIn;
      exactWidth = exactWidthIn;
    }
    void setWrongDirSpace(bool in) {
      wrongDirSpacing = true;
      wrongDirSpace = in;
    }
    void setWithinConstraint(const std::shared_ptr<frLef58SpacingEndOfLineWithinConstraint> &in) {
      withinConstraint = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingEndOfLineConstraint;
    }
  protected:
    frCoord                                                  eolSpace;
    frCoord                                                  eolWidth;
    bool                                                     exactWidth;
    bool                                                     wrongDirSpacing;
    frCoord                                                  wrongDirSpace;
    std::shared_ptr<frLef58SpacingEndOfLineWithinConstraint> withinConstraint;
  };

  class frLef58CornerSpacingSpacingConstraint;

  // SPACING Constraints
  class frSpacingConstraint : public frConstraint {
  public:
    frSpacingConstraint(): minSpacing(0) {}
    frSpacingConstraint(frCoord &minSpacingIn): minSpacing(minSpacingIn) {}

    // getter
    frCoord getMinSpacing() const {
      return minSpacing;
    }
    // setter
    void setMinSpacing(frCoord &minSpacingIn) {
      minSpacing = minSpacingIn;
    }
    // check
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcSpacingConstraint;
    }
  protected:
    frCoord minSpacing;
  };

  // PARALLELEDGE
  //class frSpacingEndOfLineParallelEdgeConstraint : public frConstraint {
  //public:
  //  // constructor
  //  frSpacingEndOfLineParallelEdgeConstraint(): parSpace(0), parWithin(0), isTwoEdges(false) {}
  //  frSpacingEndOfLineParallelEdgeConstraint(frCoord parSpaceIn, frCoord parWithinIn, bool isTwoEdgesIn)
  //    : parSpace(parSpaceIn), parWithin(parWithinIn), isTwoEdges(isTwoEdgesIn) {}
  //  // getters
  //  frCoord getParSpace() const {
  //    return parSpace;
  //  }
  //  frCoord getParWithin() const {
  //    return parWithin;
  //  }
  //  bool hasTwoEdges() const {
  //    return isTwoEdges;
  //  }
  //  // setters
  //  void setParSpace(frCoord parSpaceIn) {
  //    parSpace = parSpaceIn;
  //  }
  //  void setParWithin(frCoord parWithinIn) {
  //    parWithin = parWithinIn;
  //  }
  //  void setTwoEdges(bool isTwoEdgesIn) {
  //    isTwoEdges = isTwoEdgesIn;
  //  }
  //  frConstraintTypeEnum typeId() const override {
  //    return frConstraintTypeEnum::frcSpacingEndOfLineParallelEdgeConstraint;
  //  }

  //protected:
  //  frCoord parSpace, parWithin;
  //  bool isTwoEdges;
  //};

  //class frLef58SpacingEndOfLineParallelEdgeConstraint : public frSpacingEndOfLineParallelEdgeConstraint {
  //public:
  //  // constructors
  //  frLef58SpacingEndOfLineParallelEdgeConstraint(): frSpacingEndOfLineParallelEdgeConstraint(), subtractEolWidth(false) {}
  //  // getters
  //  bool hasSubtractEolWidth() const {
  //    return subtractEolWidth;
  //  }
  //  // setters
  //  void setSubtractEolWidth(bool in) {
  //    subtractEolWidth = in;
  //  }
  //  // others
  //  frConstraintTypeEnum typeId() const override {
  //    return frConstraintTypeEnum::frcLef58SpacingEndOfLineParallelEdgeConstraint;
  //  }
  //protected:
  //  bool subtractEolWidth;
  //};

  // EOL spacing
  class frSpacingEndOfLineConstraint : public frSpacingConstraint {
  public:
    // constructor
    frSpacingEndOfLineConstraint(): frSpacingConstraint(), 
                                    eolWidth(-1), eolWithin(-1), 
                                    parSpace(-1), parWithin(-1),
                                    isTwoEdges(false) {}
    // getter
    frCoord getEolWidth() const {
      return eolWidth;
    }
    frCoord getEolWithin() const {
      return eolWithin;
    }
    frCoord getParSpace() const {
      return parSpace;
    }
    frCoord getParWithin() const {
      return parWithin;
    }
    bool hasParallelEdge() const {
      return ((parSpace == -1) ? false : true);
    }
    bool hasTwoEdges() const {
      return isTwoEdges;
    }
    //bool hasParallelEdge() const {
    //  return (parallelEdgeConstraint) ? true : false;
    //}
    //std::shared_ptr<frSpacingEndOfLineParallelEdgeConstraint> getParallelEdge() const {
    //  return parallelEdgeConstraint;
    //}
    // setter
    void setEolWithin(frCoord &eolWithinIn) {
      eolWithin = eolWithinIn;
    }
    void setEolWidth(frCoord &eolWidthIn) {
      eolWidth = eolWidthIn;
    }
    void setParSpace(frCoord parSpaceIn) {
      parSpace = parSpaceIn;
    }
    void setParWithin(frCoord parWithinIn) {
      parWithin = parWithinIn;
    }
    void setTwoEdges(bool isTwoEdgesIn) {
      isTwoEdges = isTwoEdgesIn;
    }
    //void setParallelEdge(const std::shared_ptr<frSpacingEndOfLineParallelEdgeConstraint> &parallelEdgeConstraintIn) {
    //  parallelEdgeConstraint = parallelEdgeConstraintIn;
    //}
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcSpacingEndOfLineConstraint;
    }

  protected:
    frCoord eolWidth, eolWithin;
    frCoord parSpace, parWithin;
    bool isTwoEdges;
    //std::shared_ptr<frSpacingEndOfLineParallelEdgeConstraint> parallelEdgeConstraint;
  };

  //class frLef58SpacingEndOfLineEndToEndConstraint: public frConstraint {
  //public:
  //  ;
  //protected:
  //  ;
  //};

  //class frLef58SpacingEndOfLineConstraint : public frSpacingEndOfLineConstraint {
  //public:
  //  // constructors
  //  frLef58SpacingEndOfLineConstraint() : frSpacingEndOfLineConstraint(), sameMask(false), exactWidth(false),
  //    wrongDirSpacing(false), wrongDirSpace(0), endToEnd(false), endToEndSpace(0), maxLength(false), 
  //    minLength(false), length(0), twoSides(false) {}
  //  // getters
  //  std::shared_ptr<frLef58SpacingEndOfLineParallelEdgeConstraint> getParallelEdge() const {
  //    return std::dynamic_pointer_cast<frLef58SpacingEndOfLineParallelEdgeConstraint>(parallelEdgeConstraint);
  //  }
  //  bool hasSameMask() const {
  //    return sameMask;
  //  }
  //  bool hasExactWidth() const {
  //    return exactWidth;
  //  }
  //  bool hasWrongDirSpacing() const {
  //    return wrongDirSpacing;
  //  }
  //  frCoord getWrongDirSpace() const {
  //    return wrongDirSpace;
  //  }
  //  bool hasEndToEnd() const {
  //    return endToEnd;
  //  }
  //  frCoord getEndToEndSpace() const {
  //    return endToEndSpace;
  //  }
  //  bool hasMaxLength() const {
  //    return maxLength;
  //  }
  //  bool hasMinLength() const {
  //    return minLength;
  //  }
  //  frCoord getLength() const {
  //    return length;
  //  }
  //  bool hasTwoSides() const {
  //    return twoSides;
  //  }
  //  // setters
  //  void setSameMask(bool in) {
  //    sameMask = in;
  //  }
  //  void setExactWidth(bool in) {
  //    exactWidth = in;
  //  }
  //  void setWrongDirSpace(frCoord in) {
  //    wrongDirSpacing = true;
  //    wrongDirSpace   = in;
  //  }
  //  void setEndToEndSpace(frCoord in) {
  //    endToEnd      = true;
  //    endToEndSpace = in;
  //  }
  //  void setMaxLength(frCoord in) {
  //    maxLength = true;
  //    length = in;
  //  }
  //  void setMinLength(frCoord in) {
  //    minLength = true;
  //    length = in;
  //  }
  //  void setTwoSides(bool in) {
  //    twoSides = in;
  //  }
  //  // others
  //  frConstraintTypeEnum typeId() const override {
  //    return frConstraintTypeEnum::frcLef58SpacingEndOfLineConstraint;
  //  }
  //protected:
  //  bool sameMask;
  //  bool exactWidth;
  //  bool wrongDirSpacing;
  //  frCoord wrongDirSpace;
  //  bool endToEnd;
  //  frCoord endToEndSpace;
  //  bool maxLength;
  //  bool minLength;
  //  frCoord length;
  //  bool twoSides;
  //};


  class frLef58CutSpacingLayerConstraint : public frConstraint {
  public:
    // constructor
    frLef58CutSpacingLayerConstraint() : secondLayerName(""), className(""), 
                                         hAboveWidth(false), aboveWidth(0),
                                         hEnclosure(false), enclosure(0) {}
    // getter
    //frUInt4 getSecondLayerNum() const {
    //  return secondLayerNum;
    //}
    frString getSecondLayerName() const {
      return secondLayerName;
    }
    // cutclass
    bool hasCutClass() const {
      return (className == "");
    }
    frString getClassName() const {
      return className;
    }
    void getClassName(frString &in) const {
      in = className;
    }
    bool hasAboveWidth() const {
      return hAboveWidth;
    }
    frCoord getAboveWidth() const {
      return aboveWidth;
    }
    bool hasEnclosure() const {
      return hEnclosure;
    }
    frCoord getEnclosure() const {
      return enclosure;
    }

    // setter
    //void setSecondLayerNum(frLayerNum in) {
    //  secondLayerNum = in;
    //}
    void setSecondLayerName(const frString &in) {
      secondLayerName = in;
    }
    void setCutClass(const frString &classNameIn) {
      className = classNameIn;
    }
    void setAboveWidth(frCoord in) {
      hAboveWidth = true;
      aboveWidth  = in;
    }
    void setEnclosure(frCoord in) {
      hEnclosure = true;
      enclosure  = in;
    }

    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingLayerConstraint;
    }
  protected:
    //frUInt4  secondLayerNum;
    frString secondLayerName;
    // cutclass
    frString className;
    bool     hAboveWidth;
    frCoord  aboveWidth;
    bool     hEnclosure;
    frCoord  enclosure;
  };

  class frLef58CutSpacingAdjacentCutsConstraint : public frConstraint {
  public:
    // constructor
    frLef58CutSpacingAdjacentCutsConstraint() : numAdjCuts(0), hTwoCuts(false), 
                                                twoCuts(0), hTwoCutsSpacing(false), 
                                                twoCutsSpacing(0), sameCut(false),
                                                cutWithin1(0), cutWithin2(0),
                                                className(""), toAll(false) {}
    // getter
    frUInt4 getNumAdjCuts() const {
      return numAdjCuts;
    }
    // two cuts
    bool hasTwoCuts() const {
      return hTwoCuts;
    }
    frUInt4 getTwoCuts() const {
      return twoCuts;
    }
    bool hasTwoCutsSpacing() const {
      return hTwoCutsSpacing;
    }
    frCoord getTwoCutsSpacing() const {
      return twoCutsSpacing;
    }
    bool isSameCut() const {
      return sameCut;
    }
    // within
    frCoord getCutWithin1() const {
      return cutWithin1;
    }
    frCoord getCutWithin2() const {
      return cutWithin2;
    }
    // cutclass
    bool hasCutClass() const {
      return (className == "");
    }
    frString getClassName() const {
      return className;
    }
    void getClassName(frString &in) const {
      in = className;
    }
    bool isToAll() const {
      return toAll;
    }

    // setter
    void setNumAdjCuts(frUInt4 in) {
      numAdjCuts = in;
    }
    void setTwoCuts(frUInt4 twoCutsIn, bool sameCutIn = false) {
      hTwoCuts        = true;
      twoCuts         = twoCutsIn;
      sameCut         = sameCutIn;
    }
    void setTwoCuts(frUInt4 twoCutsIn, frCoord twoCutsSpacingIn, bool sameCutIn = false) {
      hTwoCuts        = true;
      twoCuts         = twoCutsIn;
      hTwoCutsSpacing = true;
      twoCutsSpacing  = twoCutsSpacingIn;
      sameCut         = sameCutIn;
    }
    void setWithin(frCoord cutWithin1In, frCoord cutWithin2In) {
      cutWithin1 = cutWithin1In;
      cutWithin2 = cutWithin2In;
    }
    void setCutClass(const frString &classNameIn, bool toAllIn) {
      className = classNameIn;
      toAll     = toAllIn;
    }

    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingAdjacentCutsConstraint;
    }
  protected:
    frUInt4  numAdjCuts;
    // two cuts
    bool     hTwoCuts;
    frUInt4  twoCuts;
    bool     hTwoCutsSpacing;
    frCoord  twoCutsSpacing;
    bool     sameCut;
    // within
    frCoord  cutWithin1;
    frCoord  cutWithin2;
    // cutclass
    frString className;
    bool     toAll;
  };

  class frLef58CutSpacingParallelWithinConstraint : public frConstraint {
  public:
    // constructor
    frLef58CutSpacingParallelWithinConstraint() : within(0), exceptSameNet(false), className(""), longEdgeOnly(false),
                                                  enclosure(0), above(false), parLength(0), parWithin(0) {}
    // getter
    frCoord getWithin() const {
      return within;
    }
    bool isExceptSameNet() const {
      return exceptSameNet;
    }
    bool hasCutClass() const {
      return (className == "");
    }
    frString getClassName() const {
      return className;
    }
    void getClassName(frString &in) const {
      in = className;
    }
    bool isLongEdgeOnly() const {
      return longEdgeOnly;
    }
    bool isEnclosure() const {
      return (longEdgeOnly) ? false : true;
    }
    frCoord getEnclosure() const {
      return enclosure;
    }
    bool isAbove() const {
      return above;
    }
    frCoord getParLength() const {
      return parLength;
    }
    frCoord getParWithin() const {
      return parWithin;
    }

    // setter
    void setParallelWithin(frCoord withinIn, bool exceptSameNetIn) {
      within        = withinIn;
      exceptSameNet = exceptSameNetIn;
    }
    void setClassName(const frString &in) {
      className = in;
    }
    void setLongEdgeOnly(bool in) {
      longEdgeOnly = in;
    }
    void setEnclosure(frCoord enclosureIn, bool aboveIn) {
      enclosure = enclosureIn;
      above = aboveIn;
    }
    void setParallel(frCoord parLengthIn, frCoord parWithinIn) {
      parLength = parLengthIn;
      parWithin = parWithinIn;
    }

    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingParallelWithinConstraint;
    }
  protected:
    frCoord  within;
    bool     exceptSameNet;
    frString className;
    // longedgeonly
    bool     longEdgeOnly;
    // enclosure
    frCoord  enclosure;
    bool     above;
    frCoord  parLength;
    frCoord  parWithin;
  };

  // LEF58 cut spacing
  class frLef58CutSpacingConstraint : public frConstraint {
  public:
    // constructor
    frLef58CutSpacingConstraint() {}
    // getter
    frCoord getCutSpacing() const {
      return cutSpacing;
    }
    bool hasParallelWithinConstraint() const {
      return (parallelWithinConstraint) ? true : false;
    }
    std::shared_ptr<frLef58CutSpacingParallelWithinConstraint> getParallelWithinConstraint() const {
      return parallelWithinConstraint;
    }
    bool hasAdjacentCutsConstraint() const {
      return (adjacentCutsConstraint) ? true : false;
    }
    std::shared_ptr<frLef58CutSpacingAdjacentCutsConstraint> getAdjacentCutsConstraint() const {
      return adjacentCutsConstraint;
    }
    bool hasLayerConstraint() const {
      return (layerConstraint) ? true : false;
    }
    std::shared_ptr<frLef58CutSpacingLayerConstraint> getLayerConstraint() const {
      return layerConstraint;
    }
    // setter
    void setCutSpacing(frCoord in) {
      cutSpacing = in;
    }
    void setParallelWithinConstraint(const std::shared_ptr<frLef58CutSpacingParallelWithinConstraint> &in) {
      parallelWithinConstraint = in;
    }
    void setAdjacentCutsConstraint(const std::shared_ptr<frLef58CutSpacingAdjacentCutsConstraint> &in) {
      adjacentCutsConstraint = in;
    }
    void setLayerConstraint(const std::shared_ptr<frLef58CutSpacingLayerConstraint> &in) {
      layerConstraint = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingConstraint;
    }
  protected:
    frCoord                                                    cutSpacing;
    std::shared_ptr<frLef58CutSpacingParallelWithinConstraint> parallelWithinConstraint;
    std::shared_ptr<frLef58CutSpacingAdjacentCutsConstraint>   adjacentCutsConstraint;
    std::shared_ptr<frLef58CutSpacingLayerConstraint>          layerConstraint;
  };





  class frLef58CutSpacingTableLayerConstraint : public frConstraint {
  public:
    // constructors
    frLef58CutSpacingTableLayerConstraint(): secondLayerNum(0), nonZeroEnc(false) {}
    // getters
    frLayerNum getSecondLayerNum() const {
      return secondLayerNum;
    }
    bool isNonZeroEnc() const {
      return nonZeroEnc;
    }
    // setters
    void setSecondLayerNum(frLayerNum in) {
      secondLayerNum = in;
    }
    void setNonZeroEnc(bool in) {
      nonZeroEnc = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingTableLayerConstraint;
    }
  protected:
    frLayerNum secondLayerNum;
    bool       nonZeroEnc;
  };

  class frLef58CutSpacingTablePrlConstraint : public frConstraint {
  public:
    // constructors
    frLef58CutSpacingTablePrlConstraint(): prl(0), horizontal(false), vertical(false), 
                                           maxXY(false) {}
    // getters
    frCoord getPrl() const {
      return prl;
    }
    bool isHorizontal() const {
      return horizontal;
    }
    bool isVertical() const {
      return vertical;
    }
    bool isMaxXY() const {
      return maxXY;
    }
    // setters
    void setPrl(frCoord in) {
      prl = in;
    }
    void setHorizontal(bool in) {
      horizontal = in;
    }
    void setVertical(bool in) {
      vertical = in;
    }
    void setMaxXY(bool in) {
      maxXY = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingTablePrlConstraint;
    }
  protected:
    frCoord prl;
    bool    horizontal;
    bool    vertical;
    bool    maxXY;
  };

  // LEF58 cut spacing table
  class frLef58CutSpacingTableConstraint : public frConstraint {
  public:
    // constructor
    frLef58CutSpacingTableConstraint(): cutClassHasAll(false), defaultCutSpacing(0) {}
    // getter
    std::shared_ptr<fr2DLookupTbl<frString, frString, std::pair<frCoord, frCoord> > > getCutClassTbl() const {
      return cutClassTbl;
    }
    bool hasPrlConstraint() const {
      return (prlConstraint) ? true : false;
    }
    std::shared_ptr<frLef58CutSpacingTablePrlConstraint> getPrlConstraint() const {
      return prlConstraint;
    }
    bool hasLayerConstraint() const {
      return (layerConstraint) ? true : false;
    }
    std::shared_ptr<frLef58CutSpacingTableLayerConstraint> getLayerConstraint() const {
      return layerConstraint;
    }
    bool hasAll() const {
      return cutClassHasAll;
    }
    frCoord getDefaultCutSpacing() const {
      return defaultCutSpacing;
    }
    // setter
    void setCutClassTbl(std::shared_ptr<fr2DLookupTbl<frString, frString, std::pair<frCoord, frCoord> > > &in) {
      cutClassTbl = in;
    }
    void setPrlConstraint(const std::shared_ptr<frLef58CutSpacingTablePrlConstraint> &in) {
      prlConstraint = in;
    }
    void setLayerConstraint(const std::shared_ptr<frLef58CutSpacingTableLayerConstraint> &in) {
      layerConstraint = in;
    }
    void setAll(bool in) {
      cutClassHasAll = in;
    }
    void setDefaultCutSpacing(frCoord in) {
      defaultCutSpacing = in;
    }
    // others
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CutSpacingTableConstraint;
    }
  protected:
    std::shared_ptr<fr2DLookupTbl<frString, frString, std::pair<frCoord, frCoord> > > cutClassTbl;
    std::shared_ptr<frLef58CutSpacingTablePrlConstraint>                              prlConstraint;
    std::shared_ptr<frLef58CutSpacingTableLayerConstraint>                            layerConstraint;
    bool                                                                              cutClassHasAll;
    frCoord                                                                           defaultCutSpacing;
  };

  // new SPACINGTABLE Constraints
  class frSpacingTablePrlConstraint : public frConstraint {
  public:
    // constructor
    //frSpacingTablePrlConstraint() {}
    frSpacingTablePrlConstraint(const fr2DLookupTbl<frCoord, frCoord, frCoord> &in): tbl(in) {}
    // getter
    const fr2DLookupTbl<frCoord, frCoord, frCoord>& getLookupTbl() const {
      return tbl;
    }
    fr2DLookupTbl<frCoord, frCoord, frCoord>& getLookupTbl() {
      return tbl;
    }
    frCoord find(frCoord width, frCoord prl) const {
      return tbl.find(width, prl);
    }
    frCoord findMin() const {
      return tbl.findMin();
    }
    // setter
    void setLookupTbl(const fr2DLookupTbl<frCoord, frCoord, frCoord> &in) {
      tbl = in;
    }
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcSpacingTablePrlConstraint;
    }
  protected:
    fr2DLookupTbl<frCoord, frCoord, frCoord> tbl;
  };

  struct frSpacingTableTwRowType {
    frSpacingTableTwRowType(frCoord in1, frCoord in2): width(in1), prl(in2) {}
    frCoord width;
    frCoord prl;
    bool operator<(const frSpacingTableTwRowType &b) const {
      return width < b.width || prl < b.prl;
    }
    //bool operator<=(const frSpacingTableTwRowType &b) const {
    //  return width <= b.width || prl <= b.prl;
    //}
    //bool operator>(const frSpacingTableTwRowType &b) const {
    //  return width > b.width || prl > b.prl;
    //}
    //bool operator>=(const frSpacingTableTwRowType &b) const {
    //  return width >= b.width || prl >= b.prl;
    //}
    //bool operator==(const frSpacingTableTwRowType &b) const {
    //  return width == b.width && prl == b.prl;
    //}
    //bool operator!=(const frSpacingTableTwRowType &b) const {
    //  return !(*this == b);
    //}
  };
  // new SPACINGTABLE Constraints
  class frSpacingTableTwConstraint : public frConstraint {
  public:
    // constructor
    //frSpacingTableTwConstraint() {}
    frSpacingTableTwConstraint(const fr2DLookupTbl<frSpacingTableTwRowType, frSpacingTableTwRowType, frCoord> &in): tbl(in) {}
    // getter
    const fr2DLookupTbl<frSpacingTableTwRowType, frSpacingTableTwRowType, frCoord>& getLookupTbl() const {
      return tbl;
    }
    fr2DLookupTbl<frSpacingTableTwRowType, frSpacingTableTwRowType, frCoord>& getLookupTbl() {
      return tbl;
    }
    frCoord find(frCoord width1, frCoord width2, frCoord prl) const {
      return tbl.find(frSpacingTableTwRowType(width1, prl), frSpacingTableTwRowType(width2, prl));
    }
    frCoord findMin() const {
      return tbl.findMin();
    }
    // setter
    void setLookupTbl(const fr2DLookupTbl<frSpacingTableTwRowType, frSpacingTableTwRowType, frCoord> &in) {
      tbl = in;
    }
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcSpacingTableTwConstraint;
    }
  protected:
    fr2DLookupTbl<frSpacingTableTwRowType, frSpacingTableTwRowType, frCoord> tbl;
  };
  
  // original SPACINGTABLE Constraints
  class frSpacingTableConstraint : public frConstraint {
  public:
    // constructor
    frSpacingTableConstraint(std::shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > parallelRunLengthConstraintIn) {
      parallelRunLengthConstraint = parallelRunLengthConstraintIn;
    }
    // getter
    std::shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > getParallelRunLengthConstraint() {
      return parallelRunLengthConstraint;
    }
    // setter
    void setParallelRunLengthConstraint(std::shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > &parallelRunLengthConstraintIn) {
      parallelRunLengthConstraint = parallelRunLengthConstraintIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcSpacingTableConstraint;
    }
  protected:
    std::shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > parallelRunLengthConstraint;
  };
  
  class frLef58SpacingTableConstraint : public frSpacingTableConstraint {
  public:
    // constructor
    frLef58SpacingTableConstraint(const std::shared_ptr<fr2DLookupTbl<frCoord, frCoord, frCoord> > &parallelRunLengthConstraintIn,
                                  const std::map<int, std::pair<frCoord, frCoord> > &exceptWithinConstraintIn): 
      frSpacingTableConstraint(parallelRunLengthConstraintIn), exceptWithinConstraint(exceptWithinConstraintIn), 
      wrongDirection(false), sameMask(false), exceptEol(false), eolWidth(0) {}
    // getter
    bool hasExceptWithin(frCoord val) const {
      auto rowIdx = parallelRunLengthConstraint->getRowIdx(val);
      return (exceptWithinConstraint.find(rowIdx) != exceptWithinConstraint.end());
    }
    std::pair<frCoord, frCoord> getExceptWithin(frCoord val) const {
      auto rowIdx = parallelRunLengthConstraint->getRowIdx(val);
      return exceptWithinConstraint.at(rowIdx);
    }
    bool isWrongDirection() const {
      return wrongDirection;
    }
    bool isSameMask() const {
      return sameMask;
    }
    bool hasExceptEol() const {
      return exceptEol;
    }
    frUInt4 getEolWidth() const {
      return eolWidth;
    }
    // setters
    void setExceptWithinConstraint(std::map<int, std::pair<frCoord, frCoord> > &exceptWithinConstraintIn) {
      exceptWithinConstraint = exceptWithinConstraintIn;
    }
    void setWrongDirection(bool in) {
      wrongDirection = in;
    }
    void setSameMask(bool in) {
      sameMask = in;
    }
    void setEolWidth(frUInt4 in) {
      exceptEol = true;
      eolWidth = in;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58SpacingTableConstraint;
    }
  protected:
    //std::shared_ptr<fr1DLookupTbl<frCoord, std::pair<frCoord, frCoord> > > exceptWithinConstraint;
    std::map<frCoord, std::pair<frCoord, frCoord> > exceptWithinConstraint;
    bool wrongDirection;
    bool sameMask;
    bool exceptEol;
    frUInt4 eolWidth;
  };

  // class frSpacingTableTwoWidthConstraint : public frSpacingTableConstraint {
  // public:
  //   // getter
  //   // setter
  //   // check
  //   virtual bool check() {

  //   }
  // protected:
  //   frCollection<frCoord> widths;
  //   frCollection<frCoord> lengths;
  //   frCollection<frCollection<frCoord> > spTbl;
  // };

  // ADJACENTCUTS
  class frCutSpacingConstraint : public frConstraint {
  public:
  // constructor
  frCutSpacingConstraint() {}
  frCutSpacingConstraint(
    frCoord &cutSpacingIn,
    bool centerToCenterIn,
    bool sameNetIn,
    frString secondLayerNameIn,
    bool stackIn,
    int adjacentCutsIn,
    frCoord cutWithinIn,
    bool isExceptSamePGNetIn,
    bool isParallelOverlapIn,
    frCoord cutAreaIn
  ) {
    cutSpacing = cutSpacingIn;
    centerToCenter = centerToCenterIn;
    sameNet = sameNetIn;
    secondLayerName = secondLayerNameIn;
    stack = stackIn;
    adjacentCuts = adjacentCutsIn;
    cutWithin = cutWithinIn;
    exceptSamePGNet = isExceptSamePGNetIn;
    parallelOverlap = isParallelOverlapIn;
    cutArea = cutAreaIn;
  }
  // getter
  bool hasCenterToCenter() const {
    return centerToCenter;
  }
  bool getCenterToCenter() const {
    return centerToCenter;
  }
  bool getSameNet() const {
    return sameNet;
  }
  bool hasSameNet() const {
    return sameNet;
  }
  bool getStack() const {
    return stack;
  }
  bool hasStack() const {
    return stack;
  }
  bool isLayer() const {
    return !(secondLayerName.empty());
  }
  const frString& getSecondLayerName() const {
    return secondLayerName;
  }
  bool isAdjacentCuts() const {
    return !(adjacentCuts == -1);
  }
  int getAdjacentCuts() const {
    return adjacentCuts;
  }
  frCoord getCutWithin() const {
    return cutWithin;
  }
  bool hasExceptSamePGNet() const {
    return exceptSamePGNet;
  }
  bool getExceptSamePGNet() const {
    return exceptSamePGNet;
  }
  bool isParallelOverlap() const {
    return parallelOverlap;
  }
  bool getParallelOverlap() const {
    return parallelOverlap;
  }
  bool isArea() const {
    return !(cutArea == -1);
  }
  bool getCutArea() const {
    return cutArea;
  }
  frCoord getCutSpacing() const {
    return cutSpacing;
  }
  frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcCutSpacingConstraint;
  }



  // setter

  protected:
    frCoord cutSpacing = -1;
    bool centerToCenter = false;
    bool sameNet = false;
    bool stack = false;
    bool exceptSamePGNet = false;
    bool parallelOverlap = false;
    frString secondLayerName;
    int adjacentCuts = -1;
    frCoord cutWithin = -1;
    frCoord cutArea = -1;

  };

  // Lef58_CORNERSPACING
  class frLef58CornerSpacingConvexCornerConstraint;
  class frLef58CornerSpacingConcaveCornerConstraint;

  class frLef58CornerSpacingConstraint : public frConstraint {
  public:
    // constructor
    frLef58CornerSpacingConstraint(
      std::shared_ptr<frLef58CornerSpacingConcaveCornerConstraint> concaveConstraintIn,
      frLef58CornerSpacingExceptionEnum &exceptionIn,
      frCollection<std::shared_ptr<frLef58CornerSpacingSpacingConstraint> > &spacingsIn
    ) {
      concaveConstraint = concaveConstraintIn;
      exception = exceptionIn;
      spacings = spacingsIn;
    }
    frLef58CornerSpacingConstraint(
      std::shared_ptr<frLef58CornerSpacingConvexCornerConstraint> convexConstraintIn,
      frLef58CornerSpacingExceptionEnum &exceptionIn,
      frCollection<std::shared_ptr<frLef58CornerSpacingSpacingConstraint> > &spacingsIn
    ) {
      convexConstraint = convexConstraintIn;
      exception = exceptionIn;
      spacings = spacingsIn;
    }

    // getter
    std::shared_ptr<frLef58CornerSpacingConcaveCornerConstraint> getConcaveConstraint() {
      return concaveConstraint;
    }
    std::shared_ptr<frLef58CornerSpacingConvexCornerConstraint> getConvexConstraint() {
      return convexConstraint;
    }
    frLef58CornerSpacingExceptionEnum getException() {
      return exception;
    }
    frCollection<std::shared_ptr<frLef58CornerSpacingSpacingConstraint> > getSpacings() {
      return spacings;
    }
    // setter
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CornerSpacingConstraint;
    }


  protected:
    std::shared_ptr<frLef58CornerSpacingConcaveCornerConstraint> concaveConstraint;
    std::shared_ptr<frLef58CornerSpacingConvexCornerConstraint> convexConstraint;
    frLef58CornerSpacingExceptionEnum exception = frLef58CornerSpacingExceptionEnum::NONE;
    frCollection<std::shared_ptr<frLef58CornerSpacingSpacingConstraint> > spacings;

  };

  class frLef58CornerSpacingConcaveCornerConstraint {
  public:
    // constructor
    frLef58CornerSpacingConcaveCornerConstraint(
      frCoord minLengthIn = -1,
      frCoord notchLengthIn = -1
    ) {
      if (minLengthIn != -1) {
        minLength = minLengthIn;
      }
      if (notchLengthIn != -1) {
        notchLength = notchLengthIn;
      }
    }
    // getter
    frCoord getMinLength() {
      return minLength;
    }
    frCoord getNotchLength() {
      return notchLength;
    }
  protected:
    frCoord minLength = -1;
    frCoord notchLength = -1;
  };

  class frLef58CornerSpacingConvexCornerConstraint : public frConstraint {
  public:
    // constructor
    frLef58CornerSpacingConvexCornerConstraint(
      bool &sameMaskIn, 
      bool &edgeLengthIn, 
      bool &includeLShapeIn,
      frCoord withinIn = -1,
      frCoord eolWidthIn = -1,
      frCoord lengthIn = -1
    )
      : sameMask(sameMaskIn), edgeLength(edgeLengthIn), includeLShape(includeLShapeIn)
    {
      if (withinIn != -1) {
        within = withinIn;
      }
      if (eolWidthIn != -1) {
        eolWidth = eolWidthIn;
        if (lengthIn != -1) {
          length = lengthIn;
        }
      }

    }
    // getter
    bool isSameMask() {
      return sameMask;
    }
    bool isEdgeLength() {
      return edgeLength;
    }
    bool isIncludeLShape() {
      return includeLShape;
    }
    frCoord getWithin() {
      return within;
    }
    frCoord getEOLWidth() {
      return eolWidth;
    }
    frCoord getExceptJogLength() {
      return length;
    }
    // setter
    void setSameMask(bool &sameMaskIn) {
      sameMask = sameMaskIn;
    }
    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CornerSpacingConvexCornerConstraint;
    }


  protected:
    frCoord within = -1;
    frCoord eolWidth = -1;
    frCoord length = -1;
    bool sameMask = false;
    bool edgeLength = false;
    bool includeLShape = false;
  };




  class frLef58CornerSpacingSpacingConstraint : public frConstraint {
  public:
    // constructor
    frLef58CornerSpacingSpacingConstraint(frCoord &widthIn)
      : width(widthIn)
    {}
    // getter
    frCoord getWidth() {
      return width;
    }
    // setter
    void setWidth(frCoord &widthIn) {
      width = widthIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CornerSpacingSpacingConstraint;
    }
  protected:
    frCoord width;
  };

  class frLef58CornerSpacingSpacing1DConstraint : public frLef58CornerSpacingSpacingConstraint {
  public:
    // constructor
    frLef58CornerSpacingSpacing1DConstraint(frCoord widthIn, frCoord spacingIn)
      : frLef58CornerSpacingSpacingConstraint(widthIn), spacing(spacingIn)
    {}
    // getter
    frCoord getSpacing() {
      return spacing;
    }
    // setter
    void setSpacing(frCoord &spacingIn) {
      spacing = spacingIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CornerSpacingSpacing1DConstraint;
    }
    
  protected:
    frCoord spacing = -1;
  };

  class frLef58CornerSpacingSpacing2DConstraint : public frLef58CornerSpacingSpacingConstraint {
  public:
    // constructor
    frLef58CornerSpacingSpacing2DConstraint(frCoord widthIn, frCoord horizontalSpacingIn, frCoord verticalSpacingIn)
      : frLef58CornerSpacingSpacingConstraint(widthIn), horizontalSpacing(horizontalSpacingIn), verticalSpacing(verticalSpacingIn)
    {}
    // getter
    frCoord getHorizontalSpacing() {
      return horizontalSpacing;
    }
    frCoord getVerticalSpacing() {
      return verticalSpacing;
    }
    // setter
    void setHorizontalSpacing(frCoord &horizontalSpacingIn) {
      horizontalSpacing = horizontalSpacingIn;
    }
    void setVerticalSpacing(frCoord &verticalSpacingIn) {
      verticalSpacing = verticalSpacingIn;
    }

    frConstraintTypeEnum typeId() const override {
      return frConstraintTypeEnum::frcLef58CornerSpacingSpacing2DConstraint;
    }
  protected:
    frCoord horizontalSpacing = -1, verticalSpacing = -1;
  };

}

#endif
