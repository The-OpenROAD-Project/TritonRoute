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

#include "dr/FlexGridGraph.h"

using namespace std;
using namespace fr;

/*inline*/ void FlexGridGraph::expand(FlexWavefrontGrid &currGrid, const frDirEnum &dir, 
                                      const FlexMazeIdx &dstMazeIdx1, const FlexMazeIdx &dstMazeIdx2,
                                      const frPoint &centerPt) {
  bool enableOutput = false;
  //bool enableOutput = true;
  frCost nextEstCost, nextPathCost;
  int gridX = currGrid.x();
  int gridY = currGrid.y();
  int gridZ = currGrid.z();

  getNextGrid(gridX, gridY, gridZ, dir);
  
  FlexMazeIdx nextIdx(gridX, gridY, gridZ);
  // get cost
  nextEstCost = getEstCost(nextIdx, dstMazeIdx1, dstMazeIdx2, dir);
  nextPathCost = getNextPathCost(currGrid, dir);  
  if (enableOutput) {
    std::cout << "  expanding from (" << currGrid.x() << ", " << currGrid.y() << ", " << currGrid.z() 
              << ") [pathCost / totalCost = " << currGrid.getPathCost() << " / " << currGrid.getCost() << "] to "
              << "(" << gridX << ", " << gridY << ", " << gridZ << ") [pathCost / totalCost = " 
              << nextPathCost << " / " << nextPathCost + nextEstCost << "]\n";
  }
  //FlexWavefrontGrid nextWavefrontGrid(gridX, gridY, gridZ, nextPathCost, nextPathCost + nextEstCost, currGrid.getBackTraceBuffer());
  auto lNum = getLayerNum(currGrid.z());
  //auto isH  = getZDir(currGrid.z());
  //frCoord via2viaLen = 1;// penalize length on non-pref dir
  //if ((isH  && (dir == frDirEnum::W || dir == frDirEnum::E)) ||
  //    (!isH && (dir == frDirEnum::S || dir == frDirEnum::N))) {
  //  via2viaLen = getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir); 
  //}
  //via2viaLen = (currGrid.getLength() == std::numeric_limits<frCoord>::max()) ? 0 : via2viaLen;
  auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
  frPoint currPt;
  getPoint(currPt, gridX, gridY);
  frCoord currDist = abs(currPt.x() - centerPt.x()) + abs(currPt.y() - centerPt.y());

  // vlength calculation
  frCoord currVLengthX = 0;
  frCoord currVLengthY = 0;
  currGrid.getVLength(currVLengthX, currVLengthY);
  auto nextVLengthX = currVLengthX;
  auto nextVLengthY = currVLengthY;
  bool nextIsPrevViaUp = currGrid.isPrevViaUp();
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextVLengthX = 0;
    nextVLengthY = 0;
    nextIsPrevViaUp = (dir == frDirEnum::D); // up via if current path goes down
  } else {
    if (currVLengthX != std::numeric_limits<frCoord>::max() &&
        currVLengthY != std::numeric_limits<frCoord>::max()) {
      if (dir == frDirEnum::W || dir == frDirEnum::E) {
        nextVLengthX += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      } else { 
        nextVLengthY += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      }
    }
  }
  
  // tlength calculation
  auto currTLength = currGrid.getTLength();
  auto nextTLength = currTLength;
  // if there was a turn, then add tlength
  if (currTLength != std::numeric_limits<frCoord>::max()) {
    nextTLength += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  // if current is a turn, then reset tlength
  if (currGrid.getLastDir() != frDirEnum::UNKNOWN && currGrid.getLastDir() != dir) {
    nextTLength = getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  // if current is a via, then reset tlength
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextTLength = std::numeric_limits<frCoord>::max();
  }

  FlexWavefrontGrid nextWavefrontGrid(gridX, gridY, gridZ, 
                                      currGrid.getLayerPathArea() + getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir) * pathWidth, 
                                      nextVLengthX, nextVLengthY, nextIsPrevViaUp,
                                      /*currGrid.getLength() + via2viaLen, currGrid.getLength() + via2viaLen, currGrid.isPrevViaUp(),*/
                                      nextTLength,
                                      currDist,
                                      nextPathCost, nextPathCost + nextEstCost, currGrid.getBackTraceBuffer());
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextWavefrontGrid.resetLayerPathArea();
    nextWavefrontGrid.resetLength();
    if (dir == frDirEnum::U) {
      nextWavefrontGrid.setPrevViaUp(false);
    } else {
      nextWavefrontGrid.setPrevViaUp(true);
    }
    nextWavefrontGrid.addLayerPathArea((dir == frDirEnum::U) ? getHalfViaEncArea(currGrid.z(), false) : getHalfViaEncArea(gridZ, true));
  }
  //if (currGrid.getLastDir() != frDirEnum::UNKNOWN && currGrid.getLastDir() != dir) {
  //  nextWavefrontGrid.resetLength();
  //}
  // update wavefront buffer
  auto tailDir = nextWavefrontGrid.shiftAddBuffer(dir);
  // commit grid prev direction if needed
  auto tailIdx = getTailIdx(nextIdx, nextWavefrontGrid);
  if (tailDir != frDirEnum::UNKNOWN) {
    if (getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == frDirEnum::UNKNOWN ||
        getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == tailDir) {
      setPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z(), tailDir);
      wavefront.push(nextWavefrontGrid);
      if (enableOutput) {
        std::cout << "    commit (" << tailIdx.x() << ", " << tailIdx.y() << ", " << tailIdx.z() << ") prev accessing dir = " << (int)tailDir << "\n";
      }
    }
  } else {  
    // add to wavefront
    wavefront.push(nextWavefrontGrid);
  }
  return;
}

/*inline*/ void FlexGridGraph::expandWavefront(FlexWavefrontGrid &currGrid, const FlexMazeIdx &dstMazeIdx1, 
                                               const FlexMazeIdx &dstMazeIdx2, const frPoint &centerPt) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (enableOutput) {
    cout << "start expand from (" << currGrid.x() << ", " << currGrid.y() << ", " << currGrid.z() << ")\n";
  }
  //if (currGrid.y() == 19 && currGrid.z() == 0) {
  //  cout <<"is expandable (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") NESWUD "
  //       <<isExpandable(currGrid, frDirEnum::N)
  //       <<isExpandable(currGrid, frDirEnum::E)
  //       <<isExpandable(currGrid, frDirEnum::S)
  //       <<isExpandable(currGrid, frDirEnum::W)
  //       <<isExpandable(currGrid, frDirEnum::U)
  //       <<isExpandable(currGrid, frDirEnum::D)
  //       <<endl;
  //  cout <<"has edge " 
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::N)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::E)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::S)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::W)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::U)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::D)
  //       <<endl;
  //  int gridX = currGrid.x();
  //  int gridY = currGrid.y();
  //  int gridZ = currGrid.z();
  //  if (!gridGraph.hasEdge(gridX, gridY, gridZ, frDirEnum::E)) {
  //    ;
  //  } else {
  //    getNextGrid(gridX, gridY, gridZ, frDirEnum::E);
  //    if (gridGraph.isBlocked(gridX, gridY, gridZ)) {
  //      cout <<"blocked" <<endl;
  //    } else if (gridGraph.isSrc(gridX, gridY, gridZ)) {
  //      cout <<"src" <<endl;
  //    } else if (gridGraph.getPrevAstarNodeDir(gridX, gridY, gridZ) != frDirEnum::UNKNOWN) {
  //      cout <<"visited" <<endl;
  //    } else {
  //      ;
  //    }
  //  }
  //}
  //auto tmpGrid = currWavefrontGrid;
  //// commit grid prev direction if needed
  //auto tailIdx = getTailIdx(nextIdx, nextWavefrontGrid);
  //if (tailDir != frDirEnum::UNKNOWN) {
  //  if (getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == frDirEnum::UNKNOWN) {
  //    ;
  //  }
  //}
  // N
  if (isExpandable(currGrid, frDirEnum::N)) {
    expand(currGrid, frDirEnum::N, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no N" <<endl;
  // }
  // E
  if (isExpandable(currGrid, frDirEnum::E)) {
    expand(currGrid, frDirEnum::E, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no E" <<endl;
  // }
  // S
  if (isExpandable(currGrid, frDirEnum::S)) {
    expand(currGrid, frDirEnum::S, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no S" <<endl;
  // }
  // W
  if (isExpandable(currGrid, frDirEnum::W)) {
    expand(currGrid, frDirEnum::W, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no W" <<endl;
  // }
  // U
  if (isExpandable(currGrid, frDirEnum::U)) {
    expand(currGrid, frDirEnum::U, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no U" <<endl;
  // }
  // D
  if (isExpandable(currGrid, frDirEnum::D)) {
    expand(currGrid, frDirEnum::D, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no D" <<endl;
  // }
}

/*inline*/ frCost FlexGridGraph::getEstCost(const FlexMazeIdx &src, const FlexMazeIdx &dstMazeIdx1,
                                const FlexMazeIdx &dstMazeIdx2, const frDirEnum &dir) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<"est from (" <<src.x() <<", " <<src.y() <<", " <<src.z() <<") "
         <<"to ("       <<dstMazeIdx1.x() <<", " <<dstMazeIdx1.y() <<", " <<dstMazeIdx1.z() <<") ("
                        <<dstMazeIdx2.x() <<", " <<dstMazeIdx2.y() <<", " <<dstMazeIdx2.z() <<")";
  }
  // bend cost
  int bendCnt = 0;
  frPoint srcPoint, dstPoint1, dstPoint2;
  getPoint(srcPoint, src.x(), src.y());
  getPoint(dstPoint1, dstMazeIdx1.x(), dstMazeIdx1.y());
  getPoint(dstPoint2, dstMazeIdx2.x(), dstMazeIdx2.y());
  //auto minCostX = std::abs(srcPoint.x() - dstPoint.x()) * 1;
  //auto minCostY = std::abs(srcPoint.y() - dstPoint.y()) * 1;
  //auto minCostZ = std::abs(gridGraph.getZHeight(src.z()) - gridGraph.getZHeight(dst.z())) * VIACOST;
  frCoord minCostX = max(max(dstPoint1.x() - srcPoint.x(), srcPoint.x() - dstPoint2.x()), 0) * 1;
  frCoord minCostY = max(max(dstPoint1.y() - srcPoint.y(), srcPoint.y() - dstPoint2.y()), 0) * 1;
  frCoord minCostZ = max(max(getZHeight(dstMazeIdx1.z()) - getZHeight(src.z()), 
                       getZHeight(src.z()) - getZHeight(dstMazeIdx2.z())), 0) * 1;
  if (enableOutput) {
    cout <<" x/y/z min cost = (" <<minCostX <<", " <<minCostY <<", " <<minCostZ <<") " <<endl;
  }

  bendCnt += (minCostX && dir != frDirEnum::UNKNOWN && dir != frDirEnum::E && dir != frDirEnum::W) ? 1 : 0;
  bendCnt += (minCostY && dir != frDirEnum::UNKNOWN && dir != frDirEnum::S && dir != frDirEnum::N) ? 1 : 0;
  bendCnt += (minCostZ && dir != frDirEnum::UNKNOWN && dir != frDirEnum::U && dir != frDirEnum::D) ? 1 : 0;
  //bendCnt -= bendCnt ? 1 : 0;
  // return (deltaX + deltaY + VIACOST * deltaZ + ((deltaX && deltaY) ? 1 : 0));
  if (enableOutput) {
    cout << "  est cost = " << minCostX + minCostY + minCostZ + bendCnt << endl;
  }
  return (minCostX + minCostY + minCostZ + bendCnt);

}

/*inline*/ frDirEnum FlexGridGraph::getLastDir(const std::bitset<WAVEFRONTBITSIZE> &buffer) {
  auto currDirVal = buffer.to_ulong() & 0b111u;
  return static_cast<frDirEnum>(currDirVal);
}

void FlexGridGraph::getNextGrid(frMIdx &gridX, frMIdx &gridY, frMIdx &gridZ, const frDirEnum dir) {
  switch(dir) {
    case frDirEnum::E:
      ++gridX;
      break;
    case frDirEnum::S:
      --gridY;
      break;
    case frDirEnum::W:
      --gridX;
      break;
    case frDirEnum::N:
      ++gridY;
      break;
    case frDirEnum::U:
      ++gridZ;
      break;
    case frDirEnum::D:
      --gridZ;
      break;
    default:
      ;
  }
  return;
}

void FlexGridGraph::getPrevGrid(frMIdx &gridX, frMIdx &gridY, frMIdx &gridZ, const frDirEnum dir) const {
  switch(dir) {
    case frDirEnum::E:
      --gridX;
      break;
    case frDirEnum::S:
      ++gridY;
      break;
    case frDirEnum::W:
      ++gridX;
      break;
    case frDirEnum::N:
      --gridY;
      break;
    case frDirEnum::U:
      --gridZ;
      break;
    case frDirEnum::D:
      ++gridZ;
      break;
    default:
      ;
  }
  return;
}

/*inline*/ frCost FlexGridGraph::getNextPathCost(const FlexWavefrontGrid &currGrid, const frDirEnum &dir) {
  // bool enableOutput = true;
  bool enableOutput = false;
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  frCost nextPathCost = currGrid.getPathCost();
  // bending cost
  //auto currDir = getLastDir(currGrid.getBackTraceBuffer());
  auto currDir = currGrid.getLastDir();
  // std::cout << "(currDir = " << (int)currDir << ") ";
  // if (currDir != dir && (int)currDir >= 1 && (int)currDir <= 4 &&
  //                       (int)dir     >= 1 && (int)dir     <= 4) {
  //   ++nextPathCost;
  // }
  if (currDir != dir && currDir != frDirEnum::UNKNOWN) {
    ++nextPathCost;
  }
  // // attempt to resolve minArea
  auto lNum = getLayerNum(currGrid.z());
  auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  //frCoord currArea = (currGrid.getLayerPathLength() + pathWidth) * pathWidth; 
  /* min-area enablement
  frCoord currArea = currGrid.getLayerPathArea();
  if (dir == frDirEnum::U) {
    currArea += getHalfViaEncArea(currGrid.z(), true);
  } else if (dir == frDirEnum::D) {
    currArea += getHalfViaEncArea(currGrid.z() - 1, false);
  }
  auto minAreaConstraint = getDesign()->getTech()->getLayer(lNum)->getAreaConstraint();
  if (minAreaConstraint && (dir == frDirEnum::U || dir == frDirEnum::D) && currArea < minAreaConstraint->getMinArea()) {
    nextPathCost += ggDRCCost * minAreaConstraint->getMinArea() / pathWidth;
    //nextPathCost += ggDRCCost * pathWidth;
    // if (TEST) {
    //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
    // }
  }
  */
  // via2viaminlen enablement
  //if (dir == frDirEnum::U && currGrid.getLength() != 0 && currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), true)) {
  //if (dir == frDirEnum::U && 
  //    (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), true) ? !(currGrid.getLength() == 0) : true) && 
  //    currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), true)) {
  //  nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //  // if (TEST) {
  //  //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
  //  // }
  //} else if (dir == frDirEnum::D && 
  //           (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), false) ? !(currGrid.getLength() == 0) : true) && 
  //           currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), false)) {
  //  nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //  // if (TEST) {
  //  //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
  //  // }
  //}

  // via2viaminlenNew enablement
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    frCoord currVLengthX = 0;
    frCoord currVLengthY = 0;
    currGrid.getVLength(currVLengthX, currVLengthY);
    bool isCurrViaUp = (dir == frDirEnum::U);
    // if allow zero length and both x and y = 0 
    if (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), isCurrViaUp) && currVLengthX == 0 && currVLengthY == 0) {
      ;
    } else {
      // check only y
      if (currVLengthX == 0 && currVLengthY > 0 && currVLengthY < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, true)) {
        nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
      // check only x
      } else if (currVLengthX > 0 && currVLengthY == 0 && currVLengthX < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, false)) {
        nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
      // check both x and y
      } else if (currVLengthX > 0 && currVLengthY > 0 && 
                 ((currVLengthY < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, true)) &&
                   currVLengthX < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, false))) {
        nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
      }
    }
  }

  // via2turnminlen enablement
  frCoord tLength    = std::numeric_limits<frCoord>::max();
  frCoord tLengthDummy = 0;
  frCoord tLengthReq = 0;
  //bool    isTLengthY = false;
  bool    isTLengthViaUp = false;
  if (currDir != frDirEnum::UNKNOWN && currDir != dir) {
    // next dir is a via
    if (dir == frDirEnum::U || dir == frDirEnum::D) {
      isTLengthViaUp = (dir == frDirEnum::U);
      // if there was a turn before
      if (tLength != std::numeric_limits<frCoord>::max()) {
        if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
          tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
          tLength = currGrid.getTLength();
          //isTLengthY = false;
        } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
          tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
          tLength = currGrid.getTLength();
          //isTLengthY = true;
        } else {
          ;
        }
      }
    // curr is a planar turn
    } else {
      isTLengthViaUp = currGrid.isPrevViaUp();
      if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
        tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
        //isTLengthY = false;
        currGrid.getVLength(tLength, tLengthDummy);
      } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
        tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
        //isTLengthY = true;
        currGrid.getVLength(tLengthDummy, tLength);
      } else {
        ;
      }
    }
    if (tLength < tLengthReq) {
      nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
    }
  }




  //if (currDir != dir && currDir != frDirEnum::UNKNOWN && currGrid.getLength() != 0 && currGrid.getLength() < (frCoord)pathWidth * 2) {
  //  nextPathCost += ggDRCCost * pathWidth * 2;
  //}
  //frMIdx nextGridX = gridX;
  //frMIdx nextGridY = gridY;
  //frMIdx nextGridZ = gridZ;
  //getNextGrid(nextGridX, nextGridY, nextGridZ, dir);
  //nextPathCost += gridGraph.getEdgeCost(gridX, gridY, gridZ, dir) * gridGraph.getEdgeLength(gridX, gridY, gridZ, dir);
  bool gridCost   = hasGridCost(gridX, gridY, gridZ, dir);
  bool drcCost    = hasDRCCost(gridX, gridY, gridZ, dir);
  bool markerCost = hasMarkerCost(gridX, gridY, gridZ, dir);
  bool shapeCost  = hasShapeCost(gridX, gridY, gridZ, dir);
  bool blockCost  = isBlocked(gridX, gridY, gridZ, dir);
  //bool guideCost  = hasGuide(gridX, gridY, gridZ, dir);

  // temporarily disable guideCost
  nextPathCost += getEdgeLength(gridX, gridY, gridZ, dir)
                  + (gridCost   ? GRIDCOST         * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (drcCost    ? ggDRCCost        * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  //+ (markerCost ? ggMarkerCost     * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (markerCost ? ggMarkerCost     * pathWidth                               : 0)
                  + (shapeCost  ? SHAPECOST        * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (blockCost  ? BLOCKCOST        * pathWidth                               : 0)
                  /*+ (!guideCost ? GUIDECOST        * getEdgeLength(gridX, gridY, gridZ, dir) : 0)*/;
  if (enableOutput) {
    cout <<"edge grid/shape/drc/marker/blk/length = " 
         <<hasGridCost(gridX, gridY, gridZ, dir)   <<"/"
         <<hasShapeCost(gridX, gridY, gridZ, dir)  <<"/"
         <<hasDRCCost(gridX, gridY, gridZ, dir)    <<"/"
         <<hasMarkerCost(gridX, gridY, gridZ, dir) <<"/"
         <<isBlocked(gridX, gridY, gridZ, dir) <<"/"
         //<<hasGuide(gridX, gridY, gridZ, dir) <<"/"
         <<getEdgeLength(gridX, gridY, gridZ, dir) <<endl;
  }
  return nextPathCost;

}

/*inline*/ FlexMazeIdx FlexGridGraph::getTailIdx(const FlexMazeIdx &currIdx, const FlexWavefrontGrid &currGrid) {
  int gridX = currIdx.x();
  int gridY = currIdx.y();
  int gridZ = currIdx.z();
  auto backTraceBuffer = currGrid.getBackTraceBuffer();
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) {
    int currDirVal = backTraceBuffer.to_ulong() - ((backTraceBuffer.to_ulong() >> DIRBITSIZE) << DIRBITSIZE);
    frDirEnum currDir = static_cast<frDirEnum>(currDirVal);
    backTraceBuffer >>= DIRBITSIZE;
    getPrevGrid(gridX, gridY, gridZ, currDir);
  }
  return FlexMazeIdx(gridX, gridY, gridZ);
}

/*inline*/ bool FlexGridGraph::isExpandable(const FlexWavefrontGrid &currGrid, frDirEnum dir) {
  //bool enableOutput = true;
  bool enableOutput = false;
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  //bool hg = hasEdge(gridX, gridY, gridZ, dir) && hasGuide(gridX, gridY, gridZ, dir);
  bool hg = hasEdge(gridX, gridY, gridZ, dir);
  if (enableOutput) {
    if (!hasEdge(gridX, gridY, gridZ, dir)) {
      cout <<"no edge@(" <<gridX <<", " <<gridY <<", " <<gridZ <<") " <<(int)dir <<endl;
    }
    if (hasEdge(gridX, gridY, gridZ, dir) && !hasGuide(gridX, gridY, gridZ, dir)) {
      cout <<"no guide@(" <<gridX <<", " <<gridY <<", " <<gridZ <<") " <<(int)dir <<endl;
    }
  }
  reverse(gridX, gridY, gridZ, dir);
  if (!hg || 
      isSrc(gridX, gridY, gridZ) || 
      getPrevAstarNodeDir(gridX, gridY, gridZ) != frDirEnum::UNKNOWN ||
      currGrid.getLastDir() == dir) {
    return false;
  } else {
    return true;
  }
}

void FlexGridGraph::traceBackPath(const FlexWavefrontGrid &currGrid, vector<FlexMazeIdx> &path, vector<FlexMazeIdx> &root,
                                  FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout << "    start traceBackPath...\n";
  }
  frDirEnum prevDir = frDirEnum::UNKNOWN, currDir = frDirEnum::UNKNOWN;
  int currX = currGrid.x(), currY = currGrid.y(), currZ = currGrid.z();
  // pop content in buffer
  auto backTraceBuffer = currGrid.getBackTraceBuffer();
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) {
    // current grid is src
    if (isSrc(currX, currY, currZ)) {
      break;
    }
    // get last direction
    currDir = getLastDir(backTraceBuffer);
    backTraceBuffer >>= DIRBITSIZE;
    if (currDir == frDirEnum::UNKNOWN) {
      cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    // add to root // when path makes a planar turn, dropping via on the turn is dangerous. Avoid it.
    // if (prevDir != frDirEnum::UNKNOWN && 
    //     currDir != prevDir && 
    //     (currDir != frDirEnum::U && currDir != frDirEnum::D && prevDir != frDirEnum::U && prevDir != frDirEnum::U)) {
      root.push_back(FlexMazeIdx(currX, currY, currZ));
    // }
    // push point to path
    if (currDir != prevDir) {
      path.push_back(FlexMazeIdx(currX, currY, currZ));
      if (enableOutput) {
        cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
      }
    }
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;
  }
  // trace back according to grid prev dir
  while (isSrc(currX, currY, currZ) == false) {
    // get last direction
    currDir = getPrevAstarNodeDir(currX, currY, currZ);
    // add to root
    // if (prevDir != frDirEnum::UNKNOWN && 
    //     currDir != prevDir && 
    //     (currDir != frDirEnum::U && currDir != frDirEnum::D && prevDir != frDirEnum::U && prevDir != frDirEnum::U)) {
      root.push_back(FlexMazeIdx(currX, currY, currZ));
    // }
    if (currDir == frDirEnum::UNKNOWN) {
      cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    if (currDir != prevDir) {
      path.push_back(FlexMazeIdx(currX, currY, currZ));
      if (enableOutput) {
        cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
      }
    }
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;
  }
  // add final path to src, only add when path exists; no path exists (src = dst)
  if (!path.empty()) {
    path.push_back(FlexMazeIdx(currX, currY, currZ));
    if (enableOutput) {
      cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
    }
  }
  for (auto &mi: path) {
    ccMazeIdx1.set(min(ccMazeIdx1.x(), mi.x()),
                   min(ccMazeIdx1.y(), mi.y()),
                   min(ccMazeIdx1.z(), mi.z()));
    ccMazeIdx2.set(max(ccMazeIdx2.x(), mi.x()),
                   max(ccMazeIdx2.y(), mi.y()),
                   max(ccMazeIdx2.z(), mi.z()));
  }
  if (enableOutput) {
    cout <<endl;
  }

}

bool FlexGridGraph::search(vector<FlexMazeIdx> &connComps, drPin* nextPin, vector<FlexMazeIdx> &path, 
                           FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2, const frPoint &centerPt) {
  //bool enableOutput = true;
  bool enableOutput = false;
  int stepCnt = 0;

  // prep nextPinBox
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  FlexMazeIdx dstMazeIdx1(xDim - 1, yDim - 1, zDim - 1);
  FlexMazeIdx dstMazeIdx2(0, 0, 0);
  FlexMazeIdx mi;
  for (auto &ap: nextPin->getAccessPatterns()) {
    ap->getMazeIdx(mi);
    dstMazeIdx1.set(min(dstMazeIdx1.x(), mi.x()),
                    min(dstMazeIdx1.y(), mi.y()),
                    min(dstMazeIdx1.z(), mi.z()));
    dstMazeIdx2.set(max(dstMazeIdx2.x(), mi.x()),
                    max(dstMazeIdx2.y(), mi.y()),
                    max(dstMazeIdx2.z(), mi.z()));
  }

  // std::cout << "start astarSearch\n";
  wavefront = FlexWavefront();
  //cout <<"xxx1" <<endl;
  // init wavefront
  frPoint currPt;
  for (auto &idx: connComps) {
    if (isDst(idx.x(), idx.y(), idx.z())) {
      if (enableOutput) {
        cout <<"message: astarSearch dst covered (" <<idx.x() <<", " <<idx.y() <<", " <<idx.z() <<")" <<endl;
      }
      path.push_back(FlexMazeIdx(idx.x(), idx.y(), idx.z()));
      return true;
    }
    // get min area min length
    auto lNum = getLayerNum(idx.z());
    //auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
    auto minAreaConstraint = getDesign()->getTech()->getLayer(lNum)->getAreaConstraint();
    frCoord fakeArea = minAreaConstraint ? minAreaConstraint->getMinArea() : 0;
    getPoint(currPt, idx.x(), idx.y());
    frCoord currDist = abs(currPt.x() - centerPt.x()) + abs(currPt.y() - centerPt.y());
    //frCoord fakeLen  = getVia2ViaMinLen(idx.z(), true, true);
    FlexWavefrontGrid currGrid(idx.x(), idx.y(), idx.z(), fakeArea, 
                               std::numeric_limits<frCoord>::max(), std::numeric_limits<frCoord>::max(), true, 
                               /*fakeLen, fakeLen, true,*/
                               std::numeric_limits<frCoord>::max(),
                               currDist, 0, getEstCost(idx, dstMazeIdx1, dstMazeIdx2, frDirEnum::UNKNOWN));
    wavefront.push(currGrid);
    if (enableOutput) {
      cout <<"src add to wavefront (" <<idx.x() <<", " <<idx.y() <<", " <<idx.z() <<")" <<endl;
    }
  }
  //cout <<"xxx2" <<endl;
  while(!wavefront.empty()) {
    // std::cout << "here1\n";
    auto currGrid = wavefront.top();
    wavefront.pop();
    if (getPrevAstarNodeDir(currGrid.x(), currGrid.y(), currGrid.z()) != frDirEnum::UNKNOWN) {
      continue;
    }
    // test
    if (enableOutput) {
      ++stepCnt;
    }
    // if (stepCnt % 100000 == 0) {
    //   std::cout << "wavefront size = " << wavefront.size() << " at step = " << stepCnt << "\n";
    // }
    if (isDst(currGrid.x(), currGrid.y(), currGrid.z())) {
      traceBackPath(currGrid, path, connComps, ccMazeIdx1, ccMazeIdx2);
      if (enableOutput) {
        cout << "path found. stepCnt = " << stepCnt << "\n";
      }
      return true;
    } else {
      // expand and update wavefront
      expandWavefront(currGrid, dstMazeIdx1, dstMazeIdx2, centerPt);
    }
    
  }
  return false;
}

