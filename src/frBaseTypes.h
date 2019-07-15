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

#ifndef _FR_BASE_TYPES_H_
#define _FR_BASE_TYPES_H_

#include <vector>
#include <list>
#include <map>
#include <string>
#include <utility>

//#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/connected_components.hpp>

#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
//#include <boost/geometry/geometries/point.hpp>
//#include <boost/geometry/geometries/box.hpp>
//#include <boost/geometry/index/rtree.hpp>

namespace fr {
  using frLayerNum = int;
  using frCoord = int;
  using frUInt4 = unsigned int;
  using frDist  = double;
  using frString = std::string;
  using frCost = unsigned int;
  using frMIdx = int; // negative value expected 
  template <typename T>
  using frCollection = std::vector<T>;
  template <typename T>
  using frVector     = std::vector<T>;
  template <typename T>
  using frList       = std::list<T>;
  template <typename T>
  using frListIter   = typename std::list<T>::iterator;
  //template <typename T>
  //bool frListIterComp (const frListIter<T>& a, const frListIter<T>& b) {
  //  return &(*a) < &(*b);
  //};
  enum frOrientEnum {
      frcR0       = 0, // N
      frcR90      = 1, // W
      frcR180     = 2, // S
      frcR270     = 3, // E
      frcMY       = 4, // FN
      frcMXR90    = 5, // FW
      frcMX       = 6, // FS
      frcMYR90    = 7  // FE
  };
  enum frEndStyleEnum {
      frcTruncateEndStyle = 0, // ext = 0
      frcExtendEndStyle   = 1, // ext = half width
      frcVariableEndStyle = 2  // ext = variable
  };
  enum frPrefRoutingDirEnum {
      frcNotApplicablePrefRoutingDir = 0,
      frcNonePrefRoutingDir          = 1,
      frcHorzPrefRoutingDir          = 2,
      frcVertPrefRoutingDir          = 3 
  };
  enum frBlockObjectEnum {
      frcBlockObject   = -1,
      frcNet           = 0,
      frcTerm          = 1,
      frcInst          = 2,
      frcFig           = 3,
      frcConnFig       = 4,
      frcPinFig        = 5,
      frcShape         = 6,
      frcRef           = 7,
      frcVia           = 8,
      frcPin           = 9,
      frcInstTerm      = 10,
      frcRect          = 11,
      frcPolygon       = 12,
      frcSteiner       = 13,
      frcRoute         = 14,
      frcPathSeg       = 15,
      frcGuide         = 16,
      frcBlockage      = 17,
      frcLayerBlockage = 18,
      frcBlock         = 19,
      frcBoundary      = 20,
      frcFlexPathSeg   = 21,
      frcFlexVia       = 22,
      frcInstBlockage  = 23,
      frcAccessPattern = 24,
      frcMarker        = 25,
      frcPatchWire     = 26,
      drcBlockObject,
      drcNet,
      drcPin,
      drcAccessPattern,
      drcPathSeg,
      drcVia,
      drcMazeMarker,
      drcFig,
      drcConnFig,
      drcRef,
      drcPinFig,
      drcPatchWire,
      tacBlockObject,
      tacTrack,
      tacPin,
      tacPathSeg,
      tacVia
  };
  //enum class drBlockObjectEnum {
  //  drcBlockObject = 0,
  //  drcNet,
  //  drcPin,
  //  drcAccessPattern,
  //  drcPathSeg,
  //  drcVia
  //};
  enum class frGuideEnum {
      frcGuideX,
      frcGuideGlobal,
      frcGuideTrunk,
      frcGuideShortConn
  };
  enum class frTermEnum {
    frcNormalTerm,
    frcClockTerm,
    frcPowerTerm,
    frcGroundTerm
  };
  enum class frNetEnum {
    frcNormalNet,
    frcClockNet,
    frcPowerNet,
    frcGroundNet
  };
  //enum class frLef58CutSpacingTableTypeEnum {
  //  frcCenterSpacing,
  //  frcOrthogonal,
  //  frcOther,
  //};

  enum class frConstraintTypeEnum { // check FlexDR.h fixMode
    frcShortConstraint = 0,
    frcAreaConstraint = 1,
    frcMinWidthConstraint = 2,
    frcSpacingConstraint = 3,
    frcSpacingEndOfLineConstraint = 4,
    frcSpacingEndOfLineParallelEdgeConstraint = 5, // not supported
    frcSpacingTableConstraint = 6, // not supported
    frcSpacingTablePrlConstraint = 7,
    frcSpacingTableTwConstraint = 8,
    frcLef58SpacingTableConstraint = 9, // not supported
    frcLef58CutSpacingTableConstraint = 10, // not supported
    frcLef58CutSpacingTablePrlConstraint = 11, // not supported
    frcLef58CutSpacingTableLayerConstraint = 12, // not supported
    frcLef58CutSpacingConstraint = 13, // not supported
    frcLef58CutSpacingParallelWithinConstraint = 14, // not supported
    frcLef58CutSpacingAdjacentCutsConstraint = 15, // not supported
    frcLef58CutSpacingLayerConstraint = 16, // not supported
    frcCutSpacingConstraint = 17,
    frcMinStepConstraint,
    frcMinimumcutConstraint,
    frcLef58CornerSpacingConstraint, // not supported
    frcLef58CornerSpacingConcaveCornerConstraint, // not supported
    frcLef58CornerSpacingConvexCornerConstraint, // not supported
    frcLef58CornerSpacingSpacingConstraint, // not supported
    frcLef58CornerSpacingSpacing1DConstraint, // not supported
    frcLef58CornerSpacingSpacing2DConstraint, // not supported
    frcLef58SpacingEndOfLineConstraint, // not supported
    frcLef58SpacingEndOfLineWithinConstraint, // not supported
    frcLef58SpacingEndOfLineWithinEndToEndConstraint, // not supported
    frcLef58SpacingEndOfLineWithinParallelEdgeConstraint, // not supported
    frcLef58SpacingEndOfLineWithinMaxMinLengthConstraint, // not supported
    frcLef58CutClassConstraint // not supported
  };

  enum class frMinimumcutConnectionEnum {
    UNKNOWN = -1,
    FROMABOVE = 0,
    FROMBELOW = 1
  };

  //enum class frDirEnum { UNKNOWN = 0, E = 1, S = 2, W = 3, N = 4, U = 5, D = 6 };
  //enum class frDirEnum { UNKNOWN = 0, E = 4, S = 2, W = 3, N = 1, U = 6, D = 5 };
  #define OPPOSITEDIR 7
  enum class frDirEnum { UNKNOWN = 0, D = 1, S = 2, W = 3, E = 4, N = 5, U = 6 };

  enum class frLayerTypeEnum {
    CUT,
    ROUTING,
    IMPLANT
  };


  enum class AccessPointTypeEnum {
    Ideal,
    Good,
    Offgrid,
    None
  };

  enum class MacroClassEnum {
    UNKNOWN,
    CORE,
    CORE_TIEHIGH,
    CORE_TIELOW,
    CORE_WELLTAP,
    CORE_SPACER,
    CORE_ANTENNACELL,
    ENDCAP_PRE,
    BLOCK
  };

  enum class drNetOrderingEnum {
    NETDRIVEN,
    MARKERDRIVEN
  };

  //enum frShapeEnum {
  //    frcRect    = 0,
  //    frcPolygon = 1
  //};
  class frBlockObject;
  struct vertex_properties_t {
    frBlockObject* objPtr;
    //int index;
    //boost::default_color_type color;
    //frString name;
  };
  //class frRoute;
  class frConnFig;
  class frInstTerm;
  class frTerm;
  class frInst;
  class frBlockage;
  struct edge_properties_t {
    //std::shared_ptr<frBlockObject> objPtr;
    //std::shared_ptr<frRoute> objPtr;
    std::shared_ptr<frConnFig> objPtr;
    //frString name;
  };
  // boost graph
  typedef boost::adjacency_list< boost::listS, boost::listS, boost::undirectedS, vertex_properties_t, edge_properties_t > graph_t;
  //typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, vertex_properties_t, edge_properties_t > graph_t;
  // descriptor
  typedef boost::graph_traits<graph_t>::vertex_descriptor   vertex_descriptor_t;
  typedef boost::graph_traits<graph_t>::edge_descriptor     edge_descriptor_t;
  // iterator
  typedef boost::graph_traits<graph_t>::vertex_iterator     vertex_iterator_t;
  typedef boost::graph_traits<graph_t>::edge_iterator       edge_iterator_t;
  typedef boost::graph_traits<graph_t>::out_edge_iterator   out_edge_iterator_t;
  typedef boost::graph_traits<graph_t>::adjacency_iterator  adjacency_iterator_t;

  typedef std::map<vertex_descriptor_t, std::size_t>        vertex_descriptor_map_t;
  //typedef boost::property_map<graph_t, &vertex_properties_t::objPtr>::type           tempPM;
  namespace bg  = boost::geometry;
  namespace bgi = boost::geometry::index;

  //typedef bg::model::point<int, 2, bg::cs::cartesian> boostPoint;
  typedef bg::model::d2::point_xy<frCoord, bg::cs::cartesian> boostPoint;
  typedef bg::model::box<boostPoint> boostBox;
  typedef bg::model::polygon<boostPoint> boostPolygon;
  typedef bg::model::segment<boostPoint> boostSegment;



  //typedef boost::geometry::model::point<frCoord, 2, boost::geometry::cs::cartesian>   point_t;
  typedef bg::model::d2::point_xy<frCoord, bg::cs::cartesian>                         point_t;
  typedef bg::model::box<point_t>                                                     box_t;
  typedef bg::model::segment<point_t>                                                 segment_t;
  class frConnFig;
  typedef std::pair<box_t, std::shared_ptr<frConnFig> >                               rtree_frConnFig_value_t;
  //typedef std::pair<box_t, int* >                                                     rtree_test_t;
  typedef std::pair<box_t, std::shared_ptr<frInst> > rtree_frInst_value_t;
  typedef std::pair<box_t, std::shared_ptr<frTerm> > rtree_frTerm_value_t;
  typedef std::pair<box_t, std::pair<std::shared_ptr<frTerm>, std::shared_ptr<frInstTerm> > > rtree_frTerm_frInstTerm_value_t;
  typedef std::pair<box_t, std::shared_ptr<frBlockage> > rtree_frBlockage_value_t;
  //typedef std::pair<box_t, std::string >                                              rtree_value_t;
  template <typename T>
  using rq_iter_value_t = std::pair<box_t, frListIter<T> >;
  template <typename T>
  using rq_ptr_value_t  = std::pair<box_t, std::shared_ptr<T> >;
  template <typename T>
  using rq_rptr_value_t = std::pair<box_t, T* >;
  template <typename T>
  using rq_generic_value_t = std::pair<box_t, T>;

  // KMB data types
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, vertex_properties_t, boost::property < boost::edge_weight_t, double > > KMBGraph; 
  // typedef std::pair<int, int> KMBEdge;

  // DRC check types
  typedef std::pair<boostPoint, boostPoint> boostEdge;


  // BoostPolygon
  typedef boost::polygon::rectangle_data<int>  Rectangle;
  typedef boost::polygon::polygon_90_data<int> Polygon;
  typedef std::vector<boost::polygon::polygon_90_data<int> > PolygonSet;
  typedef boost::polygon::point_data<int> Point;
  typedef boost::polygon::interval_data<int> Interval;
  typedef boost::polygon::segment_data<int> Segment;
}

#endif
