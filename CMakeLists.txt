cmake_minimum_required(VERSION 3.1)

project(TritonRoute
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7)
  message(FATAL_ERROR "TritonRoute requires GCC 7 and above! \nuse -DCMAKE_CXX_COMPILER to assign your compiler path")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

set(FLEXROUTE_HOME ${PROJECT_SOURCE_DIR} )

set(DEFLIB_HOME
  ${PROJECT_SOURCE_DIR}/module/def/5.8-p029
)
set(LEFLIB_HOME
  ${PROJECT_SOURCE_DIR}/module/lef/5.8-p029
)

## include subdirectories 
add_subdirectory(${DEFLIB_HOME})
add_subdirectory(${LEFLIB_HOME})


## configure settings
find_package(ZLIB REQUIRED)
find_package(BISON REQUIRED)

find_package(Boost 1.67.0 REQUIRED)
find_package(Threads REQUIRED)

set (FLEXROUTE_SRC
  ${FLEXROUTE_HOME}/src/global.cpp
  ${FLEXROUTE_HOME}/src/drc/frDRC_util.cpp
  ${FLEXROUTE_HOME}/src/drc/frDRC_setup.cpp
  ${FLEXROUTE_HOME}/src/drc/frDRC_init.cpp
  ${FLEXROUTE_HOME}/src/drc/frDRC_main.cpp
  ${FLEXROUTE_HOME}/src/io/io_guide.cpp
  ${FLEXROUTE_HOME}/src/io/frAPG.cpp
  ${FLEXROUTE_HOME}/src/io/io_parser_helper.cpp
  ${FLEXROUTE_HOME}/src/io/io.cpp
  ${FLEXROUTE_HOME}/src/io/defw.cpp
  ${FLEXROUTE_HOME}/src/io/io_pin.cpp
  ${FLEXROUTE_HOME}/src/io/frPinPrep.cpp
  ${FLEXROUTE_HOME}/src/io/frShapeUtil.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexGridGraph.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexGridGraph_maze.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR_maze.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR_end.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR_init.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR_conn.cpp
  ${FLEXROUTE_HOME}/src/dr/FlexDR_rq.cpp
  ${FLEXROUTE_HOME}/src/FlexRoute.cpp
  ${FLEXROUTE_HOME}/src/ta/FlexTA_rq.cpp
  ${FLEXROUTE_HOME}/src/ta/FlexTA_assign.cpp
  ${FLEXROUTE_HOME}/src/ta/FlexTA_end.cpp
  ${FLEXROUTE_HOME}/src/ta/FlexTA_init.cpp
  ${FLEXROUTE_HOME}/src/ta/FlexTA.cpp
  ${FLEXROUTE_HOME}/src/frRegionQuery.cpp
  ${FLEXROUTE_HOME}/src/utility.cpp
  ${FLEXROUTE_HOME}/src/main.cpp
  ${FLEXROUTE_HOME}/src/db/obj/frInst.cpp
  ${FLEXROUTE_HOME}/src/db/obj/frPin.cpp
  ${FLEXROUTE_HOME}/src/db/obj/frShape.cpp
  ${FLEXROUTE_HOME}/src/db/obj/frVia.cpp
  ${FLEXROUTE_HOME}/src/db/taObj/taTrack.cpp
  ${FLEXROUTE_HOME}/src/db/taObj/taShape.cpp
  ${FLEXROUTE_HOME}/src/db/infra/frBox.cpp
  ${FLEXROUTE_HOME}/src/db/infra/frTime.cpp
  ${FLEXROUTE_HOME}/src/db/infra/frTime_helper.cpp
  ${FLEXROUTE_HOME}/src/db/infra/frPoint.cpp
  ${FLEXROUTE_HOME}/src/db/drObj/drVia.cpp
  ${FLEXROUTE_HOME}/src/db/drObj/drShape.cpp
  )


set (FLEXROUTE_HEADER
  ${FLEXROUTE_HOME}/src/drc/frDRC.h
  ${FLEXROUTE_HOME}/src/FlexRoute.h
  ${FLEXROUTE_HOME}/src/io/frGuidePrep.h
  ${FLEXROUTE_HOME}/src/io/frAPG.h
  ${FLEXROUTE_HOME}/src/io/frPinPrep.h
  ${FLEXROUTE_HOME}/src/io/io.h
  ${FLEXROUTE_HOME}/src/io/frShapeUtil.h
  ${FLEXROUTE_HOME}/src/io/frUtil.h
  ${FLEXROUTE_HOME}/src/frRegionQuery.h
  ${FLEXROUTE_HOME}/src/frDesign.h
  ${FLEXROUTE_HOME}/src/frBaseTypes.h
  ${FLEXROUTE_HOME}/src/dr/FlexDR.h
  ${FLEXROUTE_HOME}/src/dr/FlexGridGraph.h
  ${FLEXROUTE_HOME}/src/dr/FlexWavefront.h
  ${FLEXROUTE_HOME}/src/dr/FlexRegionQuery.h
  ${FLEXROUTE_HOME}/src/dr/FlexMazeTypes.h
  ${FLEXROUTE_HOME}/src/ta/FlexTA.h
  ${FLEXROUTE_HOME}/src/FlexAccessPattern.h
  ${FLEXROUTE_HOME}/src/global.h
  ${FLEXROUTE_HOME}/src/db/obj/frTrackPattern.h
  ${FLEXROUTE_HOME}/src/db/obj/frPin.h
  ${FLEXROUTE_HOME}/src/db/obj/frInstTerm.h
  ${FLEXROUTE_HOME}/src/db/obj/frMarker.h
  ${FLEXROUTE_HOME}/src/db/obj/frRef.h
  ${FLEXROUTE_HOME}/src/db/obj/frFig.h
  ${FLEXROUTE_HOME}/src/db/obj/frTerm.h
  ${FLEXROUTE_HOME}/src/db/obj/frBoundary.h
  ${FLEXROUTE_HOME}/src/db/obj/frBlock.h
  ${FLEXROUTE_HOME}/src/db/obj/frVia.h
  ${FLEXROUTE_HOME}/src/db/obj/frInst.h
  ${FLEXROUTE_HOME}/src/db/obj/frShape.h
  ${FLEXROUTE_HOME}/src/db/obj/frGuide.h
  ${FLEXROUTE_HOME}/src/db/obj/frBlockObject.h
  ${FLEXROUTE_HOME}/src/db/obj/frInstBlockage.h
  ${FLEXROUTE_HOME}/src/db/obj/frNet.h
  ${FLEXROUTE_HOME}/src/db/obj/frBlockage.h
  ${FLEXROUTE_HOME}/src/db/obj/frCMap.h
  ${FLEXROUTE_HOME}/src/db/taObj/taShape.h
  ${FLEXROUTE_HOME}/src/db/taObj/taFig.h
  ${FLEXROUTE_HOME}/src/db/taObj/taVia.h
  ${FLEXROUTE_HOME}/src/db/taObj/taTrack.h
  ${FLEXROUTE_HOME}/src/db/taObj/taBlockObject.h
  ${FLEXROUTE_HOME}/src/db/taObj/taPin.h
  ${FLEXROUTE_HOME}/src/db/infra/frBox.h
  ${FLEXROUTE_HOME}/src/db/infra/frOrient.h
  ${FLEXROUTE_HOME}/src/db/infra/frPrefRoutingDir.h
  ${FLEXROUTE_HOME}/src/db/infra/frSegStyle.h
  ${FLEXROUTE_HOME}/src/db/infra/frTime.h
  ${FLEXROUTE_HOME}/src/db/infra/frPoint.h
  ${FLEXROUTE_HOME}/src/db/infra/frTransform.h
  ${FLEXROUTE_HOME}/src/db/tech/frTechObject.h
  ${FLEXROUTE_HOME}/src/db/tech/frConstraint.h
  ${FLEXROUTE_HOME}/src/db/tech/frLayer.h
  ${FLEXROUTE_HOME}/src/db/tech/frLookupTbl.h
  ${FLEXROUTE_HOME}/src/db/tech/frViaRuleGenerate.h
  ${FLEXROUTE_HOME}/src/db/tech/frViaDef.h
  ${FLEXROUTE_HOME}/src/db/drObj/drBlockObject.h
  ${FLEXROUTE_HOME}/src/db/drObj/drNet.h
  ${FLEXROUTE_HOME}/src/db/drObj/drFig.h
  ${FLEXROUTE_HOME}/src/db/drObj/drVia.h
  ${FLEXROUTE_HOME}/src/db/drObj/drMarker.h
  ${FLEXROUTE_HOME}/src/db/drObj/drShape.h
  ${FLEXROUTE_HOME}/src/db/drObj/drRef.h
  ${FLEXROUTE_HOME}/src/db/drObj/drAccessPattern.h
  ${FLEXROUTE_HOME}/src/db/drObj/drPin.h
  )

############################################################
# include directories 
############################################################
include_directories(
  ${Boost_INCLUDE_DIR}
  ${DEFLIB_HOME}/def
  ${LEFLIB_HOME}/lef

  ${FLEXROUTE_HOME}/src
)

############################################################
# Create a library 
############################################################
add_library( flexroutelib
  ${FLEXROUTE_SRC}
)

set_target_properties( flexroutelib 
  PROPERTIES OUTPUT_NAME "flexroute"
)

############################################################
# Create an executable
############################################################
add_executable( TritonRoute 
  ${FLEXROUTE_SRC}
)

add_dependencies( TritonRoute def )
add_dependencies( TritonRoute lef )

add_dependencies( flexroutelib def )
add_dependencies( flexroutelib lef )

target_link_libraries( TritonRoute 
  PUBLIC
  def
  lef

  ${ZLIB_LIBRARIES}
  Threads::Threads
  )

############################################################
# Install  
############################################################

install(FILES ${FLEXROUTE_HEADER} DESTINATION include)
install(TARGETS TritonRoute DESTINATION bin)
install(TARGETS TritonRoute DESTINATION lib)


