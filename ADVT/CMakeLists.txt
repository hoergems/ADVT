cmake_minimum_required (VERSION 3.5.1)
project (ADVT)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

include(GNUInstallDirs)
if(DEFINED ENV{OPPT_SEARCH_PATH})
   message("-- searching for oppt in $ENV{OPPT_SEARCH_PATH}")
   find_package(oppt REQUIRED PATHS $ENV{OPPT_SEARCH_PATH} NO_DEFAULT_PATH)
   message("${oppt_LIBRARY_DIRS}")
else()
   message("-- OPPT_SEARCH_PATH not set. Searching in default locations")
   find_package(oppt REQUIRED)
endif()

find_package(CGAL REQUIRED)

set(HOOT_SRC 
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/ADVT.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/PartitionNode.cpp  
               ${CMAKE_CURRENT_LIST_DIR}/src/PartitionAgent.cpp             
               ${CMAKE_CURRENT_LIST_DIR}/src/DistanceMeasure.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/DiameterEstimator.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/BeliefNode.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/ActionEdge.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/ObservationEdge.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/BeliefNodeData.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/BoundingSphere.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Partition.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/RectanglePartition.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/VoronoiPartition.cpp 
               ${CMAKE_CURRENT_LIST_DIR}/src/VDPPartition.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/LineSegmentPartition.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Tree.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/TreeElement.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/TreeElementData.cpp)
               #${CMAKE_CURRENT_LIST_DIR}/src/PrintTree.cpp)


add_executable(advt
               ${HOOT_SRC})
target_include_directories(advt 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		oppt		
		CGAL)
target_link_libraries(advt PUBLIC 
                      oppt
                      CGAL)
target_compile_features(advt PUBLIC cxx_std_11)
set_target_properties(advt
    PROPERTIES    
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin"
)
