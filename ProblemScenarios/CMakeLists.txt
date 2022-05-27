cmake_minimum_required (VERSION 2.6)
project(ProblemScenarios)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O3 -msse2 -mfpmath=sse -DEIGEN_NO_DEBUG -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -mfpmath=sse -DEIGEN_NO_DEBUG")

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release) 
endif()

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
if (NOT CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO AND NOT CMAKE_BUILD_TYPE MATCHES DEBUG AND NOT CMAKE_BUILD_TYPE MATCHES RELEASE)
   message(FATAL_ERROR "${CMAKE_BUILD_TYPE} is not a valid build type. Supported types are Debug, Release and RelWithDebInfo (default)")
endif()

add_definitions(-DUSE_DOUBLE_PRECISION=true)
include(FindPkgConfig)
include(GNUInstallDirs)
set(ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

if(DEFINED ENV{OPPT_SEARCH_PATH})
   message("-- searching for oppt in $ENV{OPPT_SEARCH_PATH}")
   find_package(oppt REQUIRED PATHS $ENV{OPPT_SEARCH_PATH} NO_DEFAULT_PATH)
   message("${oppt_LIBRARY_DIRS}")
else()
   message("-- OPPT_SEARCH_PATH not set. Searching in default locations")
   find_package(oppt REQUIRED)
endif()


SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_LIST_DIR})
FOREACH(subdir ${SUBDIRS})  
  message("Check ${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
    include(${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt)
  endif()  
ENDFOREACH()

install(FILES ${CMAKE_CURRENT_LIST_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt)
install(DIRECTORY ${ROOT_PATH}/models DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)
