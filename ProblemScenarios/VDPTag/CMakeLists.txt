include(${CMAKE_CURRENT_LIST_DIR}/initialBeliefPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/transitionPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/observationPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/rewardPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/terminalPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/heuristicPlugin/CMakeLists.txt)

target_include_directories(vdpTagInitialBeliefPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
target_include_directories(vdpTagTransitionPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
target_include_directories(vdpTagObservationPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
target_include_directories(vdpTagRewardPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
target_include_directories(vdpTagTerminalPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
target_include_directories(vdpTagHeuristicPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/model/)
