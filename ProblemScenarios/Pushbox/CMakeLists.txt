include(${CMAKE_CURRENT_LIST_DIR}/initialBeliefPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/transitionPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/observationPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/rewardPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/terminalPlugin/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/heuristicPlugin/CMakeLists.txt)

target_include_directories(pushboxInitialBeliefPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
target_include_directories(pushboxTransitionPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
target_include_directories(pushboxObservationPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
target_include_directories(pushboxRewardPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
target_include_directories(pushboxTerminalPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
target_include_directories(pushboxHeuristicPlugin PRIVATE ${CMAKE_CURRENT_LIST_DIR}/shared/)
