ADD_TRANSITION_PLUGIN(sensorPlacementTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementTransitionPlugin.cpp)
target_include_directories(sensorPlacementTransitionPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)
