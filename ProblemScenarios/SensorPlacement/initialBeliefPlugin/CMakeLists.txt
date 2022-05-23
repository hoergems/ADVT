ADD_INITIAL_BELIEF_PLUGIN(sensorPlacementInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementInitialBeliefPlugin.cpp)
target_include_directories(sensorPlacementInitialBeliefPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)
