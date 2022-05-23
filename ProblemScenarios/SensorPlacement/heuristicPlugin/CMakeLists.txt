ADD_HEURISTIC_PLUGIN(sensorPlacementHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementHeuristicPlugin.cpp)
target_include_directories(sensorPlacementHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)
