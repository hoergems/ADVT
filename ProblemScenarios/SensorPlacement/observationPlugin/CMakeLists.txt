ADD_OBSERVATION_PLUGIN(sensorPlacementObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementObservationPlugin.cpp)
target_include_directories(sensorPlacementObservationPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)

