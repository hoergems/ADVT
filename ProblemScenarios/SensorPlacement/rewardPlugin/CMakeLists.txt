ADD_REWARD_PLUGIN(sensorPlacementRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementRewardPlugin.cpp)
target_include_directories(sensorPlacementRewardPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)
