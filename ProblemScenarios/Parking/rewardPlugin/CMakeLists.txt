ADD_REWARD_PLUGIN(parkingRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingRewardPlugin.cpp)
target_include_directories(parkingRewardPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
