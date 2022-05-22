ADD_TRANSITION_PLUGIN(parkingTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingTransitionPlugin.cpp)
target_include_directories(parkingTransitionPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
