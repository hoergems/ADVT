ADD_INITIAL_BELIEF_PLUGIN(parkingInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingInitialBeliefPlugin.cpp)
target_include_directories(parkingInitialBeliefPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
