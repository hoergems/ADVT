ADD_HEURISTIC_PLUGIN(parkingHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingHeuristicPlugin.cpp)
target_include_directories(parkingHeuristicPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
