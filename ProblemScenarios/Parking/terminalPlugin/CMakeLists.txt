ADD_TERMINAL_PLUGIN(parkingTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingTerminalPlugin.cpp)
target_include_directories(parkingTerminalPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
