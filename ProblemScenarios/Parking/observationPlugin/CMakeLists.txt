ADD_OBSERVATION_PLUGIN(parkingObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingObservationPlugin.cpp)
target_include_directories(parkingObservationPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Parking/shared>)
