ADD_OBSERVATION_PLUGIN(vdpTagObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagObservationPlugin.cpp)
target_include_directories(vdpTagObservationPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
