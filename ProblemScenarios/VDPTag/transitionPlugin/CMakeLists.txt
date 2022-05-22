ADD_TRANSITION_PLUGIN(vdpTagTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagTransitionPlugin.cpp)
target_include_directories(vdpTagTransitionPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
