ADD_INITIAL_BELIEF_PLUGIN(vdpTagInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagInitialBeliefPlugin.cpp)
target_include_directories(vdpTagInitialBeliefPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
