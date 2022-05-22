ADD_HEURISTIC_PLUGIN(vdpTagHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagHeuristicPlugin.cpp)
target_include_directories(vdpTagHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
