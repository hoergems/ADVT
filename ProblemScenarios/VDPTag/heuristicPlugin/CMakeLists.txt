ADD_HEURISTIC_PLUGIN(vdpTagHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagHeuristicPlugin.cpp)
target_include_directories(vdpTagHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
#target_compile_options(vdpTagHeuristicPlugin PRIVATE ${CMAKE_CXX_FLAGS_RELEASE})
#add_compile_options(${CMAKE_CXX_FLAGS_RELEASE})
#set_target_properties(vdpTagHeuristicPlugin PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
