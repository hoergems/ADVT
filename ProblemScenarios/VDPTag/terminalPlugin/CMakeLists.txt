ADD_TERMINAL_PLUGIN(vdpTagTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagTerminalPlugin.cpp)
target_include_directories(vdpTagTerminalPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
