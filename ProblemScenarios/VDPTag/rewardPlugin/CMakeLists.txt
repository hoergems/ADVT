ADD_REWARD_PLUGIN(vdpTagRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/VDPTagRewardPlugin.cpp)
target_include_directories(vdpTagRewardPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/VDPTag/shared>)
