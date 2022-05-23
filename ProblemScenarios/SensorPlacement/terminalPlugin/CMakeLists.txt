ADD_TERMINAL_PLUGIN(sensorPlacementTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementTerminalPlugin.cpp)
target_include_directories(sensorPlacementTerminalPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SensorPlacement/shared>)
