##---------------------------------------------------------------------------------------------------------------------
##  MGUI
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

# Macro for tool using ROS components
if(${USE_REC})
	find_package(catkin COMPONENTS )
	if(${catkin_FOUND})
		catkin_package(
			CATKIN_DEPENDS roscpp
		)
		macro(build_ros_tool tool_name tool_files)
			add_definitions(-std=c++11)
			add_executable(${tool_name} ${tool_files})
			target_include_directories(${tool_name} PUBLIC ../include)
			target_link_libraries(${tool_name} LINK_PUBLIC mgui)
			target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
			target_link_libraries(${tool_name} LINK_PUBLIC ${catkin_LIBRARIES})
		endmacro()
		build_ros_tool(receiver_uav receiver_uav/main.cpp)
	endif(${catkin_FOUND})
endif(${USE_REC})

# Macro for tool without using ROS components
if(${USE_GUI})
	macro(build_tool tool_name tool_files)
		find_package(Qt5Widgets REQUIRED)
		find_package(Qt5PrintSupport REQUIRED)
		find_package(Qt5Core REQUIRED)
		add_definitions(-std=c++11)
		add_executable(${tool_name} ${tool_files})
		target_include_directories(${tool_name} PUBLIC ../include)
		target_link_libraries(${tool_name} LINK_PUBLIC mgui)
		target_link_libraries(${tool_name} LINK_PUBLIC Qt5::Widgets Qt5::PrintSupport Qt5::Core)
	endmacro()
	build_tool(gui_uav gui_uav/main.cpp)
	build_tool(gui_pcl gui_pcl/main.cpp)
endif(${USE_GUI})
