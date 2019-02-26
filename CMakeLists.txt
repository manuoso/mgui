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

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(mgui VERSION 0.1 LANGUAGES C CXX)

##########################################
######		Configure project		######
##########################################

# Communications with Fastcom
set(USE_FASTCOM ON)
set(USE_ROS OFF)

# GUIs
set(USE_GUI ON)

# Receivers
set(USE_REC ON)

# Others
set(BUILD_TOOLS ON)
set(USE_CATKIN ON)

#########################################
###### Loading 3rd party libraries ######
#########################################

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND MGUI_DEFINITIONS "")
set(MGUI_COMPILE_OPTIONS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -Wall -g -O0")

#########################################
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

#########################################
## Load PCL
if(${USE_GUI})
	find_package(PCL 1.8 QUIET REQUIRED)
	if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
		MESSAGE( STATUS "Adding properly PCL and its dependencies")
		add_definitions(-DENABLE_PCL)
		add_definitions(${PCL_DEFINITIONS})
		list(APPEND MGUI_DEFINITIONS "MGUI_USE_PCL")
	else()
		MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
	endif()
endif(${USE_GUI})

#########################################
## Load Marble
if(${USE_GUI})
	find_package(Marble REQUIRED)
	list(APPEND MGUI_DEFINITIONS "MGUI_USE_MARBLE")
endif(${USE_GUI})

#########################################
# Load rapidjson
find_package(RapidJSON REQUIRED)
list(APPEND MGUI_DEFINITIONS "MGUI_USE_RAPIDJSON")

#########################################
## Load fastcom
if(${USE_FASTCOM})
	find_package(fastcom REQUIRED)
	list(APPEND MGUI_DEFINITIONS "MGUI_USE_FASTCOM")
endif(${USE_FASTCOM})

#########################################
## Load motion_planning
if(${USE_GUI})
	find_package(motion_planning REQUIRED)
	list(APPEND MGUI_DEFINITIONS "MGUI_USE_MOTION_PLANNIG")
endif(${USE_GUI})

#########################################
## Load Qt5
if(${USE_GUI})
	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5PrintSupport REQUIRED)
	find_package(Qt5Core REQUIRED)
	MESSAGE( STATUS "Using Qt5")
	list(APPEND MGUI_DEFINITIONS "MGUI_USE_QT5")

	set(QT5_LIBRARIES Qt5::Widgets Qt5::PrintSupport Qt5::Core)

	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	set(CMAKE_AUTOMOC ON)
	set(CMAKE_AUTOUIC ON)
	# set(CMAKE_AUTORCC ON)

	# set(SOURCES_QT
	# 	src/guis/uav_gui.cpp
	# 	src/guis/pclviewer_gui.cpp
	# )

	# set(HEADERS_QT
	# 	include/mgui/guis/uav_gui.h
	# 	include/mgui/guis/pclviewer_gui.h
	# )

	# set(UI_SOURCES_QT
	# 	include/mgui/guis/uav_gui.ui
	# 	include/mgui/guis/pclviewer_gui.ui
	# )

	# QT5_WRAP_CPP(MOC_SRCS ${HEADERS_QT})
	# QT5_WRAP_UI(UI_HEADERS ${UI_SOURCES_QT})	
endif()

#########################################
# catking
if(${USE_CATKIN})
	find_package(catkin REQUIRED roscpp uav_abstraction_layer)
	if(${catkin_FOUND})
		catkin_package(
			CATKIN_DEPENDS roscpp uav_abstraction_layer
			INCLUDE_DIRS include
		)
		list(APPEND MGUI_DEFINITIONS "MGUI_USE_ROS")
	endif(${catkin_FOUND})
endif(${USE_CATKIN})

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
if(${USE_GUI})
	file(GLOB_RECURSE MGUI_GUIS_SOURCE_FILES "src/guis/*.cpp" "src/guis/*.c")
	file(GLOB_RECURSE MGUI_GUIS_HEADER_FILES "include/mgui/guis/*.h" "include/mgui/guis/*.inl")
endif(${USE_GUI})

if(${USE_REC})
	file(GLOB_RECURSE MGUI_REC_SOURCE_FILES "src/receivers/*.cpp" "src/receivers/*.c")
	file(GLOB_RECURSE MGUI_REC_HEADER_FILES "include/mgui/receivers/*.h" "include/mgui/receivers/*.inl")
endif(${USE_REC})

file(GLOB_RECURSE MGUI_LOG_SOURCE_FILES "src/LogTray.cpp")
file(GLOB_RECURSE MGUI_LOG_HEADER_FILES "include/LogTray*.h")

add_library(${PROJECT_NAME} SHARED ${MGUI_GUIS_HEADER_FILES} ${MGUI_GUIS_SOURCE_FILES} ${MGUI_REC_HEADER_FILES} ${MGUI_REC_SOURCE_FILES} ${MGUI_LOG_HEADER_FILES} ${MGUI_LOG_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${MGUI_DEFINITIONS})
string(REPLACE " " ";" MGUI_COMPILE_OPTIONS_REPLACED_FLAGS ${MGUI_COMPILE_OPTIONS})

target_compile_options(${PROJECT_NAME} PUBLIC ${MGUI_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

#########################################
# target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

target_include_directories(${PROJECT_NAME} PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RAPIDJSON_LIBRARIES})

if(${USE_FASTCOM})
	target_link_libraries(${PROJECT_NAME} PRIVATE fastcom::fastcom)
endif(${USE_FASTCOM})

if(${USE_GUI})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${QT5_LIBRARIES})
	target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Marble)
	target_link_libraries(${PROJECT_NAME} PRIVATE motion_planning::motion_planning)
endif(${USE_GUI})

if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES})
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif(${catkin_FOUND})

#########################################
######			Documentation	   ######
#########################################

# find_package(Doxygen)
# if(DOXYGEN_FOUND)
# 	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
# 	add_custom_target(	doc_rgbd_tools
# 						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
# 						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# 						COMMENT "Generating API documentation with Doxygen" VERBATIM )
# endif(DOXYGEN_FOUND)

#########################################
######		   	Tools 		   	   ######
#########################################

if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			Examples 		   ######
#########################################

# if(${BUILD_EXAMPLES})
# 	add_subdirectory(examples)
# endif(${BUILD_EXAMPLES})

#########################################
######			INSTALL 		   ######
#########################################

include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS mgui
        EXPORT mguiTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    mguiConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS mgui NAMESPACE mgui:: FILE mguiTargets.cmake)

# This generates mguiTargets.cmake
install(EXPORT mguiTargets
    FILE mguiTargets.cmake
    NAMESPACE mgui::
    DESTINATION lib/cmake/mgui
    )

configure_file(mguiConfig.cmake.in mguiConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mguiConfig.cmake"
        DESTINATION lib/cmake/mgui
        )

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
