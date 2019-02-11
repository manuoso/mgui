###############################################################
######                                                   ######
######		Author: Manuel P. J. (aka. manuoso)          ######
######                                                   ######
###############################################################

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -std=c++11 -pthread -g -O0")

set(BUILD_SHARED_LIBS OFF)

## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

## Load PCL
find_package(PCL 1.7 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

## Load Marble
find_package(Marble REQUIRED)

# Load rapidjson
find_package(RapidJSON REQUIRED)

## Load fastcom
find_package(fastcom REQUIRED)

## Load Qt
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Core REQUIRED)

set(QT5_LIBRARIES Qt5::Widgets Qt5::PrintSupport Qt5::Core)

# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTORCC ON)
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(SOURCES_QT
# 	src/guis/uav_gui.cpp
# 	src/guis/pclviewer_gui.cpp
# )

# set(HEADERS_QT
# 	include/mgui/guis/uav_gui.h
# 	include/mgui/guis/pclviewer_gui.h
# )

set(UI_SOURCES_QT
	include/mgui/guis/uav_gui.ui
	include/mgui/guis/pclviewer_gui.ui
)

# QT5_WRAP_CPP(MOC_SRCS ${HEADERS_QT})
QT5_WRAP_UI(UI_HEADERS ${UI_SOURCES_QT})

#########################################
######			Executable		   ######
#########################################
        
# Set SOURCES variable
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Set HEADERS variable
# file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

# Includes directories
GroupSources(src)

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

target_link_libraries(${PROJECT_NAME} ${QT5_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RAPIDJSON_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PRIVATE fastcom::fastcom)
