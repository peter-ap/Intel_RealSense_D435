cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(intel_cloudviewer)

find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${realsense_INCLUDE_DIR})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable (intel_cloudviewer main.cpp)
target_link_libraries (intel_cloudviewer ${PCL_LIBRARIES})
target_include_directories(intel_cloudviewer PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(intel_cloudviewer ${realsense2_LIBRARY})
target_link_libraries( intel_cloudviewer ${OpenCV_LIBS} )

