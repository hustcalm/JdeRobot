cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_visualizer_viewports)

#find_package(PCL 1.7 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

SET(INCLUDE_DIRS
    # /usr/local/include/jderobot
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
)

SET(LIBRARIES
    ${CMAKE_THREAD_LIBS_INIT}
    JderobotInterfaces
    jderobotutil
    parallelIce
    ${ZeroCIce_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${QT_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${LIBRARIES})

add_executable (pcd_viewer pcd_viewer.cpp)
target_link_libraries (pcd_viewer ${LIBRARIES})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${LIBRARIES})
