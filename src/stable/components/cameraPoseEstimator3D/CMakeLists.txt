cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cameraPoseEstimator3D)

#find_package(PCL 1.7 REQUIRED)
find_package(cvsba REQUIRED)

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
    ${QVTK_INCLUDE_DIR}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
)

include_directories(${INCLUDE_DIRS})

SET(LIBRARIES
    ${CMAKE_THREAD_LIBS_INIT}
    JderobotInterfaces
    jderobotutil
    parallelIce
    geometry
    ${ZeroCIce_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${QT_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${PCL_LIBRARIES}
    ${QVTK_LIBRARY}
    ${cvsba_LIBS}
)

set  (project_SOURCES main.cpp pclviewer.cpp Transform.cpp UConversion.cpp myprogeo.cpp)
set  (project_HEADERS pclviewer.h)
set  (project_FORMS   pclviewer.ui)
QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

add_executable ( cameraPoseEstimator3D ${project_SOURCES} ${project_FORMS_HEADERS} ${project_HEADERS_MOC})
target_link_libraries (cameraPoseEstimator3D ${LIBRARIES})

add_executable (pcl_visualizer_demo ./demos/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${LIBRARIES})

add_executable (pcd_viewer ./demos/pcd_viewer.cpp)
target_link_libraries (pcd_viewer ${LIBRARIES})

add_executable (cloud_viewer ./demos/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${LIBRARIES})
