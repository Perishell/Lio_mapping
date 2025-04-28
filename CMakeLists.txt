cmake_minimum_required(VERSION 2.8.3)
project(fusion_slam)

SET(CMAKE_BUILD_TYPE "Release")

# ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# add_definitions("-DENABLE_LIVOX")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  octomap_msgs
  pcl_ros
  tf
  # livox_ros_driver
  message_generation
  eigen_conversions
  slog
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
# find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

#GLOG (neccessary)
# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
# find_package(GLOG REQUIRED) # QUIET)
# include_directories(${GLOG_INCLUDE_DIRS})
# message("GLOG [OK]:" ${GLOG_INCLUDE_DIRS})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/common
  ${GeographicLib_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
  r_map_info_msg.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
  save_pose.srv
  gps_to_map.srv
  map_to_gps.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime slog octomap_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# add_subdirectory(src)
# add_subdirectory(app)

# add_executable(${PROJECT_NAME}_mapping src/lioMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
# target_link_libraries(${PROJECT_NAME}_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} gtsam glog ORB_LIBS)
# target_include_directories(${PROJECT_NAME}_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/lioMapping.cpp
        src/optimization.cpp
        include/ikd-Tree/ikd_Tree.cpp
        src/preprocess.cpp
        src/slog/LogMgmt.cpp
        include/Scancontext/Scancontext.cpp
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${octomap_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
        # tbb
        gtsam
        # glog
        # yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(runMapping src/runMapping.cpp)
target_link_libraries(runMapping
        ${PROJECT_NAME}
        )

# 添加 backward-cpp 源代码目录
add_subdirectory(./thrid_party/backward-cpp)
add_executable(localMapping src/localMapping.cpp ${BACKWARD_ENABLE})
add_backward(localMapping)
target_link_libraries(localMapping
        ${PROJECT_NAME}
        ${GeographicLib_LIBRARIES}
        Backward::Interface
        )
