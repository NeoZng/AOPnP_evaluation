cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
# if you want to use MKL, uncomment the above line and make sure you have set the MKLROOT environment variable

# -Ofast -march=native -flto can improve our performance(AOPnP) for Eigen
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall") # remove -g to improve performance, add -pg to enable profiling
set(cmake_export_compile_commands True)



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# set your opengv lib and include dir here
set(OpenGV_LIBS "/home/neo/test/opengv/build/lib/libopengv.a")
include_directories("/home/neo/test/opengv/include")

# set your opencv dir here, make sure its version is higher than 4.5!!!!!!!
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

set(Eigen3_DIR "/usr/lib/cmake/eigen3")
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_package()

add_library(vins_lib
    src/cpnp_solver/cpnp.cpp
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenGV_LIBS})


add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib) 

# add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
# target_link_libraries(kitti_odom_test vins_lib) 

