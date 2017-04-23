cmake_minimum_required(VERSION 2.8)
project(local_SLAM)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  rospy
  std_msgs
)


include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native -std=c++0x")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native -std=c++0x")
#SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++0x")
SET(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}
main.cc
ORBextractor.cc
Frame.cc
ORBmatcher.cc
Initializer.cc
Drawer.cc
KeyFrame.cc
MapPoint.cc
Optimizer.cc
Converter.cc
Tracking.cc
LocalMapping.cc
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
)

