cmake_minimum_required(VERSION 2.8)
project(test_matcher)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native -std=c++0x")
SET(CMAKE_BUILD_TYPE Debug)


find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
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
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
)

