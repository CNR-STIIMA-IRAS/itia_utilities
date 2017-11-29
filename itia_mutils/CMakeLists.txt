cmake_minimum_required(VERSION 2.8.3)
project(itia_mutils)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  itia_rutils
)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_mutils
  CATKIN_DEPENDS roscpp std_msgs cmake_modules itia_rutils
  DEPENDS  Eigen3 orocos_kdl Boost
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(itia_mutils src/${PROJECT_NAME}/polynomial_utils.cpp )

add_executable(itia_mutils_node src/itia_mutils_node.cpp)
target_link_libraries(itia_mutils_node
                      itia_mutils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${Eigen3_LIBRARIES}
                      ${orocos_kdl_LIBRARIES}
                     )

add_executable(test_qp src/eigenqp_test.cpp)
target_link_libraries(test_qp
                      itia_mutils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${Eigen3_LIBRARIES}
                      ${orocos_kdl_LIBRARIES}
                     )
