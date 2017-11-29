cmake_minimum_required(VERSION 2.8.3)
project(itia_rutils)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  itia_msgs
  std_msgs
  itia_futils
  cmake_modules
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_rutils
  CATKIN_DEPENDS roscpp std_msgs cmake_modules itia_msgs itia_futils
  DEPENDS  Eigen3 orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  )

add_library(itia_rutils src/itia_rutils.cpp )
add_dependencies( itia_rutils ${catkin_EXPORTED_TARGETS} )
