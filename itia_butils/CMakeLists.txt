cmake_minimum_required(VERSION 2.8.3)
project(itia_butils)

set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 ")

find_package( catkin REQUIRED COMPONENTS
              roscpp
            )

find_package( Boost REQUIRED COMPONENTS system filesystem )

include_directories(include
                    ${catkin_INCLUDE_DIRS} )

catkin_package( INCLUDE_DIRS include 
                CATKIN_DEPENDS roscpp 
                DEPENDS eigen Boost
)

add_library(itia_butils src/itia_butils.cpp )
add_dependencies( itia_butils ${catkin_EXPORTED_TARGETS} )