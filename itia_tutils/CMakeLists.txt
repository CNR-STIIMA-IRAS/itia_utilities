cmake_minimum_required(VERSION 2.8.3)
project(itia_tutils)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package( catkin REQUIRED COMPONENTS
              roscpp
              std_msgs
              itia_futils
              itia_rutils
              itia_msgs
              control_msgs
              actionlib
            )

find_package(Eigen3 REQUIRED COMPONENTS)

catkin_package( INCLUDE_DIRS include
                LIBRARIES itia_tutils
                CATKIN_DEPENDS roscpp std_msgs itia_futils itia_msgs control_msgs actionlib itia_rutils
                DEPENDS  Eigen3
              )

include_directories(  include
                      ${catkin_INCLUDE_DIRS}
                      ${Eigen3_INCLUDE_DIRS}
                    ) 

add_library( itia_tutils src/itia_tutils.cpp )
            
add_dependencies( itia_tutils ${catkin_EXPORTED_TARGETS} )

add_executable(trajectory_client src/trajection_action_client.cpp)
add_dependencies(trajectory_client  
  itia_tutils 
  ${catkin_EXPORTED_TARGETS} 
)
target_link_libraries(trajectory_client 
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)