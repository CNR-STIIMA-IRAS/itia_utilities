cmake_minimum_required(VERSION 2.8.3)
project(itia_futils)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package( catkin REQUIRED COMPONENTS
              roscpp
            )

catkin_package( INCLUDE_DIRS include
                LIBRARIES itia_futils
# 				      CATKIN_DEPENDS roscpp
# 				      DEPENDS 
              )

include_directories( ${catkin_INCLUDE_DIRS}	
                     include)

SET(  itia_futils_sources
      src/itia_futils.cpp )    
	
add_library( itia_futils ${itia_futils_sources} )

add_dependencies( itia_futils ${catkin_EXPORTED_TARGETS} )


