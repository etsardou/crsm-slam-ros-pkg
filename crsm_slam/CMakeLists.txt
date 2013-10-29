# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(crsm_slam)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp 
                                tf 
)

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS   
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(include
  ${catkin_INCLUDE_DIRS} 
)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)


############### crsm_laser ###############
add_library(crsm_laser src/crsm_slam/crsm_laser.cpp)
target_link_libraries(crsm_laser ${catkin_LIBRARIES} )

################ crsm_map ################
add_library(crsm_map src/crsm_slam/crsm_map.cpp)
target_link_libraries(crsm_map ${catkin_LIBRARIES} )

################ crsm_slam ################
add_library(crsm_slam src/crsm_slam/crsm_slam.cpp)
target_link_libraries(crsm_slam ${catkin_LIBRARIES}
						crsm_laser
						crsm_map
)

############## crsm_slam_node ##############
add_executable(crsm_slam_node  src/crsm_slam/crsm_slam_node.cpp)
target_link_libraries(crsm_slam_node ${catkin_LIBRARIES} 
						crsm_slam
)
