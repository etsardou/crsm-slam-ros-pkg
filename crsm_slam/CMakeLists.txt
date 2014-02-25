cmake_minimum_required(VERSION 2.8.3)
project(crsm_slam)

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		tf
		nav_msgs
		geometry_msgs
		sensor_msgs
	    )


catkin_package(
    CATKIN_DEPENDS
      roscpp 
      tf
      nav_msgs
      geometry_msgs
      sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES
      crsm_laser
      crsm_map
      crsm_slam
)

include_directories(include
  ${catkin_INCLUDE_DIRS} 
)

set(CMAKE_BUILD_TYPE Release)

############### crsm_laser ###############
add_library(crsm_laser src/crsm_slam/crsm_laser.cpp)
target_link_libraries(crsm_laser ${catkin_LIBRARIES} )

################ crsm_map ################
add_library(crsm_map src/crsm_slam/crsm_map.cpp)
target_link_libraries(crsm_map ${catkin_LIBRARIES} )

################ crsm_slam ################
add_library(crsm_slam src/crsm_slam/crsm_slam.cpp)
target_link_libraries(crsm_slam 
		      ${catkin_LIBRARIES}
		      crsm_laser
		      crsm_map
)

############## crsm_slam_node ##############
add_executable(crsm_slam_node  src/crsm_slam/crsm_slam_node.cpp)
target_link_libraries(crsm_slam_node 
		      ${catkin_LIBRARIES}
		      crsm_slam
)

# Install headers
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

# Install files
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install libraries
install(TARGETS crsm_laser crsm_map crsm_slam
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install excecutables
install(TARGETS crsm_slam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

