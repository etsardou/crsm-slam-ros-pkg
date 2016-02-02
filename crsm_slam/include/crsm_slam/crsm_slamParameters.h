/*
 This file is part of CrsmSlam.
 CrsmSlam is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 CrsmSlam is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with CrsmSlam.  If not, see <http://www.gnu.org/licenses/>.

 Author : Manos Tsardoulias, etsardou@gmail.com
 Organization : AUTH, PANDORA Robotics Team
*/

#ifndef CRSM_SLAM_PARAMS_HEADER
#define CRSM_SLAM_PARAMS_HEADER

#define pi 3.141592654
#define pi_double 6.283185308

#include <cstring>

namespace crsm_slam
{
  /**
    @class CrsmSlamParameters
    @brief Contains the parameters needed to execute CRSM slam. 
      These parameters are dynamically loaded from the file 
      crsm_slamParameters.yaml
   **/ 
  struct CrsmSlamParameters
  {
    int disparity; //!< Disparity of mutation in pixels at hill climbing
    int map_size; //!< Map size of initial allocated map
    //!< [OC]cupancy [G]rid [D]imentionality - the width and height in meters 
    //of a pixel
    double ocgd; 
    double density; //!< Map update density (0-127)
    double obstacle_density; //!< Coefficient for obstacle update density (0+)
    //!< Scan density lower boundary for a scan-part identification
    double scan_selection_meters; 
    int max_hill_climbing_iterations; //!< Maximum RRHC iterations
    //!< Translation in x axis of laser in comparison to robot center
    double dx_laser_robotCenter; 

    //!< The occupancy grid map publishing frequency
    double occupancy_grid_map_freq; 
    double robot_pose_tf_freq; //!< The robot pose publishing frequency
    double trajectory_freq; //!< The trajectory publishing frequency

    //!< The desired number of picked rays [algorithm specific]
    int desired_number_of_picked_rays;
    double robot_width; //!< The robot width
    double robot_length; //!< The robot length

    //!< The occupancy grid publishing topic
    std::string occupancy_grid_publish_topic; 
    //!< The trajectory publishing topic
    std::string robot_trajectory_publish_topic;
    //!< The trajectory frame ID
    std::string trajectory_publisher_frame_id;
    std::string laser_subscriber_topic; //!< The laser subscriber topic
    std::string pose_publish_topic;

    bool publish_tf; //!< whether to publish tf or not

    //!< Holds the base footprint frame - (x,y,yaw)
    std::string base_footprint_frame; 
    std::string base_frame; //!< Holds the base frame
    std::string map_frame; //!< Holds the map frame
  };

}
#endif
