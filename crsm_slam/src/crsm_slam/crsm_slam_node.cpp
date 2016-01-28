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

#include "crsm_slam/crsm_slam.h"

/**
  @brief Main node function
  @param argc [int] Number of input arguments
  @param argv [char **] The input arguments
 **/
int main (int argc, char **argv)
{
  ros::init(argc,argv,"crsm_slam_node",ros::init_options::NoSigintHandler);
  crsm_slam::CrsmSlam slam(argc,argv);

  ROS_INFO("[CrsmSlam] CRSM Slam Node initialised");

  // subscribe to laser 
  slam.startLaserSubscriber();

  // publish map and trajectory
  slam.startOGMPublisher();
  slam.startTrajectoryPublisher();

  ros::spin();
  return 0;
}
