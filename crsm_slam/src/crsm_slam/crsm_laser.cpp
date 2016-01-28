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

#include <crsm_slam/crsm_laser.h>

namespace crsm_slam
{

  /**
    @brief Default constructor. Initialises the CrsmLaser::initialised flag 
      to false
   **/
  CrsmLaser::CrsmLaser(void)
  {
    initialized = false;
  }

  /**
    @brief Initialises the CrsmLaserInfo and CrsmLaserScan when the first scan 
      arrives
    @param msg [const sensor_msgs::LaserScanConstPtr&] The 
      sensor_msgs::LaserScan message
    @return void
   **/	
  void CrsmLaser::initialize(const sensor_msgs::LaserScanConstPtr& msg)
  {
    ROS_ASSERT(angles.size() == 0);
    ROS_ASSERT(msg->angle_min < msg->angle_max);
    float angle = msg->angle_min;
    int count = 0;

    while(count < msg->ranges.size())
    {
      angles.push_back(angle);
      count++;
      angle = msg->angle_min + msg->angle_increment * count;
    }

    initialized = true;

    info.laserRays = msg->ranges.size();
    info.laserMax = msg->range_max;
    info.laserAngle = msg->angle_max - msg->angle_min;
    info.laserAngleBegin = msg->angle_min;
    info.laserAngleEnd = msg->angle_max;

    scan = CrsmLaserScan(msg->ranges.size());

    ROS_INFO("[CrsmSlam] CRSM Laser initialized");
  }
}
