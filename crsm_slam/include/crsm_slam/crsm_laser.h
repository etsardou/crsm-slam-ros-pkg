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

#ifndef CRSM_LASER_HEADER
#define CRSM_LASER_HEADER

#include <vector>
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>

#include "crsm_laserInfo.h"
#include "crsm_laserScan.h"

namespace crsm_slam
{
  /**
    @struct CrsmLaser
    @brief The container of the laser range finder.
   **/ 
  class CrsmLaser
  {
    public:
      //!< This flag becomes true when the first laser scan arrives
      bool initialized;
      //!< Container for the map info in the form of a CrsmLaserInfo structure
      CrsmLaserInfo info;
      //!< Holds the current laserScan in a CrsmLaserScan structure
      CrsmLaserScan scan;
      //!< Laser rays's angles with regard to the robot's orientation
      std::vector<float> angles;

      /**
        @brief Default constructor. Initialises the CrsmLaser::initialised 
        flag to false
       **/	
      CrsmLaser(void);

      /**
        @brief Initialises the CrsmLaserInfo and CrsmLaserScan when the first 
          scan arrives
        @param msg [const sensor_msgs::LaserScanConstPtr&] The 
          sensor_msgs::LaserScan message
        @return void
       **/	
      void initialize(const sensor_msgs::LaserScanConstPtr& msg);
  };

}

#endif
