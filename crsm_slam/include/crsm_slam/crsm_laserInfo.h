/*
 * 	This file is part of CrsmSlam.
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
* 
* Author : Manos Tsardoulias, etsardou@gmail.com
* Organization : AUTH, PANDORA Robotics Team
* */

#ifndef CRSM_LASERINFO_HEADER
#define CRSM_LASERINFO_HEADER

namespace crsm_slam{
	/**
	 @struct CrsmLaserInfo
	 @brief The container of the laser range finder information.
	 **/ 
	struct CrsmLaserInfo{
		int 	laserRays;			//!< The number of rays
		float 	laserMax;			//!< The maximum nominal laser distance (in meters)
		float 	laserAngle;			//!< The total laser angle
		float 	laserAngleBegin;	//!< The minimum rays' angle
		float 	laserAngleEnd;		//!< The maximum rays' angle
	};

}
#endif
