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

#ifndef CRSM_MAPINFO_HEADER
#define CRSM_MAPINFO_HEADER

namespace crsm_slam{
	/**
	 @struct CrsmMapInfo
	 @brief Holds the occupancy grid map information
	 **/ 
	struct CrsmMapInfo{

		unsigned int width;	//!< The map container width
		unsigned int height;	//!< The map container height
		
		unsigned int originx;	//!< The start point of mapping - x coord
		unsigned int originy;	//!< The start point of mapping - y coord
		
		/**
		@brief Default constructor. Initialises the CrsmMapInfo variables to 0.
		**/
		CrsmMapInfo(){
			width=height=0;
			originx=originy=0;
		}
	};

}

#endif
