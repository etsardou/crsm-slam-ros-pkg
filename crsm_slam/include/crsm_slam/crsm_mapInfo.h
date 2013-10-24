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
		unsigned int size;	//!< The map container size
		int xmax;			//!< Map limits : maximum X
		int xmin;			//!< Map limits : minimum X
		int ymax;			//!< Map limits : maximum Y
		int ymin;			//!< Map limits : minimum Y
		
		/**
		@brief Default constructor. Initialises the CrsmMapInfo variables to 0.
		**/
		CrsmMapInfo(){
			size=0;
			xmax=xmin=ymax=ymin=0;
		}
	};

}

#endif
