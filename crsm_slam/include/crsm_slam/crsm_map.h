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

#ifndef CRSM_MAP_HEADER
#define CRSM_MAP_HEADER

#include "crsm_mapInfo.h"
#include <map>
#include <iostream>

namespace crsm_slam
{

  enum CrsmDirection
  {
    RIGHT,
    LEFT,
    UP,
    DOWN
  };

  struct CrsmExpansion
  {
    std::map<CrsmDirection,long int> expansions;
  };

  /**
    @struct CrsmMap
    @brief Holds the occupancy grid map
   **/ 
  class CrsmMap
  {
    public:
      //!< The container of the CRSM SLAM map. Each cell holds an occupancy 
      //probability. In the specific implementation 0 states an occupied pixel 
      //and 255 a free one.
      unsigned char **p;

      CrsmMapInfo info;

      /**
        @brief Default constructor
       **/
      CrsmMap(){};

      /**
        @brief Constructor. Initialises the probability container with a 
          specific map  size
        @param size_ [unsigned int] The map's size in pixels
       **/
      CrsmMap(unsigned int size_);

      /**
        @brief Reallocs the map to a specific direction
        @params exp [crsm_slam::CrsmExpansion] The expansion to be made
        @return void
       **/
      void expandMap(CrsmExpansion exp);
  };

}
#endif
