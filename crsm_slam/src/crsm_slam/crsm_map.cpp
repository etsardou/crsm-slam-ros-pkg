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

#include <crsm_slam/crsm_map.h>

namespace crsm_slam
{

  /**
    @brief Constructor. Initialises the probability container with a specific 
      map  size
    @param size_ [unsigned int] The map's size in pixels
   **/
  CrsmMap::CrsmMap(unsigned int size_)
  {
    info.width = size_;
    info.height = size_;

    info.originx = size_ / 2;
    info.originy = size_ / 2;

    p = new unsigned char *[info.width];
    for(unsigned int i = 0 ; i < info.width ; i++)
    {
      p[i] = new unsigned char [info.height];
    }

    for(unsigned int i = 0 ; i < info.width ; i++)
    {
      for(unsigned int j = 0 ; j < info.height ; j++)
      {
        p[i][j] = 127;
      }
    }
  }

  /**
    @brief Reallocs the map to a specific direction
    @params exp [crsm_slam::CrsmExpansion] The expansion to be made
    @return void
   **/
  void CrsmMap::expandMap(CrsmExpansion expansion)
  {
    int newOriginx = info.originx+expansion.expansions[LEFT];
    int newOriginy = info.originy+expansion.expansions[UP];

    int newWidth = info.width + expansion.expansions[LEFT] + 
      expansion.expansions[RIGHT];
    int newHeight = info.height + expansion.expansions[UP] + 
      expansion.expansions[DOWN];
    
    unsigned char ** newMap;
    newMap = new unsigned char *[newWidth];
    for(unsigned int i = 0 ; i < newWidth ; i++)
    {
      newMap[i] = new unsigned char[newHeight];
    }
    for(unsigned int i = 0 ; i < newWidth ; i++)
    {
      for(unsigned int j = 0 ; j < newHeight ; j++)
      {
        newMap[i][j]=127;
      }
    }

    for(unsigned int i = 0 ; i < info.width ; i++)
    {
      for(unsigned int j = 0 ; j < info.height ; j++)
      {
        newMap[i + expansion.expansions[LEFT]][j + expansion.expansions[UP]] =
          p[i][j];
      }
    }

    for(unsigned int i = 0 ; i < info.width ; i++)
    {
      delete [] p[i];
    }
    delete [] p;

    p = newMap;
    info.width = newWidth;
    info.height = newHeight;

    info.originx = newOriginx;
    info.originy = newOriginy;
  }

}
