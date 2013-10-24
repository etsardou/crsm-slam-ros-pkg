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

#include <crsm_slam/crsm_map.h>

namespace crsm_slam{

/**
@brief Constructor. Initialises the probability container with a specific map  size
@param size_ [unsigned int] The map's size in pixels
**/
CrsmMap::CrsmMap(unsigned int size_){
	info.size=size_;
	p=new unsigned char *[info.size];
	for(unsigned int i=0;i<info.size;i++)
		p[i]=new unsigned char [info.size];

	for(unsigned int i=0;i<info.size;i++)
		for(unsigned int j=0;j<info.size;j++)
			p[i][j]=127;
			
	info.xmin=0;
	info.xmax=0;
	info.ymin=0;
	info.ymax=0;
}

/**
@brief Updates map limits
@return void
**/
void CrsmMap::findEffectiveMapLimits(void){
	int xmin=info.size/2-100;
	int xmax=info.size/2+100;
	int ymin=info.size/2-100;
	int ymax=info.size/2+100;
	bool isAllGray=false;
	while(!isAllGray){
		isAllGray=true;
		for(int i=xmin;i<xmax;i++)
			if(p[i][ymin]!=127){
				isAllGray=false;
				break;
			}
		if(!isAllGray){
			ymin-=100;
			continue;
		}
		
		for(int i=xmin;i<xmax;i++)
			if(p[i][ymax]!=127){
				isAllGray=false;
				break;
			}
		if(!isAllGray){
			ymax+=100;
			continue;
		}
		
		for(int i=ymin;i<ymax;i++)
			if(p[xmin][i]!=127){
				isAllGray=false;
				break;
			}
		if(!isAllGray){
			xmin-=100;
			continue;
		}

		for(int i=ymin;i<ymax;i++)
			if(p[xmax][i]!=127){
				isAllGray=false;
				break;
			}
		if(!isAllGray){
			xmax+=100;
			continue;
		}
	}
	info.xmin=xmin;
	info.ymin=ymin;
	info.xmax=xmax;
	info.ymax=ymax;
}

}
