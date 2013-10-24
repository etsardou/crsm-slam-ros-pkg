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

#include "crsm_slam/crsm_slam.h"

using namespace std;

namespace crsm_slam{

/**
@brief Default costructor
@param argc [int] The number of input arguments
@param argv [char **] The input arguments
@return void
**/
CrsmSlam::CrsmSlam(int argc, char **argv){

	robotPose.x=0;
	robotPose.y=0;
	robotPose.theta=0;
	
	updateParameters();

	bestTransformation.dx=bestTransformation.dy=bestTransformation.dth=0;
	map=CrsmMap(slamParams.map_size);
	int initialPatchWidth=slamParams.robot_width/slamParams.ocgd/2;
	int initialPatchLength=slamParams.robot_length/slamParams.ocgd/2;
	for(int i=-initialPatchLength;i<initialPatchLength;i++)
		for(int j=-initialPatchWidth;j<initialPatchWidth;j++)
			map.p[i+map.info.size/2-(int)(slamParams.dx_laser_robotCenter/slamParams.ocgd)][j+map.info.size/2]=200;
	map.findEffectiveMapLimits();

	_pathPublishingTimer = n.createTimer(ros::Duration(1.0/slamParams.trajectory_freq),&CrsmSlam::publishTrajectory,this,false,false);
	
	_robotPosePublishingTimer = n.createTimer(ros::Duration(1.0/slamParams.robot_pose_tf_freq),&CrsmSlam::publishRobotPoseTf,this,false,false);
	_robotPosePublishingTimer.start();
	
	_mapPublishingTimer = n.createTimer(ros::Duration(1.0/slamParams.occupancy_grid_map_freq),&CrsmSlam::publishOGM,this,false,false);
	
}

/**
@brief Reads the CRSM slam parameters from the yaml file and fills the CrsmSlamParameters structure
@return void
**/
void CrsmSlam::updateParameters(void){

	if (n.hasParam("/crsm_slam/occupancy_grid_publish_topic")) 
		n.getParam("/crsm_slam/occupancy_grid_publish_topic", slamParams.occupancy_grid_publish_topic);
	else {
		ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_publish_topic not found. Using Default");
		slamParams.occupancy_grid_publish_topic = "/crsm_slam/map" ;
	}
	
	if (n.hasParam("/crsm_slam/robot_trajectory_publish_topic")) 
		n.getParam("/crsm_slam/robot_trajectory_publish_topic", slamParams.robot_trajectory_publish_topic);
	else {
		ROS_WARN("[CrsmSlam] : Parameter robot_trajectory_publish_topic not found. Using Default");
		slamParams.robot_trajectory_publish_topic = "/crsm_slam/trajectory" ;
	}
	
	if (n.hasParam("/crsm_slam/trajectory_publisher_frame_id")) 
		n.getParam("/crsm_slam/trajectory_publisher_frame_id", slamParams.trajectory_publisher_frame_id);
	else {
		ROS_WARN("[CrsmSlam] : Parameter trajectory_publisher_frame_id not found. Using Default");
		slamParams.trajectory_publisher_frame_id = "world" ;
	}
	
	if (n.hasParam("/crsm_slam/laser_subscriber_topic")) 
		n.getParam("/crsm_slam/laser_subscriber_topic", slamParams.laser_subscriber_topic);
	else {
		ROS_WARN("[CrsmSlam] : Parameter laser_subscriber_topic not found. Using Default");
		slamParams.laser_subscriber_topic = "/crsm_slam/laser_scan" ;
	}
	
	if (n.hasParam("/crsm_slam/world_frame")) 
		n.getParam("/crsm_slam/world_frame", slamParams.world_frame);
	else {
		ROS_WARN("[CrsmSlam] : Parameter world_frame not found. Using Default");
		slamParams.world_frame = "world" ;
	}
	
	if (n.hasParam("/crsm_slam/base_footprint_frame")) 
		n.getParam("/crsm_slam/base_footprint_frame", slamParams.base_footprint_frame);
	else {
		ROS_WARN("[CrsmSlam] : Parameter base_footprint_frame not found. Using Default");
		slamParams.base_footprint_frame = "base_footprint_link" ;
	}
	
	if (n.hasParam("/crsm_slam/base_frame")) 
		n.getParam("/crsm_slam/base_frame", slamParams.base_frame);
	else {
		ROS_WARN("[CrsmSlam] : Parameter base_frame not found. Using Default");
		slamParams.base_frame = "base_link" ;
	}
	
	if (n.hasParam("/crsm_slam/map_frame")) 
		n.getParam("/crsm_slam/map_frame", slamParams.map_frame);
	else {
		ROS_WARN("[CrsmSlam] : Parameter map_frame not found. Using Default");
		slamParams.map_frame = "map" ;
	}
	
	if (n.hasParam("/crsm_slam/laser_frame")) 
		n.getParam("/crsm_slam/laser_frame", slamParams.laser_frame);
	else {
		ROS_WARN("[CrsmSlam] : Parameter laser_frame not found. Using Default");
		slamParams.laser_frame = "laser_link" ;
	}
	
	if (n.hasParam("/crsm_slam/hill_climbing_disparity")) 
		n.getParam("/crsm_slam/hill_climbing_disparity", slamParams.disparity);
	else {
		ROS_WARN("[CrsmSlam] : Parameter hill_climbing_disparity not found. Using Default");
		slamParams.disparity = 40 ;
	}
	
	if (n.hasParam("/crsm_slam/slam_container_size")) 
		n.getParam("/crsm_slam/slam_container_size", slamParams.map_size);
	else {
		ROS_WARN("[CrsmSlam] : Parameter slam_container_size not found. Using Default");
		slamParams.map_size = 4096 ;
	}
	
	if (n.hasParam("/crsm_slam/slam_occupancy_grid_dimentionality")) 
		n.getParam("/crsm_slam/slam_occupancy_grid_dimentionality", slamParams.ocgd);
	else {
		ROS_WARN("[CrsmSlam] : Parameter slam_occupancy_grid_dimentionality not found. Using Default");
		slamParams.ocgd = 0.02 ;
	}
	
	if (n.hasParam("/crsm_slam/map_update_density")) 
		n.getParam("/crsm_slam/map_update_density", slamParams.density);
	else {
		ROS_WARN("[CrsmSlam] : Parameter map_update_density not found. Using Default");
		slamParams.density = 0.8 ;
	}
	
	if (n.hasParam("/crsm_slam/map_update_obstacle_density")) 
		n.getParam("/crsm_slam/map_update_obstacle_density", slamParams.obstacle_density);
	else {
		ROS_WARN("[CrsmSlam] : Parameter map_update_obstacle_density not found. Using Default");
		slamParams.obstacle_density = 3.0 ;
	}
	
	if (n.hasParam("/crsm_slam/scan_density_lower_boundary")) 
		n.getParam("/crsm_slam/scan_density_lower_boundary", slamParams.scan_selection_meters);
	else {
		ROS_WARN("[CrsmSlam] : Parameter scan_density_lower_boundary not found. Using Default");
		slamParams.scan_selection_meters = 0.3 ;
	}
	
	if (n.hasParam("/crsm_slam/max_hill_climbing_iterations")) 
		n.getParam("/crsm_slam/max_hill_climbing_iterations", slamParams.max_hill_climbing_iterations);
	else {
		ROS_WARN("[CrsmSlam] : Parameter max_hill_climbing_iterations not found. Using Default");
		slamParams.max_hill_climbing_iterations = 40000 ;
	}
	
	if (n.hasParam("/crsm_slam/occupancy_grid_map_freq")) 
		n.getParam("/crsm_slam/occupancy_grid_map_freq", slamParams.occupancy_grid_map_freq);
	else {
		ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_map_freq not found. Using Default");
		slamParams.occupancy_grid_map_freq = 1.0 ;
	}
	
	if (n.hasParam("/crsm_slam/robot_pose_tf_freq")) 
		n.getParam("/crsm_slam/robot_pose_tf_freq", slamParams.robot_pose_tf_freq);
	else {
		ROS_WARN("[CrsmSlam] : Parameter robot_pose_tf_freq not found. Using Default");
		slamParams.robot_pose_tf_freq = 5.0 ;
	}
	
	if (n.hasParam("/crsm_slam/trajectory_freq")) 
		n.getParam("/crsm_slam/trajectory_freq", slamParams.trajectory_freq);
	else {
		ROS_WARN("[CrsmSlam] : Parameter trajectory_freq not found. Using Default");
		slamParams.trajectory_freq = 1.0 ;
	}

	// Try to find distance between laser and robot center, else initialize to zero
	tf::StampedTransform tfTransform;
	try {
		_listener.waitForTransform(slamParams.base_frame, slamParams.laser_frame, ros::Time(0), ros::Duration(1));
		_listener.lookupTransform(slamParams.base_frame, slamParams.laser_frame, ros::Time(0), tfTransform);
		tf::Vector3 origin = tfTransform.getOrigin(); 
		slamParams.dx_laser_robotCenter=origin[0];
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("[CrsmSlam] Error in tf : %s", ex.what());
		slamParams.dx_laser_robotCenter=0.2;
	}
	
	if (n.hasParam("/crsm_slam/desired_number_of_picked_rays")) 
		n.getParam("/crsm_slam/desired_number_of_picked_rays", slamParams.desired_number_of_picked_rays);
	else {
		ROS_WARN("[CrsmSlam] : Parameter desired_number_of_picked_rays not found. Using Default");
		slamParams.desired_number_of_picked_rays = 40 ;
	}
	
	if (n.hasParam("/crsm_slam/robot_width")) 
		n.getParam("/crsm_slam/robot_width", slamParams.robot_width);
	else {
		ROS_WARN("[CrsmSlam] : Parameter robot_width not found. Using Default");
		slamParams.robot_width = 0.5 ;
	}
	
	if (n.hasParam("/crsm_slam/robot_length")) 
		n.getParam("/crsm_slam/robot_length", slamParams.robot_length);
	else {
		ROS_WARN("[CrsmSlam] : Parameter robot_length not found. Using Default");
		slamParams.robot_length = 0.6 ;
	}
}

/**
@brief Calculates the transformation (translation & rotation) with RRHC
@return void
**/
void CrsmSlam::findTransformation(void){
	
	
	bestFitness=0;
	bestTransformation.dx=bestTransformation.dy=bestTransformation.dth=0;
	
	int tempx,tempy;
	float sinth,costh,tttx,ttty;
	CrsmTransformation temp;
	CrsmHillClimbingPerson trier;
	unsigned int counter=0;
	
	trier.fitness=0;
	trier.t.dx=0;
	trier.t.dy=0;
	trier.t.dth=0;
	
	bool isDone=false;
	bestFitness=0;

	while(!isDone) {
		temp.dx=robotPose.x+trier.t.dx;
		temp.dy=robotPose.y+trier.t.dy;
		temp.dth=robotPose.theta+trier.t.dth;	
		trier.fitness=0;
		float tempFitness=0;
		for(set<int>::iterator j=scanSelections.begin();j != scanSelections.end();j++){
			tempx=laser.scan.p[*j].x;
			tempy=laser.scan.p[*j].y; 
			sinth=sin(temp.dth);
			costh=cos(temp.dth);
			tttx=tempx*costh-tempy*sinth+temp.dx+map.info.size/2;
			ttty=tempx*sinth+tempy*costh+temp.dy+map.info.size/2;	
			if(map.p[(unsigned int)tttx][(unsigned int)ttty]==127) continue;
			tempFitness+=((255-map.p[(unsigned int)tttx][(unsigned int)ttty])*10+
						(255-map.p[(unsigned int)tttx-1][(unsigned int)ttty])+
						(255-map.p[(unsigned int)tttx+1][(unsigned int)ttty])+
						(255-map.p[(unsigned int)tttx][(unsigned int)ttty-1])+
						(255-map.p[(unsigned int)tttx][(unsigned int)ttty+1]))/255.0;
		}
		tempFitness/=(14.0*scanSelections.size());
		trier.fitness=tempFitness;
		if(trier.fitness>bestFitness){
			bestFitness=trier.fitness;
			bestTransformation=trier.t;
			trier.t.dx+=rand()%slamParams.disparity/4-slamParams.disparity/8;
			trier.t.dy+=rand()%slamParams.disparity/4-slamParams.disparity/8;
			trier.t.dth+=(rand()%slamParams.disparity-slamParams.disparity/2.0)/90.0;
		}
		else{
			trier.t.dx=rand()%slamParams.disparity/2-slamParams.disparity/4;
			trier.t.dy=rand()%slamParams.disparity/2-slamParams.disparity/4;
			trier.t.dth=(rand()%slamParams.disparity-slamParams.disparity/2.0)/45.0;
		}
		if(counter>slamParams.max_hill_climbing_iterations) break;
		counter++;
	}	
}

/**
@brief Chooses important rays for RRHC
@return void
**/
void CrsmSlam::calculateCriticalRays(void){
	meanDensity=0;
	float maxDensity=0;
	float maxRay=0;
	for(unsigned int i=0;i<laser.info.laserRays-1;i++){
		if(laser.scan.distance[i]>maxRay) maxRay=laser.scan.distance[i];
		laser.scan.density[i]=fabs(laser.scan.distance[i]-laser.scan.distance[i+1]);
		meanDensity+=laser.scan.density[i];
		if(maxDensity<laser.scan.density[i] && laser.scan.density[i]<slamParams.scan_selection_meters)
			maxDensity=laser.scan.density[i];
	}
	if(laser.scan.distance[laser.info.laserRays-1]>maxRay) maxRay=laser.scan.distance[laser.info.laserRays-1];
	meanDensity/=(laser.info.laserRays-1);
	static float parameter = 100;
	scanSelections.clear();
	bigChanges.clear();
	for(unsigned int i=0;i<laser.info.laserRays;i++){
		if(laser.scan.distance[i]<(laser.info.laserMax-0.05)){
			bigChanges.insert(i);
			break;
		}
	}
	unsigned int counterLaserRays=0;
	while(counterLaserRays<laser.info.laserRays){
		unsigned int a=0,b=0; 
		if(laser.scan.density[counterLaserRays]>slamParams.scan_selection_meters){
			if(laser.scan.distance[counterLaserRays]<(laser.info.laserMax-0.05)){
				bigChanges.insert(counterLaserRays);
				a=1;
			}
			if(laser.scan.distance[counterLaserRays+1]<(laser.info.laserMax-0.05)){
				bigChanges.insert(counterLaserRays+1);
				b=1;
			}
		}
		if((a+b)==0)
			counterLaserRays++;
		else  
			counterLaserRays+=a+b;
	}
	for(unsigned int i=laser.info.laserRays-1;i>0;i--){ 
		if( laser.scan.distance[i]<(laser.info.laserMax-0.05) ){
			bigChanges.insert(i);
			break;
		}
	}
	scanSelections=bigChanges;
	unsigned int start,end;
	int count=0;
	for(set<int>::const_iterator j=bigChanges.begin();j != bigChanges.end();){
		float sumDensity=0;
		start= *j;
		j++;
		if(j==bigChanges.end()) break;
		end= *j;
		if(laser.scan.distance[start+1]>(laser.info.laserMax)-0.5)continue;
		for(unsigned int i=start;i<end-1;i++)
			sumDensity+=laser.scan.density[i];
		float step = sumDensity /(end-start);
		float localPos=0;
		for(unsigned int i=start;i<end;i++){
			localPos+=laser.scan.density[i];
			if( localPos > (parameter*step/exp(laser.scan.distance[i]/sqrt(maxRay)*3.0)) ){	
				scanSelections.insert(i);	
				count++;
				localPos=0;
			}
		}
	}
	if(count > slamParams.desired_number_of_picked_rays*1.25)
		parameter += (count - slamParams.desired_number_of_picked_rays);
	else if(count < slamParams.desired_number_of_picked_rays*0.75)
		parameter -= (slamParams.desired_number_of_picked_rays - count);
		
	bool isFinished=false;
	while(!isFinished){
		isFinished=true;
		std::vector<int> tete;
		for(std::set<int>::const_iterator it=scanSelections.begin();it!=scanSelections.end();it++)
			tete.push_back(*it);
		for(unsigned int i=0;i<tete.size()-1;i++)
			if((tete[i+1]-tete[i])>25){
				isFinished=false;
				scanSelections.insert((tete[i+1]+tete[i])/2);
			}	
	}
}


/**
@brief Serves the laser scan messages
@param msg [sensor_msgs::LaserScanConstPtr&] : The laser rays distances
@return void
**/
void CrsmSlam::fixNewScans(const sensor_msgs::LaserScanConstPtr& msg){
	if(!laser.initialized)
		laser.initialize(msg);
	
	static int raysPicked=0;
	static float meanFitness=0;
	static int counter=0;
	float laserMean=0;
	for(unsigned int j=0;j<laser.info.laserRays;j++) {
		laser.scan.distance[j]=msg->ranges[j];
		laserMean+=laser.scan.distance[j];
		
		if(fabs(laser.scan.distance[j])>laser.info.laserMax)
			laser.scan.distance[j]=0;
		if(msg->ranges[j]<0)
			laser.scan.distance[j]=0;
		
		laser.scan.p[j].theta= msg->angle_min + ( j*msg->angle_increment );
		laser.scan.p[j].x=laser.scan.distance[j]/slamParams.ocgd*cos(laser.scan.p[j].theta);
		laser.scan.p[j].y=laser.scan.distance[j]/slamParams.ocgd*sin(laser.scan.p[j].theta);
	}
	
	calculateCriticalRays();

	std::vector< set<int>::iterator > toBeErased;
	for(set<int>::const_iterator it=scanSelections.begin();it!=scanSelections.end();it++)
		if(laser.scan.distance[*it]==0 || laser.scan.distance[*it]==laser.info.laserMax)
			toBeErased.push_back(it);
	for(unsigned int i=0;i<toBeErased.size();i++){
		scanSelections.erase(toBeErased[i]);
	}
	
	raysPicked+=scanSelections.size();

	findTransformation();

	meanFitness+=bestFitness;
	
	robotPose.x+=bestTransformation.dx;
	robotPose.y+=bestTransformation.dy;
	robotPose.theta+=bestTransformation.dth;
	if(robotPose.theta>pi) robotPose.theta-=pi_double;
	if(robotPose.theta<-pi) robotPose.theta+=pi_double;
	
	if(counter<40){
		robotPose.x=0;
		robotPose.y=0;
		robotPose.theta=-(laser.info.laserAngleBegin+laser.info.laserAngleEnd)/2.0;
	}

	//---robot trajectory---
	CrsmPose trajectoryTemp;
	if ( robotTrajectory.size()==0 || robotTrajectory[robotTrajectory.size()-1].x != robotPose.x || robotTrajectory[robotTrajectory.size()-1].y != robotPose.y){
		trajectoryTemp.x = robotPose.x-cos(robotPose.theta)*slamParams.dx_laser_robotCenter/slamParams.ocgd;
		trajectoryTemp.y = robotPose.y-sin(robotPose.theta)*slamParams.dx_laser_robotCenter/slamParams.ocgd;
		trajectoryTemp.theta = robotPose.theta;
		robotTrajectory.push_back( trajectoryTemp );
	}

	if(counter<10){
			meanDensity=0.5;
	}
	updateMapProbabilities();
	
	counter++;
}

/**
@brief Updates map after finding the new robot pose
@return void
**/
void CrsmSlam::updateMapProbabilities(void){
	int R=0;
	int dMeasure;
	if(meanDensity>0.2) meanDensity=0.2;
	if(meanDensity<0.05) meanDensity=0.05;
	std::set<long> prevPoints;
	
	//	Fix map size according to the laser scans
	for(unsigned int i=0;i<laser.info.laserRays;i++){
			float xPoint,yPoint;
			xPoint=laser.scan.distance[i]/slamParams.ocgd*cos(robotPose.theta+(float)i/(float)laser.info.laserRays*laser.info.laserAngle+laser.info.laserAngleBegin) + robotPose.x+map.info.size/2;
			yPoint=laser.scan.distance[i]/slamParams.ocgd*sin(robotPose.theta+(float)i/(float)laser.info.laserRays*laser.info.laserAngle+laser.info.laserAngleBegin) + robotPose.y+map.info.size/2;
	}
	
	//	Set the map's colorization
	for(unsigned int measid=0;measid<laser.info.laserRays;measid+=1){
		int xt,yt;
		if(laser.scan.distance[measid]==0) continue;
		xt=laser.scan.p[measid].x;
		yt=laser.scan.p[measid].y;
		
		if(prevPoints.find(xt*map.info.size+yt)!=prevPoints.end()) continue;
		prevPoints.insert(xt*map.info.size+yt);
		dMeasure=(int)((float)laser.scan.distance[measid]/slamParams.ocgd);
		while(R<laser.info.laserMax/slamParams.ocgd-3){
			float xPoint,yPoint;
			xPoint=R*cos(robotPose.theta+laser.angles[measid]) + robotPose.x+map.info.size/2;
			yPoint=R*sin(robotPose.theta+laser.angles[measid]) + robotPose.y+map.info.size/2;
			int tt=map.p[(unsigned int)xPoint][(unsigned int)yPoint];
			float diff=fabs(tt-127.0)/128.0;
			if(dMeasure>R || (xt==0 && yt==0))
				tt+=(1-diff)*meanDensity*slamParams.density;
			if( dMeasure+1>R && dMeasure-2<R )
				tt-=(1-diff)*meanDensity*slamParams.obstacle_density*slamParams.density;
			map.p[(unsigned int)xPoint][(unsigned int)yPoint]=tt;
			R++;
		}
		R=1;
	}
}

/**
@brief Starts the laser subscriber, listening to laser_subscriber_topic from parameters
@return void
**/
void CrsmSlam::startLaserSubscriber(){
	
	clientLaserValues = n.subscribe( slamParams.laser_subscriber_topic.c_str() , 1 , &CrsmSlam::fixNewScans , this );
}

/**
@brief Stops the laser subscriber
@return void
**/
void CrsmSlam::stopLaserSubscriber(){
	clientLaserValues.shutdown();
}

/**
@brief Starts the OccupancyGrid publisher, posting to occupancy_grid_publish_topic from parameters
@return void
**/
void CrsmSlam::startOGMPublisher(){
	_occupancyGridPublisher = n.advertise<nav_msgs::OccupancyGrid>(slamParams.occupancy_grid_publish_topic.c_str(),1);
	_mapPublishingTimer.start();
}

/**
@brief Stops the OccupancyGrid publisher
@return void
**/
void CrsmSlam::stopOGMPublisher(void){
	_mapPublishingTimer.stop();
	_occupancyGridPublisher.shutdown();
}

/**
@brief Publishes the OccupancyGrid map as nav_msgs::OccupancyGrid, posting with occupancy_grid_map_freq Hz from parameters
@param e [const ros::TimerEvent&] The timer event
@return void
**/
void CrsmSlam::publishOGM(const ros::TimerEvent& e){
	
	map.findEffectiveMapLimits();

	int width=map.info.xmax-map.info.xmin;
    int height=map.info.ymax-map.info.ymin;
    
    nav_msgs::OccupancyGrid grid;
    
    grid.header.stamp = ros::Time::now(); 
    grid.header.frame_id = slamParams.map_frame; 
    
    grid.info.resolution = slamParams.ocgd;
    grid.info.width = width;
    grid.info.height = height;

    grid.info.origin.position.x = -((float)map.info.size/2 - (float)map.info.xmin)*slamParams.ocgd ;
    grid.info.origin.position.y = -((float)map.info.size/2 - (float)map.info.ymin)*slamParams.ocgd;
    
    grid.data.resize(width*height) ;
    for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			grid.data[j*(width)+i]= 100.0-(int) (map.p[i+map.info.xmin][j+map.info.ymin]*100.0/255.0);
		}
	}
 
    _occupancyGridPublisher.publish(grid);

}

/**
@brief Returns the map occupancy probability of coordinates (x,y) ranging from 0-255 (0 is occupied, 255 is free)
@param x [int] : The x coordinate
@param y [int] : The y coordinate
@return char probability
**/
char CrsmSlam::getMapProbability(int x,int y){ 
	return map.p[x][y]; 
}

/**
@brief Returns the map info in a CrsmMapInfo structure
@return CrsmMapInfo
**/
CrsmMapInfo CrsmSlam::getMapInfo(void){
	return map.info;
}

/**
@brief Returns the robot pose in a CrsmPose structure
@return CrsmPose
**/
CrsmPose CrsmSlam::getRobotPose(void){
	return robotPose;
}

/**
@brief Returns the laser info in a CrsmLaserInfo structure
@return CrsmLaserInfo
**/
CrsmLaserInfo CrsmSlam::getLaserInfo(void){
	return laser.info;
}

/**
@brief Returns the robot trajectory in a vector of CrsmPose structures
@return std::vector<CrsmPose>
**/
std::vector<CrsmPose> CrsmSlam::getTrajectory(void){
	return robotTrajectory;
}

/**
@brief Publishes the Tf robot pose, posting with robot_pose_tf_freq Hz from parameters
@param e [const ros::TimerEvent&] The timer event
@return void
**/
void CrsmSlam::publishRobotPoseTf(const ros::TimerEvent& e){
	tf::Vector3 translation( (robotPose.x-cos(robotPose.theta)*(slamParams.dx_laser_robotCenter/slamParams.ocgd))* slamParams.ocgd, (robotPose.y-sin(robotPose.theta)*(slamParams.dx_laser_robotCenter/slamParams.ocgd))* slamParams.ocgd, 0);
	tf::Quaternion rotation;
	rotation.setRPY(0,0,robotPose.theta);

	tf::Transform transform(rotation,translation);
	_slamFrameBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
				slamParams.world_frame, slamParams.base_footprint_frame));				
}

/**
@brief Starts the Trajectory publisher, posting to robot_trajectory_publish_topic from parameters, with trajectory_publisher_frame_id as frame ID.
@return void
**/
void CrsmSlam::startTrajectoryPublisher(){
	_pathPublisher = n.advertise<nav_msgs::Path>(slamParams.robot_trajectory_publish_topic.c_str(),1);
	_pathPublishingTimer.start();	
}
	
/**
@brief Stops the Trajectory publisher.
@return void
**/	
void CrsmSlam::stopTrajectoryPublisher(void){
	_pathPublishingTimer.stop();	
	_pathPublisher.shutdown();
}

/**
@brief Publishes the robot trajectory as nav_msgs::Path, posting with trajectory_freq Hz from parameters
@param e [const ros::TimerEvent&] The timer event
@return void
**/
void CrsmSlam::publishTrajectory(const ros::TimerEvent& e){
	nav_msgs::Path pathForViz;
	geometry_msgs::PoseStamped pathPoint;
	
	for (int i=0; i<robotTrajectory.size();i++){
		pathPoint.pose.position.x = robotTrajectory[i].x*slamParams.ocgd;
		pathPoint.pose.position.y = robotTrajectory[i].y*slamParams.ocgd;
		pathForViz.poses.push_back(pathPoint);
	}
	
	pathForViz.header.stamp = ros::Time::now();
	pathForViz.header.frame_id = slamParams.trajectory_publisher_frame_id;
		
	_pathPublisher.publish(pathForViz);
}

}
