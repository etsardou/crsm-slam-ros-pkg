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

namespace crsm_slam
{

  /**
    @brief Default costructor
    @param argc [int] The number of input arguments
    @param argv [char **] The input arguments
    @return void
   **/
  CrsmSlam::CrsmSlam(int argc, char **argv)
  {

    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.theta = 0;

    updateParameters();

    bestTransformation.dx = bestTransformation.dy = bestTransformation.dth = 0;
    
    map = CrsmMap(slamParams.map_size);


    expansion.expansions.insert(std::pair<CrsmDirection ,int>(RIGHT, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(LEFT, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(UP, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(DOWN, 0));


    // initialize pose publisher
    _posePublisher =
      n.advertise<geometry_msgs::PoseWithCovarianceStamped>(slamParams.pose_publish_topic, 1);
  }

  /**
    @brief Reads the CRSM slam parameters from the yaml file and fills the 
      CrsmSlamParameters structure
    @return void
   **/
  void CrsmSlam::updateParameters(void)
  {

    if (n.hasParam("/crsm_slam/occupancy_grid_publish_topic"))
      n.getParam("/crsm_slam/occupancy_grid_publish_topic", 
        slamParams.occupancy_grid_publish_topic);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_publish_topic not found.\
        Using Default");
      slamParams.occupancy_grid_publish_topic = "/crsm_slam/map" ;
    }

    if (n.hasParam("/crsm_slam/robot_trajectory_publish_topic"))
      n.getParam("/crsm_slam/robot_trajectory_publish_topic", 
        slamParams.robot_trajectory_publish_topic);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_trajectory_publish_topic not\
        found. Using Default");
      slamParams.robot_trajectory_publish_topic = "/crsm_slam/trajectory" ;
    }

    if (n.hasParam("/crsm_slam/trajectory_publisher_frame_id"))
      n.getParam("/crsm_slam/trajectory_publisher_frame_id", 
        slamParams.trajectory_publisher_frame_id);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter trajectory_publisher_frame_id not found.\
        Using Default");
      slamParams.trajectory_publisher_frame_id = "map" ;
    }

    if (n.hasParam("/crsm_slam/laser_subscriber_topic"))
      n.getParam("/crsm_slam/laser_subscriber_topic", 
        slamParams.laser_subscriber_topic);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter laser_subscriber_topic not found.\
        Using Default");
      slamParams.laser_subscriber_topic = "/crsm_slam/laser_scan" ;
    }

    if (n.hasParam("/crsm_slam/pose_publish_topic"))
      n.getParam("/crsm_slam/pose_publish_topic", 
        slamParams.pose_publish_topic);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter pose_publish_topic not found.\
        Using Default");
      slamParams.pose_publish_topic = "/crsm_slam/pose" ;
    }

    if (n.hasParam("/crsm_slam/publish_tf"))
      n.getParam("/crsm_slam/publish_tf", 
        slamParams.publish_tf);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter publish_tf not found.\
        Using Default");
      slamParams.publish_tf = true ;
    }

    if (n.hasParam("/crsm_slam/base_footprint_frame"))
      n.getParam("/crsm_slam/base_footprint_frame", 
        slamParams.base_footprint_frame);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter base_footprint_frame not found.\
        Using Default");
      slamParams.base_footprint_frame = "base_footprint_link" ;
    }

    if (n.hasParam("/crsm_slam/base_frame"))
      n.getParam("/crsm_slam/base_frame", slamParams.base_frame);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter base_frame not found. Using Default");
      slamParams.base_frame = "base_link" ;
    }

    if (n.hasParam("/crsm_slam/map_frame"))
      n.getParam("/crsm_slam/map_frame", slamParams.map_frame);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter map_frame not found. Using Default");
      slamParams.map_frame = "map" ;
    }

    if (n.hasParam("/crsm_slam/hill_climbing_disparity"))
      n.getParam("/crsm_slam/hill_climbing_disparity", slamParams.disparity);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter hill_climbing_disparity not found.\
        Using Default");
      slamParams.disparity = 40 ;
    }

    if (n.hasParam("/crsm_slam/slam_container_size"))
      n.getParam("/crsm_slam/slam_container_size", slamParams.map_size);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter slam_container_size not found. Using\
        Default");
      slamParams.map_size = 500 ;
    }

    if (n.hasParam("/crsm_slam/slam_occupancy_grid_dimentionality"))
      n.getParam("/crsm_slam/slam_occupancy_grid_dimentionality", 
        slamParams.ocgd);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter slam_occupancy_grid_dimentionality not\
        found. Using Default");
      slamParams.ocgd = 0.02 ;
    }

    if (n.hasParam("/crsm_slam/map_update_density"))
      n.getParam("/crsm_slam/map_update_density", slamParams.density);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter map_update_density not found. Using\
        Default");
      slamParams.density = 30.0 ;
    }

    if (n.hasParam("/crsm_slam/map_update_obstacle_density"))
      n.getParam("/crsm_slam/map_update_obstacle_density", 
        slamParams.obstacle_density);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter map_update_obstacle_density not found.\
        Using Default");
      slamParams.obstacle_density = 3.0 ;
    }

    if (n.hasParam("/crsm_slam/scan_density_lower_boundary"))
      n.getParam("/crsm_slam/scan_density_lower_boundary", 
        slamParams.scan_selection_meters);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter scan_density_lower_boundary not found.\
        Using Default");
      slamParams.scan_selection_meters = 0.3 ;
    }

    if (n.hasParam("/crsm_slam/max_hill_climbing_iterations"))
      n.getParam("/crsm_slam/max_hill_climbing_iterations", 
        slamParams.max_hill_climbing_iterations);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter max_hill_climbing_iterations not found.\
        Using Default");
      slamParams.max_hill_climbing_iterations = 40000 ;
    }

    if (n.hasParam("/crsm_slam/occupancy_grid_map_freq"))
      n.getParam("/crsm_slam/occupancy_grid_map_freq", 
        slamParams.occupancy_grid_map_freq);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_map_freq not found.\
        Using Default");
      slamParams.occupancy_grid_map_freq = 1.0 ;
    }

    if (n.hasParam("/crsm_slam/robot_pose_tf_freq"))
      n.getParam("/crsm_slam/robot_pose_tf_freq", 
        slamParams.robot_pose_tf_freq);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_pose_tf_freq not found.\
        Using Default");
      slamParams.robot_pose_tf_freq = 5.0 ;
    }

    if (n.hasParam("/crsm_slam/trajectory_freq"))
      n.getParam("/crsm_slam/trajectory_freq", slamParams.trajectory_freq);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter trajectory_freq not found.\
        Using Default");
      slamParams.trajectory_freq = 1.0 ;
    }

    if (n.hasParam("/crsm_slam/desired_number_of_picked_rays"))
      n.getParam("/crsm_slam/desired_number_of_picked_rays", 
        slamParams.desired_number_of_picked_rays);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter desired_number_of_picked_rays not found.\
        Using Default");
      slamParams.desired_number_of_picked_rays = 40 ;
    }

    if (n.hasParam("/crsm_slam/robot_width"))
      n.getParam("/crsm_slam/robot_width", slamParams.robot_width);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_width not found. Using Default");
      slamParams.robot_width = 0.5 ;
    }

    if (n.hasParam("/crsm_slam/robot_length"))
      n.getParam("/crsm_slam/robot_length", slamParams.robot_length);
    else 
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_length not found. Using Default");
      slamParams.robot_length = 0.6 ;
    }
  }

  /**
   * @brief Draws a patch underneath the robot's footprint
   * @return void
   **/
  void CrsmSlam::drawInitialPatch(void)
  {
    int initialPatchWidth = slamParams.robot_width / slamParams.ocgd / 2;
    int initialPatchLength = slamParams.robot_length / slamParams.ocgd / 2;

    for(int i = -initialPatchLength ; i < initialPatchLength ; i++)
    {
      for(int j = -initialPatchWidth ; j < initialPatchWidth ; j++)
      {
        map.p[i + map.info.originx - 
          (int)(slamParams.dx_laser_robotCenter / slamParams.ocgd)]
          [j + map.info.originy] = 200;
      }
    }
  }

  /**
    @brief Calculates the transformation (translation & rotation) with RRHC
    @return void
   **/
  void CrsmSlam::findTransformation(void)
  {

    bestFitness = 0;
    bestTransformation.dx = bestTransformation.dy = bestTransformation.dth = 0;

    int tempx, tempy;
    float sinth, costh, tttx, ttty;
    CrsmTransformation temp;
    CrsmHillClimbingPerson trier;
    unsigned int counter = 0;

    trier.fitness = 0;
    trier.t.dx = 0;
    trier.t.dy = 0;
    trier.t.dth = 0;

    bool isDone = false;
    bestFitness = 0;

    while(!isDone) 
    {
      temp.dx = robotPose.x + trier.t.dx;
      temp.dy = robotPose.y + trier.t.dy;
      temp.dth = robotPose.theta + trier.t.dth;
      trier.fitness = 0;
      float tempFitness = 0;

      for(std::set<int>::iterator j = scanSelections.begin() ; 
        j != scanSelections.end() ; j++)
      {
        tempx = laser.scan.p[*j].x;
        tempy = laser.scan.p[*j].y;
        sinth = sin(temp.dth);
        costh = cos(temp.dth);
        tttx = tempx * costh - tempy * sinth + temp.dx + map.info.originx;
        ttty = tempx * sinth + tempy * costh + temp.dy + map.info.originy;

        if(checkExpansion(
            (unsigned int)(tttx - 1), (unsigned int)(ttty - 1), false))
          continue;
        if(checkExpansion(
            (unsigned int)(tttx + 1), (unsigned int)(ttty + 1), false))
          continue;

        if(map.p[(unsigned int)tttx][(unsigned int)ttty] == 127) 
          continue;
        
        tempFitness += 
          ((255 - map.p[(unsigned int)tttx][(unsigned int)ttty]) * 10+
          (255 - map.p[(unsigned int)tttx - 1][(unsigned int)ttty]) +
          (255 - map.p[(unsigned int)tttx + 1][(unsigned int)ttty]) +
          (255 - map.p[(unsigned int)tttx][(unsigned int)ttty - 1]) +
          (255 - map.p[(unsigned int)tttx][(unsigned int)ttty + 1])) / 255.0;
      }

      tempFitness /= (14.0 * scanSelections.size());
      trier.fitness = tempFitness;
      if(trier.fitness > bestFitness)
      {
        bestFitness = trier.fitness;
        bestTransformation = trier.t;
        trier.t.dx += rand() % slamParams.disparity / 4 - 
          slamParams.disparity / 8;
        trier.t.dy += rand() % slamParams.disparity / 4 - 
          slamParams.disparity / 8;
        trier.t.dth += 
          (rand() % slamParams.disparity - slamParams.disparity / 2.0) / 90.0;
      }
      else
      {
        trier.t.dx = 
          rand() % slamParams.disparity / 2 - slamParams.disparity / 4;
        trier.t.dy = rand() % slamParams.disparity / 2 - 
          slamParams.disparity / 4;
        trier.t.dth = (rand() % slamParams.disparity - 
          slamParams.disparity / 2.0) / 45.0;
      }
      if(counter > slamParams.max_hill_climbing_iterations) 
        break;
      
      counter++;
    }
  }

  /**
    @brief Chooses important rays for RRHC
    @return void
   **/
  void CrsmSlam::calculateCriticalRays(void)
  {
    meanDensity = 0;
    float maxDensity = 0;
    float maxRay = 0;

    for(unsigned int i = 0 ; i < laser.info.laserRays - 1 ; i++)
    {
      if(laser.scan.distance[i] > maxRay) 
        maxRay = laser.scan.distance[i];

      laser.scan.density[i] = 
        fabs(laser.scan.distance[i] - laser.scan.distance[i + 1]);

      meanDensity += laser.scan.density[i];

      if(maxDensity < laser.scan.density[i] && 
        laser.scan.density[i] < slamParams.scan_selection_meters)
      {
        maxDensity = laser.scan.density[i];
      }
    }

    if(laser.scan.distance[laser.info.laserRays - 1] > maxRay) 
      maxRay = laser.scan.distance[laser.info.laserRays - 1];

    meanDensity /= (laser.info.laserRays - 1);
    static float parameter = 100;
    scanSelections.clear();
    bigChanges.clear();

    for(unsigned int i = 0 ; i < laser.info.laserRays ; i++)
    {
      if(laser.scan.distance[i] < (laser.info.laserMax - 0.05))
      {
        bigChanges.insert(i);
        break;
      }
    }

    unsigned int counterLaserRays = 0;
    while(counterLaserRays < laser.info.laserRays)
    {
      unsigned int a = 0, b = 0;
      if(laser.scan.density[counterLaserRays] > 
        slamParams.scan_selection_meters)
      {
        if(laser.scan.distance[counterLaserRays] < 
          (laser.info.laserMax - 0.05))
        {
          bigChanges.insert(counterLaserRays);
          a = 1;
        }
        if(laser.scan.distance[counterLaserRays + 1] < 
          (laser.info.laserMax - 0.05))
        {
          bigChanges.insert(counterLaserRays + 1);
          b = 1;
        }
      }
      if((a + b) == 0)
        counterLaserRays++;
      else
        counterLaserRays += a + b;
    }

    for(unsigned int i = laser.info.laserRays - 1 ; i > 0 ; i--)
    {
      if( laser.scan.distance[i] < (laser.info.laserMax - 0.05))
      {
        bigChanges.insert(i);
        break;
      }
    }

    scanSelections = bigChanges;
    unsigned int start, end;
    int count = 0;

    for(std::set<int>::const_iterator j = bigChanges.begin() ; 
      j != bigChanges.end() ;)
    {
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
    if(count > slamParams.desired_number_of_picked_rays * 1.25)
      parameter += (count - slamParams.desired_number_of_picked_rays);
    else if(count < slamParams.desired_number_of_picked_rays * 0.75)
      parameter -= (slamParams.desired_number_of_picked_rays - count);

    bool isFinished = false;
    while(!isFinished)
    {
      isFinished = true;
      std::vector<int> tete;
      for(std::set<int>::const_iterator 
        it = scanSelections.begin() ; it != scanSelections.end() ; it++)
      {
        tete.push_back(*it);
      }
      for(unsigned int i = 0 ; i < tete.size() - 1 ; i++)
      {
        if((tete[i + 1] - tete[i]) > 25)
        {
          isFinished = false;
          scanSelections.insert((tete[i + 1] + tete[i]) / 2);
        }
      }
    }
  }

  /**
    @brief Serves the laser scan messages
    @param msg [sensor_msgs::LaserScanConstPtr&] : The laser rays distances
    @return void
   **/
  void CrsmSlam::fixNewScans(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if(!laser.initialized)
    {
      laser.initialize(msg);

      // Try to find distance between laser and robot center, else initialize to zero
      tf::StampedTransform tfTransform;
      try
      {
        _listener.waitForTransform(
          slamParams.base_frame, msg->header.frame_id,
          ros::Time(0), ros::Duration(1.0));

        _listener.lookupTransform(
          slamParams.base_frame, msg->header.frame_id,
          ros::Time(0), tfTransform);

        tf::Vector3 origin = tfTransform.getOrigin();
        slamParams.dx_laser_robotCenter = origin[0];
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[CrsmSlam] Error in tf : %s", ex.what());
        slamParams.dx_laser_robotCenter = 0.0;
      }

      drawInitialPatch();
    }

    static int raysPicked = 0;
    static float meanFitness = 0;
    static int counter = 0;
    float laserMean = 0;

    for(unsigned int j = 0 ; j < laser.info.laserRays ; j++) 
    {
      laser.scan.distance[j] = msg->ranges[j];

      //!< Check if laser measurement is in nominal values
      if( 0.1 <= laser.scan.distance[j] && 
        laser.scan.distance[j] <= laser.info.laserMax )
      {
        // Valid measurement
      }
      else if ( !std::isfinite(laser.scan.distance[j]) &&  
        laser.scan.distance[j] < 0 )
      {
        laser.scan.distance[j] = 0; // too close to measure
      }
      else if ( !std::isfinite(laser.scan.distance[j]) &&  
        laser.scan.distance[j] > 0 )
      {
        laser.scan.distance[j] = 0;//laser.info.laserMax; // too far to measure
      }
      else if ( std::isnan(laser.scan.distance[j]) )
      {
        laser.scan.distance[j] = 0; // erroneous measurement
      }
      else
      {
        laser.scan.distance[j] = 0; // erroneous measurement
      }

      laserMean += laser.scan.distance[j];

      laser.scan.p[j].theta = msg->angle_min + ( j * msg->angle_increment );
      laser.scan.p[j].x = 
        laser.scan.distance[j] / slamParams.ocgd * cos(laser.scan.p[j].theta);
      laser.scan.p[j].y = laser.scan.distance[j] / slamParams.ocgd * 
        sin(laser.scan.p[j].theta);
    }
    
    calculateCriticalRays();

    std::vector< std::set<int>::iterator > toBeErased;
    for(std::set<int>::const_iterator it = scanSelections.begin() ; 
      it != scanSelections.end() ; it++)
    {
      if(laser.scan.distance[*it]==0 || laser.scan.distance[*it]==laser.info.laserMax)
        toBeErased.push_back(it);
    }
    for(unsigned int i = 0 ; i < toBeErased.size() ; i++)
    {
      scanSelections.erase(toBeErased[i]);
    }

    raysPicked += scanSelections.size();

    findTransformation();

    meanFitness += bestFitness;

    robotPose.x += bestTransformation.dx;
    robotPose.y += bestTransformation.dy;
    robotPose.theta += bestTransformation.dth;

    if(robotPose.theta > pi) 
      robotPose.theta -= pi_double;
    if(robotPose.theta < -pi) 
      robotPose.theta += pi_double;

    if(counter < 40)
    {
      robotPose.x = 0;
      robotPose.y = 0;
      robotPose.theta = 
        -(laser.info.laserAngleBegin + laser.info.laserAngleEnd) / 2.0;
    }

    if(counter < 10)
    {
      meanDensity = 0.5;
    }

    updateMapProbabilities();

    publishOGM(msg->header.stamp);

    // Publish tf before trajectory in order to update it
    publishRobotPoseTf(msg->header.stamp);
    publishTrajectory(msg->header.stamp);

    counter++;
  }


  bool CrsmSlam::checkExpansion(float x, float y, bool update)
  {
    // +-50 pixels for more strict resizing
    int extra_pix = 50;
    int plus_pix = 100;
    bool changed = false;

    if(x < extra_pix)
    {
      if(update && (fabs(x)) > expansion.expansions[LEFT])
      {
        expansion.expansions[LEFT] = fabs(x) + plus_pix;
      }
      changed = true;
    }
    if(x >= (int)map.info.width - extra_pix){
      if(update && (fabs(x - map.info.width)) > expansion.expansions[RIGHT])
      {
        expansion.expansions[RIGHT] = fabs(x - map.info.width) + plus_pix;
      }
      changed = true;
    }
    if(y < extra_pix)
    {
      if(update && (fabs(y)) > expansion.expansions[UP])
      {
        expansion.expansions[UP] = fabs(y) + plus_pix;
      }
      changed = true;
    }
    if(y >= (int)map.info.height - extra_pix){
      if(update && (fabs(y - map.info.height)) > expansion.expansions[DOWN])
      {
        expansion.expansions[DOWN] = fabs(y - map.info.height) + plus_pix;
      }
      changed = true;
    }
    return changed;
  }

  void CrsmSlam::expandMap(void)
  {
    if(
      expansion.expansions[LEFT] == 0 &&
      expansion.expansions[RIGHT] == 0 &&
      expansion.expansions[UP] == 0 &&
      expansion.expansions[DOWN] == 0 ) 
    {
      return;
    }
    map.expandMap(expansion);
  }

  /**
    @brief Updates map after finding the new robot pose
    @return void
   **/
  void CrsmSlam::updateMapProbabilities(void)
  {
    int R = 0;
    int dMeasure = 0;
    if(meanDensity > 0.2) meanDensity = 0.2;
    if(meanDensity < 0.04) meanDensity = 0.04;
    std::set<long> prevPoints;

    expansion.expansions[RIGHT] = 0;
    expansion.expansions[LEFT] = 0;
    expansion.expansions[UP] = 0;
    expansion.expansions[DOWN] = 0;

    //	Fix map size according to the laser scans
    for(unsigned int i = 0 ; i < laser.info.laserRays ; i++){
      float xPoint,yPoint;
      xPoint =
        (float)laser.scan.distance[i] / (float)slamParams.ocgd *
        (float)cos(robotPose.theta + laser.angles[i]) +
        (float)robotPose.x + (float)map.info.originx + 3;
      yPoint =
        (float)laser.scan.distance[i] / (float)slamParams.ocgd *
        (float)sin(robotPose.theta + laser.angles[i]) +
        (float)robotPose.y+(float)map.info.originy + 3;

      checkExpansion(xPoint, yPoint, true);
    } // +3 since we are checking a bit more than the actual measure below

    expandMap();

    //	Set the map's colorization
    for(unsigned int measid = 0;measid < laser.info.laserRays ; measid += 1)
    {
      int xt, yt, xtt, ytt;
      if(laser.scan.distance[measid] == 0) 
        continue;

      xt = laser.scan.p[measid].x;
      yt = laser.scan.p[measid].y;

      if(prevPoints.find(xt * map.info.width + yt) != prevPoints.end()) 
      {
        continue;
      }
      prevPoints.insert(xt * map.info.width + yt);

      float prevtt = map.p
        [static_cast<int>(robotPose.x + map.info.originx)]
        [static_cast<int>(robotPose.y + map.info.originy)];

      dMeasure = (int)((float)laser.scan.distance[measid] / slamParams.ocgd);
      while(R < dMeasure + 3){ // populate this many cells as obstacle
        int xPoint, yPoint;
        xPoint = R * cos(robotPose.theta + laser.angles[measid])
          + robotPose.x + map.info.originx;
        yPoint = R * sin(robotPose.theta + laser.angles[measid])
          + robotPose.y + map.info.originy;

        if(checkExpansion(xPoint, yPoint, false))
        {
          break;
        }

        int tt = map.p[(unsigned int)xPoint][(unsigned int)yPoint];

        float oldtt = map.p[(unsigned int)xPoint][(unsigned int)yPoint];

        float diff = fabs(tt - 127.0) / 128.0;
        diff = pow(diff, 0.5);

        if(dMeasure > R || ( xt == 0 && yt == 0))
          tt += (1 - diff) * meanDensity * slamParams.density;
        if( dMeasure + 1 > R && dMeasure - 2 < R )
          tt -= (1 - diff) * meanDensity * slamParams.obstacle_density * 
            slamParams.density;

        map.p[(unsigned int)xPoint][(unsigned int)yPoint] = tt;

        prevtt = oldtt;
        R++;
      }
      R = 1;
    }
  }

  /**
    @brief Starts the laser subscriber, listening to laser_subscriber_topic 
      from parameters
    @return void
   **/
  void CrsmSlam::startLaserSubscriber()
  {
    clientLaserValues =
      n.subscribe(
        slamParams.laser_subscriber_topic.c_str(),
        1,
        &CrsmSlam::fixNewScans,
        this);
  }

  /**
    @brief Stops the laser subscriber
    @return void
   **/
  void CrsmSlam::stopLaserSubscriber()
  {
    clientLaserValues.shutdown();
  }

  /**
    @brief Starts the OccupancyGrid publisher, posting to 
      occupancy_grid_publish_topic from parameters
    @return void
   **/
  void CrsmSlam::startOGMPublisher()
  {
    _occupancyGridPublisher = 
      n.advertise<nav_msgs::OccupancyGrid>
      (slamParams.occupancy_grid_publish_topic.c_str(), 1);

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
    @brief Publishes the OccupancyGrid map as nav_msgs::OccupancyGrid, posting 
      with occupancy_grid_map_freq Hz from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishOGM(ros::Time timestamp)
  {
    int width = map.info.width;
    int height = map.info.height;

    nav_msgs::OccupancyGrid grid;

    grid.header.stamp = timestamp;
    grid.header.frame_id = slamParams.map_frame;

    grid.info.resolution = slamParams.ocgd;
    grid.info.width = width;
    grid.info.height = height;

    grid.info.origin.position.x = -(map.info.originx * slamParams.ocgd);
    grid.info.origin.position.y = -(map.info.originy * slamParams.ocgd);
    grid.info.origin.orientation.w = 1;

    grid.data.resize(width * height) ;
    for(int i = 0 ; i < width ; i++)
    {
      for(int j = 0 ; j < height ; j++)
      {
        if (map.p[i][j] == 127)
          grid.data[j * width + i] = -1;
        else
          grid.data[j * width + i] = 100.0-(int) (map.p[i][j]*100.0/255.0);
      }
    }
    _occupancyGridPublisher.publish(grid);
  }

  /**
    @brief Returns the map occupancy probability of coordinates (x,y) 
      ranging from 0-255 (0 is occupied, 255 is free)
    @param x [int] : The x coordinate
    @param y [int] : The y coordinate
    @return char probability
   **/
  char CrsmSlam::getMapProbability(int x,int y)
  {
    return map.p[x][y];
  }

  /**
    @brief Returns the map info in a CrsmMapInfo structure
    @return CrsmMapInfo
   **/
  CrsmMapInfo CrsmSlam::getMapInfo(void)
  {
    return map.info;
  }

  /**
    @brief Returns the robot pose in a CrsmPose structure
    @return CrsmPose
   **/
  CrsmPose CrsmSlam::getRobotPose(void)
  {
    return robotPose;
  }

  /**
    @brief Returns the laser info in a CrsmLaserInfo structure
    @return CrsmLaserInfo
   **/
  CrsmLaserInfo CrsmSlam::getLaserInfo(void)
  {
    return laser.info;
  }

  /**
    @brief Returns the robot trajectory in a vector of CrsmPose structures
    @return std::vector<CrsmPose>
   **/
  std::vector<CrsmPose> CrsmSlam::getTrajectory(void)
  {
    return robotTrajectory;
  }

  /**
    @brief Publishes the Tf robot pose, posting with robot_pose_tf_freq Hz 
      from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishRobotPoseTf(ros::Time timestamp)
  {
    double rx = (robotPose.x - cos(robotPose.theta) *
      (slamParams.dx_laser_robotCenter/slamParams.ocgd)) * slamParams.ocgd;
    double ry = (robotPose.y-sin(robotPose.theta) *
      (slamParams.dx_laser_robotCenter/slamParams.ocgd)) * slamParams.ocgd;
    double rth = robotPose.theta;

    tf::Vector3 translation(rx, ry, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, rth);

    if (slamParams.publish_tf)
    {
      tf::Transform transform(rotation,translation);
      _slamFrameBroadcaster.sendTransform(
        tf::StampedTransform(
          transform,
          timestamp,
          slamParams.map_frame,
          slamParams.base_footprint_frame
          )
        );
    }

    // publish pose on pose topic
    geometry_msgs::PoseWithCovarianceStamped poseOut;
    poseOut.header.stamp = timestamp;
    poseOut.header.frame_id = slamParams.map_frame;
    poseOut.pose.pose.position.x = rx;
    poseOut.pose.pose.position.y = ry;
    tf::quaternionTFToMsg(rotation, poseOut.pose.pose.orientation);
    _posePublisher.publish(poseOut);

    // Update trajectory
    geometry_msgs::PoseStamped pathPoint;
    pathPoint.header.stamp = timestamp;
    pathPoint.pose.position.x = rx;
    pathPoint.pose.position.y = ry;
    tf::Quaternion q;
    q.setRPY(0, 0, rth);
    geometry_msgs::Quaternion gq;
    tf::quaternionTFToMsg(q, gq);
    pathPoint.pose.orientation = gq;
    trajectory.poses.push_back(pathPoint);
  }

  /**
    @brief Starts the Trajectory publisher, posting to 
      robot_trajectory_publish_topic from parameters, with 
      trajectory_publisher_frame_id as frame ID.
    @return void
   **/
  void CrsmSlam::startTrajectoryPublisher()
  {
    _pathPublisher = n.advertise<nav_msgs::Path>
      (slamParams.robot_trajectory_publish_topic.c_str(), 1);
    _pathPublishingTimer.start();
  }

  /**
    @brief Stops the Trajectory publisher.
    @return void
   **/
  void CrsmSlam::stopTrajectoryPublisher(void)
  {
    _pathPublishingTimer.stop();
    _pathPublisher.shutdown();
  }

  /**
    @brief Publishes the robot trajectory as nav_msgs::Path, posting with 
      trajectory_freq Hz from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishTrajectory(ros::Time timestamp)
  {
    trajectory.header.stamp = timestamp;
    trajectory.header.frame_id = slamParams.trajectory_publisher_frame_id;
    _pathPublisher.publish(trajectory);
  }

  //---------------------- Setters for slamParameters ----------------------------//
  /**
    @brief Sets the disparity of CRSM_SlamParameters
    @param disparity [int] Disparity of mutation in pixels at hill climbing
    @return void
   **/
  void CrsmSlam::setDisparity(int disparity)
  {
    slamParams.disparity = disparity;
  }

  /**
    @brief Sets the map_size of CRSM_SlamParameters
    @param size [int] Map size of initial allocated map
    @return void
   **/
  void CrsmSlam::setInitialMapSize(int size)
  {
    slamParams.map_size = size;
  }

  /**
    @brief Sets the ocgd of CRSM_SlamParameters
    @param ocgd [double] [OC]cupancy [G]rid [D]imentionality - the width and 
      height in meters of a pixel
    @return void
   **/
  void CrsmSlam::setOcgd(double ocgd)
  {
    slamParams.ocgd = ocgd;
  }

  /**
    @brief Sets the density of CRSM_SlamParameters
    @param density [double] Map update density (0-127)
    @return void
   **/
  void CrsmSlam::setDensity(double density)
  {
    slamParams.density = density;
  }

  /**
    @brief Sets the obstacle_density of CRSM_SlamParameters
    @param ob_density [double] Coefficient for obstacle update density (0+)
    @return void
   **/
  void CrsmSlam::setObstacleDensity(double ob_density)
  {
    slamParams.obstacle_density = ob_density;
  }

  /**
    @brief Sets the scan_selection_meters of CRSM_SlamParameters
    @param scan_selection_meters [double] Scan density lower boundary for a 
      scan-part identification
    @return void
   **/
  void CrsmSlam::setScanSelectionMeters(double scan_selection_meters)
  {
    slamParams.scan_selection_meters = scan_selection_meters;
  }

  /**
    @brief Sets the max_hill_climbing_iterations of CRSM_SlamParameters
    @param iterations [int] Maximum RRHC iterations
    @return void
   **/
  void CrsmSlam::setMaxHillClimbingIterations(int iterations)
  {
    slamParams.max_hill_climbing_iterations = iterations;
  }

  /**
    @brief Sets the dx_laser_robotCenter of CRSM_SlamParameters
    @param dx [double] Translation in x axis of laser in comparison to robot 
      center
    @return void
   **/
  void CrsmSlam::setDxLaserRobotCenter(double dx)
  {
    slamParams.dx_laser_robotCenter = dx;
  }

  /**
    @brief Sets the occupancy_grid_map_freq of CRSM_SlamParameters
    @param freq [double] The occupancy grid map publishing frequency
    @return void
   **/
  void CrsmSlam::setOccupancyGridMapFreq(double freq)
  {
    slamParams.occupancy_grid_map_freq = freq;
  }

  /**
    @brief Sets the robot_pose_tf_freq of CRSM_SlamParameters
    @param freq [double] The robot pose publishing frequency
    @return void
   **/
  void CrsmSlam::setRobotPoseTfFreq(double freq)
  {
    slamParams.robot_pose_tf_freq = freq;
  }

  /**
    @brief Sets the trajectory_freq of CRSM_SlamParameters
    @param freq [double] The trajectory publishing frequency
    @return void
   **/
  void CrsmSlam::setTrajectoryFreq(double freq)
  {
    slamParams.trajectory_freq = freq;
  }

  /**
    @brief Sets the desired_number_of_picked_rays of CRSM_SlamParameters
    @param rays [int] The desired number of picked rays [algorithm specific]
    @return void
   **/
  void CrsmSlam::setDesiredNumberOfPickedRays(int rays)
  {
    slamParams.desired_number_of_picked_rays = rays;
  }

  /**
    @brief Sets the robot_width of CRSM_SlamParameters
    @param width [double] The robot width
    @return void
   **/
  void CrsmSlam::setRobotWidth(double width)
  {
    slamParams.robot_width = width;
  }

  /**
    @brief Sets the robot_length of CRSM_SlamParameters
    @param length [double] The robot length
    @return void
   **/
  void CrsmSlam::setRobotLength(double length)
  {
    slamParams.robot_length = length;
  }

  /**
    @brief Sets the occupancy_grid_publish_topic of CRSM_SlamParameters
    @param topic [std::string] The occupancy grid publishing topic
    @return void
   **/
  void CrsmSlam::setOccupancyGridPublishTopic(std::string topic)
  {
    slamParams.occupancy_grid_publish_topic = topic;
  }

  /**
    @brief Sets the robot_trajectory_publish_topic of CRSM_SlamParameters
    @param topic [std::string] The trajectory publishing topic
    @return void
   **/
  void CrsmSlam::setRobotTrajectoryPublishTopic(std::string topic)
  {
    slamParams.robot_trajectory_publish_topic = topic;
  }

  /**
    @brief Sets the trajectory_publisher_frame_id of CRSM_SlamParameters
    @param frame_id [std::string] The trajectory frame ID
    @return void
   **/
  void CrsmSlam::setTrajectoryPublisherFrameId(std::string frame_id)
  {
    slamParams.trajectory_publisher_frame_id = frame_id;
  }

  /**
    @brief Sets the laser_subscriber_topic of CRSM_SlamParameters
    @param topic [std::string] The laser subscriber topic
    @return void
   **/
  void CrsmSlam::setLaserSubscriberTopic(std::string topic)
  {
    slamParams.laser_subscriber_topic = topic;
  }

  /**
    @brief Sets the base_footprint_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the base footprint frame - (x,y,yaw)
    @return void
   **/
  void CrsmSlam::setBaseFootprintFrame(std::string frame)
  {
    slamParams.base_footprint_frame = frame;
  }

  /**
    @brief Sets the base_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the base frame
    @return void
   **/
  void CrsmSlam::setBaseFrame(std::string frame)
  {
    slamParams.base_frame = frame;
  }

  /**
    @brief Sets the map_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the map frame
    @return void
   **/
  void CrsmSlam::setMapFrame(std::string frame)
  {
    slamParams.map_frame = frame;
  }

  //------------------- Getters for slamParameters ----------------------//

  /**
    @brief Gets the disparity of CRSM_SlamParameters
    @return int Disparity of mutation in pixels at hill climbing
   **/
  int CrsmSlam::getDisparity(void)
  {
    return slamParams.disparity;
  }

  /**
    @brief Gets the map_size of CRSM_SlamParameters
    @return int Map size of initial allocated map
   **/
  int CrsmSlam::getInitialMapSize(void)
  {
    return slamParams.map_size;
  }

  /**
    @brief Gets the ocgd of CRSM_SlamParameters
    @return double [OC]cupancy [G]rid [D]imentionality - 
      the width and height in meters of a pixel
   **/
  double CrsmSlam::getOcgd(void)
  {
    return slamParams.ocgd;
  }

  /**
    @brief Gets the density of CRSM_SlamParameters
    @return double Map update density (0-127)
   **/
  double CrsmSlam::getDensity(void)
  {
    return slamParams.density;
  }

  /**
    @brief Gets the obstacle_density of CRSM_SlamParameters
    @return double Coefficient for obstacle update density (0+)
   **/
  double CrsmSlam::getObstacleDensity(void)
  {
    return slamParams.obstacle_density;
  }

  /**
    @brief Gets the scan_selection_meters of CRSM_SlamParameters
    @return double Scan density lower boundary for a scan-part identification
   **/
  double CrsmSlam::getScanSelectionMeters(void)
  {
    return slamParams.scan_selection_meters;
  }

  /**
    @brief Gets the max_hill_climbing_iterations of CRSM_SlamParameters
    @return int Maximum RRHC iterations
   **/
  int CrsmSlam::getMaxHillClimbingIterations(void)
  {
    return slamParams.max_hill_climbing_iterations;
  }

  /**
    @brief Gets the dx_laser_robotCenter of CRSM_SlamParameters
    @return double Translation in x axis of laser in comparison to robot center
   **/
  double CrsmSlam::getDxLaserRobotCenter(void)
  {
    return slamParams.dx_laser_robotCenter;
  }

  /**
    @brief Gets the occupancy_grid_map_freq of CRSM_SlamParameters
    @return double The occupancy grid map publishing frequency
   **/
  double CrsmSlam::getOccupancyGridMapFreq(void)
  {
    return slamParams.occupancy_grid_map_freq;
  }

  /**
    @brief Gets the robot_pose_tf_freq of CRSM_SlamParameters
    @return double The robot pose publishing frequency
   **/
  double CrsmSlam::getRobotPoseTfFreq(void)
  {
    return slamParams.robot_pose_tf_freq;
  }

  /**
    @brief Gets the trajectory_freq of CRSM_SlamParameters
    @return double The trajectory publishing frequency
   **/
  double CrsmSlam::getTrajectoryFreq(void)
  {
    return slamParams.trajectory_freq;
  }

  /**
    @brief Gets the desired_number_of_picked_rays of CRSM_SlamParameters
    @return int The desired number of picked rays [algorithm specific]
   **/
  int CrsmSlam::getDesiredNumberOfPickedRays(void)
  {
    return slamParams.desired_number_of_picked_rays;
  }

  /**
    @brief Gets the robot_width of CRSM_SlamParameters
    @return double The robot width
   **/
  double CrsmSlam::getRobotWidth(void)
  {
    return slamParams.robot_width;
  }

  /**
    @brief Gets the robot_length of CRSM_SlamParameters
    @return double The robot length
   **/
  double CrsmSlam::getRobotLength(void)
  {
    return slamParams.robot_length;
  }

  /**
    @brief Gets the occupancy_grid_publish_topic of CRSM_SlamParameters
    @return std::string The occupancy grid publishing topic
   **/
  std::string CrsmSlam::getOccupancyGridPublishTopic(void)
  {
    return slamParams.occupancy_grid_publish_topic;
  }

  /**
    @brief Gets the robot_trajectory_publish_topic of CRSM_SlamParameters
    @return std::string The trajectory publishing topic
   **/
  std::string CrsmSlam::getRobotTrajectoryPublishTopic(void)
  {
    return slamParams.robot_trajectory_publish_topic;
  }

  /**
    @brief Gets the trajectory_publisher_frame_id of CRSM_SlamParameters
    @return std::string The trajectory frame ID
   **/
  std::string CrsmSlam::getTrajectoryPublisherFrameId(void)
  {
    return slamParams.trajectory_publisher_frame_id;
  }

  /**
    @brief Gets the laser_subscriber_topic of CRSM_SlamParameters
    @return std::string The laser subscriber topic
   **/
  std::string CrsmSlam::getLaserSubscriberTopic(void)
  {
    return slamParams.laser_subscriber_topic;
  }

  /**
    @brief Gets the base_footprint_frame of CRSM_SlamParameters
    @return std::string Holds the base footprint frame - (x,y,yaw)
   **/
  std::string CrsmSlam::getBaseFootprintFrame(void)
  {
    return slamParams.base_footprint_frame;
  }

  /**
    @brief Gets the base_frame of CRSM_SlamParameters
    @return std::string Holds the base frame
   **/
  std::string CrsmSlam::getBaseFrame(void)
  {
    return slamParams.base_frame;
  }

  /**
    @brief Gets the map_frame of CRSM_SlamParameters
    @return std::string Holds the map frame
   **/
  std::string CrsmSlam::getMapFrame(void)
  {
    return slamParams.map_frame;
  }

}
