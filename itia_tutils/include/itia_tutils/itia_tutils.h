
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

#ifndef __ITIA_TRJ_UTILS__
#define __ITIA_TRJ_UTILS__

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <itia_futils/itia_futils.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <itia_rutils/itia_rutils.h>
namespace itia
{
namespace tutils
{

inline bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
{
  std::vector<std::vector<double>> positions;
  if (!itia::rutils::getParamMatrix(nh, trj_name+"/positions", positions))
  {
    ROS_ERROR("%s/positions does not exist", trj_name.c_str());
    return false;
  }
  unsigned int npnt = positions.size();
	
	std::vector<std::vector<double>> velocities;
	if (!itia::rutils::getParamMatrix(nh, trj_name+"/velocities", velocities))
	{
		ROS_ERROR("%s/velocities does not exist", trj_name.c_str());
	}
	
  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no points", trj_name.c_str());
    return -1;
  }
  int dimension = positions.at(0).size();
  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no dimensions", trj_name.c_str());
    return false;
  }
  
  std::vector<double> time;
  if (!nh.getParam(trj_name+"/time_from_start", time))
  {
    ROS_ERROR("%s/time_from_start does not exist", trj_name.c_str());
    return false;
  }
  if (time.size() != npnt)
  {
    ROS_ERROR("%s/time_from_start has wrong dimensions", trj_name.c_str());
    return false;
  }
  trj.points.resize(npnt);
  
  trj.joint_names.resize(dimension);
  if (!nh.getParam(trj_name+"/joint_names",trj.joint_names))
  {
    for (int idx=0;idx<dimension;idx++)
      trj.joint_names.at(idx)= "joint_"+std::to_string(idx);
  }
  
  for (int iPnt = 0;iPnt<npnt;iPnt++)
  {
    trj.points.at(iPnt).positions.resize(dimension);
		trj.points.at(iPnt).positions = positions.at(iPnt);
		if (velocities.size()>0)
		{
			trj.points.at(iPnt).velocities.resize(dimension);
			trj.points.at(iPnt).velocities = velocities.at(iPnt);
		}
    trj.points.at(iPnt).time_from_start = ros::Duration(time.at(iPnt));
  }
 return true;
}

};
};

#endif
