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
