#include <itia_tutils/itia_tutils.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_client");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting trajectory executor");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("follow_joint_trajectory",true);
  
  control_msgs::FollowJointTrajectoryGoal goal;
  
  if (!itia::tutils::getTrajectoryFromParam(nh,"/trajectory",goal.trajectory))
  {
    ROS_ERROR("ERROR LOADING TRAJECTORY");
    return -1;
  }
  
  bool log=false;
  if (!nh.getParam("start_log",log))
  {
    log=false;
  }
  
  double log_rest_time=0;
  if (log)
  {
    if (!nh.getParam("log_rest_time",log_rest_time))
    {
      ROS_INFO("/log_rest_time is not set, the log will finish 3s after  the end of the trajectory");
      log_rest_time=3;
    }
  }
  
  int repetition=1;
  if (log)
  {
    if (!nh.getParam("log_repetition",repetition))
    {
      ROS_INFO("/repetition is not set, execute trj only once");
      repetition=1;
    }
  }
  
  ros::ServiceClient start_log = nh.serviceClient<std_srvs::Empty>("/start_log");
  ros::ServiceClient stop_log  = nh.serviceClient<std_srvs::Empty>("/stop_log");
  
  control_msgs::JointTolerance tol;
  if (!nh.getParam("path_tolerance",tol.position))
    tol.position = 0.001;
  goal.path_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.path_tolerance.begin(),goal.path_tolerance.end(),tol);
  
  if (!nh.getParam("goal_tolerance",tol.position))
    tol.position = 0.00;
  goal.goal_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),tol);
  
  std::string log_name;
  nh.getParam("/binary_logger/test_name",log_name);
  
  
  ROS_INFO("number of repetition %d",repetition);
  
  for (int iR=0;iR<repetition;iR++)
  {
    ROS_INFO("Waiting for action server. repetition= %d of %d",iR+1,repetition);
    ac.waitForServer(ros::Duration(0.0));
    
    if (repetition>1)
    {
      nh.setParam("/binary_logger/test_name",log_name+std::to_string(iR+1));
    }
    if (log)
    {
      log=start_log.waitForExistence(ros::Duration(1));
      if (!log)
        ROS_WARN("no logging service available!!");
    }
    ac.cancelAllGoals();
    
    goal.trajectory.header.stamp=ros::Time::now();
    ROS_INFO("Waiting for trajectory execution");
    
    std_srvs::Empty srv;
    start_log.call(srv);
    ac.sendGoalAndWait(goal);
    
    ros::Duration(log_rest_time).sleep();
    stop_log.call(srv);
    
    if (ac.getResult()->error_code)
      ROS_ERROR("error_code = %d, string = %s",ac.getResult()->error_code,ac.getResult()->error_string.c_str());
  }
  
}
