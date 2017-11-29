# ifndef __ITIA_RUTILS_CONVERSIONS__
# define __ITIA_RUTILS_CONVERSIONS__

# include <geometry_msgs/Pose.h>
# include <geometry_msgs/Wrench.h>
# include <tf_conversions/tf_eigen.h>
# include <tf_conversions/tf_kdl.h>
# include <eigen_conversions/eigen_msg.h>

namespace itia
{
namespace rutils
{
  
geometry_msgs::Wrench changeWrenchFrame(const geometry_msgs::Wrench& w_b, const KDL::Frame& T_ab);
geometry_msgs::Wrench changeWrenchFrame(const geometry_msgs::Wrench& w_b, const geometry_msgs::Pose& T_ab);

inline geometry_msgs::Wrench changeWrenchFrame(const geometry_msgs::Wrench& w_b, const KDL::Frame& T_ab)
{
  geometry_msgs::Wrench w_a;
  KDL::Wrench kdl_w_b,  kdl_w_a;
  tf::wrenchMsgToKDL( w_b, kdl_w_b);
  kdl_w_a= T_ab*kdl_w_b;
  tf::wrenchKDLToMsg(kdl_w_a, w_a);
  return w_a;
};

inline geometry_msgs::Wrench changeWrenchFrame(const geometry_msgs::Wrench& w_b, const geometry_msgs::Pose& T_ab)
{
  
  KDL::Frame frame_ab;
  tf::poseMsgToKDL(T_ab, frame_ab);
  return changeWrenchFrame(w_b, frame_ab);
};


}
}

#endif