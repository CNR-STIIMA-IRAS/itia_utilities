#include <itia_mutils/eiquadprog.hpp>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <itia_mutils/frame_distance.h>
#include <itia_mutils/quaternion.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_mutils_test");
  ros::NodeHandle nh;
  
  Eigen::Affine3d Twa, Twb;
  Eigen::VectorXd distance(6);
  Eigen::MatrixXd J(6, 6);
  
  Twa.translation().x() = 1;
  Twa.linear().setIdentity();
  Twb.linear().setIdentity();
  Twb = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/7.0, Eigen::Vector3d::UnitZ());
                
  itia::mutils::getFrameDistanceJac(Twa, Twb, &distance, &J);
  Eigen::MatrixXd twists(6, 4);
  twists.setZero();
  twists(1, 1) =0.124;
  twists(3, 0) =1;
  twists(4, 0) =2;
  twists(5, 0) =1;
  twists(3, 1) =1;
  twists(4, 2) =1;
  twists(5, 3) =1;
  
  Eigen::MatrixXd twists2(6, 4);
  twists2.setZero();
  
  Eigen::MatrixXd motion = itia::mutils::AffineToVector(Twa, twists);
  Eigen::Affine3d Twa2 = itia::mutils::VectorToAffine(motion.col(0));
  Eigen::Affine3d Twa3 = itia::mutils::VectorToAffine(motion, &twists2);
  
  ROS_INFO_STREAM("motion      = \n" <<  motion);
  ROS_INFO_STREAM("quat_motion = \n" <<  itia::mutils::PolyVectorToQuatVector(motion));
  return 0;
  ROS_INFO_STREAM("Twa      = \n" <<  Twa.matrix());
  ROS_INFO_STREAM("Twb      = \n" <<  Twb.matrix());
  ROS_INFO_STREAM("distance = \n" <<  distance);
  ROS_INFO_STREAM("J        = \n" <<  J);
  ROS_INFO_STREAM("Twa2     = \n" <<  Twa2.matrix());
  ROS_INFO_STREAM("Twa3     = \n" <<  Twa3.matrix());
  ROS_INFO_STREAM("twist1   = \n" <<  twists);
  ROS_INFO_STREAM("twist2   = \n" <<  twists2);
  
  
//   unsigned int order=3;
//   unsigned int n_E=1;
//   unsigned int n_I=1;
//   
//   Eigen::MatrixXd G(order,order);
//   Eigen::VectorXd g0(order);
//   Eigen::MatrixXd CE;
//   Eigen::VectorXd ce0;
//   Eigen::MatrixXd CI;
//   Eigen::VectorXd ci0;
//   Eigen::VectorXd x(order);
//   G.setIdentity();
//   g0.setZero();
// 
//   if (n_E>0)
//   {
//     CE.resize(order,n_E);
//     ce0.resize(n_E);
//     CE.setZero();
//     ce0.setZero();  
//   }
//   if (n_I>0)
//   {
//     CI.resize(order,n_I);
//     ci0.resize(n_I);
//     CI.setZero();
//     ci0.setZero();
//   }
//   
// //   for (int idx=0;idx<order;idx++)
// //     g0(idx)=idx;
//   
//   CI(0,0) =  1;
//   ci0(0)  = -1;
//   
//   CE(0,0) =  -1;
//   CE(1,0) =  1;
//   ce0(0)  = -4;
//   
//   ROS_INFO("a");
//   x=-g0;
//   ros::Time t0=ros::Time::now();
//   double cost=Eigen::solve_quadprog(G,g0,CE,ce0,CI,ci0,x);
//   ros::Duration dt=ros::Time::now()-t0;
//   
//   
//   std::cout << "req.time   = " << dt.toSec() << std::endl;
//   std::cout << "solution x = " << x.transpose() << std::endl;
//   std::cout << "cost       = " << cost << std::endl;
  
  return 0;  
}