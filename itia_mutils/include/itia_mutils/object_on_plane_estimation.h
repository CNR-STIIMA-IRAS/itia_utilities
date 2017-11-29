#ifndef __ITIA_MUTILS__OBJECT_PLANE_ESTIMATION__
#define __ITIA_MUTILS__OBJECT_PLANE_ESTIMATION__

#include <Eigen/Geometry>

namespace itia
{
namespace mutils
{

inline Eigen::Affine3d computeTmatrixFromTwoPlanarPoints(const Eigen::Vector3d& p1_in_b, const Eigen::Vector3d& p2_in_b, Eigen::Vector3d Zaxis_in_b, const double& toll = 1e-3)
{
  Eigen::Vector3d tra_12_in_a = p1_in_b-p2_in_b;
  
  if (std::abs(Zaxis_in_b.dot(tra_12_in_a))>toll)
  {
    ROS_ERROR("Points 1 and 2 are not in the plane");
  }
  Eigen::Vector3d Xaxis_in_b = tra_12_in_a- tra_12_in_a.dot(Zaxis_in_b) *Zaxis_in_b;
  Xaxis_in_b /= Xaxis_in_b.norm();
  
  Eigen::Vector3d Yaxis_in_b = Zaxis_in_b.cross(Xaxis_in_b);
  
  Eigen::Matrix3d mR_ba;
  mR_ba.col(0)=Xaxis_in_b;
  mR_ba.col(1)=Yaxis_in_b;
  mR_ba.col(2)=Zaxis_in_b;
  
  Eigen::Affine3d T_b1;
  T_b1.linear() = mR_ba;
  T_b1.translation() = p1_in_b;
  return T_b1;
  
};


inline Eigen::Affine3d computeQmatrix4pose( const Eigen::Vector3d& p1a_in_c, 
                                            const Eigen::Vector3d& p2a_in_c, 
                                            const Eigen::Vector3d& p1b_in_c, 
                                            const Eigen::Vector3d& p2b_in_c, 
                                            const Eigen::Vector3d& Zaxis_in_b, 
                                            const double& toll_planarity = 1e-2, 
                                            const double& tool_distance = 2e-2
 )
{
  Eigen::Affine3d T_ca, T_cb;
  if ( std::abs((p1a_in_c-p2a_in_c).norm()-(p1b_in_c-p2b_in_c).norm()) >tool_distance )
  {
    ROS_WARN("distances of points 1 and 2 are different in pose 'a' w.r.t to pose 'b'");
  }
  T_ca  = itia::mutils::computeTmatrixFromTwoPlanarPoints(p1a_in_c, p2a_in_c, Zaxis_in_b, toll_planarity);
  T_cb  = itia::mutils::computeTmatrixFromTwoPlanarPoints(p1b_in_c, p2b_in_c, Zaxis_in_b, toll_planarity);
  
  // Q_ab_in_c = T_ca * T_bc
  // Q_ab_in_d = T_dc * Q_ab_in_c * T_cd
  Eigen::Affine3d Q_ab_in_c = T_ca *T_cb.inverse();
  return Q_ab_in_c;
  
}

}
}
#endif