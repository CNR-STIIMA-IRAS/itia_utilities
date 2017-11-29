#include <kdl/frames.hpp>
#include <eigen3/Eigen/Core>

namespace EigenKDL
{
  
  inline KDL::Vector vector(Eigen::Vector3d _Vec)
  {
    return  KDL::Vector(_Vec(0),_Vec(1),_Vec(2));
  };
  
  inline Eigen::Vector3d vector(KDL::Vector _Vec)
  {
      return  Eigen::Vector3d(_Vec.data[0],_Vec.data[1],_Vec.data[2]);
  };
  
  inline Eigen::Matrix4d frame(KDL::Frame _T)
  {
    
    Eigen::Matrix4d T;
    T(0,0) = _T.M.data[0];
    T(0,1) = _T.M.data[1];
    T(0,2) = _T.M.data[2];
    T(1,0) = _T.M.data[3];
    T(1,1) = _T.M.data[4];
    T(1,2) = _T.M.data[5];
    T(2,0) = _T.M.data[6];
    T(2,1) = _T.M.data[7];
    T(2,2) = _T.M.data[8];
    T(0,3) = _T.p.data[0];
    T(1,3) = _T.p.data[1];
    T(2,3) = _T.p.data[2];
    T(3,0) = 0.0;
    T(3,1) = 0.0;
    T(3,2) = 0.0;
    T(3,3) = 1.0;
    
    return T;
    
  };

  inline KDL::Frame frame( Eigen::Matrix4d _T )
  {
    KDL::Frame T;
    T.M.data[0] = _T(0,0);
    T.M.data[1] = _T(0,1);
    T.M.data[2] = _T(0,2);
    T.M.data[3] = _T(1,0);
    T.M.data[4] = _T(1,1);
    T.M.data[5] = _T(1,2);
    T.M.data[6] = _T(2,0);
    T.M.data[7] = _T(2,1);
    T.M.data[8] = _T(2,2);
    T.p.data[0] = _T(0,3);
    T.p.data[1] = _T(1,3);
    T.p.data[2] = _T(2,3);
    return T;
  };
    
}