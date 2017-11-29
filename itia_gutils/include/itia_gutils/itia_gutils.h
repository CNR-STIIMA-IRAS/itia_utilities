#ifndef __ITIA_GUTILS__
#define __ITIA_GUTILS__

#include <math.h>
#include <vector>
#include <iostream>
#include <exception>
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <itia_futils/itia_futils.h>
#include <itia_gutils/EigenKDL.h>


namespace itia
{
  namespace gutils
  {
	
    KDL::Vector cross(const KDL::Vector& in1, const KDL::Vector& in2);
    KDL::Rotation orthonorm(const KDL::Rotation& in);
    
    double cartesian_norm( const std::vector<double>& p1, 
                           const std::vector<double>& p2 );
	
    bool circle_trj(  const Eigen::Matrix4d& center, 
                      const double& radius, const double& eight, 
                      const int& n_pnts, 
                      std::vector<Eigen::Matrix4d>* circ_points //if eight=0 the points are only translated, otherwise they are rotate to form a cone
                   );
    
    Eigen::Matrix4d ComauToMatrix(const std::vector<double>& pose);

    Eigen::Matrix4d ArrayToMatrix(const Eigen::Array<double,1,6>& pose);
	
    Eigen::Array<double,1,6> MatrixToArray(const Eigen::Matrix4d& T);
	
    Eigen::Array<double,6,1> stdVectorToArray(const std::vector<double>& vec);
    
    
    class FourPointsMethod
    {
    private:
      int nPnt;
      Eigen::MatrixXd A;
      Eigen::VectorXd b;
      Eigen::Vector3d center;
      Eigen::Vector3d x_axis;
      Eigen::Vector3d y_axis;
      Eigen::Vector3d z_axis;
      Eigen::Vector3d distance;
      Eigen::Matrix<double,4,4> Ttool;
      double radius;
      bool circle_done;
      bool tra_done;
      bool rot_done;
      bool verbose;
      
      bool rankCompleted();
      bool getCircle();
      bool translate(const Eigen::Matrix<double,4,4>& T);
        
    public:
        /*
        FourPointsMethod identify the tool frame starting by four points.
        You need to acquire at less four points and pass them to FourPointsMethod object by using 'addpoint' method.
        It returns true when the points are enough. 
        Finally, use 'orientate' to select the desired orientation, by passing the new z-axis and the x-axis
        */
        FourPointsMethod();
        bool addpoint(  const Eigen::Matrix<double,4,4>& T);
        bool orientate( const Eigen::Matrix<double,4,4>& T,
                        const Eigen::Vector3d& _z_axis,
                        const Eigen::Vector3d& _x_axis);
        bool getFrame(Eigen::Matrix<double,4,4>* T);
    };
    
  }
}

#endif
