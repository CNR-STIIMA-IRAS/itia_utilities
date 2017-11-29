
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
