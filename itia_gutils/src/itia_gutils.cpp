
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

#include <itia_gutils/itia_gutils.h>

namespace itia
{
namespace gutils
{
   	
  KDL::Vector cross(const KDL::Vector& in1, const KDL::Vector& in2)
  {
    
    return KDL::Vector(in1.y()*in2.z()-in1.z()*in2.y(), 
                      in1.z()*in2.x()-in1.x()*in2.z(), 
                      in1.x()*in2.y()-in1.y()*in2.x());
  }
  
  KDL::Rotation orthonorm(const KDL::Rotation& in)
  {
    KDL::Vector Z = in.UnitZ();
    KDL::Vector Y = in.UnitY();
    KDL::Vector X = in.UnitX();
    Z.Normalize();
    Y = Y -dot(Y, Z) *Z;
    Y.Normalize();
    X = cross(Y, Z);
    X.Normalize();
    return KDL::Rotation(X, Y, Z);
  }
  
  double cartesian_norm(  const std::vector<double>& p1,
                          const std::vector<double>& p2 )
  {
      
    if (p1.size() != p2.size() )
      return 0.0;
    
    return sqrt( pow(p1.at(0) - p2.at(0), 2) + pow(p1.at(1) - p2.at(1), 2) + pow(p1.at(2) - p2.at(2), 2) );
  };
  
  bool circle_trj(  const Eigen::Matrix4d& center,
                    const double& radius,
                    const double& eight, 
                    const int& n_pnts,
                    std::vector<Eigen::Matrix4d>* circ_points )
  {
      
    if (n_pnts<=0)
      throw std::invalid_argument ( "[ itia_gutils - circle_trj ] Error: the number of point has to be greater than zero" );
    
    Eigen::Vector3d normal_1;
    Eigen::Vector3d normal_2;
    
    Eigen::Vector3d Zcenter;
    Eigen::Vector3d Ycenter;
    Eigen::Vector3d Ocenter;
    
    Zcenter(0) = center(0,2);
    Zcenter(1) = center(1,2);
    Zcenter(2) = center(2,2);
    Ycenter(0) = center(0,1);
    Ycenter(1) = center(1,1);
    Ycenter(2) = center(2,1);
    Ocenter(0) = center(0,3);
    Ocenter(1) = center(1,3);
    Ocenter(2) = center(2,3);
    
    circ_points->resize(n_pnts);
    double alpha = atan2(radius,eight);
    KDL::Frame _center = EigenKDL::frame(center);
    KDL::Frame T = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0.0,0.0,-eight));
    
    printf( " [ %s%s:%d%s ]\t T= \n", YELLOW, __FUNCFILE__, __LINE__, RESET );
    std::cout << EigenKDL::frame(T) << std::endl;
    
    KDL::Frame _vertex = _center*T;
    
    return true;
  };

  Eigen::Matrix4d ComauToMatrix( const std::vector<double>& pose )
  {
      
    Eigen::Matrix4d T;
    double theta1=pose.at(3)*M_PI/180.0;
    double theta2=pose.at(4)*M_PI/180.0;
    double theta3=pose.at(5)*M_PI/180.0;
    T(0,0) = -sin(theta1)*sin(theta3)+cos(theta1)*cos(theta2)*cos(theta3);
    T(0,1) = -cos(theta3)*sin(theta1)-cos(theta1)*cos(theta2)*sin(theta3);
    T(0,2) = cos(theta1)*sin(theta2);
    T(1,0) = cos(theta1)*sin(theta3)+cos(theta2)*cos(theta3)*sin(theta1);
    T(1,1) = cos(theta1)*cos(theta3)-cos(theta2)*sin(theta1)*sin(theta3);
    T(1,2) = sin(theta1)*sin(theta2);
    T(2,0) = -cos(theta3)*sin(theta2);
    T(2,1) = sin(theta2)*sin(theta3);
    T(2,2) = cos(theta2);
    
    T(0,3) = pose.at(0);
    T(1,3) = pose.at(1);
    T(2,3) = pose.at(2);
    
    T(3,0) = 0.0;
    T(3,1) = 0.0;
    T(3,2) = 0.0;
    T(3,3) = 1.0;
    return T;
  }

  Eigen::Matrix4d ArrayToMatrix( const Eigen::Array<double,1,6>& pose )
  { //Convenzioni angolari Comau (ZYZ) ma in m,rad
      
      Eigen::Matrix4d T;
      double theta1=pose(3);
      double theta2=pose(4);
      double theta3=pose(5);
      T(0,0) = -sin(theta1)*sin(theta3)+cos(theta1)*cos(theta2)*cos(theta3);
      T(0,1) = -cos(theta3)*sin(theta1)-cos(theta1)*cos(theta2)*sin(theta3);
      T(0,2) = cos(theta1)*sin(theta2);
      T(1,0) = cos(theta1)*sin(theta3)+cos(theta2)*cos(theta3)*sin(theta1);
      T(1,1) = cos(theta1)*cos(theta3)-cos(theta2)*sin(theta1)*sin(theta3);
      T(1,2) = sin(theta1)*sin(theta2);
      T(2,0) = -cos(theta3)*sin(theta2);
      T(2,1) = sin(theta2)*sin(theta3);
      T(2,2) = cos(theta2);
      
      T(0,3) = pose(0);
      T(1,3) = pose(1);
      T(2,3) = pose(2);
      
      T(3,0) = 0.0;
      T(3,1) = 0.0;
      T(3,2) = 0.0;
      T(3,3) = 1.0;
      return T;
  }
  
  Eigen::Array<double,1,6> MatrixToArray( const Eigen::Matrix4d& T )
  { //Convenzioni angolari Comau (ZYZ) ma in m,rad
  
    Eigen::Array<double,1,6> pose;
      
    pose(0) = T(0,3);
    pose(1) = T(1,3);
    pose(2) = T(2,3);
      
    pose(3) = atan2(T(1,2),T(0,2));
    pose(4) = acos(T(2,2));
    pose(5) = atan2(T(2,1),-T(2,0));
      
    return pose;
    
  }

  Eigen::Array<double,6,1> stdVectorToArray( const std::vector<double>& vec )
  {
   
    Eigen::ArrayXd res(6);
    for(int i=0;i<6;i++)
      res(i) = vec.at(i);

    return res;	
    
  }

  
  FourPointsMethod::FourPointsMethod()
  {
    nPnt = 0;
    center.setZero();
    x_axis.setZero();
    y_axis.setZero();
    z_axis.setZero();
    distance.setZero();
    Ttool.setIdentity();
    
    circle_done = false;
    tra_done    = false;
    rot_done    = false;
    verbose     = true;
  };
      
  bool FourPointsMethod::addpoint(const Eigen::Matrix<double,4,4>& T)
  {
    
    Eigen::Vector3d x;
    x(0) = T(0,3);
    x(1) = T(1,3);
    x(2) = T(2,3);

    nPnt++;
    A.conservativeResize(nPnt,4);
    b.conservativeResize(nPnt);
    
    A(nPnt-1,0) = x(0);
    A(nPnt-1,1) = x(1);
    A(nPnt-1,2) = x(2);
    A(nPnt-1,3) = 1;
    b(nPnt-1,0) = x.dot(x);
    
    
    if (this->getCircle())
      if (this->translate(T))
        return true;
      
    return false;
  };
  
  bool FourPointsMethod::orientate( const Eigen::Matrix<double,4,4>& T,
                                    const Eigen::Vector3d& _z_axis,
                                    const Eigen::Vector3d& _x_axis)
  {
    
    if (!circle_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to acquire the points before (use addpoint) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    
    if (!tra_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to translate the tool frame before (use translate) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    
    z_axis = _z_axis;
    x_axis = _x_axis;
    
    if ( std::abs(x_axis.dot(z_axis))>1e-2 )
    {
      printf( " [ %s%s:%d%s ]\t the axis X and Z are not perpendicular: x.dot(y) = %5.4f\n", YELLOW, __FUNCFILE__, __LINE__, RESET, x_axis.dot(z_axis) ); 
      return false;
    }
    
    x_axis = x_axis-(x_axis.dot(z_axis))*z_axis;
    y_axis = z_axis.cross(x_axis);
    
    Eigen::Matrix<double,3,3> orientatedToolRot;
    orientatedToolRot.col(0) = x_axis;
    orientatedToolRot.col(1) = y_axis;
    orientatedToolRot.col(2) = z_axis;
    
    Eigen::Matrix<double,4,4> T_toolFrame = T*Ttool; //T in (non-orientated) tool frame
    Eigen::Matrix<double,4,4> T_orientatedToolFrame; //T in (non-orientated) tool frame TrotTool=T*Tool*Tori
    T_orientatedToolFrame.block(0,0,3,3) = orientatedToolRot;
    T_orientatedToolFrame.block(3,0,1,4) << 0,0,0,1;
    T_orientatedToolFrame.block(0,3,3,1) = T_toolFrame.block(0,3,3,1);
    
    Eigen::Matrix<double,4,4> Tori = T_toolFrame.inverse()*T_orientatedToolFrame;
    Ttool = Ttool*Tori;
    rot_done = true;
    return true;
  };

  bool FourPointsMethod::getFrame( Eigen::Matrix< double, int ( 4 ), int ( 4 ) >* T )
  {
    
    if (!circle_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to acquire the points before (use addpoint) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    
    if (!tra_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to translate the tool frame before (use translate) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    
    if (!rot_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to orientate the tool frame before (use orientate) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    
    *T = Ttool;
    
    return true;
  }
  
  bool FourPointsMethod::rankCompleted( )
  {
    
    if (nPnt<4)
        return false;
    else
    {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
      if (svd.singularValues()(svd.singularValues().size()-1)==0)
        return false;
      
      double cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
      
      if (verbose)
        printf( " [ %s%s:%d%s ]\t condion number is %f = \n", YELLOW, __FUNCFILE__, __LINE__, RESET, cond );
      
      if (std::abs(cond)>1000)
          return false;
      
      return true;
    }
    
  };

  bool FourPointsMethod::getCircle( )
  {
    
    if (!this->rankCompleted())
    {
      printf( " [ %s%s:%d%s ]\t The matrix is rank deficient, please acquire more points \n", YELLOW, __FUNCFILE__, __LINE__, RESET );  
      return false;
    } 
    else 
    {
      Eigen::VectorXd pars(4);
      pars = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
      center(0) = pars(0)/2;
      center(1) = pars(1)/2;
      center(2) = pars(2)/2;
      radius = sqrt(pars(3)+center.dot(center));
      circle_done = true;
      
      if (verbose)
        printf( " [ %s%s:%d%s ]\t center = %4.3f %4.3f %4.3f   radius = %4.3f \n", YELLOW, __FUNCFILE__, __LINE__, RESET, center(0), center(1), center(2), radius );
      
      return true;
    }
    
  };

  bool FourPointsMethod::translate( const Eigen::Matrix<double,4,4>& T )
  {
    
    if (!circle_done)
    {
      printf( " [ %s%s:%d%s ]\t You have to acquire the points before (use addpoint) \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    }
    Eigen::Vector3d point;
    distance(0) = T(0,3)-center(0);
    distance(1) = T(1,3)-center(1);
    distance(2) = T(2,3)-center(2);
    double checkCircle = distance.dot(distance) - pow(radius,2.0);
    if (std::abs(checkCircle)>0.01*pow(radius,2.0))
    {
      printf( " [ %s%s:%d%s ]\t the point is not in the circle \n", YELLOW, __FUNCFILE__, __LINE__, RESET ); 
      return false;
    } 
    else 
    {
      distance = -T.block(0,0,3,3).transpose()*distance;
      Ttool(0,3) = distance(0);
      Ttool(1,3) = distance(1);
      Ttool(2,3) = distance(2);
      Ttool.block(0,0,3,3) = T.block(0,0,3,3);
    }
    tra_done=true;
    return true;
  }
  
}
}

