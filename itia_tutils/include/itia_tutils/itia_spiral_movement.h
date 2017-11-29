
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

#ifndef __ITIA_SPIRAL_MOVEMENTS__
#define __ITIA_SPIRAL_MOVEMENTS__

# include <Eigen/Geometry>
# include <itia_msgs/MotionStamped.h>
# include <itia_msgs/motion_conversions.h>

namespace itia
{
namespace tutils
{
  /*
   * SpiralMovement(const Eigen::Affine3d& Tbc, const double& velocity, const double& width,  const double& dt = 1e-2)
   */
  class SpiralMovement
  {
  protected:
    double m_alpha;                                         // spiral angle
    double m_velocity;                                      // velocity
    double m_dt;                                            // time increment
    double m_width;                                         // spiral "round distance"
    Eigen::Affine3d m_Tbc;                                  // spiral center
    
    double getAlphaIncrement(double dt)
    {
      return m_velocity / ( m_width / ( 2 * M_PI ) * sqrt ( pow ( m_alpha, 2 ) + 1 ) ) * dt;
    }
  public:
    SpiralMovement(const Eigen::Affine3d& Tbc, const double& velocity, const double& width,  const double& dt = 1e-2)
    {
      m_alpha = 0;
      m_dt = dt;
      m_velocity = velocity;
      m_Tbc = Tbc;
      m_width = width;
    };
    
    Eigen::Vector3d getSpiralAxis()
    {
      return m_Tbc.linear() * Eigen::Vector3d::UnitZ();
    }
    
    void rotateAxis(const Eigen::Vector3d& delta, const double& delta_max = 0.1)
    {
      double delta_angle = delta.norm();
      if (std::abs(delta_angle) <1e-10)
        return;
      Eigen::Vector3d versor = delta/delta_angle;
      if (delta_angle < -delta_max)
        delta_angle = -delta_max;
      if (delta_angle > delta_max)
        delta_angle = delta_max;
      //Eigen::Vector3d versor = m_Tbc.linear().inverse() * delta/delta_angle;
      m_Tbc.linear() = m_Tbc.linear() * Eigen::AngleAxisd(delta_angle, versor);
    }
    
    Eigen::Affine3d update(const double delta_time,  Eigen::MatrixXd* twists_in_b = NULL)
    {
      double t = 0;
      for (;t<delta_time;t += m_dt); 
        m_alpha += getAlphaIncrement(m_dt);
      m_alpha += getAlphaIncrement(delta_time-delta_time);
      
      Eigen::Affine3d Tcp;                                  // position of spiral point w.r.t. center
      Tcp.linear().setIdentity();
      Tcp.translation()(0) = m_width/(2*M_PI) *m_alpha*cos(m_alpha);
      Tcp.translation()(1) = m_width/(2*M_PI) *m_alpha*sin(m_alpha);
      Tcp.translation()(2) =  0;

      Eigen::VectorXd v_c_in_p(6);                          // velocity of spiral point in frame center
      v_c_in_p.setZero();
      Eigen::VectorXd a_c_in_p(6);                          // acceleration of spiral point in frame center
      a_c_in_p.setZero();
      
      v_c_in_p(0) = m_velocity / ( sqrt ( pow ( m_alpha, 2 ) + 1 ) ) * ( cos ( m_alpha ) - m_alpha * sin ( m_alpha ) );
      v_c_in_p(1) = m_velocity / ( sqrt ( pow ( m_alpha, 2 ) + 1 ) ) * ( sin ( m_alpha ) + m_alpha * cos ( m_alpha ) );
      
      a_c_in_p(0) = 0;                                      // TODO
      a_c_in_p(1) = 0;                                      // TODO

      Eigen::Affine3d Tbp = m_Tbc*Tcp;
      if (twists_in_b != NULL)
      {
        twists_in_b->resize(6, 2);
        twists_in_b->block(0, 0, 3, 1) =m_Tbc.linear() *v_c_in_p.block(0, 0, 3, 1);
        twists_in_b->block(3, 0, 3, 1) =m_Tbc.linear() *v_c_in_p.block(3, 0, 3, 1);
        twists_in_b->block(0, 1, 3, 1) =m_Tbc.linear() *a_c_in_p.block(0, 0, 3, 1);
        twists_in_b->block(3, 1, 3, 1) =m_Tbc.linear() *a_c_in_p.block(3, 0, 3, 1);
      }  
      
      return Tbp;
      
      
    }
  };
  
}                                                           // end of ns assembly
}                                                           // end of ns itia


# endif