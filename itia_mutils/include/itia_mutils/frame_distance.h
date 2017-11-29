
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

#ifndef __ITIA_MUTILS_FRAME_DISTANCE_
#define __ITIA_MUTILS_FRAME_DISTANCE_

#include <Eigen/Geometry>

namespace itia
{
namespace mutils
{
  /* 
   * Skew matrix of a vector
   */
  inline Eigen::Matrix3d skew(const Eigen::Vector3d& t)
  {
    Eigen::Matrix3d t_hat;
    t_hat <<  0,    -t(2), t(1),
              t(2),  0,   -t(0),
             -t(1),  t(0), 0;
    return t_hat;
  }
  
  /*
   * Distance between frames.
   * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
   * the distance are defined as a vector containing:
   *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
   *   - the vector part of the quaternion q_ab expressed in frame w (namely R_wa * imag(q_ab))
   */
  inline void getFrameDistance(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd* distance)
  {
    distance->resize(6);
    distance->block(0, 0, 3, 1) = T_wa.translation()-T_wb.translation();
    Eigen::Quaterniond q_ab(T_wa.linear().inverse() * T_wb.linear());

    if (q_ab.w() <0)
    {
      q_ab.x()=-q_ab.x();
      q_ab.y()=-q_ab.y();
      q_ab.z()=-q_ab.z();
      q_ab.w()=-q_ab.w();
    }
      
    distance->block(3, 0, 3, 1) = T_wa.linear() * q_ab.vec();
  }
  
  /*
   * Distance between frames and its jacobian.
   * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
   * the distance are defined as a vector containing:
   *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
   *   - the vector part of the quaternion q_ab expressed in frame w (namely R_wa * imag(q_ab))
   * and the jacobian of the distance error defined as
   * J=[eye(3) zeros(3, 3);zeros(3, 3) 0.5*R_wa*(eye(3)*real(q_ab)-skew(imag(q_ab)))*R_wa']
   */
  inline void getFrameDistanceJac(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd* distance,  Eigen::MatrixXd* jacobian)
  {
    distance->resize(6);
    jacobian->resize(6, 6);
    jacobian->setIdentity();
    
    distance->block(0, 0, 3, 1) = T_wb.translation()-T_wa.translation();
    Eigen::Quaterniond q_ab(T_wa.linear().inverse() * T_wb.linear());
    
    if (q_ab.w() <0)
    {
      q_ab.w()=-q_ab.w();
      q_ab.vec()=-q_ab.vec();
    }
      
      
    distance->block(3, 0, 3, 1) = T_wa.linear() * q_ab.vec();
    
    jacobian->block(3, 3, 3, 3) = 0.5* T_wa.linear() *( q_ab.w() *Eigen::MatrixXd::Identity(3, 3) - skew(q_ab.vec()) ) *T_wa.linear().inverse(); 
  }
  
  
}                                                           // namespace mutils
}                                                           // namespace itia
#endif