
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