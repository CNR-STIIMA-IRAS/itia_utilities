
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

#ifndef __ITIA_RUTILS_TF_REC__
#define __ITIA_RUTILS_TF_REC__

# include <tf/transform_listener.h>
# include <tf_conversions/tf_eigen.h>
# include <tf_conversions/tf_kdl.h>

# include <geometry_msgs/Pose.h>

namespace itia
{
namespace rutils
{
  
  class TfReceiver
  {
  private:
    unsigned int m_trial;
    unsigned int m_max_trials;
    std::string m_base;
    std::string m_tool;
    bool m_first_entry;
    
    tf::TransformListener m_listener;
    tf::StampedTransform m_transform;
    geometry_msgs::Pose m_pose;

    
  public:
    TfReceiver( const std::string& base,  
                const std::string& tool, 
                const unsigned int max_trials = 10  ) : 
                m_max_trials(max_trials), 
                m_base(base) ,
                m_tool(tool)  
    {
      m_trial       = 0; 
      m_first_entry = true;
    };
    
    bool lookUp()
    {
      try
      {
        m_listener.lookupTransform(m_base,
          m_tool,  
          ros::Time(0), 
          m_transform);
        m_first_entry = false;
        m_trial = 0;
      }
      catch (tf::TransformException ex)
      {
        ROS_DEBUG("%s",ex.what());
        m_trial++;
        if ( (m_trial >= m_max_trials) ||  m_first_entry)
        {
          ROS_WARN_THROTTLE(1, "%s",ex.what());
          return false;
        }
      }
      return true;
    };

    bool getTransform(tf::StampedTransform* transform)
    {
      if (!lookUp())
        return false;
      
      *transform = m_transform;
      return true;
    };
    
    bool getPose(geometry_msgs::Pose* pose)
    {
      if (!lookUp())
        return false;
      
      
      geometry_msgs::TransformStamped geom_tf;
      tf::transformStampedTFToMsg(m_transform,geom_tf);
      
      tf::Vector3 origin = m_transform.getOrigin();
      tf::Quaternion 	 quaternon = m_transform.getRotation();
      pose->position.x = origin.getX();
      pose->position.y = origin.getY();
      pose->position.z = origin.getZ();
      
      pose->orientation.x = quaternon.getX();
      pose->orientation.y = quaternon.getY();
      pose->orientation.z = quaternon.getZ();
      pose->orientation.w = quaternon.getW();

      return true;
    };
    
    bool getEigenAffine(Eigen::Affine3d* affine)
    {
      if (!lookUp())
        return false;
        
      tf::transformTFToEigen(m_transform, *affine);
      return true;
      
    };
  
    bool getKdlFrame(KDL::Frame* frame)
    {
      if (!lookUp())
      return false;

      tf::transformTFToKDL(m_transform, *frame);
      return true;

    };
    
    void changeBase(const std::string& base)
    {
      m_trial = 0;
      m_first_entry = true;
      m_base = base;
    };
    
    void changeTool(const std::string& tool)
    {
      m_trial = 0;
      m_first_entry = true;
      m_tool = tool;
    };
   
    bool waitForAnewData(const double& timeout)
    {
	    m_first_entry = true;
      ros::Time t0 = ros::Time::now();
      while (!lookUp())
      {
        ros::Duration(0.001).sleep();
        if (!ros::ok())
          return false;
        if ((ros::Time::now()-t0).toSec()>timeout)
        {
          ROS_ERROR("timeout (%5.4f seconds) on receiving a new tf from %s to %s !\n", (ros::Time::now()-t0).toSec(), m_base.c_str(), m_tool.c_str());
          return false;
        }
      }
      ROS_DEBUG("wait %5.4f seconds for receiving a new tf from %s to %s !\n",(ros::Time::now()-t0).toSec(), m_base.c_str(), m_tool.c_str());
      return true;
    }
   };

}
}

#endif
