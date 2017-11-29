
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

#ifndef __ITIA_DYN_PAR_UTILS__
#define __ITIA_DYN_PAR_UTILS__

#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_msgs/send_float64.h>
#include <itia_msgs/send_string.h>

namespace itia
{
namespace rutils
{
  
  template<typename topicType,typename serviceType>
  class ParameterServer
  {
  private:
    ros::Publisher m_publisher;
    ros::ServiceServer m_set_server;
    ros::ServiceServer m_reload_server;
    ros::ServiceServer m_load_server;
    ros::ServiceServer m_dump_server;
    ros::NodeHandle* m_nh;
    topicType m_max_limit;
    topicType m_min_limit;
    topicType m_msg;
    std::string m_param_name;
    
    enum {CONSTRUCTED, INITIALIZED, ERROR} m_state;
    
    typedef typename serviceType::ResponseType Response; 
    typedef typename serviceType::RequestType  Request; 
    
    bool loadCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res);
    bool dumpCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res);
    bool reinitCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    
    bool setCallback(Request& req,Response& res);
    bool clamp(Request& req);
  public:
    ParameterServer(ros::NodeHandle* nh,const std::string param_name)
    :
    m_param_name(param_name)
    {
      m_nh=nh;
      m_state = ERROR;
      ROS_ERROR("YOU NEED TO ADD YOUR TEMPLATE SPECIALIZATION!");
    };
    
    
  };
  
 
  /* 
   * Float64 data
   */
  
  template <>
  bool ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::loadCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res)
  {
    if ( m_state == INITIALIZED ) 
    { 
      int result = system(("rosparam load " + req.name.data + " /" +m_param_name).c_str());
      if (!result)
        ROS_ERROR_STREAM("FAIL: rosparam load " + req.name.data + " /" +m_param_name);
        
      if (!m_nh->hasParam(m_param_name+"/upper_bound") || !m_nh->hasParam(m_param_name+"/lower_bound") || !m_nh->hasParam(m_param_name+"/nominal"))
      {
        ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",m_param_name.c_str(),m_param_name.c_str(),m_param_name.c_str());
        m_state = ERROR;
        return false;
      }
      
      // load upper, lower and nominal values
      itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
      itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
      itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
      
      if (m_max_limit.data < m_min_limit.data )
      {
        ROS_WARN("Upper bound of '%s' is lower than its lower bound!!!",m_param_name.c_str());
        m_state=ERROR;
        return false;
      }
      
      if  (m_msg.data < m_min_limit.data || m_msg.data > m_max_limit.data)
      {
        ROS_WARN("Desired value of parameter '%s' is outside the limits",m_param_name.c_str());
        m_state=ERROR;
        return false;
      }
      
      if (m_state != INITIALIZED)
        m_publisher = m_nh->advertise<std_msgs::Float64>(m_param_name+"/topic",1,true);
      m_publisher.publish(m_msg);
      
      m_state=INITIALIZED;
      res.res=true;
      return true;
    }
    return false;
  }
  
  template <>
  bool ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::dumpCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res)
  {
    if ( m_state == INITIALIZED ) 
    { 
      std_msgs::Float64 nom_value;
      m_nh->getParam(m_param_name+"/nominal",nom_value.data);
      m_nh->setParam(m_param_name+"/nominal",m_msg.data);
      
      int result = system(("rosparam dump " + req.name.data + " /" +m_param_name).c_str());
      if (!result)
        ROS_ERROR_STREAM("rosparam dump " + req.name.data + " /" +m_param_name);
        
      m_nh->setParam(m_param_name+"/nominal",nom_value.data);
      res.res=true;
      return true;
    }
    return false;
  }
  
  template <>
  bool ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::reinitCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
  {
    if (!m_nh->hasParam(m_param_name+"/upper_bound") || !m_nh->hasParam(m_param_name+"/lower_bound") || !m_nh->hasParam(m_param_name+"/nominal"))
    {
      ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",m_param_name.c_str(),m_param_name.c_str(),m_param_name.c_str());
      m_state = ERROR;
      return false;
    }
    
    // load upper, lower and nominal values
    itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
    
    if (m_max_limit.data < m_min_limit.data )
    {
      ROS_WARN("Upper bound of '%s' is lower than its lower bound!!!",m_param_name.c_str());
      m_state=ERROR;
      return false;
    }
    
    if  (m_msg.data < m_min_limit.data || m_msg.data > m_max_limit.data)
    {
      ROS_WARN("Desired value of parameter '%s' is outside the limits",m_param_name.c_str());
      m_state=ERROR;
      return false;
    }
    
    if (m_state != INITIALIZED)
      m_publisher = m_nh->advertise<std_msgs::Float64>(m_param_name+"/topic",1,true);
    
    m_publisher.publish(m_msg);
    
    m_state=INITIALIZED;
    
    return true;
  };
  
  template <>
  bool ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::clamp(Request& req)
  {
    if (req.value < m_min_limit.data || req.value > m_max_limit.data)
      return false;
    
    m_msg.data=req.value;
    return true;
  };
  
  template <>
  bool ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::setCallback(itia_msgs::send_float64Request& req, itia_msgs::send_float64Response& res)
  {
    if (m_state != INITIALIZED )
    {
      ROS_ERROR("Parameter %s not initialized properly!",m_param_name.c_str());
      res.res=false;
      return false;
    }
    
    if (clamp(req))
    {
      m_publisher.publish<>(m_msg);
      res.res=true;
      return true;
    }
    else
    {
      ROS_WARN("Desired value of parameter '%s' is outside the limits",m_param_name.c_str());
      res.res=false;
      return false;
    }
  };
  
  template <>
  ParameterServer<std_msgs::Float64,itia_msgs::send_float64>::ParameterServer(ros::NodeHandle* nh,const std::string param_name)
  {
    m_param_name=param_name;
    m_nh=nh;
    m_state = CONSTRUCTED;
    
    m_reload_server = m_nh->advertiseService(param_name+"/reinit", &ParameterServer::reinitCallback, this);
    m_set_server    = m_nh->advertiseService(param_name+"/set",    &ParameterServer::setCallback,    this);
    m_load_server   = m_nh->advertiseService(param_name+"/load",   &ParameterServer::loadCallback,   this);
    m_dump_server   = m_nh->advertiseService(param_name+"/dump",   &ParameterServer::dumpCallback,   this);
    
    if (!m_nh->hasParam(param_name+"/upper_bound") || !m_nh->hasParam(param_name+"/lower_bound") || !m_nh->hasParam(param_name+"/nominal"))
    {
      ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",param_name.c_str(),param_name.c_str(),param_name.c_str());
      m_state = ERROR;
      return;
    }
    
    // load upper, lower and nominal values
    itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
    
    if (m_max_limit.data < m_min_limit.data )
    {
      ROS_WARN("Upper bound of '%s' is lower than its lower bound!!!",m_param_name.c_str());
      m_state=ERROR;
      return;
    }
    
    if  (m_msg.data < m_min_limit.data || m_msg.data > m_max_limit.data)
    {
      ROS_WARN("Desired value of parameter '%s' is outside the limits",m_param_name.c_str());
      m_state=ERROR;
      return;
    }
    
    // publish nominal
    m_publisher = m_nh->advertise<std_msgs::Float64>(param_name+"/topic",1,true);
    m_publisher.publish(m_msg);
    
    
    m_state=INITIALIZED;
    
  };
  
  
    /* 
   * send_string data
   */
  
  template <>
  bool ParameterServer<std_msgs::String,itia_msgs::send_string>::loadCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res)
  {
    if ( m_state == INITIALIZED ) 
    { 
      int result =system(("rosparam load " + req.name.data + " /" +m_param_name).c_str());
      if (!result)
        ROS_ERROR_STREAM("FAIL rosparam load " + req.name.data + " /" +m_param_name);
      
      if (!m_nh->hasParam(m_param_name+"/upper_bound") || !m_nh->hasParam(m_param_name+"/lower_bound") || !m_nh->hasParam(m_param_name+"/nominal"))
      {
        ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",m_param_name.c_str(),m_param_name.c_str(),m_param_name.c_str());
        m_state = ERROR;
        return false;
      }
      
      // load upper, lower and nominal values
      itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
      itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
      itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
      
      if (m_state != INITIALIZED)
        m_publisher = m_nh->advertise<std_msgs::String>(m_param_name+"/topic",1,true);
      m_publisher.publish(m_msg);
      
      m_state=INITIALIZED;
      res.res=true;
      return true;
    }
    return false;
  }
  
  template <>
  bool ParameterServer<std_msgs::String,itia_msgs::send_string>::dumpCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res)
  {
    if ( m_state == INITIALIZED ) 
    { 
      std_msgs::String nom_value;
      m_nh->getParam(m_param_name+"/nominal",nom_value.data);
      m_nh->setParam(m_param_name+"/nominal",m_msg.data);
      if (!system(("rosparam dump " + req.name.data + " /" +m_param_name).c_str()))
        ROS_ERROR_STREAM("FAIL rosparam dump " + req.name.data + " /" +m_param_name);
      m_nh->setParam(m_param_name+"/nominal",nom_value.data);
      res.res=true;
      return true;
    }
    return false;
  }
  
  template <>
  bool ParameterServer<std_msgs::String,itia_msgs::send_string>::reinitCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
  {
    if (!m_nh->hasParam(m_param_name+"/upper_bound") || !m_nh->hasParam(m_param_name+"/lower_bound") || !m_nh->hasParam(m_param_name+"/nominal"))
    {
      ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",m_param_name.c_str(),m_param_name.c_str(),m_param_name.c_str());
      m_state = ERROR;
      return false;
    }
    
    // load upper, lower and nominal values
    itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
    
    if (m_state != INITIALIZED)
      m_publisher = m_nh->advertise<std_msgs::String>(m_param_name+"/topic",1,true);
    
    m_publisher.publish(m_msg);
    
    m_state=INITIALIZED;
    
    return true;
  };
  
  template <>
  bool ParameterServer<std_msgs::String,itia_msgs::send_string>::clamp(Request& req)
  {
    m_msg.data=req.name.data;
    return true;
  };
  
  template <>
  bool ParameterServer<std_msgs::String,itia_msgs::send_string>::setCallback(itia_msgs::send_stringRequest& req, itia_msgs::send_stringResponse& res)
  {
    if (m_state != INITIALIZED )
    {
      ROS_ERROR("Parameter %s not initialized properly!",m_param_name.c_str());
      res.res=false;
      return false;
    }
    
    if (clamp(req))
    {
      m_publisher.publish<>(m_msg);
      res.res=true;
      return true;
    }
    else
    {
      ROS_WARN("Desired value of parameter '%s' is outside the limits",m_param_name.c_str());
      res.res=false;
      return false;
    }
  };
  
  template <>
  ParameterServer<std_msgs::String,itia_msgs::send_string>::ParameterServer(ros::NodeHandle* nh,const std::string param_name)
  {
    m_param_name=param_name;
    m_nh=nh;
    m_state = CONSTRUCTED;
    
    m_reload_server = m_nh->advertiseService(param_name+"/reinit", &ParameterServer::reinitCallback, this);
    m_set_server    = m_nh->advertiseService(param_name+"/set",    &ParameterServer::setCallback,    this);
    m_load_server   = m_nh->advertiseService(param_name+"/load",   &ParameterServer::loadCallback,   this);
    m_dump_server   = m_nh->advertiseService(param_name+"/dump",   &ParameterServer::dumpCallback,   this);
    
    if (!m_nh->hasParam(param_name+"/upper_bound") || !m_nh->hasParam(param_name+"/lower_bound") || !m_nh->hasParam(param_name+"/nominal"))
    {
      ROS_ERROR("Upper bound (%s/upper_bound), lower bound (%s/lower_bound), or nominal value (%s/nominal) are no specified!",param_name.c_str(),param_name.c_str(),param_name.c_str());
      m_state = ERROR;
      return;
    }
    
    // load upper, lower and nominal values
    itia::rutils::getParam(*m_nh,m_param_name+"/upper_bound",m_max_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/lower_bound",m_min_limit.data);
    itia::rutils::getParam(*m_nh,m_param_name+"/nominal",    m_msg.data);
    
      
    // publish nominal
    m_publisher = m_nh->advertise<std_msgs::String>(param_name+"/topic",1,true);
    m_publisher.publish(m_msg);
    
    
    m_state=INITIALIZED;
    
  };
}
}

#endif