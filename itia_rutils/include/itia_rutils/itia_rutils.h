
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

#ifndef __ITIA_ROS_UTILS__H__
#define __ITIA_ROS_UTILS__H__

#include <mutex>
#include <cmath>
#include <typeinfo>
#include <boost/array.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <itia_rutils/itia_tf_receiver.h>
#include <itia_rutils/itia_tf_receiver.h>
#include <itia_rutils/itia_conversions.h>
#include <itia_rutils/itia_rutils_xmlrpc.h>
#include <itia_rutils/itia_rutils_yaml.h>

namespace itia
{ 
namespace rutils
{
 
  inline bool permutationName(  const std::vector<std::string>& order_names, 
                                std::vector<std::string>& names,
                                std::vector<double>& position, 
                                std::vector<double>& velocity, 
                                std::vector<double>& effort)
  {
    assert(order_names.size() == names.size() );
    assert(order_names.size() == position.size() );
    assert(order_names.size() == velocity.size() );
    assert(order_names.size() == effort.size() );
    for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
    {
      if (names.at(iOrder).compare(order_names.at(iOrder)))
      {
        for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
        {
          if (!order_names.at(iOrder).compare(names.at(iNames)))
          { 
            std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
            std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
            std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
            std::iter_swap(effort.begin()+iOrder,   effort.begin()+iNames);
            break;
          }
          if (iNames==(names.size()-1))
          {
            ROS_ERROR("Joint %s missing",order_names.at(iOrder).c_str());
            return false;
          }
        }
      }
    }
    return true;
  }
  
  inline bool permutationName(  const std::vector<std::string>& order_names, 
                                std::vector<std::string>& names,
                                std::vector<double>& position, 
                                std::vector<double>& velocity)
  {
    assert(order_names.size() == names.size() );
    assert(order_names.size() == position.size() );
    assert(order_names.size() == velocity.size() );
    
    for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
    {
      if (names.at(iOrder).compare(order_names.at(iOrder)))
      {
        for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
        {
          if (!order_names.at(iOrder).compare(names.at(iNames)))
          { 
            std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
            std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
            std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
            break;
          }
          if (iNames==(names.size()-1))
          {
            ROS_ERROR("Joint %s missing",order_names.at(iOrder).c_str());
            return false;
          }
        }
      }
    }
    return true;
  }                                
  inline bool permutationName(  const std::vector<std::string>& order_names, 
                                std::vector<std::string>& names,
                                std::vector<double>& position)
  {
    assert(order_names.size() == names.size() );
    assert(order_names.size() == position.size() );

    for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
    {
      if (names.at(iOrder).compare(order_names.at(iOrder)))
      {
        for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
        {
          if (!order_names.at(iOrder).compare(names.at(iNames)))
          { 
            std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
            std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
            break;
          }
          if (iNames==(names.size()-1))
          {
            ROS_ERROR("Joint %s missing",order_names.at(iOrder).c_str());
            return false;
          }
        }
      }
    }
    return true;
  }
                              
  inline bool permutationName(  const std::vector<std::string>& names_a, 
                                const std::vector<std::string>& names_b, 
                                Eigen::MatrixXd* a_to_b,
                                Eigen::MatrixXd* b_to_a   )
  {
    unsigned int n_a = names_a.size();
    unsigned int n_b = names_b.size();

    a_to_b->resize( n_a, n_b );
    b_to_a->resize( n_b, n_a );

    a_to_b->setZero();
    b_to_a->setZero();

    for (unsigned int idx_a=0; idx_a<n_a; idx_a++)
    {
      ROS_DEBUG_STREAM("Searching name: " << names_a.at(idx_a) );
      for (unsigned int idx_b=0; idx_b<n_b; idx_b++)
      {
        ROS_DEBUG_STREAM("  in name: " << names_b.at(idx_b) );
        if ( !names_a.at(idx_a).compare(names_b.at(idx_b)) )
        {
          (*b_to_a)(idx_a,idx_b)=1;
          (*a_to_b)(idx_b,idx_a)=1;
          break;
        }
        if (idx_b==(n_b-1))
        {
          ROS_ERROR_STREAM("name '" << names_a.at(idx_a) << "' not found");
          return false;
        }
      }
    }
    return true;
  };

  
  
  template< class T> 
  inline bool getParamVector ( ros::NodeHandle& nh, const std::string& key,  std::vector< T >& ret )
  {
    XmlRpc::XmlRpcValue config;
    nh.getParam(key,config);
    return getParamVector<T>(config,ret, key);
  }
  
  template< class T >
  inline bool getParamMatrix ( const ros::NodeHandle& nh,  const std::string& key, std::vector< std::vector< T > >& ret )
  {
    XmlRpc::XmlRpcValue config;
    nh.getParam(key,config);
    return getParamMatrix(config,ret, key);
  }
  
  
  template< class T, size_t n > 
  inline bool getParamArray ( ros::NodeHandle& nh, const std::string& key,  boost::array< T, n >& ret )
  {
    std::vector< T > v;
    if(!getParamVector ( nh, key,  v ) )
      return false;
    if( v.size() != n )
      return false;
    
    for( size_t i=0; i<n; i++ )
      ret[i] = v[i];
    return true;
  }
  
  template<typename T>
  bool getParam(const ros::NodeHandle& nh, const std::string& key, T& ret)
  {
    bool res=nh.getParam(key,ret);
    if (!res)
      ROS_WARN_STREAM("Parameter '" << key << "' not found!");

    ROS_DEBUG_STREAM("Parameter '" << key << "' loaded!");
    return res;
  }
  
  template<> 
  inline bool getParam< Eigen::MatrixXd >( const ros::NodeHandle& nh,  const std::string& key, Eigen::MatrixXd& matrix)
  {
    std::vector<std::vector<double>> mtx;
    if (!getParamMatrix<double>(nh, key, mtx))
      return -1;
    
    int nrows, ncols;
    nrows = mtx.size();
    if (nrows>0)
      ncols = mtx.at(0).size();
    else
      ncols = 0;
    
    matrix.resize(nrows, ncols);
    for (int iR = 0;iR<nrows;iR++)
      for (int iC = 0;iC<ncols;iC++)
        matrix(iR, iC) = mtx.at(iR).at(iC);
    
    return true;
  }
  
  template<> 
  inline bool getParam< geometry_msgs::Pose >(const ros::NodeHandle& nh, const std::string& string, geometry_msgs::Pose& pose)
  {
    bool res=true;
    res=res && nh.getParam(string+"/position/x",pose.position.x);
    res=res && nh.getParam(string+"/position/y",pose.position.y);
    res=res && nh.getParam(string+"/position/z",pose.position.z);
    
    res=res && nh.getParam(string+"/orientation/x",pose.orientation.x);
    res=res && nh.getParam(string+"/orientation/y",pose.orientation.y);
    res=res && nh.getParam(string+"/orientation/z",pose.orientation.z);
    res=res && nh.getParam(string+"/orientation/w",pose.orientation.w);
    
    double mod_quat=sqrt( pow(pose.orientation.x,2.0) +
                          pow(pose.orientation.y,2.0) +
                          pow(pose.orientation.z,2.0) +
                          pow(pose.orientation.w,2.0) );
    
    pose.orientation.x/=mod_quat;
    pose.orientation.y/=mod_quat;
    pose.orientation.z/=mod_quat;
    pose.orientation.w/=mod_quat;
    
    if (!res)
      ROS_ERROR_STREAM("parameter '" << string << "' does not found");
    else
      ROS_DEBUG_STREAM("pose '" << string << "' =\n" <<  &pose);
    return res;
  }

  template<> 
  inline bool getParam< Eigen::Affine3d >(const ros::NodeHandle& nh, const std::string& string, Eigen::Affine3d& ret)
  {
    ROS_DEBUG_STREAM("QUI");
    geometry_msgs::Pose pose_msg;
    
    if (!getParam(nh, string, pose_msg))
      return false;
    
    tf::poseMsgToEigen(pose_msg, ret);
    return true;
  }
  
  inline bool getPosFromParam (const ros::NodeHandle& nh, const std::string& key, std::vector<double>& ret)
  {
    if (!getParam<double>(nh, key+"/x", ret[0])) return false;
    if (!getParam<double>(nh, key+"/y", ret[1])) return false;
    if (!getParam<double>(nh, key+"/z", ret[2])) return false;
    else
      return true;
  }
  
  

  inline void setParam(const ros::NodeHandle& nh, const std::string& string, geometry_msgs::Pose& ret)
  {
    nh.setParam(string+"/position/x",ret.position.x);
    nh.setParam(string+"/position/y",ret.position.y);
    nh.setParam(string+"/position/z",ret.position.z);
    
    nh.setParam(string+"/orientation/x",ret.orientation.x);
    nh.setParam(string+"/orientation/y",ret.orientation.y);
    nh.setParam(string+"/orientation/z",ret.orientation.z);
    nh.setParam(string+"/orientation/w",ret.orientation.w);
    return;
  }
  
  inline void setParam(const ros::NodeHandle& nh, const std::string& string, Eigen::Affine3d& ret)
  {
    geometry_msgs::Pose msg_pose;
    tf::poseEigenToMsg(ret,msg_pose);
    setParam(nh, string, msg_pose);
    return;
  }
  
  template< class T >
  inline bool setParam ( ros::NodeHandle& nh, const std::string& key, const std::vector< std::vector< T > >& mtx )
  {
    XmlRpc::XmlRpcValue data_;
    int iRow = 0;
    for ( auto itRow=mtx.begin(); itRow!=mtx.end(); itRow++ )
    {
      int iEl = 0;
      XmlRpc::XmlRpcValue row_; 
      for ( auto itCol=(*itRow).begin(); itCol!=(*itRow).end(); itCol++ )
        row_[iEl++] = *itCol; 
      
      data_[iRow++] = (XmlRpc::XmlRpcValue)row_;
    }

    nh.setParam( key, data_ );
    return true;
  }
  
  template< class T >
  inline bool setParamNum ( ros::NodeHandle& nh,  const std::string& key, const std::vector< std::vector< T > >& mtx, unsigned int precision = 0 )
  {
    const std::vector< std::type_index > allowed_type = { std::type_index(typeid(double)), 
                                                          std::type_index(typeid(long double)), 
                                                          std::type_index(typeid(float)) };
    bool ok = false;
    
    for( auto typ : allowed_type )
      if ( std::type_index(typeid( T )) == typ )
        ok = true;
    
    if( ok )
    {
      XmlRpc::XmlRpcValue data_;
      int iRow = 0;
      for ( auto itRow=mtx.begin(); itRow!=mtx.end(); itRow++ )
      {
        int iEl = 0;
        XmlRpc::XmlRpcValue row_;

        for ( auto itCol=(*itRow).begin(); itCol!=(*itRow).end(); itCol++ )
        {
          if ( precision != 0 )
            row_[iEl++] = ( lround( (long double)( *itCol ) * pow( 10, precision ) ) ) / pow( 10, precision );
          else
            row_[iEl++] = *itCol;
        }
        
        data_[iRow++] = row_;
      }

      nh.setParam(key, data_ );
      return true;
    }
    else
    {
      ROS_ERROR("Unable to detect type id in setParamNum");
      return false;
    }
    
  }
  
  
  /*
    * MsgReceiver(const std::string& _name="MsgReceiver",const int& verbose=0)
    * 
    * name = name used on info msgs
    * 
    * verbose = 0 no info
    * verbose = 1 info on firstEntry and wait (BLUE COLOR)
    * verbose = 2 info on firstEntry, wait, and newdata (GREEN COLOR)
    * 
    * Example:
    * geometry_msgs::PoseStamped msg;
    * itia::rutils::MsgReceiver<geometry_msgs::PoseStamped> msg_receiver("PoseMsgReceived");
    * ros::Subscriber  sub=nh.subscribe("pose_state",1,&itia::rutils::MsgReceiver<geometry_msgs::PoseStamped>::callback,&msg_receiver);
    * if (!msg_receiver.waitForANewData(10))
    * {
    *   std::cout <<"error!"<<std::endl;
    *   return -1;
    * }
    * msg=msg_receiver.getData();
    */
  template<typename T> class MsgReceiver
  {
  private:
    bool              m_new_data;
    int               m_verbose;
    std::string       m_name;
    unsigned long int m_msg_counter;
    std::mutex        m_mtx;
    T                 data;
    
  public:
    MsgReceiver(  const std::string& name="MsgReceiver",
                  const int& verbose=1                  );
    
    void              callback(const boost::shared_ptr<T const>& msg);    
    unsigned long int getMsgCounter();
    bool              isANewDataAvailable();
    bool              waitForANewData(const double& timeout=10.0);
    T                 getData();
  }; 

  template<typename T> 
  MsgReceiver<T>::MsgReceiver(  const std::string& name,
                                const int& verbose)
  {
    m_verbose=verbose;
    m_name=name;
    m_new_data=false;
    m_msg_counter=0;
    if (m_verbose>0)
    printf("[ MsgReceived %s] create!\n",m_name.c_str());
  }

  template<typename T> 
  void MsgReceiver<T>::callback(const boost::shared_ptr<T const>& msg)
  {
    m_mtx.lock();
    data=*msg;
    m_new_data=true;
    m_msg_counter++;
    m_mtx.unlock();

    if (m_verbose>0 && m_msg_counter==0)
      ROS_INFO("[ MsgReceived %s] first message received!\n",m_name.c_str());
    if (m_verbose>1 && m_msg_counter>0)
      printf("[MsgReceived %s] new message received!\n",m_name.c_str());

  }

  template<typename T> 
  unsigned long int MsgReceiver<T>::getMsgCounter()
  {
    m_mtx.lock();
    long unsigned int tmp=m_msg_counter;
    m_mtx.unlock();
    return tmp;
  }

  template<typename T> 
  T MsgReceiver<T>::getData()
  {
    m_mtx.lock();
    T tmp=data;
    m_new_data=false;
    m_mtx.unlock();
    return tmp;
  }

  template<typename T> 
  bool MsgReceiver<T>::isANewDataAvailable()
  {
    m_mtx.lock();
    bool tmp=m_new_data;
    m_mtx.unlock();
    return tmp;
  }

  template<typename T> 
  bool MsgReceiver<T>::waitForANewData(const double& timeout)
  {
    double t=0;
    ros::Rate loopRate(100);
    double st=loopRate.expectedCycleTime().toSec();
    ros::Time init_time = ros::Time::now();
    while((ros::Time::now()-init_time).toSec()<timeout)
    {
      if (this->isANewDataAvailable())
      {
        if (m_verbose>0)
          ROS_INFO("[ MsgReceived %s] wait %5.4f seconds to receive a new message !\n",m_name.c_str(), (ros::Time::now()-init_time).toSec());
          return true;
      }
      t+=st;
      loopRate.sleep();
      ros::spinOnce();
    }

    ROS_ERROR("[ MsgReceived %s] timeout (%5.4f seconds) on receiving a new message !\n",m_name.c_str(),(ros::Time::now()-init_time).toSec());
    return false;

  }   

}; 
};
#endif