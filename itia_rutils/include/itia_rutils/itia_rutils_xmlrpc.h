#ifndef __ITIA_ROS_UTILS__XMLRPC__H__
#define __ITIA_ROS_UTILS__XMLRPC__H__

#include <mutex>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <itia_rutils/itia_conversions.h>


namespace itia
{ 
namespace rutils
{
  inline void cast( const XmlRpc::XmlRpcValue& node, unsigned int& val )
  {
    XmlRpc::XmlRpcValue config( node );
    switch( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:    { bool   _val = static_cast< bool   >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeInt:        { int    _val = static_cast< int    >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeDouble:     { double _val = static_cast< double >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeBase64:     
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error("Type inconsistency");
        break;
    }
  }
  inline void cast( const XmlRpc::XmlRpcValue& node, double& val )
  {
    XmlRpc::XmlRpcValue config( node );
    switch( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:    { bool   _val = static_cast< bool   >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeInt:        { int    _val = static_cast< int    >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeDouble:     { double _val = static_cast< double >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeBase64:     
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error("Type inconsistency");
        break;
    }
  }
  inline void cast( const XmlRpc::XmlRpcValue& node, int& val )
  {
    XmlRpc::XmlRpcValue config( node );
    switch( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:    { bool   _val = static_cast< bool   >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeInt:        { int    _val = static_cast< int    >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeDouble:     { double _val = static_cast< double >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeBase64:     
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error("Type inconsistency");
        break;
    }
  }
  inline void cast( const XmlRpc::XmlRpcValue& node, long unsigned int& val )
  {
    XmlRpc::XmlRpcValue config( node );
    switch( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:    { bool   _val = static_cast< bool   >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeInt:        { int    _val = static_cast< int    >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeDouble:     { double _val = static_cast< double >( config ); val = _val; } break;
      case XmlRpc::XmlRpcValue::TypeBase64:     
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error("Type inconsistency");
        break;
    }
  }
  inline void cast( const XmlRpc::XmlRpcValue& node, std::string& val )
  {
    XmlRpc::XmlRpcValue config( node );
    switch( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:    { bool   _val = static_cast< bool   >( config ); val = std::to_string( _val ); } break;
      case XmlRpc::XmlRpcValue::TypeInt:        { int    _val = static_cast< int    >( config ); val = std::to_string( _val ); } break;
      case XmlRpc::XmlRpcValue::TypeDouble:     { double _val = static_cast< double >( config ); val = std::to_string( _val ); } break;
      case XmlRpc::XmlRpcValue::TypeString:     { std::string _val = static_cast< std::string >( config ); val = _val ;        } break;
      case XmlRpc::XmlRpcValue::TypeBase64:     
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error("Type inconsistency");
        break;
    }
  }
 
 
  inline bool check( const XmlRpc::XmlRpcValue& config, const std::vector< std::string >& required_fields )
  {
    for( std::vector< std::string >::const_iterator it = required_fields.begin(); it != required_fields.end(); it++ )
    {
      if( !config.hasMember( *it ) )
      {
        ROS_ERROR ( " CHECK XmlRpcValue -The field '%s' does not exist. ", it->c_str() );
        return false;
      }
    }
    return true;
  }
  
  inline double toDouble(  const XmlRpc::XmlRpcValue& node )
  {
    XmlRpc::XmlRpcValue config( node );
    double ret;
    if (config.getType() == XmlRpc::XmlRpcValue::TypeDouble )
    {
      ret = config;
    }
    else if (config.getType() == XmlRpc::XmlRpcValue::TypeInt )
    {
      int i_val = config;
      ret = i_val;
    }
    else
      throw std::runtime_error("Type Error: neither DOULBE or INT");
    return ret;
  }
  
  inline int toInt(  const XmlRpc::XmlRpcValue& node )
  {
    XmlRpc::XmlRpcValue config( node );
    int ret;
    if (config.getType() == XmlRpc::XmlRpcValue::TypeDouble )
    {
      double temp = config;
      ret = (int)temp;
    }
    else if (config.getType() == XmlRpc::XmlRpcValue::TypeInt )
    {
      ret = config;
    }
    else
      throw std::runtime_error("Type Error: neither DOULBE or INT");
    return ret;
  }
  
  
  
  template< class T> 
  inline bool getParam( const XmlRpc::XmlRpcValue& node,  T& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config( node );
    try 
    {
      ret = static_cast< T >( config );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", log_key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }
  
  template< class T> 
  inline bool getParam( const XmlRpc::XmlRpcValue& node,  std::string key,  T& ret )
  {
    try 
    {
      if( !node.hasMember( key  ) )
      {
        ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
        return false;
      }
      XmlRpc::XmlRpcValue config( node );
      
      ret = static_cast< T >( config[key] );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }
  
  template< class T> 
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  std::vector< T >& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config( node );
    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( int j=0; j<config.size(); ++j )
    {
      try
      {
        T val;
        if(typeid(T) == typeid(double))
        {
          val = toDouble( config[j] );
        }
        else if(typeid(T) == typeid(std::string))
        {
          if( config[j].getType() == XmlRpc::XmlRpcValue::TypeString )
              cast( config[j], val );
          else
            throw std::runtime_error( ("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string(  int( config[j].getType() ) ) ) .c_str() );

        }
        else
          cast( config[j], val );
        
        ret.push_back( val  );   
      }
      catch (std::exception& e )
      {
        ROS_ERROR_STREAM("Error: " << e.what() );
        return false;
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Wrong Format." );
        return false;
      }
    }
    return true;
  }
  
  template< class T, size_t n > 
  inline bool getParamArray ( const XmlRpc::XmlRpcValue& node,  boost::array< T, n >& ret, const std::string& log_key = "" )
  {
    std::vector< T > v;
    if(!getParamVector ( node, v,  log_key ) )
      return false;
    if( v.size() != n )
      return false;
    
    for( size_t i=0; i<n; i++ )
      ret[i] = v[i];
    return true;
  }
  
  
  template< class T> 
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector< T >& ret  )
  {
    try 
    {
      if( !node.hasMember( key  ) )
      {
        ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
        return false;
      }
      
      XmlRpc::XmlRpcValue config( node );
      return getParamVector ( config[key],  ret, key  );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }
  
  template< class T, size_t n > 
  inline bool getParamArray ( const XmlRpc::XmlRpcValue& node,  const std::string& key, boost::array< T, n >& ret  )
  {
    std::vector< T > v;
    if(!getParamVector ( node, key, v ) )
      return false;
    if( v.size() != n )
      return false;
    
    for( size_t i=0; i<n; i++ )
      ret[i] = v[i];
    return true;
  }
  
  
  
  template< class T >
  inline bool getParamMatrix ( const XmlRpc::XmlRpcValue& node, std::vector< std::vector< T > >& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config(node);
    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( auto i=0; i<config.size(); ++i )
    {
    
      XmlRpc::XmlRpcValue row = config[i];
      if ( row.getType() != XmlRpc::XmlRpcValue::TypeArray )
      {
        ROS_ERROR ( "The row[%d] is not an array.", i );
        return false;
      }
      std::vector< T > vct;
      for ( int j=0; j<row.size(); ++j )
      {
        T value;
        try
        {
          XmlRpc::XmlRpcValue rpcval = row[j];
          if(typeid(T) == typeid(double))
          {
            value = toDouble( row[j] );
          }
          else if(typeid(T) == typeid(std::string))
          {
            if( row[j].getType() == XmlRpc::XmlRpcValue::TypeString )
                cast( row[j], value );
            else
              throw std::runtime_error( ("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string(  int( row[j].getType() ) ) ).c_str() );

          }
          else
          {
            cast( row[j], value );
          }   
        }
        catch (std::exception& e )
        {
          ROS_ERROR_STREAM("Error: " << e.what() );
          if (typeid(T) == typeid(double))
            ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
          return false;
        }
        catch (...)
        {
          if (typeid(T) == typeid(double))
            ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
          return false;
        }
        vct.push_back ( value );
      }
      ret.push_back ( vct );
    }
    return true;
  }
  
  
  template< class T> 
  inline bool getParamMatrix ( const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector< T >& ret  )
  {
    try 
    {
      if( !node.hasMember( key  ) )
      {
        ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
        return false;
      }
      
      XmlRpc::XmlRpcValue config( node );
      return getParamMatrix ( config[key],  ret, key  );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }

}; 
};
#endif














/*
  template<>
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  std::vector< double >& ret, const std::string& log_key)
  {
    XmlRpc::XmlRpcValue config( node );
    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( int j=0; j<config.size(); ++j )
    {
      try
      {
        double v = toDouble( config[j] );
        ret.push_back( v );   
      }
      catch (std::exception& e )
      {
        ROS_ERROR_STREAM("Error: " << e.what() );
        return false;
      }
      catch (...)
      {
        ROS_ERROR_STREAM("WRONG FORMAT IN PARAMETER " );
        return false;
      }
    }
    return true;
  }
  
  template<>
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  std::vector< std::string >& ret, const std::string& log_key)
  {
    XmlRpc::XmlRpcValue config( node );

    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( int j=0; j<config.size(); ++j )
    {
      try
      {
        std::string t = config[j];
        ret.push_back ( t );   
      }
      catch (std::exception& e )
      {
        ROS_ERROR_STREAM("Error: " << e.what() );
        return false;
      }
      catch (...)
      {
        ROS_ERROR_STREAM("WRONG FORMAT IN PARAMETER " );
        return false;
      }
    }
    return true;
  }*/



/*  
inline bool getParam ( const ros::NodeHandle& nh,  const std::string& log_key, std::vector< std::vector< double > >& mtx )
{
  XmlRpc::XmlRpcValue config;
  bool ret = nh.getParam(log_key, config);
  if( !ret )
  {
    ROS_ERROR( "The log_key '%s' has not been found in the paramserver", log_key.c_str() );
    return false;
  }
    
  if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR ( "The input value (XmlRpcValue) is not of type array. %d/%d", int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
    return false;
  }
  
  mtx.clear();
  for ( size_t i=0; i<config.size(); ++i )
  {
    XmlRpc::XmlRpcValue& row = config[i];
    if ( row.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The row[%zu] is not an array.", i );
      return false;
    }
    
    std::vector< double > vct;
    for ( int j=0; j<row.size(); ++j )
    {
      double _val;
      XmlRpc::XmlRpcValue& val = row[j];
      if( val.getType() == XmlRpc::XmlRpcValue::TypeDouble )
        _val = double( val );
      else if( val.getType() == XmlRpc::XmlRpcValue::TypeInt )
      {
        int v = val;
        _val = double( v );
      }
      else
        throw std::runtime_error("Type error!");
    
      vct.push_back ( _val );
    }
    mtx.push_back ( vct );
  }
  return true;
}


inline bool getParam ( const ros::NodeHandle& nh,  const std::string& log_key, std::vector< std::vector< std::string > >& mtx )
{
  XmlRpc::XmlRpcValue config;
  bool ret = nh.getParam(log_key, config);
  if( !ret )
  {
    ROS_ERROR( "The log_key '%s' has not been found in the paramserver", log_key.c_str() );
    return false;
  }
    
  if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR ( "The input value (XmlRpcValue) is not of type array. %d/%d", int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
    return false;
  }
  
  mtx.clear();
  for ( size_t i=0; i<config.size(); ++i )
  {
    XmlRpc::XmlRpcValue& row = config[i];
    if ( row.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The row[%zu] is not an array.", i );
      return false;
    }
    
    std::vector< std::string > vct;
    for ( int j=0; j<row.size(); ++j )
    {
      XmlRpc::XmlRpcValue& val = row[j];
      if( val.getType() == XmlRpc::XmlRpcValue::TypeString )
        vct.push_back ( val );
      else 
        throw std::runtime_error("Type error!");

    }
    mtx.push_back ( vct );
  }
  return true;
}*/

