#ifndef __ITIA_ROS_UTILS__YAML__H__
#define __ITIA_ROS_UTILS__YAML__H__

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <itia_futils/itia_futils.h>

namespace itia
{ 
namespace rutils
{
 
inline bool check( const YAML::Node& node, const std::vector< std::string >& required_fields )
{
  for( std::vector< std::string >::const_iterator it = required_fields.begin(); it != required_fields.end(); it++ )
  {
    if( node[ *it ].IsNull() )
    {
      ROS_ERROR ( "The input value (YAML::Node) has not the '%s' field. ", it->c_str() );
      return false;
    }
  }
  return true;
}



template< class T >
inline bool getParam( const YAML::Node& node, T& ret )
{
  try 
  {
    ret = node.as< T >( );
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM( "YAML::Exception " << std::endl << BOLDYELLOW << e.msg << std::endl << RESET      << "Node: " << node << std::endl << "return." ;);
    return false;
  }
  catch( std::exception& e)
  {
    ROS_ERROR_STREAM( "std::exception "         << std::endl << BOLDYELLOW << e.what()     << std::endl << RESET  << "Node: "  << node << std::endl << "return." );
    return false;
  }
  catch( ... )
  {
    ROS_ERROR_STREAM( "Unhandled Exception " << std::endl);
    return false;
  }
  return true;
};


template< class T >
inline bool getParam( const YAML::Node& node, const std::string& key, T& ret )
{
  bool ok;
  try 
  {
    if( key != "" )
    {
      ok = getParam( node[ key ], ret );
    }
    else
      ok = getParam( node, ret );
    
    if( !ok )
    {
      ROS_ERROR_STREAM( __FUNCTION__ << "Error in processing the key '"<< BOLDCYAN<<key << "'" << std::endl);
    }
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "YAML::Exception " << std::endl 
                                  << BOLDYELLOW << e.msg << std::endl 
                                  << BOLDRED    << "Key: '"<< key << "'" << std::endl 
                                  << RESET      << "Node: "  << node );
    return false;
  }
  catch( std::exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "std::exception "         << std::endl 
                                  << BOLDYELLOW << e.what()     << std::endl 
                                  << BOLDRED    << "Key: '"<< key << "'" << std::endl 
                                  << RESET      << "Node: "  << node );
    return false;
  }
  catch( ... )
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "Unhandled Exception " << std::endl);
    return false;
  }
  return ok;
};




template< class T >
inline bool getParamVector(const YAML::Node& node, std::vector< T >& ret)
{ 

  try 
  {
    YAML::Node config(node);
    
    if( config.Type() != YAML::NodeType::Sequence )
    {
      ROS_ERROR_STREAM( __FUNCTION__ << "Error " << std::endl 
                                << BOLDYELLOW << "Is not a sequence" << std::endl 
                                << RESET      << "Node: "  << node );  
  
      return false;
    }
  
    ret.clear();
    for( size_t i= 0; i< config.size(); i++ )
    {
      T n;
      n = config[ i ].as< T >( );
      ret.push_back( n );
    }
  
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "YAML::Exception " << std::endl 
                                  << BOLDYELLOW << e.msg << std::endl 
                                  << RESET      << "Node: "  << node );
    return false;
  }
  catch( std::exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "std::exception "         << std::endl 
                                  << BOLDYELLOW << e.what()     << std::endl 
                                  << RESET      << "Node: "     << node );
    return false;
  }
  catch( ... )
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "Unhandled Exception " << std::endl);
    return false;
  }
  
  return true;
}

template< class T, size_t n >
inline bool getParamArray(const YAML::Node& node, boost::array< T, n >& ret)
{ 
  std::vector< T > v;
  if(!getParamVector ( node, v ) )
    return false;
  if( v.size() != n )
    return false;
  
  for( size_t i=0; i<n; i++ )
    ret[i] = v[i];
  return true;
}


template< class T >
inline bool getParamVector(const YAML::Node& node, const std::string& key, std::vector< T >& ret)
{ 

  bool ok = false;
  try 
  {
    if( key != "" )
    {
      ok = getParam( node[ key ], ret );
    }
    else
      ok = getParam( node, ret );
    
    if( !ok )
    {
      ROS_ERROR_STREAM( __FUNCTION__ << "Error in processing the key '"<< BOLDCYAN<<key << "'" << std::endl);
    }  
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "YAML::Exception " << std::endl 
                                  << BOLDYELLOW << e.msg << std::endl 
                                  << BOLDRED    << "Key: '"<< key << "'" << std::endl 
                                  << RESET      << "Node: "  << node );
    return false;
  }
  catch( std::exception& e)
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "std::exception "         << std::endl 
                                  << BOLDYELLOW << e.what()     << std::endl 
                                  << BOLDRED    << "Key: '"     << key << "'" << std::endl 
                                  << RESET      << "Node: "     << node );
    return false;
  }
  catch( ... )
  {
    ROS_ERROR_STREAM( __FUNCTION__ << "Unhandled Exception " << std::endl);
    return false;
  }
  
  return ok;
}



template< class T, size_t n >
inline bool getParamArray(const YAML::Node& node, const std::string& key, boost::array< T, n >& ret)
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


}; 
};
#endif