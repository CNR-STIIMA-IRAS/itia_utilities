#ifndef __ITIA_FILE_UTILS__STRING__
#define __ITIA_FILE_UTILS__STRING__

#include <sstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <map>
#include <iostream>

#include <itia_futils/itia_futils.h>

namespace itia
{
namespace futils
{
  
template< typename T > std::string to_string  ( const std::vector< T >& v
                                              , const std::string prefix = "["
                                              , const std::string delimeter = ", "
                                              , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(v.size() == 0)
    return "";
  
  for( size_t i=0; i < v.size()-1; i++)
    ret += std::to_string( v[i] ) + delimeter;
  ret += std::to_string( v.back() ) + terminator;
  
  return ret;
}

  
template<> inline std::string to_string<>( const std::vector< std::string >& v
                              , const std::string prefix
                              , const std::string delimeter
                              , const std::string terminator )
{
  
  std::string ret = prefix;
  if( v.size() > 0 )
  { 
    for( size_t i=0; i < v.size()-1; i++)
      ret += v[i] + delimeter;
    ret += v.back() + terminator;
  }
  else
    ret += terminator;
  
  return ret;
}
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}

template< typename T > std::string to_string_keys ( const std::map< std::string, T >& m
                                                  , const std::string prefix = "["
                                                  , const std::string delimeter = ", "
                                                  , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(m.size() == 0)
    return "";
  
  for( auto im = m.begin(); im != m.end(); im++)
    ret += im->first + delimeter;
  ret += terminator;
  
  return ret;
}



 
}
}


#endif
