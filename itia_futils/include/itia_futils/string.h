
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
