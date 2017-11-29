
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

#ifndef __ITIA_FILE_UTILS__
#define __ITIA_FILE_UTILS__

#include <sys/types.h>
#include <dirent.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <map>



#define __FUNCFILE__ (strrchr(__FILE__,'/') ? strrchr(__FILE__,'/') + 1 : __FILE__ )

static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";

namespace itia
{
namespace futils
{
  inline std::string check_colors()
  {
    std::string ret; 
    ret += std::string( DEFAULT      ); 
    ret += std::string( RESET        ); 
    ret += std::string( BLACK        ); 
    ret += std::string( RED          ); 
    ret += std::string( GREEN        ); 
    ret += std::string( YELLOW       ); 
    ret += std::string( BLUE         ); 
    ret += std::string( MAGENTA      ); 
    ret += std::string( CYAN         ); 
    ret += std::string( WHITE        ); 
    ret += std::string( BOLDBLACK    ); 
    ret += std::string( BOLDRED      ); 
    ret += std::string( BOLDGREEN    ); 
    ret += std::string( BOLDYELLOW   ); 
    ret += std::string( BOLDBLUE     ); 
    ret += std::string( BOLDMAGENTA  ); 
    ret += std::string( BOLDCYAN     ); 
    ret += std::string( BOLDWHITE    ); 
    return ret; 
  }
  
  inline int getdir (std::string dir, std::vector<std::string> &files)
  {
    DIR *dp;
    struct dirent *dirp;
    
    if((dp  = opendir(dir.c_str())) == NULL) 
    {
      std::cout << "Error(" << errno << ") opening " << dir << std::endl;
      return errno;
    }

    while ((dirp = readdir(dp)) != NULL) 
      files.push_back( std::string(dirp->d_name) );    
    
    closedir(dp);
    
    return 0;
  }


inline std::vector< std::string > ls( const std::string& path, const std::vector< std::string >& extensions = std::vector< std::string >(), const std::string& what="RELATIVE" )
{
  
  assert(what == "RELATIVE" || what == "ABSOLUTE" );
  
  std::vector< std::string > fn;
  
  boost::filesystem::path p( path );
  if (!boost::filesystem::exists(p))    // does p actually exist?
  {
    std::cout << "The path " << p << " does not exist\n";
    return fn;
  }

  if (!boost::filesystem::is_directory(p) )
  {
    std::cout << "The path " << p << " is a not directory\n";
      return fn;
  }

  boost::filesystem::recursive_directory_iterator endit;
  for(boost::filesystem::recursive_directory_iterator it(p); it != endit; it++ )
  {
    if( !boost::filesystem::is_regular_file(*it) )
    {
      continue;
    }
    if( extensions.size() == 0 )
    {
      if( what=="ABSOLUTE" )
       fn.push_back( it->path().string( ) );
      else
       fn.push_back( it->path().leaf().string( ) );
    }
    else
    {
      for(size_t i = 0; i < extensions.size(); i++ )
      {
        if( (extensions[i] == it->path().extension() )
        ||  ("."+extensions[i] == it->path().extension() ) )
        {
         if( what=="ABSOLUTE" )
         {
          fn.push_back( it->path().string( ) );
         }
         else
         {
          fn.push_back( it->path().leaf().string( ) );
         }
          break;
        }
      }
    }
  }
  return fn;
}


inline std::vector< std::string > lsDirectoryOnly( const std::string& path )
{
  
  std::vector< std::string > dir;
  
  boost::filesystem::path p( path );
  if (!boost::filesystem::exists(p))    // does p actually exist?
  {
    std::cout << "The path " << p << "does not exist\n";
    return dir;
  }

  if (!boost::filesystem::is_directory(p) )
  {
    std::cout << "The path " << p << "is a not directory\n";   
    return dir;
  }

  boost::filesystem::recursive_directory_iterator endit;
  for(boost::filesystem::recursive_directory_iterator it(p); it != endit; it++ )
  {
    if( boost::filesystem::is_directory(*it) )
    {
      dir.push_back( boost::filesystem::canonical(*it).string() );
    }
  }
  return dir;
}


inline std::vector< boost::filesystem::path > lsDirectoryOnly( const boost::filesystem::path& p )
{
  
  std::vector< boost::filesystem::path > dir;
  
  if (!boost::filesystem::exists(p))    // does p actually exist?
  {
    std::cout << "The path " << p << "does not exist\n";
    return dir;
  }

  if (!boost::filesystem::is_directory(p) )
  {
    std::cout << "The path " << p << "is a not directory\n";   
    return dir;
  }

  boost::filesystem::recursive_directory_iterator endit;
  for(boost::filesystem::recursive_directory_iterator it(p); it != endit; it++ )
  {
    if( boost::filesystem::is_directory(*it) )
    {
      dir.push_back( *it );
    }
  }
  return dir;
}


  
inline bool remove_files( const std::string& path, const std::vector< std::string >& extensions = std::vector< std::string >() )
{
  boost::filesystem::path p( path );
  if (!boost::filesystem::exists(p))    // does p actually exist?
  {
    std::cout << "The path " << p << "does not exist\n";
    return false;
  }

  if (!boost::filesystem::is_directory(p) )
  {
    std::cout << "The path " << p << "is a not directory\n";   
      return false;
  }

  boost::filesystem::recursive_directory_iterator endit;
  for(boost::filesystem::recursive_directory_iterator it(p); it != endit; it++ )
  {
    if( !boost::filesystem::is_regular_file(*it) )
    {
      continue;
    }
    if( extensions.size() == 0 )
      boost::filesystem::remove( *it );
    else
    {
      for(size_t i = 0; i < extensions.size(); i++ )
      {
        if( extensions[i] == it->path().extension() )
        {
          boost::filesystem::remove( *it );
          break;
        }
      }
    }
  }
  return true;
}
inline bool create_directory_if_needed( const std::string& directory, std::string& path) 
{
  
  boost::filesystem::path p( directory );
  
  if (!boost::filesystem::exists(p))   
  {
    if( boost::filesystem::create_directory( p ) ) 
    {
      path = boost::filesystem::canonical( p ).string();
      return true;
    }
    else
      return false;
  }
  else if (boost::filesystem::is_directory(p) )
  {
    path = boost::filesystem::canonical( p ).string();
    return true;
  }
  else
  {
    std::cout << "The path " << p << "is a not directory\n";   
    return false;
  }
  return true;
}



inline bool create_directory_if_needed( const std::string& root, const std::string& directory, std::string& path) 
{
  boost::filesystem::path p( root );
  if (!boost::filesystem::exists(p))   
  {
    std::cerr << "The path" << p << "does not exist\n";
    return false;
  }
  if (!boost::filesystem::is_directory( root ) )
  {
    std::cerr << "The path" << p << "is a not directory\n";
      return false;
  }
  
  std::string rh = directory;
  if( boost::filesystem::canonical( p ).leaf() == rh )
  {
    path = boost::filesystem::canonical( p ).string();
    return true;
  }
  else
  {
    boost::filesystem::path pp( p / (directory) );
    if (!boost::filesystem::exists(pp))   
    {
      if( boost::filesystem::create_directory( pp ) ) 
      {
        path = boost::filesystem::canonical( pp ).string();
        return true;
      }
      else
        return false;
    } 
    else if (!boost::filesystem::is_directory( root ) )
    {
      std::cerr <<"The path" <<  pp << "is a not directory\n";
      return false;
    }
    else 
    {
      path = boost::filesystem::canonical( pp ).string();
      return true;
    }
  }
  return true;
}


inline bool check_if_directory_exist( const std::string& root, const std::string& directory, std::string* path = NULL) 
{
  boost::filesystem::path p( root );
  if (!boost::filesystem::exists(p))   
  {
    std::cerr <<"The path" <<  p << "does not exist\n";
    return false;
  }
  
  if (!boost::filesystem::is_directory( root ) )
  {
    std::cerr << "The path" << p << "is a not directory\n";
      return false;
  }
  
  std::string rh = directory;
  if( boost::filesystem::canonical( p ).leaf() == rh )
  {
    *path = boost::filesystem::canonical( p ).string();
    return true;
  }
  else
  {
    boost::filesystem::path pp( p / (directory) );
    if (!boost::filesystem::exists(pp))   
    {
      return false;
    } 
    else if (!boost::filesystem::is_directory( pp ) )
    {
      std::cerr << pp << "is a not directory\n";
      return false;
    }
    else 
    {
      *path = boost::filesystem::canonical( pp ).string();
      return true;
    }
  }
  return true;
}



inline bool check_if_directory_exist( const std::string& path, std::string* path_in_canonical_form = NULL ) 
{
  boost::filesystem::path p( path );
  if (!boost::filesystem::exists(p))   
  {
    std::cerr <<"The path" <<  p << "does not exist\n";
    return false;
  }
  
  if (!boost::filesystem::is_directory( path ) )
  {
    std::cerr << "The path" << p << "is a not directory\n";
      return false;
  }
  if( path_in_canonical_form != NULL )
  {
    *path_in_canonical_form = boost::filesystem::canonical( path ).string();
  }
  
  return true;
}



inline bool check_if_file_exist( const std::string& path) 
{
  boost::filesystem::path p( path );
  if (!boost::filesystem::exists(p))   
  {
    std::cerr << "The path" <<  p << "does not exist\n";
    return false;
  }
  if(boost::filesystem::is_directory( path ) )
  {
    std::cerr << "The path" << p << "is a directory\n";
      return false;
  }
  if(!boost::filesystem::is_regular_file( path ) )
  {
    std::cerr << "The path" << p << "is not a regular file\n";
    return false;
  }
  
  return true;
}



}
}


#endif
