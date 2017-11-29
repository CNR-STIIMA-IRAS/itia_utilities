
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

#ifndef __ITIA_LINEAR_MOVEMENTS__
#define __ITIA_LINEAR_MOVEMENTS__

#include <stdio.h>
#include <math.h>
#include <vector>

#include <itia_futils/itia_futils.h>
#include <itia_gutils/itia_gutils.h>

namespace itia
{
namespace tutils
{
  inline bool get3DCartLinerInterpolation(const std::vector<double>& st_pnt, 
                                          const std::vector<double>& end_pnt, 
                                          const double& step,
                                          std::vector< std::vector<double> >& trajectory )
  {
    // 3D Linear Interpolation between two points with a fixed step 
    // NOT DEBUGGED
    double distance = itia::gutils::cartesian_norm( end_pnt, st_pnt );
     
    if (step > abs(distance) )
    {
      printf ( " [ %s%s:%d%s ]\t %s ERROR: the required step is less than the norm between strating and end position! %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
      return false;
    }
    
    if ( ( st_pnt.size() != 3 ) && ( end_pnt.size() != 3 ) )
    {
      printf ( " [ %s%s:%d%s ]\t %s ERROR: wrong number of element in input vector! %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
      return false;
    }
    
    int trjPnt = floor( abs(distance)/step );
    
    trajectory.clear();
    trajectory.push_back( st_pnt );
          
    std::vector<double> direct;
    for( size_t i=0; i<3; i++ )
      direct.push_back( ( end_pnt.at(i) - st_pnt.at(i) ) / distance );
    
    for(size_t iPnt=1; iPnt<trjPnt+1; iPnt++)
    {
      std::vector<double> tmp_;
      for(size_t iEl=0; iEl<3; iEl++)
        tmp_.push_back( trajectory.at(iPnt-1).at(iEl) + step * direct.at(iEl) );
        
      trajectory.push_back( tmp_ );
    }
    
    trajectory.push_back( end_pnt );
    
    return true;
    
  }
 

}
}

#endif
