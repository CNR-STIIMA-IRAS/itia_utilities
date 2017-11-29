
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

#ifndef __ITIA_MUTILS__POLYNOMIAL_UTILS__
#define __ITIA_MUTILS__POLYNOMIAL_UTILS__

#include <unsupported/Eigen/Polynomials>
#include <ros/console.h>

namespace itia {
namespace mutils {
  
Eigen::MatrixXd evalMatrix( const double& t,
                            const unsigned int& n_coeff,
                            const unsigned int& order);

Eigen::VectorXd derivative(const Eigen::VectorXd& in_poly);

Eigen::VectorXd computeCoefficients(const Eigen::VectorXd& init_state,
                                    const Eigen::VectorXd& final_state,
                                    const double& init_time,
                                    const double& final_time);

Eigen::VectorXd computeCoefficients(const double& init_state,
                                    const double& final_state,
                                    const double& init_time,
                                    const double& final_time);

void computeMaxMinValues(const Eigen::VectorXd coeff, const double& init_time, const double& final_time, double* min_value, double* max_value);

void computeMaxMinValuesDer(  const Eigen::VectorXd coeff, const double& init_time, const double& final_time, const unsigned int& derivative_order, Eigen::VectorXd* min_values, Eigen::VectorXd* max_values);
}
}

#endif 