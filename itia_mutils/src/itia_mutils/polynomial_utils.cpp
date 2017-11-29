
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

#include <itia_mutils/polynomial_utils.h>

namespace itia {
namespace mutils {
  
Eigen::MatrixXd evalMatrix( const double& t,
                            const unsigned int& n_coeff,
                            const unsigned int& order)
{
  Eigen::MatrixXd matrix;
  matrix.resize(order,n_coeff);
  matrix.setZero();
  
  for (int ir=0;ir<order;ir++)
  {
    if (ir==0)
      for (int ic=0;ic<n_coeff;ic++)
        matrix(ir,ic)=1;
    else
      for (int ic=1;ic<(n_coeff);ic++)
        matrix(ir,ic)=(ic)*matrix(ir-1,ic-1);
  }
  for (int ir=0;ir<order;ir++)
  {
    for (int ic=ir;ic<n_coeff;ic++)
    {
      matrix(ir,ic)*=pow(t,ic-ir);
    }
  }
  
  return matrix;
};

Eigen::VectorXd derivative(const Eigen::VectorXd& in_poly)
{
  Eigen::VectorXd out_poly(in_poly.size());
  out_poly.setZero();
  for (int idx=0;idx<(in_poly.size()-1);idx++)
    out_poly(idx)=in_poly(idx+1)*(idx+1);
  
  return out_poly;
}

Eigen::VectorXd computeCoefficients(const Eigen::VectorXd& init_state, const Eigen::VectorXd& final_state, const double& init_time, const double& final_time)
{
  Eigen::MatrixXd time_matrix;
  if (final_state.rows() !=init_state.rows())
  {
    ROS_ERROR("initial and final states have different dimension!! initial state dim = %d, final state dim = %d",(int)init_state.rows(),(int)final_state.rows());
    throw("wrong dimension of the initial and final states");
  }
  unsigned int order=final_state.rows();
  unsigned int n_coeff=2*order;
  
  time_matrix.resize(n_coeff,n_coeff);
  time_matrix.setZero();
  time_matrix.block(0,0,order,n_coeff)     = itia::mutils::evalMatrix(init_time, n_coeff,order);
  time_matrix.block(order,0,order,n_coeff) = itia::mutils::evalMatrix(final_time,n_coeff,order);
  
  Eigen::VectorXd ini_fin_conditions;
  ini_fin_conditions.resize(n_coeff);
  ini_fin_conditions.block(0,0,order,1)     = init_state;
  ini_fin_conditions.block(order,0,order,1) = final_state;
  
  Eigen::VectorXd coefficients=time_matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ini_fin_conditions);
  
  return coefficients;
};


Eigen::VectorXd computeCoefficients(const double& init_state, const double& final_state, const double& init_time, const double& final_time)
{
  Eigen::VectorXd init_state_(1);
  init_state_(0) =init_state;
  Eigen::VectorXd final_state_(1);
  final_state_(0) =final_state;
  return computeCoefficients(init_state_, final_state_, init_time, final_time);
};

void computeMaxMinValues(const Eigen::VectorXd coeff, const double& init_time, const double& final_time, double* min_value, double* max_value)
{
  ros::Time t0_=ros::Time::now();
  ros::Time t0;
  ros::Duration dt;
  double init_value=poly_eval(coeff,init_time);
  double final_value=poly_eval(coeff,final_time);
  
  double max=std::max(init_value,final_value);
  double min=std::min(init_value,final_value);
  
  Eigen::VectorXd Dcoeff=derivative(coeff);
  int degree=0;
  for (int idx=0;idx<Dcoeff.rows();idx++)
    if (std::abs(Dcoeff(idx))>1.0e-8)
      degree=idx;

  if (degree>=2)
  {
    
    
    Eigen::EigenSolver<Eigen::MatrixXd> es;
    Eigen::MatrixXd companion(degree,degree);
    companion.setZero();
    companion.block(0,1,degree-1,degree-1).setIdentity();
    companion.block(degree-1,0,1,degree)=-Dcoeff.block(0,0,degree,1).transpose()/Dcoeff(degree);
    t0=ros::Time::now();es.compute(companion, /* computeEigenvectors = */ false);
    dt=ros::Time::now()-t0;
    Eigen::VectorXcd complex_roots = es.eigenvalues();
    
    
    
    for (int idx=0;idx<complex_roots.rows();idx++)
      if (std::abs(complex_roots(idx).imag())<1e-8)
      {
        double extremum=poly_eval(coeff,complex_roots(idx).real());
        max=std::max(max,extremum);
        min=std::min(min,extremum);
      }
  }
  else if (degree==1)
  {
    double root=-Dcoeff(0)/Dcoeff(1);
    double extrema=poly_eval(coeff,root);
    max=std::max(max,extrema);
    min=std::min(min,extrema);
  }
  
  *max_value=max;
  *min_value=min;
  ros::Duration dt_=ros::Time::now()-t0_;
  ROS_INFO("dt_=%f",dt_.toSec());
  ROS_INFO("dt=%f",dt.toSec());
    
}

void computeMaxMinValuesDer(const Eigen::VectorXd coeff, const double& init_time, const double& final_time, const unsigned int& derivative_order, Eigen::VectorXd* min_values, Eigen::VectorXd* max_values)
{
  max_values->resize(derivative_order);
  min_values->resize(derivative_order);
  
  
  Eigen::VectorXd der_coeff=coeff;
  for (int idx=0;idx<derivative_order;idx++)
  {
    der_coeff=derivative(der_coeff);
    computeMaxMinValues(der_coeff,init_time,final_time,&((*min_values)(idx)),&((*max_values)(idx)));
  }
  
}


}
}




