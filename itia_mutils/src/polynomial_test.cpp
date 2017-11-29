
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
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include <lapacke.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_mutils_test");
  ros::NodeHandle nh;
  
  int order=5;
  Eigen::VectorXd poly(order+1);
  
  int ntrial=1000;
  std::vector<Eigen::MatrixXd> companion(ntrial);
  
  for (int idx=0;idx<ntrial;idx++)
  {
    std::srand((unsigned int) std::time(0));
    poly=Eigen::MatrixXd::Random(order+1, 1);
    
    Eigen::MatrixXd t_companion(order,order);
    t_companion.setZero();
    t_companion.block(0,1,order-1,order-1).setIdentity();
    t_companion.block(order-1,0,1,order)=(-poly.block(0,0,order,1)/poly(order)).transpose();
    companion.at(idx)=t_companion;
  }
  
  
  ros::Time t0;
  ros::Duration dt,dt2;
  Eigen::VectorXcd eig, eig2;
  Eigen::EigenSolver<Eigen::MatrixXd> es,es2;
  
  t0=ros::Time::now();
  for (int idx=0;idx<ntrial;idx++)
  {
    es.compute(companion.at(idx), /* computeEigenvectors = */ false);
    eig = es.eigenvalues();
  }
  dt=ros::Time::now()-t0;
  std::cout << "EIGEN time = " << dt.toSec()/(double)ntrial*1000.0 /*<< ", eig = " << eig.transpose() << std::endl*/;
  
  int info;
  Eigen::MatrixXcd eig_vet;
  double wr[order], wi[order], vl[order*order], vr[order*order];
  t0=ros::Time::now();
  for (int idx=0;idx<ntrial;idx++)
    info = LAPACKE_dgeev( LAPACK_COL_MAJOR, 'N', 'N', order, companion.at(idx).data(), order, wr, wi,vl, order, vr, order );
  dt2=ros::Time::now()-t0;
  
  std::cout << ",\tLAPACK time = " << dt2.toSec()/(double)ntrial*1000.0 << ", EIGEN/LAPACK time ratio = " << dt.toSec()/dt2.toSec() << std::endl;
    

  return 0;  
}