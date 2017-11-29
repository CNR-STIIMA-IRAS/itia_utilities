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