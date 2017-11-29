#include <itia_gutils/itia_gutils.h>

int main(int argc, char **argv)
{

  Eigen::Matrix4d center = Eigen::Matrix4d::Identity();
  
  center(1,1) =   0.0;
  center(1,2) =  -1.0;
  center(2,1) =   1.0;
  center(2,2) =   0.0;
  
  std::cout << "center =\n" << center << std::endl;
  
  double n_pnts = 4;
  double radius = 1;
  double eight  = 1;
  
  std::vector<Eigen::Matrix4d> points;
  points.resize(n_pnts);
  
  itia::gutils::circle_trj(center,radius,eight,n_pnts,&points);
    
  for (int idx=0;idx<n_pnts;idx++)
      std::cout << "points.at("<<idx<<") =\n" << points.at(idx) << std::endl;
  
  return 0;
}