#ifndef __ITIA_NORM_DERIVATIVE__

# include <boost/math/special_functions/binomial.hpp>

namespace itia
{
namespace mutils
{
  
inline double power_derivative(const double& x, const double& exponential_order, const unsigned int der_order)
{
  double der_exponential_order = exponential_order - der_order;
  double coeff = 1;
  for (unsigned int idx = 0;idx<der_order;idx++)
  {
    coeff *= (exponential_order-idx);
    ROS_INFO_STREAM("idx = " << idx <<  ", c = " << coeff);
  }
  
  return coeff*std::pow(x, der_exponential_order);
  
};

}
}
#endif
