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