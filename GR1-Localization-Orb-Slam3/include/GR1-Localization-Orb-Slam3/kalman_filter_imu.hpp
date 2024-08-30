#ifndef MULTI_KALMAN_FILTER_HPP_
#define MULTI_KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Dense>

class MultiKalmanFilter {
public:
  MultiKalmanFilter(double process_noise, double measurement_noise, double estimated_error, const Eigen::VectorXd& initial_value);
  Eigen::VectorXd update(const Eigen::VectorXd& measurement);

private:
  double process_noise_;
  double measurement_noise_;
  Eigen::MatrixXd estimated_error_cov_;
  Eigen::VectorXd state_;
};

#endif  // MULTI_KALMAN_FILTER_HPP_