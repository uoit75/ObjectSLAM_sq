#include "kalman_filter_imu.hpp"

MultiKalmanFilter::MultiKalmanFilter(double process_noise, double measurement_noise, double estimated_error, const Eigen::VectorXd& initial_value)
: process_noise_(process_noise), measurement_noise_(measurement_noise), state_(initial_value) {
  estimated_error_cov_ = Eigen::MatrixXd::Identity(state_.size(), state_.size()) * estimated_error;
}

Eigen::VectorXd MultiKalmanFilter::update(const Eigen::VectorXd& measurement) {
  // Prediction update (时间更新)
  estimated_error_cov_ += Eigen::MatrixXd::Identity(state_.size(), state_.size()) * process_noise_;

  // Measurement update (测量更新)
  Eigen::MatrixXd kalman_gain = estimated_error_cov_ * (estimated_error_cov_ + Eigen::MatrixXd::Identity(state_.size(), state_.size()) * measurement_noise_).inverse();
  state_ += kalman_gain * (measurement - state_);
  estimated_error_cov_ = (Eigen::MatrixXd::Identity(state_.size(), state_.size()) - kalman_gain) * estimated_error_cov_;

  return state_;
}