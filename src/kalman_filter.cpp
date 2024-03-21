#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */

  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose() + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * (z - H_ * x_);
  P = ( MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  float square_root_sum = std::pow(x_(0) * x_(0) + x_(1) * x_(1),0.5);
  float cross_product = x_(3) * x_(1) - x_(4) * x_(0);
  x_est = VectorXd(4);
  x_est << square_root_sum, atan2(x_(0),x_(1)), cross_product/square_root_sum;
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * (z - x_est);
  P = (MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
}
