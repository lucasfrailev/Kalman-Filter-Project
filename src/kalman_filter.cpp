#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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

  x_ = F_ * x_;

  P_ = F_ * P_ * F_.transpose() + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * (z - H_ * x_);
  P_ = ( MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  float square_root_sum = std::pow(x_(0) * x_(0) + x_(1) * x_(1),0.5);
  float aux_sum = x_(0) * x_(2) + x_(1) * x_(3);
  VectorXd x_est = VectorXd(3);
  cout << "Line 1" << endl;
  x_est << square_root_sum, atan2(x_(0),x_(1)), aux_sum/square_root_sum;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  cout << "Line 2" << endl;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * (z - x_est);
  P_ = (MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
  cout << "Line 3" << endl;
}
