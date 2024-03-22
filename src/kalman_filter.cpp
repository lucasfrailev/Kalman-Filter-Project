#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::cout;
using std::endl;

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
  x_est << square_root_sum, atan2(x_(1),x_(0)), aux_sum/square_root_sum;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  VectorXd y = (z - x_est);
  while (y(1) > M_PI) y(1) -= 2 * M_PI;
  while (y(1) < -M_PI) y(1) += 2 * M_PI;
  x_ = x_ + K_ * y;
  P_ = (MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
}

VectorXd KalmanFilter::ComputeError(const VectorXd &z){
    /**
   * This function computes the error between state estimates and measurement. We will use it for the Iterated EKF
   */

  float square_root_sum = std::pow(x_(0) * x_(0) + x_(1) * x_(1),0.5);
  float aux_sum = x_(0) * x_(2) + x_(1) * x_(3);
  VectorXd x_est = VectorXd(3);
  x_est << square_root_sum, atan2(x_(1),x_(0)), aux_sum/square_root_sum;
  VectorXd y = (z - x_est);
  while (y(1) > M_PI) y(1) -= 2 * M_PI;
  while (y(1) < -M_PI) y(1) += 2 * M_PI;
  return y;
}

void KalmanFilter::UpdateIteratedEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  VectorXd y = KalmanFilter::ComputeError(z);
  x_ = x_ + K_ * y;
  int n = 0;
  while ((K_ * y).squaredNorm() > std::pow(0.1,10) && n<10){
    cout << "error = " << (K_ * y).squaredNorm() << endl;
    cout << "i = " << n << endl;
    n++;
    H_ = tools.CalculateJacobian(x_);
    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();
    y = KalmanFilter::ComputeError(z);
    x_ = x_ + K_ * y;
  }
  P_ = (MatrixXd::Identity(x_.size(),x_.size())- K_ * H_) * P_;
}