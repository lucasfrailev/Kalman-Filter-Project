#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
  */

  // Initialize RSME vector as zeros and compute length of ground_truth
  Eigen::MatrixXd RSME_;
  RSME_ = VectorXd::Zero(4);
  int n = estimations.size();

  // If the length of 'estimations' does not match that of 'ground_truth', or the later is zero, return a warning
  if (ground_truth.size() != estimations.size() || n == 0 ){
   cout << "Invalid estimation or ground_truth data" << endl;
   return RSME_;
  }

  // We use a simple algebra trick to compute the weighted square of the scalar differences in a single line
  for (int i=0;i<n;i++){
   RSME_ += 1.0 / n * ((estimations[i] - ground_truth[i]) * (estimations[i] - ground_truth[i]).transpose()).diagonal();
  }
  // Return the square root of the sum
   return RSME_.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   // These will come out handy to keep the code "clean"
   float sum_of_squares = x_state(0) * x_state(0) + x_state(1) * x_state(1);
   float square_root_sum = std::pow(sum_of_squares,0.5);
   float cross_product = x_state(3) * x_state(0) - x_state(2) * x_state(1);

   Eigen::MatrixXd Hj_;
   Hj_ = MatrixXd::Zero(3,4);
  // Avoid division by zero
  if (fabs(sum_of_squares) < 0.00001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj_;
  }
   Hj_(0,0) = x_state(0) / square_root_sum;
   Hj_(0,1) = x_state(1) / square_root_sum;
   Hj_(1,0) =-x_state(1) / sum_of_squares;
   Hj_(1,1) =-x_state(0) / sum_of_squares;
   Hj_(2,0) =-x_state(1) * cross_product / (square_root_sum * sum_of_squares);
   Hj_(2,1) = x_state(0) * cross_product / (square_root_sum * sum_of_squares);
   Hj_(2,2) = x_state(0) / square_root_sum;
   Hj_(2,3) = x_state(1) / square_root_sum;
   return Hj_;
}
