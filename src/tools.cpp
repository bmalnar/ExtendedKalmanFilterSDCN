#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // Create an initial value of the rmse vector, set all values to zero
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Estimations and ground truth need to match in terms of size 
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "CalculateRMSE: Invalid estimation or ground_truth data" << std::endl; 
    return rmse;
  }

  // For each data point, calculate the residual and add to the rmse vector
  for (unsigned int i = 0; i < estimations.size(); i++) {

    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Normalize and calculate square root to get the final rmse value
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if (px == 0 && py == 0) { 

    // If both px and py are zero, we have division by zero, cannot proceed
    std::cout << "CalculateJacobian: Division by zero" << std::endl;

  } else {

    // Calculate the Jacobian matrix based on the equations from the 
    // Udacity lectures on Extended Kalman Filter

    Hj(0,2) = 0;
    Hj(0,3) = 0;
    Hj(1,2) = 0;
    Hj(1,3) = 0;

    float px_sq = px * px;
    float py_sq = py * py;
    float sum_sq = px_sq + py_sq;
    float sqrt_sum_sq = sqrt(sum_sq);
    float pow_sum_sq = pow(sum_sq, 1.5);

    Hj(0,0) = px / sqrt_sum_sq; 
    Hj(0,1) = py / sqrt_sum_sq;
    Hj(1,0) = -1 * py / sum_sq;; 
    Hj(1,1) = px / sum_sq;; 
    Hj(2,0) = py * (vx * py - vy * px) / pow_sum_sq; 
    Hj(2,1) = px * (vx * py + vy * px) / pow_sum_sq;
    Hj(2,2) = Hj(0,0);
    Hj(2,3) = Hj(0,1);
  }

  return Hj;
}

float Tools::NormalizeAngle(float angle) { 

  // Normalize angle to be in the range from
  // -pi to +pi

  while (angle > M_PI || angle < -1 * M_PI) {

    if (angle > M_PI) { 
      // If angle > pi, deduct 2*pi
      angle -= 2 * M_PI;
    } else { 
      // If angle < -pi, add 2*pi
      angle += 2 * M_PI;
    }
  }

  return angle;
}

VectorXd Tools::CalculateRadarHFunction(const VectorXd& x_state) {

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Calculate translaion from the state vector to the radar coordinates

  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  // Normalize the angle phi to be within the range -pi to +pi

  float phi_norm = this->NormalizeAngle(phi);

  VectorXd h = VectorXd(3);

  h << rho, phi_norm, rho_dot;
  
  return h;
}

void Tools::PrintDebug(const std::string s) {

  // Print the string if the debug flag is set to true, otherwise do nothing
  if (Tools::print_flag_ == true) { 
    std::cout << s << std::endl;
  }
}

void Tools::PrintDebugMatrix(const std::string s, const MatrixXd mat) {

  // Print the matrix if the debug flag is set to true, otherwise do nothing
  if (Tools::print_flag_ == true) {
    std::cout << s << ":" << std::endl << mat << std::endl;
  }
}

void Tools::PrintDebugVector(const std::string s, const VectorXd vec) {

  // Print the vector if the debug flag is set to true, otherwise do nothing
  if (Tools::print_flag_ == true) {
    std::cout << s << ":" << std::endl << vec << std::endl;
  }
}
