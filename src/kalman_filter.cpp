#include<iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
          MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_ekf_in, MatrixXd &Q_in) {

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;

  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);

  // H is the measurement function of the laser
  H_ = H_in;
  // R_ is the measurement noise of the laser
  R_ = R_in;
  // R_ekf_ is the measurement noise of the radar
  R_ekf_ = R_ekf_in;
}

void KalmanFilter::Predict() {

  Tools::PrintDebug("Enter Predict()");

  Tools::PrintDebugVector("x_", x_);
  Tools::PrintDebugMatrix("F_", F_);

  MatrixXd F_transpose = F_.transpose();
  Tools::PrintDebugMatrix("F_transpose", F_transpose);

  x_ = F_ * x_;
  Tools::PrintDebugVector("x_", x_);

  Tools::PrintDebugMatrix("P_", P_);
  Tools::PrintDebugMatrix("Q_", Q_);

  P_ = F_ * P_ * F_transpose + Q_;
  Tools::PrintDebugMatrix("P_", P_);
  
  Tools::PrintDebug("Exit Predict()");
}

void KalmanFilter::Update(const VectorXd &z) {

  Tools::PrintDebug("Enter Update()");

  Tools::PrintDebugMatrix("H_", H_);

  MatrixXd H_transpose = H_.transpose();
  Tools::PrintDebugMatrix("H_transpose", H_transpose);

  Tools::PrintDebugVector("x_", x_);
  Tools::PrintDebugVector("z", z);

  VectorXd y = z - H_ * x_;
  Tools::PrintDebugVector("y", y);

  MatrixXd S = H_ * P_ * H_transpose + R_;
  Tools::PrintDebugMatrix("S", S);

  MatrixXd K = P_ * H_transpose * S.inverse();
  Tools::PrintDebugMatrix("K", K);

  // New estimate
  x_ = x_ + (K * y);
  Tools::PrintDebugVector("x_", x_);

  P_ = (I_ - K * H_) * P_;
  Tools::PrintDebugMatrix("P_", P_);

  Tools::PrintDebug("Exit Update()");
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  Tools t;

  Tools::PrintDebug("Enter UpdateEKF()");
  Tools::PrintDebugVector("z", z);
  Tools::PrintDebugVector("x_", x_);

  VectorXd h = t.CalculateRadarHFunction(x_);
  Tools::PrintDebugVector("h", h);

  MatrixXd Hj = t.CalculateJacobian(x_);
  Tools::PrintDebugMatrix("Hj", Hj);

  MatrixXd Hj_transpose = Hj.transpose();
  Tools::PrintDebugMatrix("Hj_transpose", Hj_transpose);

  VectorXd y = z - h;
  y(1) = t.NormalizeAngle(y(1));

  Tools::PrintDebugVector("y", y);

  Tools::PrintDebugMatrix("P_", P_);
  Tools::PrintDebugMatrix("R_ekf_", R_ekf_);

  MatrixXd S = Hj * P_ * Hj_transpose + R_ekf_;
  Tools::PrintDebugMatrix("S", S);

  MatrixXd K = P_ * Hj_transpose * S.inverse();
  Tools::PrintDebugMatrix("K", K);

  // New estimate
  x_ = x_ + (K * y);
  Tools::PrintDebugVector("x_", x_);

  P_ = (I_ - K * Hj) * P_;
  Tools::PrintDebugMatrix("P_", P_);

  Tools::PrintDebug("Exit UpdateEKF()");
}
