#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0, 
             0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    VectorXd x_init = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      Tools::PrintDebug("EKF: Initializing with RADAR measuremet");

      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      // float rho_dot = measurement_pack.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = 0.0f;
      float vy = 0.0f;

      x_init << px, py, vx, vy;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      Tools::PrintDebug("EKF: Initializing with LASER measuremet");

      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      float vx = 0.0f;
      float vy = 0.0f;

      x_init << px, py, vx, vy;
    }

    MatrixXd P_init = MatrixXd(4, 4);
    P_init << 1, 0, 0, 0, 
              0, 1, 0, 0, 
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    MatrixXd F_init = MatrixXd(4,4);
    F_init << 1, 0, 0, 0, 
              0, 1, 0, 0, 
              0, 0, 1, 0, 
              0, 0, 0, 1; 

    MatrixXd Q_init(4,4);

    ekf_.Init( 
        x_init,
        P_init,
        F_init,
        H_laser_,
        R_laser_,
        R_radar_,
        Q_init);

    previous_timestamp_ = measurement_pack.timestamp_;

    Tools::PrintDebugVector("EKF: Initialized x_", ekf_.x_);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  float noise_ax = 9.0f;
  float noise_ay = 9.0f;

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0f;
  float dt_2 = dt * dt;
  float dt_3 = dt * dt_2;
  float dt_4 = dt * dt_3;
  float dt_3_o2 = dt_3 / 2.0f;
  float dt_4_o4 = dt_4 / 4.0f;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0, 
             0, 1, 0, dt, 
             0, 0, 1, 0, 
             0, 0, 0, 1; 

  ekf_.Q_  = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_o4 * noise_ax, 0, dt_3_o2 * noise_ax, 0,
             0, dt_4_o4 * noise_ay, 0, dt_3_o2 * noise_ay,
             dt_3_o2 * noise_ax, 0, dt_2 * noise_ax, 0, 
             0, dt_3_o2 * noise_ay, 0, dt_2 * noise_ay;
  
  ekf_.Predict();

  previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
