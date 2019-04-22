/*
* @Original Author: Udacity
* @Last Modified by:   debasis123
*/

#include "FusionEKF.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // noise covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  // noise covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  // measurement matrix for laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Measurement matrix for radar
  // will be initialized based on Jacobian (predicted state)
  Hj_ = MatrixXd(3, 4);

  // process noise initialized in class definition
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& meas) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    initialize_KF_with_first_measurement(meas);
    cout << "initialized: should be called once!" << endl;

    // done initializing, no need to predict or update
    previous_timestamp_ = meas.timestamp_;
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements
  const double dt = (meas.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = meas.timestamp_;

  //1. Modify the F_ matrix so that the time is integrated
  ekf_.F_(0,2) = dt;  // for vx
  ekf_.F_(1,3) = dt;  // for vy

  //2. Set the process covariance matrix Q_
  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt3*dt;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ <<  dt4/4*noise_ax, 0,              dt3/2*noise_ax, 0,
              0,              dt4/4*noise_ay, 0,              dt3/2*noise_ay,
              dt3/2*noise_ax, 0,              dt2*noise_ax,   0,
              0,              dt3/2*noise_ay, 0,              dt2*noise_ay;


  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
    // Radar updates with the most recent raw measurements_
    Hj_ = Tools::CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(meas.raw_measurements_);
  }
  else if (meas.sensor_type_ == MeasurementPackage::SensorType::LASER) {
    // Laser updates with the most recent raw measurements_
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(meas.raw_measurements_);
  }

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::initialize_KF_with_first_measurement(const MeasurementPackage& meas) {
  /**
    * Initialize the state ekf_.x_ with the first measurement from sensor.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */
  // first measurement
  cout << "EKF initialization: " << endl;
  // state vector
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1; // imp for RMSE!

  if (meas.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    const double rho = meas.raw_measurements_(0); // range
    const double phi = meas.raw_measurements_(1); // bearing
    const double rho_dot = meas.raw_measurements_(2); // range rate

    ekf_.x_ << rho*cos(phi),
               rho*sin(phi),
               rho_dot*cos(phi), // not really correct!! but may be ok for initialization
               rho_dot*sin(phi); // same!
  }
  else if (meas.sensor_type_ == MeasurementPackage::SensorType::LASER) {
    /**
    Initialize state.
    */
    //set the state with the initial location and zero velocity
    ekf_.x_ << meas.raw_measurements_(0),
               meas.raw_measurements_(1),
               0,
               0;
  }

  //state covariance matrix P_
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0,    0,
              0, 1, 0,    0,
              0, 0, 1000, 0,
              0, 0, 0,    1000;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 0, 0, // diagonal values set to 1
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
}
