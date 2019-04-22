/*
* @Original Author: Udacity
* @Last Modified by: debasis123
*/

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"

/**
 * Data Structure to initialize the filter,
 * Orchestrate the calls to the predict function and the update function
 */
class FusionEKF {
public:
  /**
  * Constructor.
  * Initializes the H and R matrices that are different for Laser and Radar
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Initializes other matrices and
  * run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage& meas);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // noise covariance and measurement matrices
  Eigen::MatrixXd R_laser_; // noise covariance matrix - laser
  Eigen::MatrixXd R_radar_; // noise covariance matrix - radar
  Eigen::MatrixXd H_laser_; // measurement matrix for laser
  Eigen::MatrixXd Hj_; // H_jacobian: measurement matrix for radar

  // process noise
  const double noise_ax = 9,
               noise_ay = 9;

  void initialize_KF_with_first_measurement(const MeasurementPackage& meas);
};

#endif /* FusionEKF_H_ */
