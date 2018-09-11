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
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  // process noise
  const double noise_ax = 9, noise_ay = 9;

  void initialize_KF_with_first_measurement(const MeasurementPackage& measurement_pack);
};

#endif /* FusionEKF_H_ */
