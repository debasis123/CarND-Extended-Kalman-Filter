/*
* @Original Author: Udacity
* @Last Modified by:   debasis123
*/

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:

  Eigen::VectorXd x_;   // state vector
  Eigen::MatrixXd P_;   // state covariance matrix
  Eigen::MatrixXd F_;   // state transition function matrix
  Eigen::MatrixXd Q_;   // process noise covariance matrix
  Eigen::MatrixXd H_;   // measurement function matrix
  Eigen::MatrixXd R_;   // measurement noise covariance matrix

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance using the process model
   */
  void Predict();

  /**
   * Updates the state by using "standard" Kalman Filter equations (for lidar)
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd& z);

  /**
   * Updates the state by using "Extended" Kalman Filter equations (for radar)
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd& z);

private:
  void CommonUpdateForLidarAndRadar(const Eigen::VectorXd& y);

};

#endif /* KALMAN_FILTER_H_ */
