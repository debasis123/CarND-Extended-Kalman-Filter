#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

/**
 * define the following:
 * - predict function,
 * - update function for lidar,
 * - update function for radar
 */
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
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd& x_in,
            Eigen::MatrixXd& P_in,
            Eigen::MatrixXd& F_in,
            Eigen::MatrixXd& H_in,
            Eigen::MatrixXd& R_in,
            Eigen::MatrixXd& Q_in);

  /**
   * Prediction Predicts the state and the state covariance using the process model
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations (for lidar)
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd& z);

  /**
   * Updates the state by using Extended Kalman Filter equations (for radar)
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd& z);

private:
  void CommonUpdateForLidarAndRadar(const Eigen::VectorXd& y);

};

#endif /* KALMAN_FILTER_H_ */
