/*
* @Original Author: Udacity
* @Last Modified by:   debasis123
*/

#include "kalman_filter.h"
#include <cmath>  // for atan2

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// NOTE: Eigen library does not initialize VectorXd or MatrixXd objects
// with zeros upon creation. So initialize accordingly.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  // predict the state
  x_          = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_          = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd& z) {
  // Because lidar measurement update function is a linear function h,
  // the update step will use the basic Kalman filter equations
  VectorXd z_pred = H_*x_;
  VectorXd y      = z-z_pred;

  CommonUpdateForLidarAndRadar(y);
}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations (for radar)
    * Radar measurement update function is a non-linear function h,
    * so the update step involves linearizing the H_j with the Jacobian
  */
  const double px = x_(0),
               py = x_(1),
               vx = x_(2),
               vy = x_(3);

  const double ZERO_DIV_LIMIT = 0.00001;
  const double rho = sqrt(px*px + py*py);
  const double phi = fabs(px) > ZERO_DIV_LIMIT ? atan2(py, px) : 0.0; // atan2 returns values between -pi and pi
  const double rho_dot = fabs(rho) > ZERO_DIV_LIMIT ? (px*vx + py*vy) / rho : 0.0;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z-z_pred;

  // correction for phi in y to be not in the range -pi to pi
  const double PI = 3.14159265;
  y(1) = y(1)>PI ? y(1)-2*PI
       : y(1)<-PI ? y(1)+2*PI
       : y(1);

  CommonUpdateForLidarAndRadar(y);
}

void KalmanFilter::CommonUpdateForLidarAndRadar(const VectorXd& y) {
  MatrixXd Ht     = H_.transpose();
  MatrixXd S      = H_*P_*Ht + R_;
  MatrixXd Si     = S.inverse();
  MatrixXd PHt    = P_*Ht;
  MatrixXd K      = PHt*Si;

  //new estimate
  x_          = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I  = MatrixXd::Identity(x_size, x_size);
  P_          = (I-K*H_) * P_;
}
