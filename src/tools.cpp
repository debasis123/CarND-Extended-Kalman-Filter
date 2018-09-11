#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  eigen_assert(estimations.size() != 0 
    && "estimations vector size cannot be 0");
  eigen_assert(estimations.size() == ground_truth.size() 
    && "estimations and ground_truth size should be same");

  //accumulate squared residuals
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  for (uint16_t i=0; i != estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //check division by zero
  eigen_assert(!(px==0 && py==0));

  //compute the Jacobian matrix
  double px2py2 = px*px + py*py;
  double pxpy   = sqrt(px2py2);
  double px3py3 = px2py2 * pxpy;

  // in case of division by zero, set state to small float value
  if (fabs(px2py2) < 0.00001) {
    VectorXd x(4);
    x << 0.001, 0.01, 0.001, 0.001;
    return CalculateJacobian(x);
  }

  MatrixXd Hj(3,4);
  Hj << px/pxpy,                  py/pxpy,                   0,       0,
        -py/px2py2,               px/px2py2,                 0,       0,
        py*(vx*py-vy*px)/px3py3,  px*(vy*px-vx*py)/px3py3,   px/pxpy, py/pxpy;

  return Hj;
}
