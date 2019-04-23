/*
* @Original Author: Udacity
* @Last Modified by: debasis123
*/

#ifndef TOOLS_H_
#define TOOLS_H_

#include "Eigen/Dense"
#include <vector>

/**
 * Utility class to calculate RMSE values and the Jacobian matrix
 */
struct Tools {
  /**
  * A helper method to calculate RMSE.
  */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                       const std::vector<Eigen::VectorXd>& ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};
#endif /* TOOLS_H_ */
