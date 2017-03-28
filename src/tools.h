#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:

  /**
  * A helper method to calculate RMSE.
  */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  static Eigen::VectorXd Tools::RadarMeasurementFunction(const Eigen::VectorXd &x_state);

  static Eigen::VectorXd Tools::RadarToCartesian(const Eigen::VectorXd &z);

};

#endif /* TOOLS_H_ */
