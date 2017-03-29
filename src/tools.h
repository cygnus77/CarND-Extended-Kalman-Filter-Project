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

  /**
  * A helper method to map cartesian state (px,py,vx,vy) to radar's polar space (rho, phi, rhodot)
  */
  static Eigen::VectorXd RadarMeasurementFunction(const Eigen::VectorXd &x_state);

  /**
  * A helper function to map radar's polar space (rho, phi, rhodot) back to cartesian space (px,py,vx,vy)
  */
  static Eigen::VectorXd RadarToCartesian(const Eigen::VectorXd &z);

};

#endif /* TOOLS_H_ */
