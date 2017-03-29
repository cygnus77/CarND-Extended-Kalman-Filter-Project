#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
  const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size() || estimations.size() == 0) return rmse;

  //accumulate squared residuals
  VectorXd residual(4);
  residual << 0, 0, 0, 0;
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd d = estimations[i] - ground_truth[i];
    residual = residual.array() + (d.array() * d.array());
  }
  residual /= estimations.size();
  rmse = residual.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px2py2 = px*px + py*py;
  if (px2py2 == 0) {
    std::cout << "Div by zero" << std::endl;
  }
  else {
    float sqpxy = sqrt(px2py2);
    float tt = sqpxy * px2py2;
    //compute the Jacobian matrix
    Hj << px / sqpxy, py / sqpxy, 0, 0,
          -py / px2py2, px / px2py2, 0, 0,
          py*(vx*py - vy*px) / tt, px*(vy*px - vx*py) / tt, px / sqpxy, py / sqpxy;
  }
  return Hj;

}

VectorXd Tools::RadarMeasurementFunction(const VectorXd &x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float rhodot = (px*vx + py*vy) / rho;

  VectorXd result = VectorXd(3);
  result << rho, phi, rhodot;
  return result;
}

Eigen::VectorXd Tools::RadarToCartesian(const Eigen::VectorXd &z)
{
  float rho = z(0);
  float phi = z(1);
  float rhodot = z(2);

  float py = rho * sin(phi);
  float px = rho * cos(phi);
  float vx = rhodot * cos(phi);
  float vy = rhodot * sin(phi);
  VectorXd result(4);
  result << px, py, vx, vy;
  return result;
}