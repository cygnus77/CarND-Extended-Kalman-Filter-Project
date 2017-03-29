#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(const Eigen::MatrixXd& R_lidar, const Eigen::MatrixXd& R_radar) : 
  x_(4), // estimate
  P_(4,4), // uncertainty covariance matrix
  H_(2,4), // measurement matrix - lidar
  F_(4,4), // state transition matrix
  Q_(4,4), // process covariance matrix
  R_lidar_(R_lidar), // 2x2 lidar measurement noise
  R_radar_(R_radar)   // 3x3 radar measurement noise
{
  //measurement matrix - for lidar
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  //the initial transition matrix F_
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
}


void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_lidar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  // Prevent div-by-zero
  if (x_(0) == 0) return;

  // y = z - h(x')
  VectorXd y = z - Tools::RadarMeasurementFunction(x_);
  while (y(1) < -M_PI)
    y(1) += M_PI;
  while (y(1) > M_PI)
    y(1) -= M_PI;

  MatrixXd Hj = Tools::CalculateJacobian(x_);
  MatrixXd Hjt = Hj.transpose();

  // S = Hj P HjT + R
  MatrixXd S = Hj * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();

  // K = P HjT Sinv
  MatrixXd K = P_ * Hjt * Si;

  // x = x + Ky
  x_ = x_ + (K * y);

  // P = (I - K Hj)P
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
