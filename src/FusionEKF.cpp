#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() : 
  ekf_( 
        //measurement covariance matrix - laser
        (MatrixXd(2,2) << 0.0225, 0, 
                          0, 0.0225).finished(), 

        //measurement covariance matrix - radar
        (MatrixXd(3,3) << 0.09, 0, 0, 
                          0, 0.0009, 0, 
                          0, 0, 0.09).finished()),
  is_initialized_(false),
  previous_timestamp_(0)
{
}

Eigen::VectorXd FusionEKF::getEstimate() const
{
  return ekf_.x_;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
  
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ = Tools::RadarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Initialize uncertainty covariance matrix
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
  * Update the state transition matrix F according to the new elapsed time.
    - Time is measured in seconds.
  * Update the process noise covariance matrix.
  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */
    //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // update dt in state transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // precalc pow2, pow3/2 and pow4/4
  double dt2 = dt * dt;
  double dt3 = (dt2 * dt);
  double dt4 = (dt3 * dt);
  dt3 /= 2;
  dt4 /= 4;

  // update process covariance matrix
  ekf_.Q_ << dt4 * noise_ax, 0, dt3 * noise_ax, 0,
            0, dt4 * noise_ay, 0, dt3 * noise_ay,
            dt3 * noise_ax, 0, dt2 * noise_ax, 0,
            0, dt3 * noise_ay, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
  * Use the sensor type to perform the update step.
  * Update the state and covariance matrices.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
