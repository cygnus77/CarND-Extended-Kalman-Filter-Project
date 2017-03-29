#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Getter for estimate
  */
  Eigen::VectorXd getEstimate() const;

private:
  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  const int noise_ax = 9;
  const int noise_ay = 9;
};

#endif /* FusionEKF_H_ */
