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
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  
  // Object is created here of the class KalmanFilter
  KalmanFilter ekf_;
  

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;
  long long dt;
  // tool object used to compute Jacobian and RMSE
  // Jacobian and the RMSE are calculated and then given to the simulator to test it. 
  Tools tools;

  // These variables are private and are stored in the FusionEKF.cpp. So, I think these variables cannot be - 
  // - accessed in the file kallam_filter.cpp. 
  
  // Eigen::MatrixXd H_laser_;
  // Eigen::MatrixXd Hj_;

};

#endif /* FusionEKF_H_ */
