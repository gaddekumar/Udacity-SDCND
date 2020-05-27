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

FusionEKF::FusionEKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  MatrixXd R_laser_ = MatrixXd(2, 2);
  MatrixXd R_radar_ = MatrixXd(3, 3);
  MatrixXd H_laser_ = MatrixXd(2,4);

  R_laser_ << 0.0225, 0,
            0, 0.0225;
  
  R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.H_laser_ = H_laser_;

  ekf_.R_laser_ = MatrixXd(2,2);
  ekf_.R_laser_ = R_laser_;

  ekf_.R_radar_ = MatrixXd(3,3);
  ekf_.R_radar_ = R_radar_;
  
}   

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    
    ekf_.x_ = VectorXd(4);
    ekf_.x_<< 1,1,1,1;
  
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd values_r = measurement_pack.raw_measurements_;

      // double rho = measurement_pack.raw_measurements_[0];
      // double phi = measurement_pack.raw_measurements_[1];
      // double ro_dot = measurement_pack.raw_measurements_[2];

      float px = values_r[0]*cos(values_r[1]);
      float py = values_r[0]*sin(values_r[1]);
      float vx = values_r[2]*cos(values_r[1]);
      float vy = values_r[2]*sin(values_r[1]);

      ekf_.x_ << px, py, vx, vy;

      if(fabs(px) < 0.0001){
          px = 0.0001;
      }
      if(fabs(py) < 0.0001){
          py = 0.0001;
       }
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      VectorXd values_l = measurement_pack.raw_measurements_;
      ekf_.x_ <<  values_l[0], values_l[1], 0, 0;

    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;

  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix Q.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  double dt = measurement_pack.timestamp_ - previous_timestamp_;
  dt /= 1000000.0 ;
  previous_timestamp_ = measurement_pack.timestamp_;


  // The below two steps updates the F and the Q matrices. 
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,dt,0,
              0,1,0,dt,
              0,0,1,0,
              0,0,0,1;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << pow(dt,4)*pow(noise_ax,2)/4,0,pow(dt,3)*pow(noise_ax,2)/2,0,
              0,pow(dt,4)*pow(noise_ay,2)/4,0,pow(dt,3)*pow(noise_ay,2)/2,
              pow(dt,3)*pow(noise_ax,2)/2,0,pow(dt,2)*pow(noise_ax,2),0,
              0,pow(dt,3)*pow(noise_ay,2)/2,0,pow(dt,2)*pow(noise_ay,2);
           
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /** First we predict the next position and then we do the measurement update. 
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
