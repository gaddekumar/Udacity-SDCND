#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */

UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/4.0;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off.
  */

  prevtimestamp_ = 0;

  P_ = 0.75*MatrixXd::Identity(5,5);

//  P_<<0.75,0,0,0,0,
//      0,0.75,0,0,0,
//      0,0,0.75,0,0,
//      0,0,0,0.75,0,
//      0,0,0,0,0.75;

  n_x_ = 5;
  n_z_ = 3;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  sqrt_value = sqrt(n_aug_ + lambda_);

  weights    = VectorXd(2*n_aug_ + 1);
  weights(0) = lambda_/(lambda_ + n_aug_);

  for (int i=1; i<2*n_aug_ + 1; i++){
    weights(i) = 0.5/(lambda_ + n_aug_);
  }

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+ 1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  cout<<" -- PROCESS MEASUREMENT -- "<<endl;
  if(!is_initialized_){

    if(meas_package.sensor_type_==MeasurementPackage::LASER){

      VectorXd values1 = meas_package.raw_measurements_;
      float px;
      float py;
      px = values1(0);
      py = values1(1);
      x_ << px,py,0,0,0;
    }

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){

      VectorXd values2 = meas_package.raw_measurements_;
      float rho;
      float phi;
      float phi_dot;

      rho = values2(0);
      phi = values2(1);
      phi_dot = values2(2);

      // Do the as usual angle 
      while(phi>M_PI){phi -= M_PI;}
      while(phi<M_PI){phi += M_PI;}

      x_ << rho*cos(phi), rho*sin(phi), 0,0,0;

    }


    prevtimestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t = meas_package.timestamp_ - prevtimestamp_;
  delta_t = delta_t/1000000.0;
  prevtimestamp_ = meas_package.timestamp_;

  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }

  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

void UKF::Prediction(double delta_t) {

  cout<< "-- PREDICTION METHOD --"<<endl;

  VectorXd X_aug = VectorXd( n_aug_ );

  for(int i=0; i < n_x_; i++){
    X_aug(i) = x_(i);
  }
  X_aug(5) = 0;
  X_aug(6) = 0;

  cout<<"X_aug vector is: "<<X_aug<<endl;
  
  MatrixXd P_aug_ = MatrixXd(7,7);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;

  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd Xsig = MatrixXd( n_aug_, 2*n_aug_ + 1);
  Xsig.col(0)   = X_aug;

  MatrixXd A = P_aug_.llt().matrixL();
  cout<< "A in prediction method is: "<<A<<endl;

  for(int i=0;i< n_aug_ ;i++){
    Xsig.col(i+1)          = X_aug + sqrt_value*A.col(i);
    Xsig.col(n_aug_ + i+1) = X_aug - sqrt_value*A.col(i);
  }

  cout<<"Xsig is: "<<Xsig<<endl;
  // It is correct till here.

  for (int i=0; i < 2*n_aug_+1; i++){
    VectorXd presentColumn = Xsig.col(i);
    const double p_x = presentColumn(0);
    const double p_y = presentColumn(1);
    const double v_k = presentColumn(2);
    const double phi = presentColumn(3);
    const double phi_dot = presentColumn(4);
    const double a_dot = presentColumn(5);
    const double yaw_dot = presentColumn(6);

    if(phi_dot==0){
      cout<<"phi dot is zero in this case"<<endl;
      Xsig_pred_(0,i)  = Xsig(0,i) + v_k*cos(phi)*(delta_t) + 0.5*delta_t*delta_t*cos(phi)*a_dot;
      Xsig_pred_(1,i)  = Xsig(1,i) + v_k*sin(phi)*(delta_t) + 0.5*delta_t*delta_t*sin(phi)*a_dot;
      Xsig_pred_(2,i)  = Xsig(2,i) + 0 + (delta_t)*a_dot;
      Xsig_pred_(3,i)  = Xsig(3,i) + phi_dot*delta_t + 0.5*delta_t*delta_t*yaw_dot;
      Xsig_pred_(4,i)  = Xsig(4,i) + 0 + (delta_t)*yaw_dot;
    }

    else{
      cout<<"Phi dot is NOT ZERO in this sample"<<endl;
      Xsig_pred_(0,i) = Xsig(0,i) + (v_k/phi_dot)*(sin(phi + phi_dot*delta_t)-sin(phi)) + 0.5*delta_t*delta_t*cos(phi)*a_dot;
      Xsig_pred_(1,i) = Xsig(1,i) + (v_k/phi_dot)*(-cos(phi+phi_dot*delta_t)+cos(phi))+0.5*delta_t*delta_t*sin(phi)*a_dot;
      Xsig_pred_(2,i) = Xsig(2,i) + 0 + delta_t*a_dot;
      Xsig_pred_(3,i) = Xsig(3,i) + phi_dot*delta_t + 0.5*delta_t*delta_t*yaw_dot;
      Xsig_pred_(4,i) = Xsig(4,i) + 0 + delta_t*yaw_dot;
    }
  }

  x_.fill(0.0);
  x_ = Xsig_pred_*weights;
  P_.fill(0.0);
  for ( int i=0; i < 2*n_aug_+1; i++){
    P_ += weights(i)*(Xsig_pred_.col(i) - x_)*((Xsig_pred_.col(i) - x_).transpose());
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the RADAR NIS.
  */ 

  cout<< " -------------------------> RADAR UPDATE --"<<endl;
  MatrixXd Z = MatrixXd(3, 2*n_aug_+1);

  for (int i=0; i<2*n_aug_+1; i++){
      VectorXd presentColumn = Xsig_pred_.col(i);
      const double p_x = presentColumn(0);
      const double p_y = presentColumn(1);
      const double v_k = presentColumn(2);
      const double phi = presentColumn(3);
      const double phi_dot = presentColumn(4);

      const double sum_square = sqrt(p_x*p_x + p_y*p_y);

      Z(0,i) = sum_square;
      Z(1,i) = atan2(p_y,p_x);

      if(sum_square<0.0001){
        Z(2,i) = (p_x*cos(phi)*v_k+p_y*sin(phi)*v_k)/0.0001;
      }
      else{
        Z(2,i) = (p_x*cos(phi)*v_k+p_y*sin(phi)*v_k)/sum_square; 
      }      
  }

  VectorXd z_mean = VectorXd(3);
  z_mean.fill(0.0);

  for(int i=0; i<2*n_aug_+1; i++){ 
    z_mean += weights(i)*Z.col(i);
  }
  
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
    S += weights(i)*(Z.col(i) - z_mean)*((Z.col(i) - z_mean).transpose());
  }

  S(0,0) += std_radr_*std_radr_;
  S(1,1) += std_radphi_*std_radphi_;
  S(2,2) += std_radrd_*std_radrd_;

  MatrixXd Tc = MatrixXd( n_x_, n_z_);
  Tc.fill(0.0);

  for (int i=0;i<2*n_aug_+1;i++){
      Tc += weights(i)*(Xsig_pred_.col(i) - x_)*(( Z.col(i) - z_mean).transpose());
  }

  VectorXd measurements = meas_package.raw_measurements_;
  cout << "measurements RADAR: " <<  measurements << endl;
  cout << "z_mean ----- RADAR: " << z_mean << endl;

  MatrixXd K = Tc*(S.inverse());
  x_ += K*(measurements - z_mean);

  cout<< "   x___________RADAR -->  "<<x_<<endl;
  P_ -= K*S*(K.transpose());

  cout<<"Returned from the radar update"<<endl;
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package  */

void UKF::UpdateLidar(MeasurementPackage meas_package) {

  /** TODO: Complete this function! Use Lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS  */  

  MatrixXd Z_lidar = MatrixXd(2, 2*n_aug_+1);

  Z_lidar.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
      VectorXd presentColumn = Xsig_pred_.col(i);
      const double p_x = presentColumn(0);
      const double p_y = presentColumn(1);

      Z_lidar(0,i) = p_x;
      Z_lidar(1,i) = p_y;

  }

  VectorXd z_mean = VectorXd(2);
  z_mean.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
    z_mean += weights(i)*Z_lidar.col(i);
  }
  
  MatrixXd S = MatrixXd(2,2);
  S.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
    S += weights(i)*(Z_lidar.col(i) - z_mean)*((Z_lidar.col(i) - z_mean).transpose());
  }

  S(0,0) += std_laspx_*std_laspx_;
  S(1,1) += std_laspy_*std_laspy_;

  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  // MatrixXd Xsig_pred_2 = Xsig_pred_.block(0,0,2,2*n_aug_+1);
  // MatrixXd x_2 = x_.block(0,0,2,1);

  for (int i=0; i<2*n_aug_+1; i++){
      Tc += weights(i)*( Xsig_pred_.col(i) - x_)*(( Z_lidar.col(i) - z_mean).transpose());
  }
  
  VectorXd measurements = meas_package.raw_measurements_;

  MatrixXd K = Tc*(S.inverse());
  x_ += K*(measurements - z_mean);
  P_ -= K*S*(K.transpose());

}
