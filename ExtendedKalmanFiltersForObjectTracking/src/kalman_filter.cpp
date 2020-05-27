#include <iostream>
#include <cmath>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;  
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Q_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in) {

  
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_in;
  Q_ = Q_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;

}

void KalmanFilter::Predict() {
  /** TODO:
    * predict the state
  */
  // Mean is equal to zero.
  x_ = F_*x_;
  P_ = F_*P_*(F_.transpose()) + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_laser_*x_;
  MatrixXd S = H_laser_*P_*(H_laser_.transpose()) + R_laser_;

  MatrixXd K = P_*(H_laser_.transpose())*(S.inverse());
  x_ = x_ + (K*y);

  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = ( I - K * H_laser_ )*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  MatrixXd Hj_(3,4);

  Hj_ = tools.CalculateJacobian(x_);
  
  double rho = sqrt(px * px + py * py);
  if (fabs(rho) < 0.00001) {
    rho = 0.00001;
  }

  // atan2 returns values in between -pi and pi.
  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;

  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New Estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;

}
