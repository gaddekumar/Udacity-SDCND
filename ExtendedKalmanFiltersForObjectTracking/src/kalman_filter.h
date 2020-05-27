#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"


class KalmanFilter {

public:

  // These all are public variables and can be accessed in all the methods in the class. 
  // state vector
  Eigen::VectorXd x_;
  
  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;
  
  // process covariance matrix
  Eigen::MatrixXd Q_;
  
  // measurement matrix
  Eigen::MatrixXd H_laser_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

  
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in,  Eigen::MatrixXd &Q_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */

  Tools tools;

  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
