#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
 
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i) {
      VectorXd res = estimations[i] - ground_truth[i];
      res = res.array() * res.array();
      rmse += res;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  /**
  TODO:
    * Calculate a Jacobian here.
  */

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = (px*px)+(py*py);
  float c2 = sqrt(c1);
  float c3 = vx*py - vy*px;
  float c4 = vy*px - vx*py;


  MatrixXd Hj_(3,4);
  
  Hj_ << (px/c2), (py/c2),0,0,
        (-py/c1),(px/c1),0,0,
        (py*c3)/(c1*c2),(px*c4)/(c1*c2),px/c2,py/c2;

  return Hj_;

}
