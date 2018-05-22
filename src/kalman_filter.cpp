#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::print_all_variables() {
    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "x:" << std::endl << x_ << std::endl;
    std::cout << "P:" << std::endl << P_ << std::endl;
    std::cout << "F:" << std::endl << F_ << std::endl;
    std::cout << "Q:" << std::endl << Q_ << std::endl;
    std::cout << "H:" << std::endl << H_ << std::endl;
    std::cout << "R:" << std::endl << R_ << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
    return;
  }

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Predict the new state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  // update the state by using Kalman Filter equations
  //print_all_variables();
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
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
  // update the state by using Extended Kalman Filter equations
  
  VectorXd z_pred = VectorXd(3);
  
  /* Transfer the cartsian
  *  predicted state to polar, also denoted h(x').
  */ 
  //TODO: Clearn up how the h(x') function uses varibles and flows.
  //      Alos test to make sure it is handling the 0 cases correctly
  /**
  * Range
  * - you can always calculate rho without fear of undefined behavior
  */
  rho = sqrt(pow(x_[0],2) + pow(x_[1],2)
  z_pred[0] = rho;
  
  // Numerator of rho_dot
  rho_dot_numerator = x_[0]*x_[2] + x_[1]*x_[3];
  // Check for division by 0
  if (x_[0] == 0)
  {
    // Compute the arctan of Py/Px.
    z_pred[1] = atan2(x_[1], almost_zero);
  }
  else if (rho == 0)
  {
    // Radial Velocity with correction of 0 in the denominator
    z_pred[2] = (rho_dot_numerator)/
                (almost_zero);
  }
  else
  {
    // Bearing
    phi = atan2(x_[1],x_[0]);
    
    // Radial Velocity
    rho_dot = (rho_dot_numerator)/
                (rho);
                
    z_pred[1] = phi;
    z_pred[2] = rho_dot;
  }
  

  VectorXd y = z - z_pred;
  // Adjust the pi angle to be between -pi and pi
  while (y[1] < -pi || y[1] > pi)
  {
    if (y[1] > pi)
    {
      y[1] -= 2*pi;
    }
    else
    {
      y[1] += 2*pi;
    }
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_* P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
