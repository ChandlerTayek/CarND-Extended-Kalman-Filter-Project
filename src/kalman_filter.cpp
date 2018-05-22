#include "kalman_filter.h"
#include "tools.h"
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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  print_all_variables();
  VectorXd z_pred = H_ * x_;
  std::cout << "*****************" << std::endl;
  std::cout << z << std::endl;
  std::cout << " --------------- " << std::endl; 
  std::cout << z_pred << std::endl;
  VectorXd y = z - z_pred;
  std::cout << "2!" << std::endl;
  MatrixXd Ht = H_.transpose();
  std::cout << "3" << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  std::cout << "Finish!" << std::endl;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd z_pred = VectorXd(3);
  
  double almost_zero = 0.000001;
  
  // Check there is no division by 0.
  std::cout << "HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "---------" << std::endl;
  std::cout << x_[0] << std::endl;
  if (x_[0] == 0)
  {
    // Compute the arctan of Py/Px.
    z_pred[1] = atan2(x_[1], almost_zero);
  }
  else if (sqrt(pow(x_[0],2) + pow(x_[1],2)) == 0)
  {
    z_pred[2] = (x_[0]*x_[2] + x_[1]*x_[3])/
                (almost_zero);
  }
  else
  {
    z_pred[1] = atan2(x_[1],x_[0]);
    z_pred[2] = (x_[0]*x_[2] + x_[1]*x_[3])/
                (sqrt(pow(x_[0],2) + pow(x_[1],2)));
  }
  z_pred[0] = sqrt(pow(x_[0],2) + pow(x_[1],2));
  

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
  MatrixXd Hj =  tools.CalculateJacobian(z);
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj* P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
