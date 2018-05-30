#include <iostream>

#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0,0,0,0;

  
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size() == 0)
	{
	    cerr << "vector estimation size is 0" << endl;
	    return rmse;
	}
	
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size())
	{
	    cerr << "vector sizes don't match" << endl;
	    return rmse;
	}
	VectorXd residual(4);
	VectorXd squared_res(4);
	vector<VectorXd> squared_res_cont;
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
 		residual = estimations[i] - ground_truth[i];
 		squared_res = residual.array() * residual.array();
 		squared_res_cont.push_back(squared_res);
	}

	//calculate the mean
  for(int i=0; i < squared_res_cont.size(); ++i)
      rmse += squared_res_cont[i];
  rmse /= estimations.size();
	//calculate the squared root
  rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
  
    //pre-compute a term to avoid repeated calculation
  float pythag = pow(px, 2) + pow(py, 2);
  
  if(fabs(pythag) < 0.0001)
  {
      cerr << "Divide by zero" << endl;
      pythag = 0.0001;
      
  }
	//compute the Jacobian matrix
  Hj(0, 0) = px/sqrt(pythag);
  Hj(0, 1) = py/sqrt(pythag);
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;
  Hj(1, 0) = -py/pythag;
  Hj(1, 1) = px/pythag;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;
  Hj(2, 0) = py*(vx*py - vy*px)/pow(pythag, 3.0/2);
  Hj(2, 1) = px*(vy*px - vx*py)/pow(pythag, 3.0/2);
  Hj(2, 2) = px/sqrt(pythag);
  Hj(2, 3) = py/sqrt(pythag);

	return Hj;
}
