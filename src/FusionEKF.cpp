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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  
  // Laser measurement matrix 
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  
  //State covariance matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  //Declare the Process noise Covariance matrix
  ekf_.Q_ = MatrixXd(4,4);
  
  // Initial state transition matrix F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
			       0, 1, 0, 1,
             0, 0, 1, 0,
			       0, 0, 0, 1;

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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double P_x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      double P_y =  (measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]));
      ekf_.x_ << P_x, P_y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // TODO: 
  // Fix the bug that lives between the "Set up Q and R print to the predict call!"
  /**
   TODO: 
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Set the process noice for x and y
  int noise_ax = 9;
  int noise_ay = 9;
   
  // Calculate the change in time.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  //Add the change in time to the state transition function 
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //Initialize the process noice covariance matrix Q;
  ekf_.Q_ << pow(dt,4)*noise_ax/4, 0, pow(dt,3)*noise_ax/2, 0,
	           0, pow(dt,4)*noise_ay/4, 0, pow(dt,3)*noise_ay/2,
	           pow(dt,3)*noise_ax/2, 0, pow(dt,2)*noise_ax, 0,
	           0, (pow(dt,3)/2)*noise_ay, 0, pow(dt,2)*noise_ay;
            
  ekf_.Predict();
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Set the measurement covariance matrix
    ekf_.R_ = R_radar_;
    
    // Set the measurement matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // Set the measurement covariance matrix
    ekf_.R_ = R_laser_;
    
    // Set the measurement matrix
    ekf_.H_ = H_laser_;
    
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  return;
}
