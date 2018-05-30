#ifndef CarND_Extended_Kalman_Filter_Project_MEASUREMENT_PACKAGE_H_
#define CarND_Extended_Kalman_Filter_Project_MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};

#endif /* CarND_Extended_Kalman_Filter_Project_MEASUREMENT_PACKAGE_H_ */
