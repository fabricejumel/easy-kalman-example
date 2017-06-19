//
// Created by misiu on 19-6-17.
//

#ifndef KALMAN_FILTER_EXAMPLES_MEASUREMENT_PACKAGE_H
#define KALMAN_FILTER_EXAMPLES_MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
public:
    long timestamp_;

    enum SensorType {
        LASER, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

#endif //KALMAN_FILTER_EXAMPLES_MEASUREMENT_PACKAGE_H
