#ifndef KALMAN_FILTER_EXAMPLES_TRACKING_H
#define KALMAN_FILTER_EXAMPLES_TRACKING_H

#include <fstream>
#include <string>
#include <vector>

#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    long previous_timestamp_;

    // I guess only this example specific
    float noise_ax;
    float noise_ay;
};

#endif //KALMAN_FILTER_EXAMPLES_TRACKING_H
