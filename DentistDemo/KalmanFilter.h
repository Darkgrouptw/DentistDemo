#pragma once
#include <math.h>

class KalmanFilter
{
public:
    KalmanFilter(double _q = 0.0, double _r = 0.0)
        : x(0.0), q(_q), r(_r), p(sqrt(_q*_q + _r*_r))
    {

    }

    double Update(double v)
    {
        p += q;
        k = p / (p + r);
        x += k * (v - x);
        p *= (1 - k);
        return x;
    }

private:
    double k;    // Kalman gain
    double p;    // Estimation error covariance
    double q;    // Process noise covariance
    double r;    // Measurement noise covariance
    double x;    // Result value
};