#pragma once
#include <cmath>

// #include <string>
// #include <vector>
// #include <eigen3/Eigen/Eigen>
// #define _USE_MATH_DEFINES
// #include "math.h"

namespace farol_utils {

  inline double wrapTo2Pi(double theta) {
    theta = std::fmod(theta, 2.0 * M_PI);
    if (theta < 0.0) theta += 2.0 * M_PI;
    return theta;
  }

  inline double wrapToPi(double theta) {
    theta = std::fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0.0) theta += 2.0 * M_PI;
    return theta - M_PI;
  }

} // namespace farol_utils

