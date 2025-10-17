#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Eigen>
#define _USE_MATH_DEFINES
#include "math.h"

namespace FarolUtils{
  double wrapTo2Pi(double theta){
    theta = fmod(theta, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;
    return theta ;
  }

  double wrapToPi(double theta){
    theta = fmod(theta + M_PI, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;
    theta = theta - M_PI;
    return theta ;
  }
}