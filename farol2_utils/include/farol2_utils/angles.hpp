#pragma once
#include <cmath>

namespace farol2_utils {

  // High-precision pi constant (no <numbers>, no M_PI)
  template <typename T>
  inline constexpr T pi_v = static_cast<T>(3.141592653589793238462643383279502884L);

  template <typename T>
  inline constexpr T deg2rad(T deg) noexcept {
    return deg * pi_v<T> / static_cast<T>(180);
  }

  template <typename T>
  inline constexpr T rad2deg(T rad) noexcept {
    return rad * static_cast<T>(180) / pi_v<T>;
  }

  template <typename T>
  inline T wrapTo2Pi(T theta) {
    theta = std::fmod(theta, static_cast<T>(2) * pi_v<T>);
    if (theta < static_cast<T>(0))
      theta += static_cast<T>(2) * pi_v<T>;
    return theta;
  }

  template <typename T>
  inline T wrapToPi(T theta) {
    theta = std::fmod(theta + pi_v<T>, static_cast<T>(2) * pi_v<T>);
    if (theta < static_cast<T>(0))
      theta += static_cast<T>(2) * pi_v<T>;
    return theta - pi_v<T>;
  }

} // namespace farol2_utils
