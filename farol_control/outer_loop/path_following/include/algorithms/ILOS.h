#pragma once
#include "PathFollowing.h"

/**
 * @brief Path following using the Integral Line-of-Sight (ILOS) algorithm
 *
 * This algorithm supports:
 *    Controls:
 *      - yaw
 *      - surge
 *    Supports Cooperative Path Following - True
 *    Contains Currents Observers - False
 *
 * The ILOS algorithm augments the classical LOS guidance law with an integral
 * action on the cross-track error to achieve offset-free path following in the
 * presence of constant environmental disturbances (e.g. ocean currents).
 *
 * Reference: Caharija et al., "Integral LOS Path Following for Curved Paths
 * Based on a Monotone Cubic Hermite Spline Parametrization", IEEE Trans. on
 * Control Systems Technology, 2016.
 *
 * Heading reference:
 *   yaw_d = psi_path + atan2(-(y_e + ki * sigma), delta)
 * with sigma dynamics:
 *   sigma_dot = y_e * delta / sqrt(delta^2 + (y_e + ki * sigma)^2)
 *
 * Gains:
 *   delta - lookahead distance (> 0)
 *   ki    - integral gain (>= 0)
 *
 * @author    [Your Name]
 * @version   1.0a
 * @date      2026
 * @copyright MIT
 */
class ILOS : public PathFollowing {

  public:
    /**
     * @brief Constructor
     *
     * @param delta       Lookahead distance (metres)
     * @param ki          Integral gain for cross-track error
     * @param surge_pub   ROS surge reference publisher
     * @param yaw_pub     ROS yaw reference publisher
     * @param gamma_pub   ROS gamma publisher (for cooperative PF)
     * @param mode_client ROS service client to switch path to closest-point mode
     */
    ILOS(double delta,
         double ki,
         rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub,
         rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub,
         rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_pub,
         rclcpp::Client<paths::srv::SetMode>::SharedPtr mode_client);

    /**
     * @brief Set / update controller gains at runtime
     *
     * @param gains  Vector of 2 gains: [delta, ki]
     * @return true  on success, false if wrong number of gains
     */
    bool setPFGains(std::vector<double> gains) override;

    /**
     * @brief Compute the ILOS heading and surge references
     *
     * @param dt  Time elapsed since the previous call (seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief Publish surge, yaw and gamma references
     */
    void publish_private() override;

    /**
     * @brief First-iteration setup — requests closest-point path mode
     */
    void start() override;

    /**
     * @brief Returns true when gamma has reached gamma_max
     */
    bool stop() override;

    /**
     * @brief Reset all internal state variables
     */
    bool reset() override;

  private:

    /* Controller gains */
    double delta_{10.0};   ///< Lookahead distance
    double sigma_{0.0};       ///< Integral gain

    /* Integral state */
    double y_int_{0.0};    ///< Integral of cross-track error (ILOS state)

    /* Desired references */
    double desired_surge_{0.0};
    double desired_yaw_{0.0};

    /* ROS publishers */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_pub_;

    /* ROS service client to enable closest-point path mode */
    rclcpp::Client<paths::srv::SetMode>::SharedPtr mode_client_;
};
