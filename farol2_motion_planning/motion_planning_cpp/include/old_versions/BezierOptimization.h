#ifndef BEZIER_OPTIMIZATION_H
#define BEZIER_OPTIMIZATION_H

#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <numeric> // For std::binomial_coefficient (C++20)
#include <ros/ros.h>
#include <cstdlib>  // for rand()
#include <ctime>    // for time()
// Define a structure to hold position and angle (current or goal)
struct State
{
    double x;
    double y;
    double theta; // Angle (yaw) of the vehicle
};

// Structure to hold Optimization Bounds
struct Bounds
{
    casadi::DM lbx;
    casadi::DM ubx;
    casadi::DM g_lbx;
    casadi::DM g_ubx;
};

// Define the optimization problem class
class BezierOptimization
{
public:
    // Constructor: Initialize the number of vehicles, Bezier degree, and other parameters
    BezierOptimization(int NVehicles,
                       int BezierDegree,
                       const std::vector<int> &nSplit,
                       const std::vector<State> &current_states,
                       const std::vector<State> &goal_states,
                       const std::vector<double> &v0,
                       const std::vector<double> &vT,
                       const Eigen::Matrix<double, 3, Eigen::Dynamic> &circ_obs,
                       const Eigen::Matrix<double, 3, Eigen::Dynamic> &line_obs);

    
    /**
     * @brief  Runs optimization in a centralized setting
     */
    void runOptimization(bool first_iteration, Eigen::Tensor<double, 3> control_P_Guess, double Tf_Guess, int BezierDegree);

    /**
     * @brief  Runs optimization in a distributed setting
     */
    void runOptimization(bool first_iteration, bool goal_reached, bool first_section, Eigen::Tensor<double, 3> control_P_Guess, Eigen::Tensor<double, 3> control_P_Prev, double Tf_Guess, double Tf_prev, double Tf_min, int BezierDegree);
    
    // Getter for the optimal control points
    Eigen::Tensor<double, 3> getControlPoints() const;
    // Getter for the optimal Tf
    double getTf() const;
    
    /**
     * @brief  Defines cotrol points in terms of the optimization variables
     */
    void defineVariables();

    /**
     * @brief  Defines cotrol points in terms of the optimization variables for distributed case
     */
    void defineVariablesDist();

    /**
     * @brief  Defines constraints in terms of the optimization variables
     */
    void defineConstraints(bool first_iter);

    /**
     * @brief  Defines constraints in terms of the optimization variables Updated
     */
    void defineConstraintsNew();

    /**
     * @brief  Auxiliary method to rearrange constraints in more practical way
     */
    std::vector<casadi::SX> split_constraints(const casadi::SX& flat_constraints, int n_segments, const std::vector<int>& n_local_vals);

    /**
     * @brief  Solves Optimization problem for the centralized setting
     */
    void solveOptimization();

    /**
     * @brief  Solves Optimization problem for the distributed setting
     */
    void solveOptimizationDist();

    /**
     * @brief  Computes The bounds on optimization variables and constraints
     */
    Bounds computeBounds(bool first_iter);
    
    /**
     * @brief  Computes The bounds on optimization variables and constraints for the centralized setting
     */
    Bounds computeBoundsNew();

    /**
     * @brief  Computes The bounds on optimization variables and constraints for the distributed setting
     */
    Bounds computeBoundsDist();

    /**
     * @brief  Computes initial guesses of the Optimization variables in the centralized setting
     */
    casadi::DM computeGuesses();

    /**
     * @brief  Computes initial guesses of the Optimization variables in the distributed setting
     */
    casadi::DM computeGuessesDist();

    /**
     * @brief  Extracts optimal Control points from solution in centralized setting
     *
     * @param opt_sol Value returned by the solver where all optimization variables are sequential
     */
    Eigen::Tensor<double, 3> extractOptimalControlPoints(const casadi::DM &opt_sol);
    
    /**
     * @brief  Extracts optimal Control points from solution in distributed setting
     *
     * @param opt_sol Value returned by the solver where all optimization variables are sequential
     */
    Eigen::Tensor<double, 3> extractOptimalControlPointsDist(const casadi::DM &opt_sol);

    /**
     * @brief  Recursive function to compute linear velocity constraints
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param Tf Symbolic variable that represents Time of arrival
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_velocity_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit);

    /**
     * @brief  Recursive function to compute angular velocity constraints
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param Tf Symbolic variable that represents Time of arrival
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_ang_velocity_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit);

    /**
     * @brief  Recursive function to compute angular velocity constraints
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param Tf Symbolic variable that represents Time of arrival
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_acceleration_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit);

    /**
     * @brief  Recursive function to compute all dynamic constraints in one go
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param Tf Symbolic variable that represents Time of arrival
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_dynamic_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit);
    /**
     * @brief  Recursive function to compute interVehicle avoidance constraints
     *
     * @param P1 Symbolic variable that represents the control points of the first curve
     * @param P2 Symbolic variable that represents the control points of the second curve
     * @param radius radius of each vehicle
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_interV_constraints(const casadi::SX &P1, const casadi::SX &P2, double radius, int nsplit);
    
    /**
     * @brief  Recursive function to compute interVehicle avoidance constraints
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param obs obstacle coordinates
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_obstacle_constraints(const casadi::SX &P, Eigen::Vector3d obs, int nsplit);

    /**
     * @brief  Recursive function to compute interVehicle avoidance constraints
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param circ_obs circular obstacle coordinates
     * @param line_obs infinite line parameters
     * @param nsplit Number of pieces the curve gets divided into = 2^nsplit
     */
    casadi::SX adaptive_all_obstacle_constraints(
        const casadi::SX &P,
        const Eigen::Matrix<double, 3, Eigen::Dynamic> &circ_obs,
        const Eigen::Matrix<double, 3, Eigen::Dynamic> &line_obs,
        int nsplit);
    /**
     * @brief  Function to get the control points of the derivative of a Bézier curve
     *
     * @param P Symbolic variable that represents the control points of the curve
     */
    casadi::SX bezier_derivative_Casadi(const casadi::SX &P);

    /**
     * @brief  Function to get the control points of the degree elevation of a Bézier curve
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param degree How many degrees it gets elevated by
     */
    casadi::SX elevate_degree_Casadi(const casadi::SX &P, int degree);

    /**
     * @brief  Function to get the control points of the result on the multiplication of two Bézier curves
     *
     * @param P1 Symbolic variable that represents the control points of the first curve
     * @param P2 Symbolic variable that represents the control points of the second curve
     */
    casadi::SX multiply_Bezier_Casadi(const casadi::SX &P1, const casadi::SX &P2);

    /**
     * @brief  Function to get the control points of the result on the multiplication of two Bézier curves
     *
     * @param P Symbolic variable that represents the control points of the curve
     * @param u tau value at which it is split
     */
    std::pair<casadi::SX, casadi::SX> divide_bezier_Casadi(const casadi::SX &P, double u);

    /**
     * @brief  Function to perform De Casteljau's algorithm recursively
     *
     */
    casadi::SX deCasteljau_Casadi(double u, const casadi::SX &pts, int i, int j);

    Eigen::MatrixXd adaptive_ang_velocity_constraints(const Eigen::MatrixXd &P, double Tf, int nsplit);
    Eigen::MatrixXd bezier_derivative(const Eigen::MatrixXd &P);
    Eigen::MatrixXd elevate_degree(const Eigen::MatrixXd &P, int degree);
    Eigen::MatrixXd multiply_Bezier(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> divide_bezier(const Eigen::MatrixXd &P, double u);
    Eigen::VectorXd deCasteljau(double u, const Eigen::MatrixXd &pts, int i, int j);

private:
    // ROS handle for logging and parameters (optional)
    // ros::NodeHandle nh_;

    // Number of vehicles and degree of the Bézier curve
    int NVehicles_;
    int BezierDegree_;

    // Input states: current and goal positions and angles
    std::vector<int> nSplit_;
    std::vector<State> current_states_;
    std::vector<State> goal_states_;
    std::vector<double> v0_;
    std::vector<double> vT_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;

    bool first_iteration_;
    bool goal_reached_;
    bool first_section_;
    bool see_goal_;
    double Tf_Guess_;
    double Tf_Prev_;
    double max_distance_;
    double Tf_min_;
    Eigen::Tensor<double, 3> control_P_Guess_;
    Eigen::Tensor<double, 3> control_P_Prev_;
    

    // CasADi symbolic variables
    casadi::SX Tf_;                                  // Final time (optimization variable)
    casadi::SX l1_, l2_;                             // Control point distances
    casadi::SX P_intermediate_x_, P_intermediate_y_; // Intermediate control points
    std::vector<casadi::SX> controlPoints_;          // Store control points per vehicle

    // Constraints and optimization variables
    casadi::SX constr_;  // Constraints vector
    casadi::Function J_; // Objective function (final time to minimize)

    // Solver
    casadi::Function solver_;

    // Solution variables
    Eigen::Tensor<double, 3> optimal_ContP_;
    Eigen::Tensor<double, 3> opt_control_points_;

    // Time of arrival
    double Tf_opt_;

    // Constraint bounds as class-level constants
    static constexpr double VEL_MIN = 0.001;
    static constexpr double VEL_MAX = 0.3;
    static constexpr double ACC_MIN = -0.006;
    static constexpr double ACC_MAX = 0.006;//1000000000;
    static constexpr double ANG_VEL_MIN = -0.15;
    static constexpr double ANG_VEL_MAX = 0.15;
    static constexpr double ANG_ACC_MIN = -0.005;
    static constexpr double ANG_ACC_MAX = 0.005;
    static constexpr double OBS_MIN = 0.0;
    static constexpr double OBS_MAX = std::numeric_limits<double>::infinity();
    static constexpr double RADIUS = 2.0;
    static constexpr double ALPHA = 1.0;
    static constexpr double BETA = 5.0;
};

#endif // BEZIER_OPTIMIZATION_H
