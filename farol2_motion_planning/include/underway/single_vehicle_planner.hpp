#ifndef SINGLE_VEHICLE_MOTION_PLAN_H
#define SINGLE_VEHICLE_MOTION_PLAN_H

#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <numeric>
#include <ros/ros.h>
#include <cstdlib>
#include <ctime> 
#include "BezierUtils.h"
#include <stdexcept>

// Define a structure to hold position and angle (current or goal)
struct State
{
    double x;
    double y;
    double theta;
    double v;
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
class SingleVehicleMotionPlan
{
public:
    // Constructor: Initialize the number of vehicles, Bezier degree, and other parameters
    SingleVehicleMotionPlan(int BezierDegree, const std::vector<int> &nSplit,const std::vector<bool> &constr_flag);

    void setBoundsAndGains(double vel_min, double vel_max,
                         double acc_min, double acc_max,
                         double ang_vel_min, double ang_vel_max,
                         double ang_acc_min, double ang_acc_max,
                         double obs_min, double obs_max,
                         double radius, double alpha, double beta, double gamma, double delta);

    void setupSymbolic();

    void setOptimizationProblem(const State& current_states, const State& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& neighbor_lines,
                            bool first_section, bool first_iter);

    void createConstraintVector();
    void createConstraintVectorGoalSeen();

    void createDecisionVector(Eigen::Matrix<double, 2, Eigen::Dynamic>& ContP_guess, double Tf_guess);
    void createDecisionVectorGoalSeen(Eigen::Matrix<double, 2, Eigen::Dynamic>& ContP_guess, double Tf_guess);

    void computeCostFunction();
    void computeCostFunctionGoalSeen();

    void solveOptimizationProblem();

    Eigen::Matrix<double, 2, Eigen::Dynamic> getControlPoints() const;

    double getTf() const;

private:
    int BezierDegree_;
    State current_states_;
    State goal_states_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> neighbor_lines_;
    Eigen::Matrix<double, 2, Eigen::Dynamic> ContP_guess_;
    double Tf_guess_;

    // --- For warm-starting ---
    Eigen::Matrix<double, 2, Eigen::Dynamic> scaled_prev_P_; // control points of the previous solution scaled by their corresponding time and degree


    std::vector<int> nSplit_;
    std::vector<bool> constr_flag_;
    bool first_iter_;
    bool first_section_;
    
    // --- NLP entries ---
    Bounds problem_Bounds_;
    casadi::SX constraints_;
    casadi::SX decision_vec_;
    casadi::SX cost_func_;
    casadi::DM initial_guess_;

    // --- CasADi symbolic variables ---
    casadi::SX sym_Tf_;                           // Final time (optimization variable)
    casadi::SX sym_P_all_;                        // All control points (2 x (BezierDegree+1))
    casadi::SX sym_circ_obs_;                     // Circular obstacle parameters (3 x 1)
    casadi::SX sym_line_obs_;                     // Linear obstacle parameters (3 x 1)
    casadi::SX sym_neigh_coefs_;                  // Neighbor coefficients (3 x 1)
    casadi::SX sym_vel_constr_;                   // Velocity constraints
    casadi::SX sym_ang_vel_constr_;               // Angular velocity constraints
    casadi::SX sym_acc_constr_;                   // Linear acceleration constraints
    casadi::SX sym_ang_acc_constr_;               // Angular acceleration constraints
    casadi::SX sym_circ_constr_;                  // Circular obstacle avoidance constraints
    casadi::SX sym_line_constr_;                  // Line obstacle avoidance constraints
    casadi::SX sym_neigh_constr_;                 // Neighbor-related constraints
    casadi::SX sym_cost_;                         // Initial Fully symbolic cost

    // Solution variables
    Eigen::Matrix<double, 2, Eigen::Dynamic> optimal_ContP_;

    // Time of arrival
    double Tf_opt_;

    // Constraint bounds as class-level constants
    double VEL_MIN = 0.001;
    double VEL_MAX = 0.3;
    double ACC_MIN = -0.03;
    double ACC_MAX = 0.03;//1000000000;
    double ANG_VEL_MIN = -0.15;
    double ANG_VEL_MAX = 0.15;
    double ANG_ACC_MIN = -0.005;
    double ANG_ACC_MAX = 0.005;
    double OBS_MIN = 0.0;
    double OBS_MAX = std::numeric_limits<double>::infinity();
    double RADIUS = 1.5;
    double ALPHA = 1.0; // TF weight
    double BETA = 5.0;  // Energy weight
    double GAMMA = 1.0; // end point weight
    double DELTA = 1.0; // distance to goal weight
};

#endif // SINGLE_VEHICLE_MOTION_PLAN_H
