#ifndef SINGLE_VEHICLE_MOTION_PLAN_H
#define SINGLE_VEHICLE_MOTION_PLAN_H

#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <numeric>
#include <cstdlib>
#include <ctime> 
#include "algorithms/bezier_utils.hpp"
#include <stdexcept>
#include <atomic>

// Define a structure to hold position and angle (current or goal)
struct vehicle_state
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

class CancelCallback : public casadi::Callback {
public:
    std::atomic<bool>* cancel_flag_;
    std::vector<casadi::Sparsity> sparsities_;

    CancelCallback(const std::string& name,
                   std::atomic<bool>* flag,
                   const std::vector<casadi::Sparsity>& sparsities)
        : cancel_flag_(flag), sparsities_(sparsities)
    {
        construct(name);
    }

    casadi_int get_n_in() override { return sparsities_.size(); }
    casadi_int get_n_out() override { return 1; }

    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        return sparsities_[i];   // EXACT MATCH
    }

    casadi::Sparsity get_sparsity_out(casadi_int) override {
        return casadi::Sparsity::dense(1,1);
    }

    std::vector<casadi::DM> eval(const std::vector<casadi::DM>&) const override {
        if (cancel_flag_ && cancel_flag_->load()) {
            throw std::runtime_error("User requested cancellation");
        }
        return {casadi::DM(0)};
    }
};


// Define the optimization problem class
class MultipleVehiclePlanner
{
public:
    // Constructor: Initialize the number of vehicles, Bezier degree, and other parameters
    MultipleVehiclePlanner(int BezierDegree, int NVehicles, const std::vector<int> &nSplit,const std::vector<uint8_t> &constr_flag);

    ~MultipleVehiclePlanner();  // destructor
    
    void setBoundsAndGains(double vel_min, double vel_max,
                         double acc_min, double acc_max,
                         double ang_vel_min, double ang_vel_max,
                         double ang_acc_min, double ang_acc_max,
                         double obs_min, double obs_max,
                         double radius, double alpha, double beta, double gamma);

    void setupSymbolic();

    void computeCostFunction();

    void setOptimizationProblem(const std::vector<vehicle_state>& current_states, const std::vector<vehicle_state>& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs,
                            const Eigen::Tensor<double, 3>& ContP_guess, double Tf_guess, bool first_iter);
    void setOptimizationProblem(const std::vector<vehicle_state>& current_states, const std::vector<vehicle_state>& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs);
    void setOptimizationProblem(const std::vector<vehicle_state>& current_states, const std::vector<vehicle_state>& goal_states);


    void createConstraintVector();

    void createDecisionVector();

    void createConstraintVector(const std::vector<int>& active_vehicles);

    void createDecisionVector(const std::vector<int>& active_vehicles, double Tf_min);

    void solveOptimizationProblem(std::atomic<bool>* cancel_flag);

    Eigen::Tensor<double, 3>  getControlPoints() const;

    Eigen::Tensor<double, 3>  getOptimalTrajectories(int numPoints) const;

    std::string getOptimizationstatus() const;

    double getTf() const;

private:
    int BezierDegree_;
    int NVehicles_;
    std::vector<vehicle_state> current_states_;
    std::vector<vehicle_state> goal_states_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;

    Eigen::Tensor<double, 3> ContP_guess_;
    double Tf_guess_;

    std::vector<int> nSplit_;
    std::vector<uint8_t> constr_flag_;
    bool first_iter_;
    
    // --- NLP entries ---
    Bounds problem_Bounds_;
    casadi::SX constraints_;
    casadi::SX decision_vec_;
    casadi::SX cost_func_;
    casadi::DM initial_guess_;

    // --- CasADi symbolic variables ---
    casadi::SX sym_Tf_;                           // Final time (optimization variable)
    std::vector<casadi::SX> sym_P_all_;           // All control points N x (2 x (BezierDegree+1))
    casadi::SX sym_circ_obs_;                     // Circular obstacle parameters (3 x 1)
    casadi::SX sym_line_obs_;                     // Linear obstacle parameters (3 x 1)
    std::vector<casadi::SX> sym_vel_constr_;      // Velocity constraints
    std::vector<casadi::SX> sym_ang_vel_constr_;  // Angular velocity constraints
    std::vector<casadi::SX> sym_acc_constr_;      // Linear acceleration constraints
    std::vector<casadi::SX> sym_ang_acc_constr_;  // Angular acceleration constraints
    std::vector<casadi::SX> sym_circ_constr_;     // Circular obstacle avoidance constraints
    std::vector<casadi::SX> sym_line_constr_;     // Line obstacle avoidance constraints
    casadi::SX sym_inter_vehicle_constr_;         // Inter-Vehicle constraints
    casadi::SX sym_cost_;                         // Initial Fully symbolic cost

    // Solution variables
    Eigen::Tensor<double, 3> optimal_ContP_;

    // Time of arrival
    double Tf_opt_;

    std::string return_status_;

    // Constraint bounds as class-level constants
    double VEL_MIN = 0.001;
    double VEL_MAX = 0.3;
    double ACC_MIN = -0.03;
    double ACC_MAX = 0.03;
    double ANG_VEL_MIN = -0.15;
    double ANG_VEL_MAX = 0.15;
    double ANG_ACC_MIN = -0.005;
    double ANG_ACC_MAX = 0.005;
    double OBS_MIN = 2.0;
    double OBS_MAX = std::numeric_limits<double>::infinity();
    double RADIUS = 1.5;
    double ALPHA = 1.0; // TF weight
    double BETA = 5.0;  // Energy weight
    double GAMMA = 1.0; // end point weight
};

#endif // SINGLE_VEHICLE_MOTION_PLAN_H
