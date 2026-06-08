#include "SingleVehicleMotionPlan.h"

// Constructor for the SingleVehicleMotionPlan class
SingleVehicleMotionPlan::SingleVehicleMotionPlan(int BezierDegree, const std::vector<int> &nSplit, const std::vector<bool> &constr_flag)
    : BezierDegree_(BezierDegree), nSplit_(nSplit), constr_flag_(constr_flag) 
{
    // Validate input parameters
    if (BezierDegree_ <= 0) {
        throw std::invalid_argument("BezierDegree must be positive");
    }
    if (nSplit_.size() < 2) {
        throw std::invalid_argument("nSplit must have at least 2 elements");
    }
    if (constr_flag_.size() < 4) {
        throw std::invalid_argument("constr_flag must have at least 4 elements");
    }
}


void SingleVehicleMotionPlan::setBoundsAndGains(double vel_min, double vel_max,
                         double acc_min, double acc_max,
                         double ang_vel_min, double ang_vel_max,
                         double ang_acc_min, double ang_acc_max,
                         double obs_min, double obs_max,
                         double radius, double alpha, double beta, double gamma, double delta)
{
    // Validate constraint bounds
    if (vel_min >= vel_max) throw std::invalid_argument("vel_min must be less than vel_max");
    if (acc_min >= acc_max) throw std::invalid_argument("acc_min must be less than acc_max");
    if (ang_vel_min >= ang_vel_max) throw std::invalid_argument("ang_vel_min must be less than ang_vel_max");
    if (ang_acc_min >= ang_acc_max) throw std::invalid_argument("ang_acc_min must be less than ang_acc_max");
    
    VEL_MIN = vel_min;
    VEL_MAX = vel_max;
    ACC_MIN = acc_min;
    ACC_MAX = acc_max;
    ANG_VEL_MIN = ang_vel_min;
    ANG_VEL_MAX = ang_vel_max;
    ANG_ACC_MIN = ang_acc_min;
    ANG_ACC_MAX = ang_acc_max;
    OBS_MIN = obs_min;
    OBS_MAX = obs_max;
    RADIUS = radius;
    ALPHA = alpha;
    BETA = beta;
    GAMMA = gamma;
    DELTA = delta;
}

void SingleVehicleMotionPlan::setupSymbolic()
{
    using namespace casadi;

    sym_Tf_ = SX::sym("Tf");
    sym_P_all_ = SX::sym("P_all", 2, BezierDegree_ + 1);
    
    sym_circ_obs_ = SX::sym("circ_obs", 3, 1);
    sym_line_obs_ = SX::sym("line_obs", 3, 1);
    sym_neigh_coefs_ = SX::sym("neigh_coefs", 3, 1);

    sym_circ_constr_ = BezierUtils::adaptive_all_obstacle_constraints(sym_P_all_, sym_circ_obs_, SX::sym("empty_circ", 3, 0), nSplit_[1]);
    sym_line_constr_ = BezierUtils::adaptive_all_obstacle_constraints(sym_P_all_, SX::sym("empty_circ", 3, 0), sym_line_obs_, nSplit_[1]);
    sym_neigh_constr_ = BezierUtils::adaptive_all_obstacle_constraints(sym_P_all_, SX::sym("empty_circ", 3, 0), sym_neigh_coefs_, nSplit_[1]);

    if (constr_flag_[0] == true)
    {
        sym_vel_constr_ = BezierUtils::adaptive_dynamic_constraints(sym_P_all_, sym_Tf_, nSplit_[0], {true, false, false, false});
    }
    if (constr_flag_[1] == true)
    {
        sym_ang_vel_constr_ = BezierUtils::adaptive_dynamic_constraints(sym_P_all_, sym_Tf_, nSplit_[0], {false, true, false, false});
    }
    if (constr_flag_[2] == true)
    {
        sym_acc_constr_ = BezierUtils::adaptive_dynamic_constraints(sym_P_all_, sym_Tf_, nSplit_[0], {false, false, true, false});
    }
    if (constr_flag_[3] == true)
    {               
        sym_ang_acc_constr_ = BezierUtils::adaptive_dynamic_constraints(sym_P_all_, sym_Tf_, nSplit_[0], {false, false, false, true});
    }
}

void SingleVehicleMotionPlan::setOptimizationProblem(const State& current_states, const State& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& neighbor_lines,
                            bool first_section, bool first_iter)
{
    // Store states
    current_states_ = current_states;
    goal_states_    = goal_states;
    circ_obs_ = circ_obs;
    line_obs_ = line_obs;
    neighbor_lines_ = neighbor_lines;
    first_iter_  = first_iter;
    first_section_ = first_section;

    if (circ_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: circ_obs_ must be 3×N.\n";
    }

    if (line_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: line_obs_ must be 3×N.\n";
    }

    if (Tf_guess_ <= 0) {
        std::cerr << "[setOptimizationProblem] WARNING: Tf_guess_ <= 0, using default 10s.\n";
        Tf_guess_ = 10.0;
    }

    if (nSplit_.empty()) {
        std::cerr << "[setOptimizationProblem] WARNING: nSplit_ is empty.\n";
    }
}

void SingleVehicleMotionPlan::createConstraintVectorGoalSeen()
{
    using namespace casadi;

    // --- Substitute symbolic variables with actual values for the current and goal states in the constraints and cost function ---
    std::vector<SX> vars_to_substitute;
    std::vector<SX> values_to_substitute;

    vars_to_substitute.push_back(sym_P_all_(0,0));
    vars_to_substitute.push_back(sym_P_all_(1,0));
    vars_to_substitute.push_back(sym_P_all_(0,BezierDegree_));
    vars_to_substitute.push_back(sym_P_all_(1,BezierDegree_));
    values_to_substitute.push_back(SX(current_states_.x));
    values_to_substitute.push_back(SX(current_states_.y));
    values_to_substitute.push_back(SX(goal_states_.x));
    values_to_substitute.push_back(SX(goal_states_.y));

    SX P1x = current_states_.x + current_states_.v * cos(current_states_.theta) * sym_Tf_ / BezierDegree_;
    SX P1y = current_states_.y + current_states_.v * sin(current_states_.theta) * sym_Tf_ / BezierDegree_;
    SX PNx = goal_states_.x - goal_states_.v * cos(goal_states_.theta) * sym_Tf_ / BezierDegree_;
    SX PNy = goal_states_.y - goal_states_.v * sin(goal_states_.theta) * sym_Tf_ / BezierDegree_;

    vars_to_substitute.push_back(sym_P_all_(0,1));
    vars_to_substitute.push_back(sym_P_all_(1,1));
    vars_to_substitute.push_back(sym_P_all_(0,BezierDegree_-1));
    vars_to_substitute.push_back(sym_P_all_(1,BezierDegree_-1));
    values_to_substitute.push_back(P1x);
    values_to_substitute.push_back(P1y);
    values_to_substitute.push_back(PNx);
    values_to_substitute.push_back(PNy);

    if (! first_section_) {
        // scaled_prev_P contains the control points of the previous solution scaled by their corresponding time and degree
        SX P2x = (scaled_prev_P_(0, 2) - scaled_prev_P_(0, 1) * 2 + scaled_prev_P_(0, 0)) * sym_Tf_ /(BezierDegree_ * (BezierDegree_ - 1))
                + 2* P1x - current_states_.x;
        SX P2y = (scaled_prev_P_(1, 2) - scaled_prev_P_(1, 1) * 2 + scaled_prev_P_(1, 0)) * sym_Tf_ /(BezierDegree_ * (BezierDegree_ - 1)) 
                + 2* P1y - current_states_.y;
        
        vars_to_substitute.push_back(sym_P_all_(0,2));
        vars_to_substitute.push_back(sym_P_all_(1,2));
        values_to_substitute.push_back(P2x);
        values_to_substitute.push_back(P2y);
    }

    std::vector<SX> all_obs_constraints;
    for (int i = 0; i < circ_obs_.cols(); ++i) {
        SX obs = SX::vertcat({SX(circ_obs_(0,i)), SX(circ_obs_(1,i)), SX(circ_obs_(2,i))});
        SX substituted_constraint = SX::substitute(sym_circ_constr_, {sym_circ_obs_}, {obs});
        all_obs_constraints.push_back(substituted_constraint);
    }

    for (int i = 0; i < line_obs_.cols(); ++i) {
        SX line = SX::vertcat({SX(line_obs_(0,i)), SX(line_obs_(1,i)), SX(line_obs_(2,i))});
        SX substituted_constraint = SX::substitute(sym_line_constr_, {sym_line_obs_}, {line});
        all_obs_constraints.push_back(substituted_constraint);
    }

    for (int i = 0; i < neighbor_lines_.cols(); ++i) {
        SX line = SX::vertcat({SX(neighbor_lines_(0,i)), SX(neighbor_lines_(1,i)), SX(neighbor_lines_(2,i))});
        SX substituted_constraint = SX::substitute(sym_neigh_constr_, {sym_neigh_coefs_}, {line});
        all_obs_constraints.push_back(substituted_constraint);
    }

    SX all_obs_constr;
    if(!all_obs_constraints.empty()){
        all_obs_constr = SX::vertcat(all_obs_constraints);
    } else {
        all_obs_constr = SX();
    }


    SX constraint_vector = vertcat(sym_vel_constr_, sym_ang_vel_constr_, sym_acc_constr_, sym_ang_acc_constr_, all_obs_constr);

    constraints_ = constraint_vector;
    cost_func_   = sym_cost_;

    for (size_t i = 0; i < vars_to_substitute.size(); ++i)
    {
        constraints_ = SX::substitute(constraints_, vars_to_substitute[i], values_to_substitute[i]);

        cost_func_   = SX::substitute(cost_func_, vars_to_substitute[i], values_to_substitute[i]);

        sym_P_all_ = SX::substitute(sym_P_all_, vars_to_substitute[i], values_to_substitute[i]);
    }

    // --- Set bounds for the optimization problem ---
    int obs_size = all_obs_constr.size1();
    int vel_size = sym_vel_constr_.size1();
    int ang_vel_size = sym_ang_vel_constr_.size1();
    int acc_size = sym_acc_constr_.size1();
    int ang_acc_size = sym_ang_acc_constr_.size1();
    int total_num = obs_size + vel_size + ang_vel_size + acc_size + ang_acc_size;

    problem_Bounds_.g_lbx = DM::zeros(total_num);
    problem_Bounds_.g_ubx = DM::zeros(total_num);
    int idx = 0;
    problem_Bounds_.g_lbx(Slice(idx, idx + vel_size)) = DM::ones(vel_size) * VEL_MIN * VEL_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + vel_size)) = DM::ones(vel_size) * VEL_MAX * VEL_MAX;
    idx = idx + vel_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + ang_vel_size)) = DM::ones(ang_vel_size) * ANG_VEL_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + ang_vel_size)) = DM::ones(ang_vel_size) * ANG_VEL_MAX;
    idx = idx + ang_vel_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + acc_size)) = DM::ones(acc_size) * (-ACC_MIN) * ACC_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + acc_size)) = DM::ones(acc_size) * ACC_MAX * ACC_MAX;
    idx = idx + acc_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + ang_acc_size)) = DM::ones(ang_acc_size) * ANG_ACC_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + ang_acc_size)) = DM::ones(ang_acc_size) * ANG_ACC_MAX;
    idx = idx + ang_acc_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + obs_size)) = DM::ones(obs_size) * OBS_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + obs_size)) = DM::ones(obs_size) * OBS_MAX;
}

void SingleVehicleMotionPlan::createConstraintVector()
{
    using namespace casadi;

    std::vector<SX> vars_to_substitute;
    std::vector<SX> values_to_substitute;

    vars_to_substitute.push_back(sym_P_all_(0,0));
    vars_to_substitute.push_back(sym_P_all_(1,0));
    
    values_to_substitute.push_back(SX(current_states_.x));
    values_to_substitute.push_back(SX(current_states_.y));
    
    SX P1x = current_states_.x + current_states_.v * cos(current_states_.theta) * sym_Tf_ / BezierDegree_;
    SX P1y = current_states_.y + current_states_.v * sin(current_states_.theta) * sym_Tf_ / BezierDegree_;
    
    vars_to_substitute.push_back(sym_P_all_(0,1));
    vars_to_substitute.push_back(sym_P_all_(1,1));

    values_to_substitute.push_back(P1x);
    values_to_substitute.push_back(P1y);

    if (! first_section_) {
        // scaled_prev_P contains the control points of the previous solution scaled by their corresponding time and degree
        SX P2x = (scaled_prev_P_(0, 2) - scaled_prev_P_(0, 1) * 2 + scaled_prev_P_(0, 0)) * sym_Tf_ /(BezierDegree_ * (BezierDegree_ - 1))
                + 2* P1x - current_states_.x;
        SX P2y = (scaled_prev_P_(1, 2) - scaled_prev_P_(1, 1) * 2 + scaled_prev_P_(1, 0)) * sym_Tf_ /(BezierDegree_ * (BezierDegree_ - 1)) 
                + 2* P1y - current_states_.y;
        
        vars_to_substitute.push_back(sym_P_all_(0,2));
        vars_to_substitute.push_back(sym_P_all_(1,2));
        values_to_substitute.push_back(P2x);
        values_to_substitute.push_back(P2y);
    }

    std::vector<SX> all_obs_constraints;
    for (int i = 0; i < line_obs_.cols(); ++i) {
        SX line = SX::vertcat({SX(line_obs_(0,i)), SX(line_obs_(1,i)), SX(line_obs_(2,i))});
        SX substituted_constraint = SX::substitute(sym_line_constr_, {sym_line_obs_}, {line});
        all_obs_constraints.push_back(substituted_constraint);
    }

    for (int i = 0; i < neighbor_lines_.cols(); ++i) {
        SX line = SX::vertcat({SX(neighbor_lines_(0,i)), SX(neighbor_lines_(1,i)), SX(neighbor_lines_(2,i))});
        SX substituted_constraint = SX::substitute(sym_neigh_constr_, {sym_neigh_coefs_}, {line});
        all_obs_constraints.push_back(substituted_constraint);
    }

    SX all_obs_constr;
    if(!all_obs_constraints.empty()){
        all_obs_constr = SX::vertcat(all_obs_constraints);
    } else {
        all_obs_constr = SX();
    }


    SX constraint_vector = vertcat(sym_vel_constr_, sym_ang_vel_constr_, sym_acc_constr_, sym_ang_acc_constr_, all_obs_constr);

    constraints_ = constraint_vector;
    cost_func_   = sym_cost_;

    for (size_t i = 0; i < vars_to_substitute.size(); ++i)
    {
        constraints_ = SX::substitute(constraints_, vars_to_substitute[i], values_to_substitute[i]);

        cost_func_   = SX::substitute(cost_func_, vars_to_substitute[i], values_to_substitute[i]);

        sym_P_all_ = SX::substitute(sym_P_all_, vars_to_substitute[i], values_to_substitute[i]);
    }

    // --- Set bounds for the optimization problem ---
    int obs_size = all_obs_constr.size1();
    int vel_size = sym_vel_constr_.size1();
    int ang_vel_size = sym_ang_vel_constr_.size1();
    int acc_size = sym_acc_constr_.size1();
    int ang_acc_size = sym_ang_acc_constr_.size1();
    int total_num = obs_size + vel_size + ang_vel_size + acc_size + ang_acc_size;

    problem_Bounds_.g_lbx = DM::zeros(total_num);
    problem_Bounds_.g_ubx = DM::zeros(total_num);
    int idx = 0;
    problem_Bounds_.g_lbx(Slice(idx, idx + vel_size)) = DM::ones(vel_size) * VEL_MIN * VEL_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + vel_size)) = DM::ones(vel_size) * VEL_MAX * VEL_MAX;
    idx = idx + vel_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + ang_vel_size)) = DM::ones(ang_vel_size) * ANG_VEL_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + ang_vel_size)) = DM::ones(ang_vel_size) * ANG_VEL_MAX;
    idx = idx + ang_vel_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + acc_size)) = DM::ones(acc_size) * (-ACC_MIN) * ACC_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + acc_size)) = DM::ones(acc_size) * ACC_MAX * ACC_MAX;
    idx = idx + acc_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + ang_acc_size)) = DM::ones(ang_acc_size) * ANG_ACC_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + ang_acc_size)) = DM::ones(ang_acc_size) * ANG_ACC_MAX;
    idx = idx + ang_acc_size;
    problem_Bounds_.g_lbx(Slice(idx, idx + obs_size)) = DM::ones(obs_size) * OBS_MIN;
    problem_Bounds_.g_ubx(Slice(idx, idx + obs_size)) = DM::ones(obs_size) * OBS_MAX;
}

void SingleVehicleMotionPlan::createDecisionVectorGoalSeen(Eigen::Matrix<double, 2, Eigen::Dynamic>& ContP_guess, double Tf_guess)
{
    using namespace casadi;

    std::vector<SX> vars;
    std::vector<DM> guess;
    if (first_section_) {
        for (int i = 2; i < sym_P_all_.size2() - 2; ++i) {
            for (int j = 0; j < sym_P_all_.size1(); ++j) {
                vars.push_back(sym_P_all_(j,i));
                guess.push_back(DM(ContP_guess(j,i)));
            }
        }
        vars.push_back(sym_Tf_);
        guess.push_back(Tf_guess);
    } else {
        for (int i = 3; i < sym_P_all_.size2() - 2; ++i) {
            for (int j = 0; j < sym_P_all_.size1(); ++j) {
                vars.push_back(sym_P_all_(j,i));
                guess.push_back(DM(ContP_guess(j,i)));
            }
        }
        vars.push_back(sym_Tf_);
        guess.push_back(Tf_guess);
    }

    decision_vec_ = SX::vertcat(vars);
    initial_guess_ = DM::vertcat(guess);
    problem_Bounds_.lbx = DM::ones(initial_guess_.size1()) * (-std::numeric_limits<double>::infinity());
    problem_Bounds_.ubx = DM::ones(initial_guess_.size1()) * (std::numeric_limits<double>::infinity());
    problem_Bounds_.lbx(initial_guess_.size1() - 1) = DM(0.0);

}

// TODO: Implement this function alternatively using velocity and angle insted...
void SingleVehicleMotionPlan::createDecisionVector(Eigen::Matrix<double, 2, Eigen::Dynamic>& ContP_guess, double Tf_guess)
{
    using namespace casadi;

    std::vector<SX> vars;
    std::vector<DM> guess;
    if (first_section_) {
        for (int i = 2; i < sym_P_all_.size2(); ++i) {
            for (int j = 0; j < sym_P_all_.size1(); ++j) {
                vars.push_back(sym_P_all_(j,i));
                guess.push_back(DM(ContP_guess_(j,i)));
            }
        }
        vars.push_back(sym_Tf_);
        guess.push_back(Tf_guess_);
    } else {
        for (int i = 3; i < sym_P_all_.size2(); ++i) {
            for (int j = 0; j < sym_P_all_.size1(); ++j) {
                vars.push_back(sym_P_all_(j,i));
                guess.push_back(DM(ContP_guess_(j,i)));
            }
        }
        vars.push_back(sym_Tf_);
        guess.push_back(Tf_guess_);
    }

    decision_vec_ = SX::vertcat(vars);
    initial_guess_ = DM::vertcat(guess);
    problem_Bounds_.lbx = DM::ones(initial_guess_.size1()) * (-std::numeric_limits<double>::infinity());
    problem_Bounds_.ubx = DM::ones(initial_guess_.size1()) * (std::numeric_limits<double>::infinity());
    problem_Bounds_.lbx(initial_guess_.size1() - 1) = DM(0.0);

}

void SingleVehicleMotionPlan::computeCostFunctionGoalSeen()
{
    using namespace casadi;

    int ang_vel_size = sym_ang_vel_constr_.size1();
    int acc_size = sym_acc_constr_.size1();
    int ang_acc_size = sym_ang_acc_constr_.size1();

    SX energy_cost;
    SX end_cost;
    // Compute cost associated with energy (minimize acceleration squared terms)
    if ((constr_flag_[1] == false) || (constr_flag_[2] == false))
    {
        energy_cost = SX(0);
        end_cost = SX(0);
    }
    else {
        SX angular_cost = pow(sum1(sym_ang_acc_constr_)/ang_acc_size, 2);
        angular_cost = angular_cost/pow(ANG_VEL_MAX,2);
        SX acceleration_cost = sum1(sym_acc_constr_)/acc_size;
        acceleration_cost = acceleration_cost/pow(ACC_MAX,2);
        energy_cost = angular_cost/2+ acceleration_cost/2;

        // Compute cost associated with initial and final acc and angular rate
        end_cost = (sym_acc_constr_(0)/pow(ACC_MAX,2) + sym_acc_constr_(acc_size - 1)/pow(ACC_MAX,2) + pow(sym_ang_vel_constr_(0),2)/pow(ANG_VEL_MAX,2) + pow(sym_ang_vel_constr_(ang_vel_size - 1),2)/pow(ANG_VEL_MAX,2))/4;
    }

    // Sotf plus function to penalize high velocity at the end of the trajectory
    SX P_Deriv = BezierUtils::bezier_derivative_Casadi(sym_P_all_)/sym_Tf_;
    SX P_Deriv_squared = BezierUtils::multiply_bezier_Casadi(P_Deriv, P_Deriv);
    SX end_vel_squared = P_Deriv_squared(0, P_Deriv_squared.size1() - 1) + P_Deriv_squared(1, P_Deriv_squared.size1() - 1);

    // Parameters
    double eps = 1e-3;                      // smoothing parameter
    double a   = pow(VEL_MAX-0.02,2);       // cutoff point
    
    // Define penalty functions
    SX softplus = log(1 + exp((end_vel_squared - a)/eps));
    
    sym_cost_ = sym_Tf_ * (ALPHA + BETA * energy_cost + GAMMA * end_cost) + 1000.0 * softplus;
}

void SingleVehicleMotionPlan::computeCostFunction()
{
    using namespace casadi;

    int ang_vel_size = sym_ang_vel_constr_.size1();
    int acc_size = sym_acc_constr_.size1();
    int ang_acc_size = sym_ang_acc_constr_.size1();

    SX energy_cost;
    SX end_cost;
    // Compute cost associated with energy (minimize acceleration squared terms)
    if ((constr_flag_[1] == false) || (constr_flag_[2] == false))
    {
        energy_cost = SX(0);
        end_cost = SX(0);
    }
    else {
        SX angular_cost = pow(sum1(sym_ang_acc_constr_)/ang_acc_size, 2);
        angular_cost = angular_cost/pow(ANG_VEL_MAX,2);
        SX acceleration_cost = sum1(sym_acc_constr_)/acc_size;
        acceleration_cost = acceleration_cost/pow(ACC_MAX,2);
        energy_cost = angular_cost/2+ acceleration_cost/2;

        // Compute cost associated with initial and final acc and angular rate
        end_cost = (sym_acc_constr_(0)/pow(ACC_MAX,2) + sym_acc_constr_(acc_size - 1)/pow(ACC_MAX,2) + pow(sym_ang_vel_constr_(0),2)/pow(ANG_VEL_MAX,2) + pow(sym_ang_vel_constr_(ang_vel_size - 1),2)/pow(ANG_VEL_MAX,2))/4;
    }
    
    SX dist_to_Goal = sqrt(pow(sym_P_all_(0, sym_P_all_.size2() - 1) - goal_states_.x, 2) + pow(sym_P_all_(1, sym_P_all_.size2() - 1) - goal_states_.y, 2));

    // Sotf plus function to penalize high velocity at the end of the trajectory
    SX P_Deriv = BezierUtils::bezier_derivative_Casadi(sym_P_all_)/sym_Tf_;
    SX P_Deriv_squared = BezierUtils::multiply_bezier_Casadi(P_Deriv, P_Deriv);
    SX end_vel_squared = P_Deriv_squared(0, P_Deriv_squared.size1() - 1) + P_Deriv_squared(1, P_Deriv_squared.size1() - 1);

    // Parameters
    double eps = 1e-3;                      // smoothing parameter
    double a   = pow(VEL_MAX-0.02,2);       // cutoff point
    
    // Define penalty functions
    SX softplus = log(1 + exp((end_vel_squared - a)/eps));
    
    sym_cost_ = sym_Tf_ * (ALPHA + BETA * energy_cost + GAMMA * end_cost + DELTA * dist_to_Goal) + 1000.0 * softplus;
}

void SingleVehicleMotionPlan::solveOptimizationProblem()
{
    using namespace casadi;

    SXDict nlp; // NLP declaration
    nlp["x"] = decision_vec_; 
    nlp["f"] = cost_func_;
    nlp["g"] = constraints_;

    // Set solver options
    Dict opts;
    opts["ipopt.print_level"] = 5;
    opts["ipopt.tol"] = 1e-10;
    opts["ipopt.max_iter"] = 100000;

    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    DMDict solver_inputs = {
        {"x0", DM(initial_guess_)},
        {"lbx", DM(problem_Bounds_.lbx)},
        {"ubx", DM(problem_Bounds_.ubx)},
        {"lbg", DM(problem_Bounds_.g_lbx)},
        {"ubg", DM(problem_Bounds_.g_ubx)}};

    auto sol = solver(solver_inputs);

    // Extract the solution
    DM opt_sol = sol["x"];

    // Extract the final time (Tf)
    Tf_opt_ = opt_sol(opt_sol.size1() - 1, 0).scalar();

    std::vector<SX> vars;
    for (int i = 0; i < decision_vec_.size1(); ++i) {
        vars.push_back(decision_vec_(i));
    }
    std::vector<DM> vals;
    for (int i = 0; i < opt_sol.size1(); ++i) {
        vals.push_back(opt_sol(i));
    }
    SX P_all_numeric_sx = sym_P_all_;

    for (size_t i = 0; i < vars.size(); ++i)
    {
        P_all_numeric_sx = SX::substitute(P_all_numeric_sx , vars[i], vals[i]);
    }
    DM P_all_numeric = DM(P_all_numeric_sx);
    int rows = P_all_numeric.size1(); 
    int cols = P_all_numeric.size2(); 
    optimal_ContP_.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            optimal_ContP_(i, j) = static_cast<double>(P_all_numeric(i, j));
        }
    }
}

Eigen::Matrix<double, 2, Eigen::Dynamic> SingleVehicleMotionPlan::getControlPoints() const
{
    return optimal_ContP_;
}

double SingleVehicleMotionPlan::getTf() const
{
    return Tf_opt_;
}

