#include "MultipleVehicleMotionPlan.h"

// Constructor for the MultipleVehicleMotionPlan class
MultipleVehicleMotionPlan::MultipleVehicleMotionPlan(int BezierDegree, int NVehicles, const std::vector<int> &nSplit, const std::vector<uint8_t> &constr_flag)
    : BezierDegree_(BezierDegree), NVehicles_(NVehicles), nSplit_(nSplit), constr_flag_(constr_flag) 
{
    // Validate input parameters
    if (BezierDegree_ <= 0) {
        throw std::invalid_argument("BezierDegree must be positive");
    }
    if (NVehicles_ <= 0) {
        throw std::invalid_argument("Number of Vehicles must be positive");
    }
    if (nSplit_.size() < 2) {
        throw std::invalid_argument("nSplit must have at least 2 elements");
    }
    if (constr_flag_.size() < 4) {
        throw std::invalid_argument("constr_flag must have at least 4 elements");
    }
}

// In MultipleVehicleMotionPlan.cpp
MultipleVehicleMotionPlan::~MultipleVehicleMotionPlan()
{
    // Destructor implementation (if needed)
}


void MultipleVehicleMotionPlan::setBoundsAndGains(double vel_min, double vel_max,
                         double acc_min, double acc_max,
                         double ang_vel_min, double ang_vel_max,
                         double ang_acc_min, double ang_acc_max,
                         double obs_min, double obs_max,
                         double radius, double alpha, double beta, double gamma)
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
}

void MultipleVehicleMotionPlan::setupSymbolic()
{
    using namespace casadi;

    // Shared final time
    sym_Tf_ = SX::sym("Tf");
    sym_circ_obs_ = SX::sym("circ_obs", 3, 1);
    sym_line_obs_ = SX::sym("line_obs", 3, 1);

    // Resize containers
    sym_P_all_.resize(NVehicles_);
    sym_circ_constr_.resize(NVehicles_);
    sym_line_constr_.resize(NVehicles_);
    sym_vel_constr_.resize(NVehicles_);
    sym_ang_vel_constr_.resize(NVehicles_);
    sym_acc_constr_.resize(NVehicles_);
    sym_ang_acc_constr_.resize(NVehicles_);

    for (int i = 0; i < NVehicles_; ++i)
    {
        sym_P_all_[i] = SX::sym("P_" + std::to_string(i), 2, BezierDegree_ + 1);

        // Obstacle constraints
        sym_circ_constr_[i] = BezierUtils::adaptive_all_obstacle_constraints(sym_P_all_[i], sym_circ_obs_, SX::sym("empty_circ", 3, 0), nSplit_[1]);
        sym_line_constr_[i] = BezierUtils::adaptive_all_obstacle_constraints(sym_P_all_[i], SX::sym("empty_circ", 3, 0), sym_line_obs_, nSplit_[1]);

        // Dynamic constraints (shared Tf)
        if (constr_flag_[0])
            sym_vel_constr_[i] = BezierUtils::adaptive_dynamic_constraints(sym_P_all_[i], sym_Tf_, nSplit_[0], {true, false, false, false});

        if (constr_flag_[1])
            sym_ang_vel_constr_[i] = BezierUtils::adaptive_dynamic_constraints(sym_P_all_[i], sym_Tf_, nSplit_[0], {false, true, false, false});

        if (constr_flag_[2])
        {
            ROS_INFO("Setting up acceleration constraints.");
            sym_acc_constr_[i] = BezierUtils::adaptive_dynamic_constraints(sym_P_all_[i], sym_Tf_, nSplit_[0], {false, false, true, false});
        }
        if (constr_flag_[3])
        {
            ROS_INFO("Setting up angular acceleration constraints.");
            sym_ang_acc_constr_[i] = BezierUtils::adaptive_dynamic_constraints(sym_P_all_[i], sym_Tf_, nSplit_[0], {false, false, false, true});
        }
    }
    for(int i = 0; i < NVehicles_; ++i){
        for (int j = i+1; j < NVehicles_; ++j)
        {
            casadi::SX inter_vehicle_constr = BezierUtils::adaptive_inter_vehicle_constraints(sym_P_all_[i], sym_P_all_[j], RADIUS, nSplit_[2]);
            sym_inter_vehicle_constr_ = vertcat(sym_inter_vehicle_constr_, inter_vehicle_constr);
        }
    }
}

void MultipleVehicleMotionPlan::computeCostFunction()
{
    using namespace casadi;

    sym_cost_ = SX(0); 
    SX energy_cost;
    SX end_cost;
    for (int i = 0; i < NVehicles_; ++i)
    {
        int ang_vel_size = sym_ang_vel_constr_[i].size1();
        int acc_size = sym_acc_constr_[i].size1();
        int ang_acc_size = sym_ang_acc_constr_[i].size1();

        // Compute cost associated with energy (minimize acceleration squared terms)
        if ((!constr_flag_[1]) || (!constr_flag_[2]))
        {
            energy_cost = SX(0);
            end_cost = SX(0);
        }
        else {
            SX angular_cost = pow(sum1(sym_ang_acc_constr_[i])/ang_acc_size, 2);
            angular_cost = angular_cost/pow(ANG_VEL_MAX,2);
            SX acceleration_cost = sum1(sym_acc_constr_[i])/acc_size;
            acceleration_cost = acceleration_cost/pow(ACC_MAX,2);
            energy_cost = angular_cost/2+ acceleration_cost/2;

            // Compute cost associated with initial and final acc and angular rate
            end_cost = (sym_acc_constr_[i](0)/pow(ACC_MAX,2) + sym_acc_constr_[i](acc_size - 1)/pow(ACC_MAX,2) + pow(sym_ang_vel_constr_[i](0),2)/pow(ANG_VEL_MAX,2) + pow(sym_ang_vel_constr_[i](ang_vel_size - 1),2)/pow(ANG_VEL_MAX,2))/4;
        }
        

        sym_cost_ = sym_cost_ + sym_Tf_ * (ALPHA + BETA * energy_cost + GAMMA * end_cost);
    }
}

void MultipleVehicleMotionPlan::setOptimizationProblem(const std::vector<vehicle_State>& current_states, const std::vector<vehicle_State>& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs,
                            const Eigen::Tensor<double, 3>& ContP_guess, double Tf_guess, bool first_iter)
{
    // Store states
    current_states_ = current_states;
    goal_states_    = goal_states;
    circ_obs_ = circ_obs;
    line_obs_ = line_obs;
    ContP_guess_ = ContP_guess;
    Tf_guess_ = Tf_guess;
    first_iter_  = first_iter;

    if (ContP_guess_.dimension(2) == 0) {
        std::cerr << "[setOptimizationProblem] WARNING: ContP_guess_ is empty.\n";
    }

    if (circ_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: circ_obs_ must be 3×N.\n";
    }

    if (line_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: line_obs_ must be 3×N.\n";
    }

    if (ContP_guess_.dimension(1) != 2) {
        std::cerr << "[setOptimizationProblem] ERROR: ContP_guess_ must be 2×(deg+1).\n";
    }

    if (Tf_guess_ <= 0) {
        std::cerr << "[setOptimizationProblem] WARNING: Tf_guess_ <= 0, using default 10s.\n";
        Tf_guess_ = 10.0;
    }

    if (nSplit_.empty()) {
        std::cerr << "[setOptimizationProblem] WARNING: nSplit_ is empty.\n";
    }
}

void MultipleVehicleMotionPlan::setOptimizationProblem(const std::vector<vehicle_State>& current_states, const std::vector<vehicle_State>& goal_states,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& circ_obs,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& line_obs)
{
    // Store states
    current_states_ = current_states;
    goal_states_    = goal_states;
    circ_obs_ = circ_obs;
    line_obs_ = line_obs;
    first_iter_  = true;

    if (circ_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: circ_obs_ must be 3×N.\n";
    }

    if (line_obs_.rows() != 3) {
        std::cerr << "[setOptimizationProblem] ERROR: line_obs_ must be 3×N.\n";
    }

    if (nSplit_.empty()) {
        std::cerr << "[setOptimizationProblem] WARNING: nSplit_ is empty.\n";
    }
}

void MultipleVehicleMotionPlan::setOptimizationProblem(const std::vector<vehicle_State>& current_states, const std::vector<vehicle_State>& goal_states)
{
    // Store states
    current_states_ = current_states;
    goal_states_    = goal_states;
    first_iter_  = true;
}

void MultipleVehicleMotionPlan::createConstraintVector()
{
    using namespace casadi;

    std::vector<SX> vars_to_substitute;
    std::vector<SX> values_to_substitute;
    std::vector<SX> all_obs_constraints;
    
    SX sym_vel_constr_all;
    SX sym_ang_vel_constr_all;
    SX sym_acc_constr_all;
    SX sym_ang_acc_constr_all;

    for (int i = 0; i < NVehicles_; ++i)
    {
        vars_to_substitute.push_back(sym_P_all_[i](0,0));
        vars_to_substitute.push_back(sym_P_all_[i](1,0));
        vars_to_substitute.push_back(sym_P_all_[i](0,BezierDegree_));
        vars_to_substitute.push_back(sym_P_all_[i](1,BezierDegree_));
        values_to_substitute.push_back(SX(current_states_[i].x));
        values_to_substitute.push_back(SX(current_states_[i].y));
        values_to_substitute.push_back(SX(goal_states_[i].x));
        values_to_substitute.push_back(SX(goal_states_[i].y));

        SX P1x = current_states_[i].x + current_states_[i].v * cos(current_states_[i].theta) * sym_Tf_ / BezierDegree_;
        SX P1y = current_states_[i].y + current_states_[i].v * sin(current_states_[i].theta) * sym_Tf_ / BezierDegree_;
        SX PNx = goal_states_[i].x - goal_states_[i].v * cos(goal_states_[i].theta) * sym_Tf_ / BezierDegree_;
        SX PNy = goal_states_[i].y - goal_states_[i].v * sin(goal_states_[i].theta) * sym_Tf_ / BezierDegree_;

        vars_to_substitute.push_back(sym_P_all_[i](0,1));
        vars_to_substitute.push_back(sym_P_all_[i](1,1));
        vars_to_substitute.push_back(sym_P_all_[i](0,BezierDegree_-1));
        vars_to_substitute.push_back(sym_P_all_[i](1,BezierDegree_-1));
        values_to_substitute.push_back(P1x);
        values_to_substitute.push_back(P1y);
        values_to_substitute.push_back(PNx);
        values_to_substitute.push_back(PNy);


        for (int k = 0; k < circ_obs_.cols(); ++k) {
            SX substituted_constraint = sym_circ_constr_[i];
            SX obs = SX::vertcat({SX(circ_obs_(0,k)), SX(circ_obs_(1,k)), SX(circ_obs_(2,k))});
            for (casadi_int j = 0; j < obs.size1(); ++j)
            {
                substituted_constraint = SX::substitute(substituted_constraint, sym_circ_obs_(j), obs(j));
            }
            all_obs_constraints.push_back(substituted_constraint);
        }

        for (int k = 0; k < line_obs_.cols(); ++k) {
            SX substituted_constraint = sym_line_constr_[i];
            SX line = SX::vertcat({SX(line_obs_(0,k)), SX(line_obs_(1,k)), SX(line_obs_(2,k))});
            for (casadi_int j = 0; j < line.size1(); ++j)
            {
                substituted_constraint = SX::substitute(substituted_constraint, sym_line_obs_(j), line(j));
            }
            all_obs_constraints.push_back(substituted_constraint);
        }

        if (!sym_vel_constr_[i].is_empty())
        {
            if (sym_vel_constr_all.is_empty())
                sym_vel_constr_all = sym_vel_constr_[i];
            else
                sym_vel_constr_all = SX::vertcat({sym_vel_constr_all, sym_vel_constr_[i]});
        }
        if (!sym_ang_vel_constr_[i].is_empty())
        {
            if (sym_ang_vel_constr_all.is_empty())
                sym_ang_vel_constr_all = sym_ang_vel_constr_[i];
            else
                sym_ang_vel_constr_all = SX::vertcat({sym_ang_vel_constr_all, sym_ang_vel_constr_[i]});
        }
        if (!sym_acc_constr_[i].is_empty())
        {
            if (sym_acc_constr_all.is_empty())  
                sym_acc_constr_all = sym_acc_constr_[i];
            else
                sym_acc_constr_all = SX::vertcat({sym_acc_constr_all, sym_acc_constr_[i]});
        }
        if (!sym_ang_acc_constr_[i].is_empty())
        {
            if (sym_ang_acc_constr_all.is_empty())
                sym_ang_acc_constr_all = sym_ang_acc_constr_[i];
            else
                sym_ang_acc_constr_all = SX::vertcat({sym_ang_acc_constr_all, sym_ang_acc_constr_[i]});
        }
    }

    SX all_obs_constr;
    if(!all_obs_constraints.empty()){
        

        all_obs_constr = SX::vertcat(all_obs_constraints);
        ROS_INFO("Obstacle_constraint size: lines - %d; columns - %d;", all_obs_constr.size1(), all_obs_constr.size2());
    } else {
        all_obs_constr = SX();
    }
    if(!sym_inter_vehicle_constr_.is_empty()){
        all_obs_constr = SX::vertcat({all_obs_constr, sym_inter_vehicle_constr_});
    }
    constraints_ = vertcat(sym_vel_constr_all , sym_ang_vel_constr_all, sym_acc_constr_all, sym_ang_acc_constr_all, all_obs_constr);
    cost_func_   = sym_cost_;

    for (size_t i = 0; i < vars_to_substitute.size(); ++i)
    {
        constraints_ = SX::substitute(constraints_, vars_to_substitute[i], values_to_substitute[i]);

        cost_func_   = SX::substitute(cost_func_, vars_to_substitute[i], values_to_substitute[i]);

        for (int j = 0; j < NVehicles_; ++j)
        {
            sym_P_all_[j] = SX::substitute(sym_P_all_[j], vars_to_substitute[i], values_to_substitute[i]);
        }
    }
    
    int obs_size = all_obs_constr.size1();
    int vel_size = sym_vel_constr_all.size1();
    int ang_vel_size = sym_ang_vel_constr_all.size1();
    int acc_size = sym_acc_constr_all.size1();
    int ang_acc_size = sym_ang_acc_constr_all.size1();
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

void MultipleVehicleMotionPlan::createDecisionVector()
{
    using namespace casadi;
    std::vector<SX> vars;
    std::vector<DM> guess;
    double Tf_guess = 0.0;
    
    for (int i = 0; i < NVehicles_; ++i)
    {
        Tf_guess = std::max(Tf_guess,sqrt(pow(current_states_[i].x-goal_states_[i].x, 2) + pow(current_states_[i].y-goal_states_[i].y, 2)) / (VEL_MAX/2));
    }
    for (int k = 0; k < NVehicles_; ++k)
    {
        if (first_iter_) {
            Function eval_endpoints = Function("eval_endpoints", {sym_Tf_}, {sym_P_all_[k](0,1), sym_P_all_[k](1,1), sym_P_all_[k](0, BezierDegree_-1), sym_P_all_[k](1, BezierDegree_-1)});
            std::vector<DM> res = eval_endpoints({DM(Tf_guess)});

            double x_init_val  = res[0].scalar();
            double y_init_val  = res[1].scalar();
            double x_final_val = res[2].scalar();
            double y_final_val = res[3].scalar();
            
            //std::srand(static_cast<unsigned>(std::time(nullptr)));
            //auto random_offset = []() {
            //    return ((std::rand() / static_cast<double>(RAND_MAX)) * 0.2) - 0.1;
            //};

            //x_init_val  += random_offset();
            //y_init_val  += random_offset();
            //x_final_val += random_offset();
            //y_final_val += random_offset();
            
            if (BezierDegree_ >= 5)
            {
                x_init_val = 2*x_init_val-current_states_[k].x;
                y_init_val = 2*y_init_val-current_states_[k].y;
                x_final_val = 2*x_final_val-goal_states_[k].x;
                y_final_val = 2*y_final_val-goal_states_[k].y;
                //ROS_INFO("Initial guess for vehicle %d: P2=(%.2f, %.2f), PN-2=(%.2f, %.2f)", k, x_init_val, y_init_val, x_final_val, y_final_val);
            } else {
                x_init_val = x_init_val * (BezierDegree_ - 3) / (BezierDegree_ -2) + x_final_val / (BezierDegree_ -2);
                y_init_val = y_init_val * (BezierDegree_ - 3) / (BezierDegree_ -2) + y_final_val / (BezierDegree_ -2);
                x_final_val = x_final_val * (BezierDegree_ - 3) / (BezierDegree_ -2) + x_init_val / (BezierDegree_ -2);
                y_final_val = y_final_val * (BezierDegree_ - 3) / (BezierDegree_ -2) + y_init_val / (BezierDegree_ -2);
            }
            for (int i = 2; i < sym_P_all_[k].size2() - 2; ++i) {
                for (int j = 0; j < sym_P_all_[k].size1(); ++j) {
                    vars.push_back(sym_P_all_[k](j,i));
                }
                double aux = (i-2)/double((sym_P_all_[k].size2() - 5));
                double val_x = aux * x_final_val + (1 - aux) * x_init_val;
                double val_y = aux * y_final_val + (1 - aux) * y_init_val;
                guess.push_back(DM(val_x));
                guess.push_back(DM(val_y));
                //ROS_INFO("Initial guess for vehicle %d: P%d=(%.2f, %.2f)", k, i, val_x, val_y);
            }
        } else {
            for (int i = 2; i < sym_P_all_[k].size2() - 2; ++i) {
                for (int j = 0; j < sym_P_all_[k].size1(); ++j) {
                    vars.push_back(sym_P_all_[k](j,i));
                    guess.push_back(DM(ContP_guess_(k,j,i)));
                }
            }
            Tf_guess = Tf_guess_;
        }
    }
    

    vars.push_back(sym_Tf_);
    guess.push_back(Tf_guess);

   
    ROS_INFO("Initial guess Tf = %.2f", static_cast<double>(guess.back().scalar()));

    decision_vec_ = SX::vertcat(vars);
    initial_guess_ = DM::vertcat(guess);
    problem_Bounds_.lbx = DM::ones(initial_guess_.size1()) * (-std::numeric_limits<double>::infinity());
    problem_Bounds_.ubx = DM::ones(initial_guess_.size1()) * (std::numeric_limits<double>::infinity());
    problem_Bounds_.lbx(initial_guess_.size1() - 1) = DM(0.0);

}

void MultipleVehicleMotionPlan::createConstraintVector(const std::vector<int>& active_vehicles)
{
    using namespace casadi;

    std::vector<SX> vars_to_substitute;
    std::vector<SX> values_to_substitute;
    std::vector<SX> all_obs_constraints;
    
    SX sym_vel_constr_all;
    SX sym_ang_vel_constr_all;
    SX sym_acc_constr_all;
    SX sym_ang_acc_constr_all;

    for (int i = 0; i < NVehicles_; ++i)
    {
        if (std::find(active_vehicles.begin(), active_vehicles.end(), i) == active_vehicles.end()) {
            // Inactive vehicles do not count for constraints, we assume their trajectories are fine

            for (int d = 0; d < ContP_guess_.dimension(1); ++d) {
                for (int pt = 0; pt < ContP_guess_.dimension(2); ++pt) {
                    vars_to_substitute.push_back(sym_P_all_[i](d, pt));
                    values_to_substitute.push_back(SX(ContP_guess_(i, d, pt)));
                }
            }
        }
        else {
            vars_to_substitute.push_back(sym_P_all_[i](0,0));
            vars_to_substitute.push_back(sym_P_all_[i](1,0));
            vars_to_substitute.push_back(sym_P_all_[i](0,BezierDegree_));
            vars_to_substitute.push_back(sym_P_all_[i](1,BezierDegree_));
            values_to_substitute.push_back(SX(current_states_[i].x));
            values_to_substitute.push_back(SX(current_states_[i].y));
            values_to_substitute.push_back(SX(goal_states_[i].x));
            values_to_substitute.push_back(SX(goal_states_[i].y));

            SX P1x = current_states_[i].x + current_states_[i].v * cos(current_states_[i].theta) * sym_Tf_ / BezierDegree_;
            SX P1y = current_states_[i].y + current_states_[i].v * sin(current_states_[i].theta) * sym_Tf_ / BezierDegree_;
            SX PNx = goal_states_[i].x - goal_states_[i].v * cos(goal_states_[i].theta) * sym_Tf_ / BezierDegree_;
            SX PNy = goal_states_[i].y - goal_states_[i].v * sin(goal_states_[i].theta) * sym_Tf_ / BezierDegree_;

            vars_to_substitute.push_back(sym_P_all_[i](0,1));
            vars_to_substitute.push_back(sym_P_all_[i](1,1));
            vars_to_substitute.push_back(sym_P_all_[i](0,BezierDegree_-1));
            vars_to_substitute.push_back(sym_P_all_[i](1,BezierDegree_-1));
            values_to_substitute.push_back(P1x);
            values_to_substitute.push_back(P1y);
            values_to_substitute.push_back(PNx);
            values_to_substitute.push_back(PNy);


            for (int k = 0; k < circ_obs_.cols(); ++k) {
                SX substituted_constraint = sym_circ_constr_[i];
                SX obs = SX::vertcat({SX(circ_obs_(0,k)), SX(circ_obs_(1,k)), SX(circ_obs_(2,k))});
                for (casadi_int j = 0; j < obs.size1(); ++j)
                {
                    substituted_constraint = SX::substitute(substituted_constraint, sym_circ_obs_(j), obs(j));
                }
                all_obs_constraints.push_back(substituted_constraint);
            }

            for (int k = 0; k < line_obs_.cols(); ++k) {
                SX substituted_constraint = sym_line_constr_[i];
                SX line = SX::vertcat({SX(line_obs_(0,k)), SX(line_obs_(1,k)), SX(line_obs_(2,k))});
                for (casadi_int j = 0; j < line.size1(); ++j)
                {
                    substituted_constraint = SX::substitute(substituted_constraint, sym_line_obs_(j), line(j));
                }
                all_obs_constraints.push_back(substituted_constraint);
            }

            if (!sym_vel_constr_[i].is_empty())
            {
                if (sym_vel_constr_all.is_empty())
                    sym_vel_constr_all = sym_vel_constr_[i];
                else
                    sym_vel_constr_all = SX::vertcat({sym_vel_constr_all, sym_vel_constr_[i]});
            }
            if (!sym_ang_vel_constr_[i].is_empty())
            {
                if (sym_ang_vel_constr_all.is_empty())
                    sym_ang_vel_constr_all = sym_ang_vel_constr_[i];
                else
                    sym_ang_vel_constr_all = SX::vertcat({sym_ang_vel_constr_all, sym_ang_vel_constr_[i]});
            }
            if (!sym_acc_constr_[i].is_empty())
            {
                if (sym_acc_constr_all.is_empty())  
                    sym_acc_constr_all = sym_acc_constr_[i];
                else
                    sym_acc_constr_all = SX::vertcat({sym_acc_constr_all, sym_acc_constr_[i]});
            }
            if (!sym_ang_acc_constr_[i].is_empty())
            {
                if (sym_ang_acc_constr_all.is_empty())
                    sym_ang_acc_constr_all = sym_ang_acc_constr_[i];
                else
                    sym_ang_acc_constr_all = SX::vertcat({sym_ang_acc_constr_all, sym_ang_acc_constr_[i]});
            }
        }
    }

    SX all_obs_constr;
    if(!all_obs_constraints.empty()){
        all_obs_constr = SX::vertcat(all_obs_constraints);
        ROS_INFO("Obstacle_constraint size: lines - %d; columns - %d;", all_obs_constr.size1(), all_obs_constr.size2());
    } else {
        all_obs_constr = SX();
    }
    if(!sym_inter_vehicle_constr_.is_empty()){
        all_obs_constr = SX::vertcat({all_obs_constr, sym_inter_vehicle_constr_});
    }
    constraints_ = vertcat(sym_vel_constr_all , sym_ang_vel_constr_all, sym_acc_constr_all, sym_ang_acc_constr_all, all_obs_constr);
    cost_func_   = sym_cost_;

    for (size_t i = 0; i < vars_to_substitute.size(); ++i)
    {
        constraints_ = SX::substitute(constraints_, vars_to_substitute[i], values_to_substitute[i]);

        cost_func_   = SX::substitute(cost_func_, vars_to_substitute[i], values_to_substitute[i]);

        for (int j = 0; j < NVehicles_; ++j)
        {
            sym_P_all_[j] = SX::substitute(sym_P_all_[j], vars_to_substitute[i], values_to_substitute[i]);
        }
    }
    
    int obs_size = all_obs_constr.size1();
    int vel_size = sym_vel_constr_all.size1();
    int ang_vel_size = sym_ang_vel_constr_all.size1();
    int acc_size = sym_acc_constr_all.size1();
    int ang_acc_size = sym_ang_acc_constr_all.size1();
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

void MultipleVehicleMotionPlan::createDecisionVector(const std::vector<int>& active_vehicles, double Tf_min)
{
    using namespace casadi;
    std::vector<SX> vars;
    std::vector<DM> guess;

    for (int k = 0; k < NVehicles_; ++k)
    {
        if (std::find(active_vehicles.begin(), active_vehicles.end(), k) == active_vehicles.end()) {
            std::cout << "[DecisionVector] Not processing vehicle " << k << std::endl;
            continue;
        }
        else {
            std::cout << "[DecisionVector] Processing vehicle " << k << std::endl;
            for (int i = 2; i < sym_P_all_[k].size2() - 2; ++i) {
                for (int j = 0; j < sym_P_all_[k].size1(); ++j) {
                    vars.push_back(sym_P_all_[k](j,i));
                    guess.push_back(DM(ContP_guess_(k,j,i)));
                }
            }
        }
    }
    

    vars.push_back(sym_Tf_);
    guess.push_back(DM(Tf_guess_));

    ROS_INFO("Initial guess Tf = %.2f", static_cast<double>(guess.back().scalar()));

    decision_vec_ = SX::vertcat(vars);
    initial_guess_ = DM::vertcat(guess);

    std::cout << "initial_guess shape: "
          << initial_guess_.size1() << " x "
          << initial_guess_.size2() << std::endl;
    problem_Bounds_.lbx = DM::ones(initial_guess_.size1()) * (-std::numeric_limits<double>::infinity());
    problem_Bounds_.ubx = DM::ones(initial_guess_.size1()) * (std::numeric_limits<double>::infinity());
    problem_Bounds_.lbx(initial_guess_.size1() - 1) = DM(Tf_min);

}

void MultipleVehicleMotionPlan::solveOptimizationProblem(std::atomic<bool>* cancel_flag)
{
    using namespace casadi;

    SXDict nlp; // NLP declaration
    nlp["x"] = decision_vec_; 
    nlp["f"] = cost_func_;
    nlp["g"] = constraints_;

    std::cout << "Callback n_in: " << decision_vec_.size1() << "x" << decision_vec_.size2() << std::endl;
    // First create temporary solver to inspect outputs
    Function tmp_solver = nlpsol("tmp_solver", "ipopt", nlp);

    int solver_outputs = tmp_solver.n_out();


    std::vector<casadi::Sparsity> sparsities;
    for (int i = 0; i < solver_outputs; ++i) {
        sparsities.push_back(tmp_solver.sparsity_out(i));
    }
    CancelCallback cancel_cb("cancel_cb", cancel_flag, sparsities);

    // Set solver options
    Dict opts;
    opts["ipopt.print_level"] = 3;
    opts["ipopt.tol"] = 1e-6;
    opts["ipopt.max_iter"] = 5000;

    // Assign the callback
    opts["iteration_callback"] = cancel_cb;
    opts["iteration_callback_step"] = 1; // called every iteration

    std::cout << "Callback n_in: " << cancel_cb.n_in() << std::endl;
    std::cout << "Callback n_out: " << cancel_cb.n_out() << std::endl;
    Function solver = nlpsol("solver", "ipopt", nlp, opts);
   
    DMDict solver_inputs = {
        {"x0", DM(initial_guess_)},
        {"lbx", DM(problem_Bounds_.lbx)},
        {"ubx", DM(problem_Bounds_.ubx)},
        {"lbg", DM(problem_Bounds_.g_lbx)},
        {"ubg", DM(problem_Bounds_.g_ubx)}};

    try {
        auto sol = solver(solver_inputs);

        // Extract the solution
        DM opt_sol = sol["x"];
        return_status_ = solver.stats().at("return_status").to_string();
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
        

        // Assume all vehicles have the same number of control points
        int rows = sym_P_all_[0].size1();
        int cols = sym_P_all_[0].size2();

        // Resize tensor: [vehicle][row][col]
        optimal_ContP_.resize(NVehicles_, rows, cols);
        for (int k = 0; k < NVehicles_; ++k)
        {
            SX P_all_numeric_sx = sym_P_all_[k];
            for (size_t i = 0; i < vars.size(); ++i)
            {
                P_all_numeric_sx = SX::substitute(P_all_numeric_sx , vars[i], vals[i]);
            }
            DM P_all_numeric = DM(P_all_numeric_sx);
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    optimal_ContP_(k, i, j) = static_cast<double>(P_all_numeric(i, j));
                }
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[Solver] Aborted: " << e.what() << "\n";
    }
}

Eigen::Tensor<double, 3> MultipleVehicleMotionPlan::getControlPoints() const
{
    return optimal_ContP_;
}

Eigen::Tensor<double, 3> MultipleVehicleMotionPlan::getOptimalTrajectories(int numPoints) const
{

    Eigen::Tensor<double, 3> optimal_Trajectories(NVehicles_, 2, numPoints);
    const auto& CP = optimal_ContP_;

    for(int i = 0; i < NVehicles_; ++i){
        Eigen::MatrixXd pts(2, BezierDegree_ + 1);
        for (int k = 0; k <= BezierDegree_; ++k) {
            pts(0, k) = CP(i, 0, k);
            pts(1, k) = CP(i, 1, k);
        }
        for (int j = 0; j < numPoints; ++j){
            double u = static_cast<double>(j) / (numPoints - 1);
            
            Eigen::VectorXd point = BezierUtils::deCasteljau(u, pts);
            
            optimal_Trajectories(i, 0, j) = point(0);
            optimal_Trajectories(i, 1, j) = point(1);
        }
    }
    return optimal_Trajectories;
}

std::string MultipleVehicleMotionPlan::getOptimizationstatus() const
{
    return return_status_;
}

double MultipleVehicleMotionPlan::getTf() const
{
    return Tf_opt_;
}

