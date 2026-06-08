// #include "BezierOptimization.h"
#include "BezierOptimization.h"
// Constructor for the BezierOptimization class
BezierOptimization::BezierOptimization(int NVehicles,
                                       int BezierDegree,
                                       const std::vector<int> &nSplit,
                                       const std::vector<State> &current_states,
                                       const std::vector<State> &goal_states,
                                       const std::vector<double> &v0,
                                       const std::vector<double> &vT,
                                       const Eigen::Matrix<double, 3, Eigen::Dynamic> &circ_obs,
                                       const Eigen::Matrix<double, 3, Eigen::Dynamic> &line_obs)
    : NVehicles_(NVehicles), BezierDegree_(BezierDegree), nSplit_(nSplit),
      current_states_(current_states), goal_states_(goal_states),
      circ_obs_(circ_obs), line_obs_(line_obs)
{
    // Initialize any other members as needed
    v0_ = v0; // Copy v0 into v0_
    vT_ = vT; // Copy v0 into v0_
}

// ######################################################################################################//
//______________________________________________CENTRALIZED_____________________________________________//
// ######################################################################################################//
void BezierOptimization::runOptimization(bool first_iteration, Eigen::Tensor<double, 3> control_P_Guess, double Tf_Guess, int BezierDegree)
{
    first_iteration_ = first_iteration;
    control_P_Guess_ = control_P_Guess;
    BezierDegree_ = BezierDegree;
    Tf_Guess_ = Tf_Guess;

    defineVariables();
    defineConstraintsNew();
    solveOptimization();
}

void BezierOptimization::defineVariables()
{
    using namespace casadi;

    // Initialize symbolic variables for distances and intermediate points
    P_intermediate_x_ = SX::sym("P_intermediate_x", BezierDegree_ - 3, NVehicles_); // Free x for inner points
    P_intermediate_y_ = SX::sym("P_intermediate_y", BezierDegree_ - 3, NVehicles_); // Free y for inner points
    Tf_ = SX::sym("Tf");                                                            // Final time

    // Initialize separate control point matrices
    controlPoints_.resize(NVehicles_);
    for (int i = 0; i < NVehicles_; ++i)
    {
        controlPoints_[i] = SX::zeros(2, BezierDegree_ + 1);
    }

    // Define control points for each vehicle
    for (int i = 0; i < NVehicles_; ++i)
    {
        // First and last control points (start and goal positions)
        controlPoints_[i](0, 0) = current_states_[i].x; // Start x
        controlPoints_[i](1, 0) = current_states_[i].y; // Start y

        controlPoints_[i](0, BezierDegree_) = goal_states_[i].x; // Goal x
        controlPoints_[i](1, BezierDegree_) = goal_states_[i].y; // Goal y

        // Control point 1 (P1) depends on initial angle and distance l1
        controlPoints_[i](0, 1) = cos(current_states_[i].theta) * v0_[i] * Tf_ / BezierDegree_ + current_states_[i].x;
        controlPoints_[i](1, 1) = sin(current_states_[i].theta) * v0_[i] * Tf_ / BezierDegree_ + current_states_[i].y;

        // Control point d-1 (Pd-1) depends on final angle and distance l2
        controlPoints_[i](0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * vT_[i] * Tf_ / BezierDegree_ + goal_states_[i].x;
        controlPoints_[i](1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * vT_[i] * Tf_ / BezierDegree_ + goal_states_[i].y;

        // Free intermediate points (P2 to P(d-2)) for higher Bézier degrees
        for (int k = 2; k < BezierDegree_ - 1; ++k)
        {
            controlPoints_[i](0, k) = P_intermediate_x_(k - 2, i);
            controlPoints_[i](1, k) = P_intermediate_y_(k - 2, i);
        }
    }
}

void BezierOptimization::solveOptimization()
{
    using namespace casadi;

    double alpha = ALPHA; // Weight for energy
    double beta = BETA;  // Weight for end_points

    int num_vel = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_acc = (BezierDegree_ * 4 - 5) * (1 << nSplit_[0]) * NVehicles_;

    SX vel_constraints = constr_(Slice(0, num_vel));
    SX ang_constraints = constr_(Slice(num_vel, num_vel + num_ang));
    SX acc_constraints = constr_(Slice(num_vel + num_ang, num_vel + num_ang + num_acc));

    SX angular_cost = pow(sum1(ang_constraints)/num_ang, 2);
    angular_cost = angular_cost/pow(ANG_VEL_MAX,2);
    SX acceleration_cost = sum1(acc_constraints)/num_acc;
    acceleration_cost = acceleration_cost/pow(ACC_MAX,2);
    SX energy_cost = angular_cost/3 + 2*acceleration_cost/3;

    SX yaw_end_pts = casadi::SX::zeros(2*NVehicles_);
    SX acc_end_pts = casadi::SX::zeros(2*NVehicles_);
    int index_ang, index_acc;
    for(int i=0; i<NVehicles_; i++){
        index_ang = i*num_ang/NVehicles_; 
        index_acc = i*num_acc/NVehicles_;
        yaw_end_pts(2*i) = pow(ang_constraints(index_ang),2)/pow(ANG_VEL_MAX,2);
        yaw_end_pts(2*i+1) = pow(ang_constraints(index_ang+num_ang/NVehicles_-1),2)/pow(ANG_VEL_MAX,2);
        acc_end_pts(2*i) = acc_constraints(index_acc)/pow(ACC_MAX,2);
        acc_end_pts(2*i+1) = acc_constraints(index_acc+num_acc/NVehicles_-1)/pow(ACC_MAX,2);
    }
    angular_cost = sum1(yaw_end_pts)/ (2*NVehicles_);
    acceleration_cost = sum1(acc_end_pts)/(2*NVehicles_);
    SX end_point_cost = angular_cost + acceleration_cost;

    SX total_cost = Tf_ *(1 + alpha*energy_cost + beta*end_point_cost);

    SX P_intermediate_x_flat = reshape(P_intermediate_x_, (BezierDegree_ - 3) * NVehicles_, 1);
    SX P_intermediate_y_flat = reshape(P_intermediate_y_, (BezierDegree_ - 3) * NVehicles_, 1);

    SXDict nlp;                                                            // NLP declaration
    nlp["x"] = vertcat(P_intermediate_x_flat, P_intermediate_y_flat, Tf_); // decision vopts_dict["print_time"] = 0;ars
    nlp["f"] = total_cost;                                                 // objective
    nlp["g"] = constr_;                                                    // constraints

    // Set solver options
    Dict opts;
    opts["ipopt.print_level"] = 5;
    opts["ipopt.tol"] = 1e-10;
    opts["ipopt.max_iter"] = 100000;

    // Use IPOPT solver from CasADi
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Compute bounds and initial guess using the helper functions
    // Bounds bounds = computeBounds(first_iteration_);
    Bounds bounds = computeBoundsNew();
    DM guess = computeGuesses();
    // std::cout << "Initial Guesses: " << guess << std::endl;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << guess;
    std::cout << "Initial Guesses:\n"
              << stream.str() << std::endl;

    // std::cout << "Initial Guesses: " << std::fixed << std::setprecision(3) << guess << "\n";

    // Use DMDict for solver inputs
    DMDict solver_inputs = {
        {"x0", DM(guess)},
        {"lbx", DM(bounds.lbx)},
        {"ubx", DM(bounds.ubx)},
        {"lbg", DM(bounds.g_lbx)},
        {"ubg", DM(bounds.g_ubx)}};

    auto sol = solver(solver_inputs);

    // Extract the solution
    casadi::DM opt_sol = sol["x"];
    // Extract the final time (Tf)
    Tf_opt_ = opt_sol(opt_sol.size1() - 1, 0).scalar();
    std::cout << "Optimal T_f: " << Tf_opt_ << std::endl;
    opt_control_points_ = extractOptimalControlPoints(opt_sol);
}

Bounds BezierOptimization::computeBoundsNew()
{
    // 1. Define the number of intermediate points per vehicle
    int N_interm = NVehicles_ * (BezierDegree_ - 3);

    Bounds bounds; // Create an instance of the Bounds struct

    // Create casadi::DM arrays for lbx and ubx
    bounds.lbx = -casadi::DM::inf(N_interm * 2 + 1);
    bounds.ubx = casadi::DM::inf(N_interm * 2 + 1);

    max_distance_ = 0.0;
    for (int i = 0; i < NVehicles_; i++)
    {
        double dx = goal_states_[i].x - current_states_[i].x;
        double dy = goal_states_[i].y - current_states_[i].y;
        double distance = std::sqrt(dx * dx + dy * dy); // Compute Euclidean distance

        max_distance_ = std::max(max_distance_, distance); // Update with the max distance
    }
    // Modify lower bounds for Final Time
    bounds.lbx(2 * N_interm) = max_distance_ / VEL_MAX; // Final time cannot be faster than going in straight line at max speed

    int num_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_acc_constraints = (BezierDegree_ * 4 - 5) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang_acc_constraints = (BezierDegree_ * 4 - 3) * (1 << nSplit_[0]) * NVehicles_;


    int num_interV_constraints = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[2]) * (NVehicles_ * (NVehicles_ - 1)) / 2;
    
    int N_circ_obs = circ_obs_.cols();
    int N_line_obs = line_obs_.cols();
    int num_circ_obs = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[1]) * NVehicles_ * N_circ_obs;
    int num_line_obs = (BezierDegree_) * (1 << nSplit_[1]) * NVehicles_ * N_line_obs;
    int num_obs_constraints = num_circ_obs + num_line_obs;

    int total_constraints = num_vel_constraints + num_ang_vel_constraints + num_acc_constraints + num_ang_acc_constraints + num_interV_constraints + num_obs_constraints;

    // Initialize g_lbx and g_ubx as casadi::DM arrays
    bounds.g_lbx = casadi::DM::zeros(total_constraints);
    bounds.g_ubx = casadi::DM::inf(total_constraints);

    int idx = 0;
    bounds.g_lbx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MIN * VEL_MIN;
    idx += num_vel_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MIN;
    idx += num_ang_vel_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * (-ACC_MIN) * ACC_MIN;
    idx += num_acc_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_ang_acc_constraints)) = casadi::DM::ones(num_ang_acc_constraints) * ANG_ACC_MIN;
    idx += num_ang_acc_constraints;
    
    bounds.g_lbx(casadi::Slice(idx, idx + num_interV_constraints)) = casadi::DM::ones(num_interV_constraints) * OBS_MIN;
    idx += num_interV_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * RADIUS * RADIUS * 0;

    idx = 0;
    bounds.g_ubx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MAX * VEL_MAX;
    idx += num_vel_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MAX;
    idx += num_ang_vel_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * ACC_MAX * ACC_MAX;
    idx += num_acc_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_ang_acc_constraints)) = casadi::DM::ones(num_ang_acc_constraints) * ANG_ACC_MAX;
    idx += num_ang_acc_constraints;

    bounds.g_ubx(casadi::Slice(idx, idx + num_interV_constraints)) = casadi::DM::ones(num_interV_constraints) * OBS_MAX;
    idx += num_interV_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * OBS_MAX;

    return bounds; // Return the struct containing all bounds
}

casadi::DM BezierOptimization::computeGuesses()
{
    // Pre-allocate the vector with the exact size
    int guess_size = NVehicles_ * (BezierDegree_ - 3) * 2 + 1;
    casadi::DM guess = casadi::DM::ones(guess_size);
    casadi::DM Px_guess = casadi::DM::zeros(NVehicles_ * (BezierDegree_ - 3));
    casadi::DM Py_guess = casadi::DM::zeros(NVehicles_ * (BezierDegree_ - 3));

    if (first_iteration_)
    {
        double Tf_guess_for_vel = max_distance_ * 0.75 / (VEL_MAX); // Guess assumes for the purposes of calculating P Guesses that the trajectory can be executed faster than allowed
        // 2. Compute Px_guess and Py_guess
        for (int i = 0; i < NVehicles_; i++)
        {
            
            // As the cost function will try to reach 0 values for r and v_dot, we can approximate the third and third to last contro points
            if (BezierDegree_ >= 5) // Only if there are free points
            {
                casadi::SX x_init_expr  = controlPoints_[i](0, 1);
                casadi::SX y_init_expr  = controlPoints_[i](1, 1);
                casadi::SX x_final_expr = controlPoints_[i](0, BezierDegree_-1);
                casadi::SX y_final_expr = controlPoints_[i](1, BezierDegree_-1);
                // Build CasADi Function with Tf_ as the *only* explicit input
                casadi::Function eval_endpoints = casadi::Function("eval_endpoints", {Tf_}, {x_init_expr, y_init_expr, x_final_expr, y_final_expr});
                std::vector<casadi::DM> res = eval_endpoints({casadi::DM(Tf_guess_for_vel)});

                // Extract numeric values
                double x_init_val  = res[0].scalar();
                double y_init_val  = res[1].scalar();
                double x_final_val = res[2].scalar();
                double y_final_val = res[3].scalar();
                
                double dx_cf = x_final_val - current_states_[i].x;
                double dy_cf = y_final_val - current_states_[i].y;
                double baseline = std::sqrt(dx_cf*dx_cf + dy_cf*dy_cf); // distance from current to final

                // normalized epsilon
                double eps = 1 * baseline * baseline;  // small fraction of area relative to baseline


                // area of triangle (current, init, final)
                double area1 = std::abs((x_init_val - current_states_[i].x) * 
                                        (x_final_val - current_states_[i].y) -
                                        (y_init_val - current_states_[i].y) * 
                                        (x_final_val - current_states_[i].x));

                // area of triangle (init, final, goal)
                double area2 = std::abs((x_final_val - x_init_val) * 
                                        (goal_states_[i].y - y_init_val) -
                                        (y_final_val - y_init_val) * 
                                        (goal_states_[i].x - x_init_val));

                

                if (true) {
                    std::cout << "Almost colinear!" << std::endl;
                    ROS_WARN("Almost colinear! Adjusting initial and final points to avoid numerical issues.");

                    // Seed the random number generator once at the start of your program
                    std::srand(std::time(nullptr));

                    // Define rotation range in radians
                    double theta_min = -0.05;
                    double theta_max =  0.05;

                    // Generate random angle for initial point rotation
                    double theta_init = theta_min + (0 - theta_min) * ((double) std::rand() / RAND_MAX);
                    double cos_t = cos(theta_init);
                    double sin_t = sin(theta_init);


                    // Rotate initial point (x_init_val, y_init_val) around current
                    double dx = x_init_val - current_states_[i].x;
                    double dy = y_init_val - current_states_[i].y;
                    x_init_val = current_states_[i].x + cos_t*dx - sin_t*dy;
                    y_init_val = current_states_[i].y + sin_t*dx + cos_t*dy;
                    
                    theta_init = theta_min + (theta_max - theta_min) * ((double) std::rand() / RAND_MAX);
                    cos_t = cos(theta_init);
                    sin_t = sin(theta_init);
                    // Rotate final point (x_final_val, y_final_val) around goal
                    dx = x_final_val - goal_states_[i].x;
                    dy = y_final_val - goal_states_[i].y;
                    x_final_val = goal_states_[i].x + cos_t*dx - sin_t*dy;
                    y_final_val = goal_states_[i].y + sin_t*dx + cos_t*dy;
                }
                
                double P2_x = 2*x_init_val-current_states_[i].x;
                double P2_y = 2*y_init_val-current_states_[i].y;
                double Pn2_x = 2*x_final_val-goal_states_[i].x;
                double Pn2_y = 2*y_final_val-goal_states_[i].y;

                int index = i * (BezierDegree_ - 3);

                Px_guess(index) = P2_x;
                Py_guess(index) = P2_y;
                Px_guess(index +(BezierDegree_ - 3)-1) = Pn2_x; 
                Py_guess(index +(BezierDegree_ - 3)-1) = Pn2_y;
                // Linear interpolation for the rest of the points between P2 and Pn2
                double alpha;
                for (int j = 3; j < BezierDegree_ - 2; j++)
                {
                    index = i * (BezierDegree_ - 3) + (j - 2);
                    alpha = (j - 2) / static_cast<double>(BezierDegree_ - 4);
                    Px_guess(index) = alpha * Pn2_x + (1 - alpha) * P2_x;
                    Py_guess(index) = alpha * Pn2_y + (1 - alpha) * P2_y;
                }
            }
            else
            {
                for (int j = 2; j < BezierDegree_ - 1; j++)
                {
                    int index = i * (BezierDegree_ - 3) + (j - 2);
                    Px_guess(index) = (goal_states_[i].x * (j / static_cast<double>(BezierDegree_))) +
                                    (current_states_[i].x * ((BezierDegree_ - j) / static_cast<double>(BezierDegree_)));
                    Py_guess(index) = (goal_states_[i].y * (j / static_cast<double>(BezierDegree_))) +
                                    (current_states_[i].y * ((BezierDegree_ - j) / static_cast<double>(BezierDegree_)));
                }
            }
        }

        // std::cout << "total Px_guess dimensions: " << Px_guess.size1() << " x " << Px_guess.size2() << std::endl;
        // Concatenate Px_guess and Py_guess into guess
        int idx = 0;
        guess(casadi::Slice(idx, idx + NVehicles_ * (BezierDegree_ - 3))) = Px_guess;
        idx += NVehicles_ * (BezierDegree_ - 3);
        guess(casadi::Slice(idx, idx + NVehicles_ * (BezierDegree_ - 3))) = Py_guess;
        idx += NVehicles_ * (BezierDegree_ - 3);

        guess(idx) = max_distance_ / (VEL_MAX * 0.75); // Guess assumes that the longest trajectory will be executed at 75% of max velocity
        std::cout << "Px_guess (first_iter): " << Px_guess << std::endl;
        std::cout << "Py_guess (first_iter): " << Py_guess << std::endl;
    }
    else
    {
        // 2. Compute Px_guess and Py_guess
        int X = control_P_Guess_.dimension(2); // Get the last dimension dynamically
        for (int i = 0; i < NVehicles_; i++)
        {
            // Extract a slice along the 3rd dimension
            Eigen::MatrixXd aux_P_guess(2, X);
            // Extract data from tensor
            for (int row = 0; row < 2; row++)
            {
                for (int col = 0; col < X; col++)
                {
                    aux_P_guess(row, col) = control_P_Guess_(i, row, col);
                }
            }

            int cols = aux_P_guess.cols();
            if (cols < BezierDegree_ + 1)
            {
                aux_P_guess = elevate_degree(aux_P_guess, BezierDegree_ + 1 - cols);
            }
            for (int j = 2; j < BezierDegree_ - 1; j++)
            {
                int index = i * (BezierDegree_ - 3) + (j - 2);

                Px_guess(index) = aux_P_guess(0, j);
                Py_guess(index) = aux_P_guess(1, j);
            }
        }

        // Concatenate Px_guess and Py_guess into guess
        int idx = 0;
        guess(casadi::Slice(idx, idx + NVehicles_ * (BezierDegree_ - 3))) = Px_guess;
        idx += NVehicles_ * (BezierDegree_ - 3);
        guess(casadi::Slice(idx, idx + NVehicles_ * (BezierDegree_ - 3))) = Py_guess;
        idx += NVehicles_ * (BezierDegree_ - 3);

        guess(idx) = Tf_Guess_; // Tf
    }

    return guess;
}

Eigen::Tensor<double, 3> BezierOptimization::extractOptimalControlPoints(const casadi::DM &opt_sol)
{
    using namespace casadi;

    int idx = 0;

    // Extract intermediate control points into Eigen matrices
    Eigen::MatrixXd opt_P_intermediate_x(BezierDegree_ - 3, NVehicles_);
    Eigen::MatrixXd opt_P_intermediate_y(BezierDegree_ - 3, NVehicles_);

    idx = 0;
    for (int c = 0; c < NVehicles_; ++c)
    {
        for (int r = 0; r < BezierDegree_ - 3; ++r)
        {
            opt_P_intermediate_x(r, c) = opt_sol(idx, 0).scalar();
            opt_P_intermediate_y(r, c) = opt_sol(idx + (BezierDegree_ - 3) * NVehicles_, 0).scalar();
            idx++;
        }
    }
    Tf_opt_ = opt_sol(opt_sol.size1() - 1, 0).scalar();
    // std::cout << "Tf_opt_ value: " << Tf_opt_ << std::endl;  // Print the value

    // Define the output tensor: NVehicles_ × (BezierDegree_ + 1) × 2
    Eigen::Tensor<double, 3> optimal_ContP_(NVehicles_, 2, BezierDegree_ + 1);
    optimal_ContP_.setZero(); // Initialize with zeros

    // Populate control points
    for (int i = 0; i < NVehicles_; ++i)
    {
        // Start and goal positions
        optimal_ContP_(i, 0, 0) = current_states_[i].x;
        optimal_ContP_(i, 1, 0) = current_states_[i].y;

        optimal_ContP_(i, 0, BezierDegree_) = goal_states_[i].x;
        optimal_ContP_(i, 1, BezierDegree_) = goal_states_[i].y;

        // Control point 1 (P1)
        optimal_ContP_(i, 0, 1) = cos(current_states_[i].theta) * v0_[i] * Tf_opt_ / BezierDegree_ + current_states_[i].x;
        optimal_ContP_(i, 1, 1) = sin(current_states_[i].theta) * v0_[i] * Tf_opt_ / BezierDegree_ + current_states_[i].y;

        // Control point d-1 (Pd-1)
        optimal_ContP_(i, 0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * vT_[i] * Tf_opt_ / BezierDegree_ + goal_states_[i].x;
        optimal_ContP_(i, 1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * vT_[i] * Tf_opt_ / BezierDegree_ + goal_states_[i].y;

        // Free intermediate points (P2 to P(d-2))
        for (int k = 2; k < BezierDegree_ - 1; ++k)
        {
            optimal_ContP_(i, 0, k) = opt_P_intermediate_x(k - 2, i);
            optimal_ContP_(i, 1, k) = opt_P_intermediate_y(k - 2, i);
        }
    }

    return optimal_ContP_;
}

// ######################################################################################################//
//______________________________________________DISTRIBUTED_____________________________________________//
// ######################################################################################################//
void BezierOptimization::runOptimization(bool first_iteration, bool goal_reached, bool first_section, Eigen::Tensor<double, 3> control_P_Guess, Eigen::Tensor<double, 3> control_P_Prev, double Tf_Guess, double Tf_prev, double Tf_min, int BezierDegree)
{
    first_iteration_ = first_iteration;
    control_P_Guess_ = control_P_Guess;
    BezierDegree_ = BezierDegree;
    Tf_Guess_ = Tf_Guess;
    // Variables needed in the distributed case
    goal_reached_ = goal_reached;
    first_section_ = first_section;
    control_P_Prev_ = control_P_Prev;
    Tf_Prev_ = Tf_prev;
    Tf_min_ = Tf_min;

    defineVariablesDist();
    defineConstraintsNew();
    solveOptimizationDist();
}

void BezierOptimization::defineVariablesDist()
{
    using namespace casadi;
    int num_interM = 0;
    // If this is the first section of the path there is no need to impose continuous acceleration
    if (first_section_)
    {
        num_interM = BezierDegree_ - 3;
    }
    else // To ensure C2 continuity there is an additional control point that is determined
    {
        num_interM = BezierDegree_ - 4;
    }
    P_intermediate_x_ = SX::sym("P_intermediate_x", num_interM, 1); // Free x for inner points
    P_intermediate_y_ = SX::sym("P_intermediate_y", num_interM, 1); // Free y for inner points
    Tf_ = SX::sym("Tf");                                            // Final time
    // If the goal is outside our voronoi cell we cannot impose final velocity
    if (!see_goal_)
    {
        l2_ = SX::sym("l2", 1);
    }
    // Initialize separate control point matrices VAMOS FINGIR que NVehicles_ não é 1
    controlPoints_.resize(1);
    controlPoints_[0] = SX::zeros(2, BezierDegree_ + 1);

    // In the distributed case the optimization is done for one only vehicle thus i=0 serves only
    // to preserve the structure and class variables used in the centralized approach
    int i = 0;
    // First and last control points (start and goal positions)
    controlPoints_[i](0, 0) = current_states_[i].x; // Start x
    controlPoints_[i](1, 0) = current_states_[i].y; // Start y

    controlPoints_[i](0, BezierDegree_) = goal_states_[i].x; // Goal x
    controlPoints_[i](1, BezierDegree_) = goal_states_[i].y; // Goal y

    // Control point 1 (P1) depends on initial angle and initial velocity
    controlPoints_[i](0, 1) = cos(current_states_[i].theta) * v0_[i] * Tf_ / BezierDegree_ + current_states_[i].x;
    controlPoints_[i](1, 1) = sin(current_states_[i].theta) * v0_[i] * Tf_ / BezierDegree_ + current_states_[i].y;

    // Control point d-1 (Pd-1) depends on final angle and end velocity
    if (see_goal_)
    {
        controlPoints_[i](0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * vT_[i] * Tf_ / BezierDegree_ + goal_states_[i].x;
        controlPoints_[i](1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * vT_[i] * Tf_ / BezierDegree_ + goal_states_[i].y;
    }
    else
    {
        controlPoints_[i](0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * l2_(i) * Tf_ / BezierDegree_ + goal_states_[i].x;
        controlPoints_[i](1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * l2_(i) * Tf_ / BezierDegree_ + goal_states_[i].y;
    }

    if (first_section_)
    {
        // Free intermediate points (P2 to P(d-2)) for higher Bézier degrees
        for (int k = 2; k < BezierDegree_ - 1; ++k)
        {
            controlPoints_[i](0, k) = P_intermediate_x_(k - 2, i);
            controlPoints_[i](1, k) = P_intermediate_y_(k - 2, i);
        }
    }
    else
    { // Enforce C2 continuity
        int prev_degree = control_P_Prev_.dimension(2) - 1;
        SX aux = (prev_degree * (prev_degree - 1) * Tf_ * Tf_) / (BezierDegree_ * (BezierDegree_ - 1) * Tf_Prev_ * Tf_Prev_);
        controlPoints_[i](0, 2) = aux * (control_P_Prev_(i, 0, prev_degree) - 2 * control_P_Prev_(i, 0, prev_degree - 1) + control_P_Prev_(i, 0, prev_degree - 2)) + 2 * controlPoints_[i](0, 1) - controlPoints_[i](0, 0);
        controlPoints_[i](1, 2) = aux * (control_P_Prev_(i, 1, prev_degree) - 2 * control_P_Prev_(i, 1, prev_degree - 1) + control_P_Prev_(i, 1, prev_degree - 2)) + 2 * controlPoints_[i](1, 1) - controlPoints_[i](1, 0);
        ;

        // Free intermediate points (P3 to P(d-2)) for higher Bézier degrees
        for (int k = 3; k < BezierDegree_ - 1; ++k)
        {
            controlPoints_[i](0, k) = P_intermediate_x_(k - 3, i);
            controlPoints_[i](1, k) = P_intermediate_y_(k - 3, i);
        }
    }
}

void BezierOptimization::solveOptimizationDist()
{
    using namespace casadi;

    double alpha = 1.0; // Weight for time
    double beta = 1.0;  // Weight for energy (tune as needed)

    int num_vel = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]);
    SX vel_constraints = constr_(Slice(0, num_vel));
    SX energy_cost = sum1(vel_constraints);
    SX energy_int = Tf_ * energy_cost / (BezierDegree_ + 1);
    SX total_cost = alpha * Tf_ + beta * energy_int;

    SXDict nlp; // NLP declaration
    if (goal_reached_)
    {
        nlp["x"] = vertcat(P_intermediate_x_, P_intermediate_y_, Tf_); // decision vopts_dict["print_time"] = 0;ars
    }
    else
    {
        nlp["x"] = vertcat(P_intermediate_x_, P_intermediate_y_, l2_, Tf_); // decision vopts_dict["print_time"] = 0;ars
    }
    nlp["f"] = total_cost; // objective
    nlp["g"] = constr_;    // constraints

    // Set solver options
    Dict opts;
    opts["ipopt.print_level"] = 5;
    opts["ipopt.tol"] = 1e-10;
    opts["ipopt.max_iter"] = 100000;

    // Use IPOPT solver from CasADi
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Compute bounds and initial guess using the helper functions
    Bounds bounds = computeBoundsNew();
    DM guess = computeGuessesDist();

    // Use DMDict for solver inputs
    DMDict solver_inputs = {
        {"x0", DM(guess)},
        {"lbx", DM(bounds.lbx)},
        {"ubx", DM(bounds.ubx)},
        {"lbg", DM(bounds.g_lbx)},
        {"ubg", DM(bounds.g_ubx)}};

    auto sol = solver(solver_inputs);

    // Extract the solution
    casadi::DM opt_sol = sol["x"];
    // Extract the final time (Tf)
    Tf_opt_ = opt_sol(opt_sol.size1() - 1, 0).scalar();
    std::cout << "Optimal T_f: " << Tf_opt_ << std::endl;
    opt_control_points_ = extractOptimalControlPointsDist(opt_sol);
}

Bounds BezierOptimization::computeBoundsDist()
{
    int num_interM = 0;
    // If this is the first section of the path there is no need to impose continuous acceleration
    if (first_section_)
    {
        num_interM = BezierDegree_ - 3;
    }
    else // To ensure C2 continuity there is an additional control point that is determined
    {
        num_interM = BezierDegree_ - 4; // Less intermidiate points
    }

    int guess_size = 0;
    // Pre-allocate the vector with the exact size
    if (goal_reached_)
    {
        guess_size = num_interM * 2 + 1; // +1 because of Tf variable
    }
    else
    {
        guess_size = num_interM * 2 + 2; // +2 because of Tf and l2 variables
    }

    Bounds bounds; // Create an instance of the Bounds struct

    // Create casadi::DM arrays for lbx and ubx
    bounds.lbx = -casadi::DM::inf(guess_size);
    bounds.ubx = casadi::DM::inf(guess_size);
    if (!goal_reached_)
    {
        bounds.lbx(guess_size - 2) = VEL_MIN; // l2 cannot be outside the velocity bounds
        bounds.ubx(guess_size - 2) = VEL_MAX; // l2 cannot be outside the velocity bounds
    }
    int i = 0;
    double dx = goal_states_[i].x - current_states_[i].x;
    double dy = goal_states_[i].y - current_states_[i].y;
    max_distance_ = std::sqrt(dx * dx + dy * dy);             // Compute Euclidean distance
    double min_time = max_distance_ / VEL_MAX;                // Final time cannot be faster than going in straight line at max speed
    bounds.lbx(guess_size - 1) = std::max(min_time, Tf_min_); // Update with the max time;

    // 2. Constraint Bounds (g_lbx, g_ubx)

    int num_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[1]) * NVehicles_;
    int num_acc_constraints = (2 * ((BezierDegree_) + (BezierDegree_ - 1) - 1) - 1) * (1 << nSplit_[3]) * NVehicles_;
    int N_circ_obs = circ_obs_.cols();
    int N_line_obs = line_obs_.cols();
    int num_circ_obs = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[1]) * NVehicles_ * N_circ_obs;
    int num_line_obs = (BezierDegree_) * (1 << nSplit_[1]) * NVehicles_ * N_line_obs;
    int num_obs_constraints = num_circ_obs + num_line_obs;

    int total_constraints = num_vel_constraints + num_ang_vel_constraints + num_acc_constraints + num_obs_constraints;

    // Initialize g_lbx and g_ubx as casadi::DM arrays
    bounds.g_lbx = casadi::DM::zeros(total_constraints);
    bounds.g_ubx = casadi::DM::inf(total_constraints);

    int idx = 0;
    bounds.g_lbx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MIN * VEL_MIN;
    idx += num_vel_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MIN;
    idx += num_ang_vel_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * (-ACC_MIN) * ACC_MIN;
    idx += num_acc_constraints;
    bounds.g_lbx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * OBS_MIN;

    idx = 0;
    bounds.g_ubx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MAX * VEL_MAX;
    idx += num_vel_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MAX;
    idx += num_ang_vel_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * ACC_MAX * ACC_MAX;
    idx += num_acc_constraints;
    bounds.g_ubx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * OBS_MAX;

    return bounds; // Return the struct containing all bounds
}

casadi::DM BezierOptimization::computeGuessesDist()
{
    int num_interM = 0;
    // If this is the first section of the path there is no need to impose continuous acceleration
    if (first_section_)
    {
        num_interM = BezierDegree_ - 3;
    }
    else // To ensure C2 continuity there is an additional control point that is determined
    {
        num_interM = BezierDegree_ - 4; // Less intermidiate points
    }

    int guess_size = 0;
    // Pre-allocate the vector with the exact size
    if (goal_reached_)
    {
        guess_size = num_interM * 2 + 1; // +1 because of Tf variable
    }
    else
    {
        guess_size = num_interM * 2 + 2; // +2 because of Tf and l2 variables
    }

    casadi::DM guess = casadi::DM::ones(guess_size);
    casadi::DM Px_guess = casadi::DM::zeros(num_interM);
    casadi::DM Py_guess = casadi::DM::zeros(num_interM);

    if (first_iteration_)
    {
        // 2. Compute Px_guess and Py_guess
        int i = 0;
        int first_idx = BezierDegree_ - 1 - num_interM;
        for (int j = first_idx; j < BezierDegree_ - 1; j++)
        {
            int index = (j - first_idx);
            Px_guess(index) = (goal_states_[i].x * (j / static_cast<double>(BezierDegree_))) +
                              (current_states_[i].x * ((BezierDegree_ - j) / static_cast<double>(BezierDegree_)));
            Py_guess(index) = (goal_states_[i].y * (j / static_cast<double>(BezierDegree_))) +
                              (current_states_[i].y * ((BezierDegree_ - j) / static_cast<double>(BezierDegree_)));
        }

        // Concatenate Px_guess and Py_guess into guess
        int idx = 0;
        guess(casadi::Slice(idx, idx + num_interM)) = Px_guess;
        idx += num_interM;
        guess(casadi::Slice(idx, idx + num_interM)) = Py_guess;
        idx += num_interM;

        if (!goal_reached_)
        {
            guess(idx) = VEL_MAX * 0.75; // Guess assumes that final velocity is 75% of max velocity
            idx++;
        }

        guess(idx) = max_distance_ / (VEL_MAX * 0.75); // Guess assumes that the trajectory is being executed at 75% of max velocity
    }
    else
    {
        // 2. Compute Px_guess and Py_guess
        int guess_degree = control_P_Guess_.dimension(2) - 1; // Get the last dimension dynamically
        int i = 0;

        // Extract a slice along the 3rd dimension
        Eigen::MatrixXd aux_P_guess(2, guess_degree + 1);
        // Extract data from tensor
        for (int row = 0; row < 2; row++)
        {
            for (int col = 0; col < guess_degree + 1; col++)
            {
                aux_P_guess(row, col) = control_P_Guess_(i, row, col);
            }
        }

        if (guess_degree < BezierDegree_)
        {
            aux_P_guess = elevate_degree(aux_P_guess, BezierDegree_ - guess_degree);
        }

        int first_idx = BezierDegree_ - 1 - num_interM;
        for (int j = first_idx; j < BezierDegree_ - 1; j++)
        {
            int index = (j - first_idx);

            Px_guess(index) = aux_P_guess(0, j);
            Py_guess(index) = aux_P_guess(1, j);
        }

        // Concatenate Px_guess and Py_guess into guess
        int idx = 0;
        guess(casadi::Slice(idx, idx + num_interM)) = Px_guess;
        idx += num_interM;
        guess(casadi::Slice(idx, idx + num_interM)) = Py_guess;
        idx += num_interM;

        if (!goal_reached_)
        {
            guess(idx) = VEL_MAX * 0.75; // Guess assumes that final velocity is 75% of max velocity
            idx++;
        }

        guess(idx) = Tf_Guess_; // Guesses on final velocity based on previous computation
    }

    return guess;
}

Eigen::Tensor<double, 3> BezierOptimization::extractOptimalControlPointsDist(const casadi::DM &opt_sol)
{
    using namespace casadi;
    int num_interM = 0;
    // If this is the first section of the path there is no need to impose continuous acceleration
    if (first_section_)
    {
        num_interM = BezierDegree_ - 3;
    }
    else // To ensure C2 continuity there is an additional control point that is determined
    {
        num_interM = BezierDegree_ - 4; // Less intermidiate points
    }

    int guess_size;
    // Pre-allocate the vector with the exact size
    if (goal_reached_)
    {
        guess_size = num_interM * 2 + 1; // +1 because of Tf variable
    }
    else
    {
        guess_size = num_interM * 2 + 2; // +2 because of Tf and l2 variables
    }

    Tf_opt_ = opt_sol(opt_sol.size1() - 1, 0).scalar();

    // Extract intermediate control points into Eigen matrices
    Eigen::MatrixXd opt_P_intermediate_x(num_interM, 1);
    Eigen::MatrixXd opt_P_intermediate_y(num_interM, 1);

    // Free intermediate points (P2 or P3 to P(d-1)) for higher Bézier degrees
    for (int k = 0; k < num_interM; ++k)
    {
        opt_P_intermediate_x(k) = opt_sol(k, 0).scalar();
        opt_P_intermediate_y(k) = opt_sol(k + num_interM, 0).scalar();
    }

    // Define the output tensor: NVehicles_ × (BezierDegree_ + 1) × 2
    Eigen::Tensor<double, 3> optimal_ContP_(NVehicles_, 2, BezierDegree_ + 1);
    optimal_ContP_.setZero(); // Initialize with zeros

    // Populate control points

    int i = 0;
    // Start and goal positions
    optimal_ContP_(i, 0, 0) = current_states_[i].x;
    optimal_ContP_(i, 1, 0) = current_states_[i].y;

    optimal_ContP_(i, 0, BezierDegree_) = goal_states_[i].x;
    optimal_ContP_(i, 1, BezierDegree_) = goal_states_[i].y;

    // Control point 1 (P1)
    optimal_ContP_(i, 0, 1) = cos(current_states_[i].theta) * v0_[i] * Tf_opt_ / BezierDegree_ + current_states_[i].x;
    optimal_ContP_(i, 1, 1) = sin(current_states_[i].theta) * v0_[i] * Tf_opt_ / BezierDegree_ + current_states_[i].y;

    if (goal_reached_)
    {
        // Control point d-1 (Pd-1)
        optimal_ContP_(i, 0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * vT_[i] * Tf_opt_ / BezierDegree_ + goal_states_[i].x;
        optimal_ContP_(i, 1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * vT_[i] * Tf_opt_ / BezierDegree_ + goal_states_[i].y;
    }
    else
    {
        double l2 = opt_sol(opt_sol.size1() - 2, 0).scalar();
        // Control point d-1 (Pd-1)
        optimal_ContP_(i, 0, BezierDegree_ - 1) = -cos(goal_states_[i].theta) * l2 * Tf_opt_ / BezierDegree_ + goal_states_[i].x;
        optimal_ContP_(i, 1, BezierDegree_ - 1) = -sin(goal_states_[i].theta) * l2 * Tf_opt_ / BezierDegree_ + goal_states_[i].y;
    }

    if (!first_section_)
    {
        // Enforce C2 continuity
        int prev_degree = control_P_Prev_.dimension(2) - 1;
        double aux = (prev_degree * (prev_degree - 1) * Tf_opt_ * Tf_opt_) /
                     (BezierDegree_ * (BezierDegree_ - 1) * Tf_Prev_ * Tf_Prev_);

        optimal_ContP_(i, 0, 2) = aux * (control_P_Prev_(i, 0, prev_degree) - 2 * control_P_Prev_(i, 0, prev_degree - 1) + control_P_Prev_(i, 0, prev_degree - 2)) + 2 * optimal_ContP_(i, 0, 1) - optimal_ContP_(i, 0, 0);

        optimal_ContP_(i, 1, 2) = aux * (control_P_Prev_(i, 1, prev_degree) - 2 * control_P_Prev_(i, 1, prev_degree - 1) + control_P_Prev_(i, 1, prev_degree - 2)) + 2 * optimal_ContP_(i, 1, 1) - optimal_ContP_(i, 1, 0);
    }

    int first_idx = BezierDegree_ - 1 - num_interM;
    // Free intermediate points (P2 to P(d-2))
    for (int k = first_idx; k < BezierDegree_ - 1; ++k)
    {
        optimal_ContP_(i, 0, k) = opt_P_intermediate_x(k - first_idx);
        optimal_ContP_(i, 1, k) = opt_P_intermediate_y(k - first_idx);
    }

    return optimal_ContP_;
}

// ######################################################################################################//
//______________________________________________SHARED__________________________________________________//
// ######################################################################################################//
void BezierOptimization::defineConstraintsNew()
{
    using namespace casadi;
    // Initialize the dynamic constraints vector
    int num_vel = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_acc = (BezierDegree_ * 4 - 5) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang_acc = (BezierDegree_ * 4 - 3) * (1 << nSplit_[0]) * NVehicles_;

    SX vel_constraints = SX::zeros(num_vel, 1);
    SX ang_constraints = SX::zeros(num_ang, 1);
    SX acc_constraints = SX::zeros(num_acc, 1);
    SX ang_acc_constraints = SX::zeros(num_ang_acc, 1);

    int index_vel = 0;
    int index_ang = 0;
    int index_acc = 0;
    int index_ang_acc = 0;

    // Initialize the collision constraints vector
    int aux_num_interV = (NVehicles_ * (NVehicles_ - 1)) / 2;
    int num_interV = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[2]) * aux_num_interV;
    int N_circ_obs = circ_obs_.cols();
    int N_line_obs = line_obs_.cols();
    int num_circ_obs = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[1]) * NVehicles_ * N_circ_obs;
    int num_line_obs = (BezierDegree_) * (1 << nSplit_[1]) * NVehicles_ * N_line_obs;
    int num_obs = num_circ_obs + num_line_obs;

    SX interV_constraints = SX::zeros(num_interV, 1);
    SX obs_constraints = SX::zeros(num_obs, 1);

    // Fill constraints in a single loop
    int index_interV = 0;
    int index_obs = 0;

    for (int i = 0; i < NVehicles_; ++i)
    {
        // Dynamic constraints
        SX dyn = adaptive_dynamic_constraints(controlPoints_[i], Tf_, nSplit_[0]);

        int n_segments = 1 << nSplit_[0];
        std::vector<int> dyn_sizes = {
            // Respect the order in which the constraints are concatenated in adaptive_dynamic_constraints
            (BezierDegree_ * 2 - 1),                    // vel
            (BezierDegree_ * 2 - 1),                    // ang
            (BezierDegree_ * 4 - 5),                     // acc
            (BezierDegree_ * 4 - 3)                     // ang_acc
        };
        std::vector<casadi::SX> split_result = split_constraints(dyn, n_segments, dyn_sizes);

        int vel_size = dyn_sizes[0] * n_segments;
        int ang_size = dyn_sizes[1] * n_segments;
        int acc_size = dyn_sizes[2] * n_segments;
        int ang_acc_size = dyn_sizes[3] * n_segments;

        vel_constraints(Slice(index_vel, index_vel + vel_size)) = split_result[0];
        ang_constraints(Slice(index_ang, index_ang + ang_size)) = split_result[1];
        acc_constraints(Slice(index_acc, index_acc + acc_size)) = split_result[2];
        ang_acc_constraints(Slice(index_ang_acc, index_ang_acc + ang_acc_size)) = split_result[3];

        index_vel += vel_size;
        index_ang += ang_size;
        index_acc += acc_size;
        index_ang_acc += ang_acc_size;

        if (num_obs > 0)
        {
            SX obs = adaptive_all_obstacle_constraints(controlPoints_[i], circ_obs_, line_obs_, nSplit_[1]);
            obs_constraints(Slice(index_obs, index_obs + num_obs / NVehicles_)) = obs;
            index_obs += num_obs / (NVehicles_);
        }

        // Inter-vehicle constraints (only when i < j)
        for (int j = i + 1; j < NVehicles_; ++j)
        {
            SX interV = adaptive_interV_constraints(controlPoints_[i], controlPoints_[j], RADIUS, nSplit_[2]);
            interV_constraints(Slice(index_interV, index_interV + num_interV / aux_num_interV)) = interV;
            index_interV += num_interV / aux_num_interV;
        }
    }
    // Concatenate all constraints sequentially into constr_
    constr_ = vertcat(vel_constraints, ang_constraints, acc_constraints, ang_acc_constraints, interV_constraints, obs_constraints);
}

std::vector<casadi::SX> BezierOptimization::split_constraints(const casadi::SX &flat_constraints, int n_segments, const std::vector<int> &n_local_vals)
{
    using namespace casadi;
    std::vector<casadi::SX> output(n_local_vals.size());
    std::vector<std::vector<casadi::SX>> temp(n_local_vals.size());

    int block_size = 0;
    for (int n : n_local_vals)
        block_size += n;

    for (int k = 0; k < n_segments; ++k)
    {
        int offset = k * block_size;
        int local_offset = 0;

        for (size_t i = 0; i < n_local_vals.size(); ++i)
        {
            int size = n_local_vals[i];
            casadi::SX part = flat_constraints(casadi::Slice(offset + local_offset, offset + local_offset + size));
            temp[i].push_back(part);
            local_offset += size;
        }
    }

    for (size_t i = 0; i < n_local_vals.size(); ++i)
    {
        output[i] = casadi::SX::vertcat(temp[i]);
    }

    return output;
}

Eigen::Tensor<double, 3> BezierOptimization::getControlPoints() const
{
    return opt_control_points_;
}

double BezierOptimization::getTf() const
{
    return Tf_opt_;
}

casadi::SX BezierOptimization::adaptive_dynamic_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX deriv_p = bezier_derivative_Casadi(P) / Tf;
        casadi::SX sec_deriv_p = bezier_derivative_Casadi(deriv_p) / Tf;
        casadi::SX third_deriv_p = bezier_derivative_Casadi(sec_deriv_p) / Tf;

        // Surge calculation
        casadi::SX square = multiply_Bezier_Casadi(deriv_p, deriv_p);
        casadi::SX square_vel = square(0, casadi::Slice()) + square(1, casadi::Slice());

        // Heading-rate Calculation
        casadi::SX num_1 = multiply_Bezier_Casadi(deriv_p(0, casadi::Slice()), sec_deriv_p(1, casadi::Slice()));
        casadi::SX num_2 = multiply_Bezier_Casadi(deriv_p(1, casadi::Slice()), sec_deriv_p(0, casadi::Slice()));
        casadi::SX num_ang = num_1 - num_2;
        casadi::SX num_ang_el = elevate_degree_Casadi(num_ang, 1);
        casadi::SX ang_constr = num_ang_el / square_vel;

        // Acceleration Calculation
        casadi::SX num_x = multiply_Bezier_Casadi(deriv_p(0, casadi::Slice()), sec_deriv_p(0, casadi::Slice()));
        casadi::SX num_y = multiply_Bezier_Casadi(deriv_p(1, casadi::Slice()), sec_deriv_p(1, casadi::Slice()));
        casadi::SX num_acc_sqrt = num_x + num_y;

        casadi::SX num_acc = multiply_Bezier_Casadi(num_acc_sqrt, num_acc_sqrt);

        int num_size = num_acc.size2();
        int den_size = square_vel.size2();

        casadi::SX den = elevate_degree_Casadi(square_vel, num_size - den_size);
        casadi::SX acc_constr = num_acc / den;

        // Angular acceleration Calculation
        // term_a = (y'''x'-x'''y')(x'^2+y'^2)
        casadi::SX term1_a = multiply_Bezier_Casadi(third_deriv_p(1, casadi::Slice()), deriv_p(0, casadi::Slice()));
        casadi::SX term2_a = multiply_Bezier_Casadi(third_deriv_p(0, casadi::Slice()), deriv_p(1, casadi::Slice()));
        casadi::SX term_a_res = term1_a - term2_a;
        casadi::SX term_a = multiply_Bezier_Casadi(term_a_res, square_vel);
        // term_b = (y''x'-x''y')(2x'x''+2y'y'')
        casadi::SX term_b = 2*multiply_Bezier_Casadi(num_acc_sqrt, num_ang);

        casadi::SX num_ang_acc = term_a - term_b;
        casadi::SX den_ang_acc = multiply_Bezier_Casadi(square_vel, square_vel);
        int num_ang_acc_size = num_ang_acc.size2();
        int den_ang_acc_size = den_ang_acc.size2();
        num_ang_acc = elevate_degree_Casadi(num_ang_acc, den_ang_acc_size - num_ang_acc_size);
        casadi::SX ang_acc_constr = num_ang_acc / den_ang_acc;
        
        return casadi::SX::vertcat({square_vel.T(), ang_constr.T(), acc_constr.T(), ang_acc_constr.T()});
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_dynamic_constraints(A1, Tf / 2, nsplit - 1),
            adaptive_dynamic_constraints(B1, Tf / 2, nsplit - 1));
    }
}

casadi::SX BezierOptimization::adaptive_all_obstacle_constraints(const casadi::SX &P, const Eigen::Matrix<double, 3, Eigen::Dynamic> &circ_obs, const Eigen::Matrix<double, 3, Eigen::Dynamic> &line_obs, int nsplit)
{
    if (nsplit == 0)
    {
        std::vector<casadi::SX> all_constraints;

        // circular obstacle
        for (int i = 0; i < circ_obs.cols(); ++i)
        {
            Eigen::Vector3d obs = circ_obs.col(i);
            std::vector<casadi::SX> obs_vec = {casadi::SX(obs(0)), casadi::SX(obs(1))};
            casadi::SX obs_xy = casadi::SX::vertcat(obs_vec);
            casadi::SX P_new = P - obs_xy;
            casadi::SX P_obs = multiply_Bezier_Casadi(P_new, P_new);
            casadi::SX radiuses = (obs(2)) * (obs(2)) * casadi::SX::ones(1, P_obs.size2());
            casadi::SX constraint = (P_obs(0, casadi::Slice()) + P_obs(1, casadi::Slice()) - radiuses).T();
            all_constraints.push_back(constraint);
        }

        // Infinite line obstacle
        for (int i = 0; i < line_obs.cols(); ++i)
        {
            Eigen::Vector3d line = line_obs.col(i);
            casadi::SX a = casadi::SX(line(0));
            casadi::SX b = casadi::SX(line(1));
            casadi::SX c = casadi::SX(line(2));

            // P is 2 x N
            int N = P.size2();
            casadi::SX ones_row = casadi::SX::ones(1, N);

            // P_new is 3 x N, line_vec is 3 x 1
            casadi::SX result = a*P(0, casadi::Slice()) +
                                b*P(1, casadi::Slice()) +
                                c*ones_row(0, casadi::Slice());
            
            // If all points have the same sign they are on the same side of that line
            casadi::SX sign_ref = result(0);
            // Multiplying by the first point will result in a positive value
            casadi::SX constraint = sign_ref * result(casadi::Slice(1, result.size2()));

            all_constraints.push_back(constraint.T());
        }

        return casadi::SX::vertcat(all_constraints);
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_all_obstacle_constraints(A1, circ_obs, line_obs, nsplit - 1),
            adaptive_all_obstacle_constraints(B1, circ_obs, line_obs, nsplit - 1));
    }
}

casadi::SX BezierOptimization::adaptive_interV_constraints(const casadi::SX &P1, const casadi::SX &P2, double radius, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX P_new = P1 - P2;
        casadi::SX P_obs = multiply_Bezier_Casadi(P_new, P_new);
        casadi::SX radiuses = radius * radius * casadi::SX::ones(1, P_obs.size2());
        return (P_obs(0, casadi::Slice()) + P_obs(1, casadi::Slice()) - radiuses).T();
    }
    else
    {
        auto [v1_A1, v1_B1] = divide_bezier_Casadi(P1, 0.5);
        auto [v2_A1, v2_B1] = divide_bezier_Casadi(P2, 0.5);
        return vertcat(
            adaptive_interV_constraints(v1_A1, v2_A1, radius, nsplit - 1),
            adaptive_interV_constraints(v1_B1, v2_B1, radius, nsplit - 1));
    }
}

// ######################################################################################################//
//________________________________________BEZIER_OPERATIONS_____________________________________________//
// ######################################################################################################//
casadi::SX BezierOptimization::bezier_derivative_Casadi(const casadi::SX &P)
{
    // Get the dimensions and number of control points
    int dim = P.size1(); // Typically 2 (for 2D curves)
    int n = P.size2();   // Number of control points (n)

    // Degree of the Bézier curve (degree = n - 1)
    int degree = n - 1;

    // Initialize the derivative control points
    casadi::SX P_derivative = casadi::SX::zeros(dim, degree); // (dim, degree) for derivative points

    // Compute the derivative control points
    for (int i = 0; i < degree; ++i)
    {
        P_derivative(casadi::Slice(), i) = degree * (P(casadi::Slice(), i + 1) - P(casadi::Slice(), i));
    }

    return P_derivative;
}

casadi::SX BezierOptimization::elevate_degree_Casadi(const casadi::SX &P, int degree)
{
    int n = P.size2() - 1; // Degree of the Bézier curve (number of control points - 1)

    // Initialize the new control points matrix with zeros (degree + n control points)
    casadi::SX new_control_P = casadi::SX::zeros(P.size1(), n + degree + 1);
    // Perform degree elevation directly inside the function
    for (int i = 0; i <= n + degree; ++i)
    {
        for (int j = std::max(0, i - degree); j <= std::min(n, i); ++j)
        {
            // Compute the binomial coefficient directly for each term
            double val1 = tgamma(n + 1) / (tgamma(j + 1) * tgamma(n - j + 1));
            double val2 = tgamma(degree + 1) / (tgamma(i - j + 1) * tgamma(degree - (i - j) + 1));
            double val3 = tgamma(n + degree + 1) / (tgamma(i + 1) * tgamma(n + degree - i + 1));

            // Compute the weighted control point contribution
            new_control_P(casadi::Slice(), i) += (val1 * val2 / val3) * P(casadi::Slice(), j);
        }
    }

    return new_control_P;
}

casadi::SX BezierOptimization::multiply_Bezier_Casadi(const casadi::SX &P1, const casadi::SX &P2)
{
    // Get the degrees of the Bézier curves
    int np1 = P1.size2() - 1; // Degree of the first Bézier curve
    int np2 = P2.size2() - 1; // Degree of the second Bézier curve
    // Example inside your function:
    // std::cout << "P1 dimensions: " << P1.size1() << " x " << P1.size2() << std::endl;
    // std::cout << "P2 dimensions: " << P2.size1() << " x " << P2.size2() << std::endl;

    // Initialize the product Bézier curve with the appropriate number of control points
    casadi::SX product_P = casadi::SX::zeros(P1.size1(), np1 + np2 + 1);
    // std::cout << "product_P dimensions: " << product_P.size1() << " x " << product_P.size2() << std::endl;
    //  Nested loops to calculate the control points of the product curve
    for (int i = 0; i <= np1; ++i)
    {
        for (int j = 0; j <= np2; ++j)
        {
            // Compute the binomial coefficient directly for each term
            double val1 = tgamma(np1 + 1) / (tgamma(i + 1) * tgamma(np1 - i + 1));
            double val2 = tgamma(np2 + 1) / (tgamma(j + 1) * tgamma(np2 - j + 1));
            double val3 = tgamma(np1 + np2 + 1) / (tgamma(i + j + 1) * tgamma(np1 + np2 - i - j + 1));
            double coef = val1 * val2 / val3;

            // Add the weighted control points to the correct position in the product Bézier curve
            product_P(casadi::Slice(), i + j) = product_P(casadi::Slice(), i + j) + P2(casadi::Slice(), j) * P1(casadi::Slice(), i) * coef;
        }
    }

    return product_P;
}

std::pair<casadi::SX, casadi::SX> BezierOptimization::divide_bezier_Casadi(const casadi::SX &P, double u)
{
    // Get the number of control points (P is of size (dim, n))
    int n = P.size2(); // P is of shape (dim, n)

    // Initialize P1 and P2 as empty matrices
    casadi::SX P1 = casadi::SX::zeros(P.size1(), n);
    casadi::SX P2 = casadi::SX::zeros(P.size1(), n);

    // Loop to perform the division using Casteljau's algorithm
    for (int i = 0; i < n; ++i)
    {
        // Apply Casteljau's algorithm for P1
        P1(casadi::Slice(), i) = deCasteljau_Casadi(u, P(casadi::Slice(), casadi::Slice(0, i + 1)), i + 1, i + 1);
        // Apply Casteljau's algorithm for P2
        P2(casadi::Slice(), i) = deCasteljau_Casadi(u, P(casadi::Slice(), casadi::Slice(i, n)), n - i, n - i);
    }

    // Return the divided Bézier curves as a pair
    return std::make_pair(P1, P2);
}

casadi::SX BezierOptimization::deCasteljau_Casadi(double u, const casadi::SX &pts, int i, int j)
{
    // Base case: if i == 1, return the point
    if (i == 1)
    {
        return pts(casadi::Slice(), j - 1);
    }
    else
    {
        // Recursive case: apply the de Casteljau formula
        casadi::SX left = deCasteljau_Casadi(u, pts, i - 1, j);
        casadi::SX right = deCasteljau_Casadi(u, pts, i - 1, j - 1);
        return u * left + (1 - u) * right;
    }
}

Eigen::MatrixXd BezierOptimization::adaptive_ang_velocity_constraints(const Eigen::MatrixXd &P, double Tf, int nsplit)
{
    if (nsplit == 0)
    {
        Eigen::MatrixXd deriv_p = bezier_derivative(P) / Tf;
        Eigen::MatrixXd sec_deriv_p = bezier_derivative(deriv_p) / Tf;
        sec_deriv_p = elevate_degree(sec_deriv_p, 1);

        Eigen::MatrixXd square = multiply_Bezier(deriv_p, deriv_p);
        Eigen::MatrixXd square1 = multiply_Bezier(deriv_p.row(0), sec_deriv_p.row(1));
        Eigen::MatrixXd square2 = multiply_Bezier(deriv_p.row(1), sec_deriv_p.row(0));

        Eigen::MatrixXd num = square1 - square2;
        Eigen::MatrixXd den = square.row(0) + square.row(1);

        return (num.array() / den.array()).transpose();
    }
    else
    {
        auto [A1, B1] = divide_bezier(P, 0.5);
        return (Eigen::MatrixXd() << adaptive_ang_velocity_constraints(A1, Tf / 2, nsplit - 1),
                adaptive_ang_velocity_constraints(B1, Tf / 2, nsplit - 1))
            .finished();
    }
}

Eigen::MatrixXd BezierOptimization::bezier_derivative(const Eigen::MatrixXd &P)
{
    int dim = P.rows();
    int n = P.cols();
    int degree = n - 1;

    Eigen::MatrixXd P_derivative(dim, degree);
    for (int i = 0; i < degree; ++i)
    {
        P_derivative.col(i) = degree * (P.col(i + 1) - P.col(i));
    }
    return P_derivative;
}

Eigen::MatrixXd BezierOptimization::elevate_degree(const Eigen::MatrixXd &P, int degree)
{
    int n = P.cols() - 1;
    Eigen::MatrixXd new_control_P = Eigen::MatrixXd::Zero(P.rows(), n + degree + 1);

    for (int i = 0; i <= n + degree; ++i)
    {
        for (int j = std::max(0, i - degree); j <= std::min(n, i); ++j)
        {
            double val1 = std::tgamma(n + 1) / (std::tgamma(j + 1) * std::tgamma(n - j + 1));
            double val2 = std::tgamma(degree + 1) / (std::tgamma(i - j + 1) * std::tgamma(degree - (i - j) + 1));
            double val3 = std::tgamma(n + degree + 1) / (std::tgamma(i + 1) * std::tgamma(n + degree - i + 1));

            new_control_P.col(i) += (val1 * val2 / val3) * P.col(j);
        }
    }
    return new_control_P;
}

Eigen::MatrixXd BezierOptimization::multiply_Bezier(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2)
{
    int np1 = P1.cols() - 1;
    int np2 = P2.cols() - 1;

    Eigen::MatrixXd product_P = Eigen::MatrixXd::Zero(P1.rows(), np1 + np2 + 1);

    for (int i = 0; i <= np1; ++i)
    {
        for (int j = 0; j <= np2; ++j)
        {
            double val1 = std::tgamma(np1 + 1) / (std::tgamma(i + 1) * std::tgamma(np1 - i + 1));
            double val2 = std::tgamma(np2 + 1) / (std::tgamma(j + 1) * std::tgamma(np2 - j + 1));
            double val3 = std::tgamma(np1 + np2 + 1) / (std::tgamma(i + j + 1) * std::tgamma(np1 + np2 - i - j + 1));

            product_P.col(i + j) += P2.col(j).cwiseProduct(P1.col(i)) * (val1 * val2 / val3);
        }
    }
    return product_P;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> BezierOptimization::divide_bezier(const Eigen::MatrixXd &P, double u)
{
    int n = P.cols(); // Number of control points

    // Initialize P1 and P2
    Eigen::MatrixXd P1(P.rows(), n);
    Eigen::MatrixXd P2(P.rows(), n);

    // Loop to perform the division using Casteljau's algorithm
    for (int i = 0; i < n; ++i)
    {
        P1.col(i) = deCasteljau(u, P.leftCols(i + 1), i + 1, i + 1);
        P2.col(i) = deCasteljau(u, P.rightCols(n - i), n - i, n - i);
    }

    return {P1, P2};
}

Eigen::VectorXd BezierOptimization::deCasteljau(double u, const Eigen::MatrixXd &pts, int i, int j)
{
    if (i == 1)
    {
        return pts.col(j - 1);
    }
    else
    {
        Eigen::VectorXd left = deCasteljau(u, pts, i - 1, j);
        Eigen::VectorXd right = deCasteljau(u, pts, i - 1, j - 1);
        return u * left + (1 - u) * right;
    }
}

// ######################################################################################################//
//______________________________________________NOT_USED________________________________________________//
// ######################################################################################################//
Bounds BezierOptimization::computeBounds(bool first_iter)
{
    // 1. Define the number of intermediate points per vehicle
    int N_interm = NVehicles_ * (BezierDegree_ - 3);

    Bounds bounds; // Create an instance of the Bounds struct

    // Create casadi::DM arrays for lbx and ubx
    bounds.lbx = casadi::DM::zeros(N_interm * 2 + 1);
    bounds.ubx = casadi::DM::inf(N_interm * 2 + 1);

    // Modify lower bounds for Point variables
    int idx = 0;
    bounds.lbx(casadi::Slice(idx, idx + 2 * N_interm)) = -casadi::DM::inf(2 * N_interm); // Assign -∞ to the next N_interm elements

    // 2. Constraint Bounds (g_lbx, g_ubx)
    if (first_iter)
    {
        int N_obs = circ_obs_.cols();
        int num_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
        int num_obs_constraints = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[4]) * NVehicles_ * N_obs;
        // Initialize g_lbx and g_ubx as casadi::DM arrays
        bounds.g_lbx = casadi::DM::zeros(num_vel_constraints + num_obs_constraints);
        bounds.g_ubx = casadi::DM::inf(num_vel_constraints + num_obs_constraints);
        idx = 0;
        bounds.g_lbx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MIN * VEL_MIN;
        idx = 0;
        bounds.g_ubx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MAX * VEL_MAX;
    }
    else
    {
        int N_obs = circ_obs_.cols();
        int num_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
        int num_ang_vel_constraints = (BezierDegree_ * 2 - 1) * (1 << nSplit_[1]) * NVehicles_;
        int num_interV_constraints = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[2]) * (NVehicles_ * (NVehicles_ - 1)) / 2;
        int num_acc_constraints = (2 * ((BezierDegree_) + (BezierDegree_ - 1) - 1) - 1) * (1 << nSplit_[3]) * NVehicles_;
        int num_obs_constraints = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[4]) * NVehicles_ * N_obs;

        int total_constraints = num_vel_constraints + num_ang_vel_constraints + num_interV_constraints + num_acc_constraints + num_obs_constraints;

        // Initialize g_lbx and g_ubx as casadi::DM arrays
        bounds.g_lbx = casadi::DM::zeros(total_constraints);
        bounds.g_ubx = casadi::DM::inf(total_constraints);

        idx = 0;
        bounds.g_lbx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MIN * VEL_MIN;
        idx += num_vel_constraints;
        bounds.g_lbx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MIN;
        idx += num_ang_vel_constraints;
        bounds.g_lbx(casadi::Slice(idx, idx + num_interV_constraints)) = casadi::DM::ones(num_interV_constraints) * OBS_MIN;
        idx += num_interV_constraints;
        bounds.g_lbx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * (-ACC_MIN) * ACC_MIN;
        idx += num_acc_constraints;
        bounds.g_lbx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * OBS_MIN;

        idx = 0;
        bounds.g_ubx(casadi::Slice(idx, idx + num_vel_constraints)) = casadi::DM::ones(num_vel_constraints) * VEL_MAX * VEL_MAX;
        idx += num_vel_constraints;
        bounds.g_ubx(casadi::Slice(idx, idx + num_ang_vel_constraints)) = casadi::DM::ones(num_ang_vel_constraints) * ANG_VEL_MAX;
        idx += num_ang_vel_constraints;
        bounds.g_ubx(casadi::Slice(idx, idx + num_interV_constraints)) = casadi::DM::ones(num_interV_constraints) * OBS_MAX;
        idx += num_interV_constraints;
        bounds.g_ubx(casadi::Slice(idx, idx + num_acc_constraints)) = casadi::DM::ones(num_acc_constraints) * ACC_MAX * ACC_MAX;
        idx += num_acc_constraints;
        bounds.g_ubx(casadi::Slice(idx, idx + num_obs_constraints)) = casadi::DM::ones(num_obs_constraints) * OBS_MAX;
    }
    return bounds; // Return the struct containing all bounds
}

void BezierOptimization::defineConstraints(bool first_iter)
{
    using namespace casadi;
    // Initialize the constraints vector
    int N_obs = circ_obs_.cols();
    int aux_num_interV = (NVehicles_ * (NVehicles_ - 1)) / 2;
    int num_vel = (BezierDegree_ * 2 - 1) * (1 << nSplit_[0]) * NVehicles_;
    int num_ang = (BezierDegree_ * 2 - 1) * (1 << nSplit_[1]) * NVehicles_;
    int num_interV = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[2]) * aux_num_interV;
    int num_acc = (2 * ((BezierDegree_) + (BezierDegree_ - 1) - 1) - 1) * (1 << nSplit_[3]) * NVehicles_;
    int num_obs = ((BezierDegree_ + 1) * 2 - 1) * (1 << nSplit_[4]) * NVehicles_ * N_obs;

    SX vel_constraints = SX::zeros(num_vel, 1);
    SX ang_constraints = SX::zeros(num_ang, 1);
    SX interV_constraints = SX::zeros(num_interV, 1);
    SX acc_constraints = SX::zeros(num_acc, 1);
    SX obs_constraints = SX::zeros(num_obs, 1);

    // Fill constraints in a single loop
    int index_vel = 0;
    int index_ang = 0;
    int index_interV = 0;
    int index_acc = 0;
    int index_obs = 0;

    for (int i = 0; i < NVehicles_; ++i)
    {
        // Velocity constraints
        SX vel = adaptive_velocity_constraints(controlPoints_[i], Tf_, nSplit_[0]);
        vel_constraints(Slice(index_vel, index_vel + num_vel / NVehicles_)) = vel;
        index_vel += num_vel / NVehicles_;

        for (int i = 0; i < circ_obs_.cols(); ++i)
        {
            SX obs = adaptive_obstacle_constraints(controlPoints_[i], circ_obs_.col(i), nSplit_[4]);
            obs_constraints(Slice(index_obs, index_obs + num_obs / NVehicles_)) = obs;
            index_obs += num_obs / (NVehicles_ * N_obs);
        }
        if (!first_iter)
        {
            // Angular velocity constraints
            SX ang = adaptive_ang_velocity_constraints(controlPoints_[i], Tf_, nSplit_[1]);
            ang_constraints(Slice(index_ang, index_ang + num_ang / NVehicles_)) = ang;
            index_ang += num_ang / NVehicles_;
            // Acceleration velocity constraints
            SX acc = adaptive_acceleration_constraints(controlPoints_[i], Tf_, nSplit_[3]);
            std::cout << "total acc dimensions: " << acc.size1() << " x " << acc.size2() << std::endl;
            acc_constraints(Slice(index_acc, index_acc + num_acc / NVehicles_)) = acc;
            index_acc += num_acc / NVehicles_;

            // Inter-vehicle constraints (only when i < j)
            for (int j = i + 1; j < NVehicles_; ++j)
            {
                SX interV = adaptive_interV_constraints(controlPoints_[i], controlPoints_[j], RADIUS, nSplit_[2]);
                interV_constraints(Slice(index_interV, index_interV + num_interV / aux_num_interV)) = interV;
                index_interV += num_interV / aux_num_interV;
            }
        }
    }
    // Concatenate all constraints sequentially into constr_
    if (first_iter)
    {
        constr_ = vertcat(vel_constraints, obs_constraints);
    }
    else
    {
        constr_ = vertcat(vel_constraints, ang_constraints, interV_constraints, acc_constraints, obs_constraints);
    }
}

casadi::SX BezierOptimization::adaptive_velocity_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX deriv_p = bezier_derivative_Casadi(P) / Tf;
        casadi::SX square = multiply_Bezier_Casadi(deriv_p, deriv_p);
        return (square(0, casadi::Slice()) + square(1, casadi::Slice())).T(); // Sum first and second rows
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_velocity_constraints(A1, Tf / 2, nsplit - 1),
            adaptive_velocity_constraints(B1, Tf / 2, nsplit - 1));
    }
}

casadi::SX BezierOptimization::adaptive_ang_velocity_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX deriv_p = bezier_derivative_Casadi(P) / Tf;
        casadi::SX sec_deriv_p = bezier_derivative_Casadi(deriv_p) / Tf;
        sec_deriv_p = elevate_degree_Casadi(sec_deriv_p, 1);

        casadi::SX square = multiply_Bezier_Casadi(deriv_p, deriv_p);
        casadi::SX square1 = multiply_Bezier_Casadi(deriv_p(0, casadi::Slice()), sec_deriv_p(1, casadi::Slice()));
        casadi::SX square2 = multiply_Bezier_Casadi(deriv_p(1, casadi::Slice()), sec_deriv_p(0, casadi::Slice()));
        casadi::SX num = square1 - square2;
        casadi::SX den = square(0, casadi::Slice()) + square(1, casadi::Slice());

        return (num / den).T();
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_ang_velocity_constraints(A1, Tf / 2, nsplit - 1),
            adaptive_ang_velocity_constraints(B1, Tf / 2, nsplit - 1));
    }
}

casadi::SX BezierOptimization::adaptive_acceleration_constraints(const casadi::SX &P, const casadi::SX &Tf, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX deriv_p = bezier_derivative_Casadi(P) / Tf;
        casadi::SX sec_deriv_p = bezier_derivative_Casadi(deriv_p) / Tf;

        casadi::SX square = multiply_Bezier_Casadi(deriv_p, deriv_p);
        casadi::SX square1 = multiply_Bezier_Casadi(deriv_p(0, casadi::Slice()), sec_deriv_p(0, casadi::Slice()));
        casadi::SX square2 = multiply_Bezier_Casadi(deriv_p(1, casadi::Slice()), sec_deriv_p(1, casadi::Slice()));
        casadi::SX sums = square1 + square2;
        // std::cout << "total sums1 dimensions: " << sums.size1() << " x " << sums.size2() << std::endl;
        sums = multiply_Bezier_Casadi(sums, sums);
        // std::cout << "total sums2 dimensions: " << sums.size1() << " x " << sums.size2() << std::endl;

        casadi::SX den = square(0, casadi::Slice()) + square(1, casadi::Slice());

        int sums_size = sums.size2();
        int den_size = den.size2();

        den = elevate_degree_Casadi(den, sums_size - den_size);

        casadi::SX num = 4 * sums;

        return (num / den).T();
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_acceleration_constraints(A1, Tf / 2, nsplit - 1),
            adaptive_acceleration_constraints(B1, Tf / 2, nsplit - 1));
    }
}

casadi::SX BezierOptimization::adaptive_obstacle_constraints(const casadi::SX &P, Eigen::Vector3d obs, int nsplit)
{
    if (nsplit == 0)
    {
        casadi::SX obs_xy = casadi::SX::vertcat({casadi::SX(obs(0)), casadi::SX(obs(1))}); // 2x1 vector
        casadi::SX P_new = P - obs_xy;
        casadi::SX P_obs = multiply_Bezier_Casadi(P_new, P_new);
        casadi::SX radiuses = obs(2) * obs(2) * casadi::SX::ones(1, P_obs.size2());
        return (P_obs(0, casadi::Slice()) + P_obs(1, casadi::Slice()) - radiuses).T();
    }
    else
    {
        auto [A1, B1] = divide_bezier_Casadi(P, 0.5);
        return vertcat(
            adaptive_obstacle_constraints(A1, obs, nsplit - 1),
            adaptive_obstacle_constraints(B1, obs, nsplit - 1));
    }
}
