// benchmark_planner.cpp
//
// Computational-cost / success-rate benchmarking harness for MultipleVehiclePlanner.
//
// Goal: sweep one (or more) problem parameters while holding the rest fixed,
// run N trials per scenario point, and report:
//   - average solve time over SUCCESSFUL optimizations only
//   - worst-case (max) solve time over SUCCESSFUL optimizations only
//   - percentage of trials that failed (did not converge / infeasible / threw)
//
// This file assumes MultipleVehiclePlanner exposes (or will expose) two
// convenience accessors in addition to what's already in the header you shared:
//
//   bool   MultipleVehiclePlanner::wasSuccessful() const;
//   double MultipleVehiclePlanner::getSolveTimeMs() const;
//
// If those don't exist yet, the fallback in runSingleTrial() below shows how
// to derive success from getOptimizationstatus() and time via wall-clock
// timing around solveOptimizationProblem(). Swap in the "official" accessors
// once you've added them, since internal solver timing (excluding problem
// setup/symbolic construction) is usually what you want to report.
//
// Randomization of the actual scenario (start/goal states, obstacle
// placement, etc.) is intentionally stubbed out in RandomProblemGenerator —
// fill that in once you've decided the sampling scheme.

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <random>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <atomic>
#include <stdexcept>

#include "algorithms/multiple_vehicle_planner.hpp"  // MultipleVehiclePlanner, vehicle_state

// ---------------------------------------------------------------------------
// Scenario definition: one point in the parameter sweep
// ---------------------------------------------------------------------------
struct ScenarioParams {
    std::string label;                 // human-readable tag, e.g. "N=4"
    int bezier_degree      = 7;
    int n_vehicles          = 2;
    std::vector<int> nSplit = {0, 0, 0};   // as used by createConstraintVector etc.
    std::vector<uint8_t> constr_flags = {true, false, false, false}; // vel/acc/ang_vel/ang_acc on/off
    int n_circ_obs          = 0;
    int n_elip_obs          = 0;
    int n_line_obs          = 0;
    double conservativeness = 0.5;     // 0 = loose bounds, 1 = tight bounds
    double area_size        = 50.0;    // spatial extent for random state/obstacle placement
};

struct TrialResult {
    bool success = false;
    double solve_time_ms = 0.0;
    double tf = 0.0;
    Eigen::Tensor<double, 3> ControlPts;
    Eigen::Tensor<double, 3> opt_traj;
    std::string status;

    bool success_strategy = false;
    double solve_time_ms_strategy = 0.0;
    double tf_strategy = 0.0;
    std::string status_strategy;
};

struct AggregateResult {
    ScenarioParams params;

    int n_trials = 0;
    int n_success = 0;
    double failure_pct = 0.0;
    double avg_time_success_ms = std::nan("");
    double worst_time_success_ms = std::nan("");
    double std_time_success_ms = std::nan("");
    double tf_value = std::nan("");
    std::vector<Eigen::Tensor<double, 3>> ControlPts;
    std::vector<Eigen::Tensor<double, 3>> opt_traj;
    std::vector<int> ControlPts_trial;
    

    int n_success_strategy = 0;
    double failure_pct_strategy = 0.0;
    double avg_time_success_ms_strategy = std::nan("");
    double worst_time_success_ms_strategy = std::nan("");
    double std_time_success_ms_strategy = std::nan("");
    double tf_value_strategy = std::nan("");

    double avg_percentage_strategy = std::nan("");
    double worst_percentage_strategy = std::nan("");
    double std_percentage_strategy = std::nan("");};

// ---------------------------------------------------------------------------
// Random scenario generator (STUB — fill in your sampling scheme later)
// ---------------------------------------------------------------------------
struct RandomProblemGenerator {
    std::mt19937 rng;

    explicit RandomProblemGenerator(unsigned seed) : rng(seed) {}

    std::vector<vehicle_state> randomStates(
        int n,
        double area,
        double radius)
    {
        std::uniform_real_distribution<double> pos(
            -area / 2.0,
             area / 2.0
        );

        std::uniform_real_distribution<double> theta(-M_PI, M_PI);

        const double min_dist = 2.0 * radius;
        const double min_dist_sq = min_dist * min_dist;

        std::vector<vehicle_state> states;
        states.reserve(n);

        for (int i = 0; i < n; ++i) {

            bool valid = false;
            vehicle_state candidate;

            while (!valid) {

                candidate.x = pos(rng);
                candidate.y = pos(rng);
                candidate.theta = theta(rng);
                candidate.v = 0.02;

                valid = true;

                // Check candidate against all previously generated states
                for (const auto& s : states) {
                    const double dx = candidate.x - s.x;
                    const double dy = candidate.y - s.y;

                    const double dist_sq = dx * dx + dy * dy;

                    if (dist_sq < min_dist_sq) {
                        valid = false;
                        break;
                    }
                }
            }

            states.push_back(candidate);
        }

        return states;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> randomCircObs(int n, double area) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> obs(3, n);
        std::uniform_real_distribution<double> pos(-area / 2.0, area / 2.0);
        std::uniform_real_distribution<double> rad(1.0, 3.0);
        for (int i = 0; i < n; ++i) {
            obs(0, i) = pos(rng);
            obs(1, i) = pos(rng);
            obs(2, i) = rad(rng);
        }
        return obs;
    }

    Eigen::Matrix<double, 5, Eigen::Dynamic> randomElipObs(int n, double area) {
        Eigen::Matrix<double, 5, Eigen::Dynamic> obs(5, n);
        std::uniform_real_distribution<double> pos(-area / 2.0, area / 2.0);
        std::uniform_real_distribution<double> axis(1.0, 3.0);
        std::uniform_real_distribution<double> orient(-M_PI, M_PI);
        for (int i = 0; i < n; ++i) {
            obs(0, i) = pos(rng);
            obs(1, i) = pos(rng);
            obs(2, i) = axis(rng);        // semi-major
            obs(3, i) = axis(rng) * 0.6;  // semi-minor
            obs(4, i) = orient(rng);
        }
        return obs;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> randomLineObs(int n, double area) {
        // Placeholder: same layout as circ obs for now; replace with actual
        // line-obstacle parametrization once defined.
        return randomCircObs(n, area);
    }
};

// ---------------------------------------------------------------------------
// Single trial: build + solve one random instance of a given scenario
// ---------------------------------------------------------------------------
TrialResult runSingleTrial(const ScenarioParams &p, RandomProblemGenerator &gen) {
    TrialResult result;

    vehicle_state initial{3.66, -1.86, 1.4, 0.02};
    vehicle_state goal{0.0, 0.0, -1.4, 0.02};

    std::vector<vehicle_state> current_states{initial};
    std::vector<vehicle_state> goal_states{goal};

    auto t0 = std::chrono::high_resolution_clock::now();
    MultipleVehiclePlanner solver(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    solver.setBoundsAndGains(0.01, 1.0,
                         -5, 5,
                         -5, 5,
                         -5, 5,
                         1.5, 5000.0,
                         1.5, 1.0, 0.0, 0.0);
    

    solver.setFinalVelHeadMode(false, false);

    Eigen::MatrixXd Current(2, 2);
    Current << 0, 1,
        0, 0;
    solver.setupSymbolic(Current);
    solver.computeCostFunction();
    solver.setOptimizationProblem(current_states, goal_states);
    solver.createConstraintVector();
    solver.createDecisionVector();

    std::atomic<bool> cancel_flag{false};

    
    solver.solveOptimizationProblem(&cancel_flag);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.status = solver.getOptimizationstatus();
    result.tf = solver.getTf();
    result.ControlPts = solver.getControlPoints();
    result.opt_traj = solver.getOptimalTrajectories(2000);

    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success")   != std::string::npos ||
        result.status.find("SUCCESS")   != std::string::npos;

    return result;
}

TrialResult runSingleTrial_with_lower_order_guess(const ScenarioParams &p, RandomProblemGenerator &gen) {
    TrialResult result;


    // Map "conservativeness" in [0,1] to tighter bounds. 0 = nominal/loose,
    // 1 = fully tight. Adjust the interpolation to match what "conservative"
    // means physically for your vehicle (e.g. shrinking vel/acc envelopes,
    // growing obstacle safety margins).
    double c = p.conservativeness;
    double vel_max   = 0.3  * (1.0 - 0.5 * c);
    double acc_max   = 0.03 * (1.0 - 0.5 * c);
    double ang_vel_max = 0.15  * (1.0 - 0.5 * c);
    double ang_acc_max = 0.005 * (1.0 - 0.5 * c);
    double obs_min    = 2.0 * (1.0 + 0.5 * c);

    auto current_states = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto goal_states     = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto circ_obs = gen.randomCircObs(p.n_circ_obs, p.area_size);
    auto elip_obs = gen.randomElipObs(p.n_elip_obs, p.area_size);
    auto line_obs = gen.randomLineObs(p.n_line_obs, p.area_size);

    std::atomic<bool> cancel_flag{false};
    double Tf_guess = -std::numeric_limits<double>::infinity();
    Eigen::Tensor<double, 3> all_control_points;

    auto t0 = std::chrono::high_resolution_clock::now();
    MultipleVehiclePlanner solver_guess(5, p.n_vehicles, p.nSplit, p.constr_flags);
    solver_guess.setupSymbolic();
    solver_guess.computeCostFunction();
    solver_guess.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs);
    solver_guess.createConstraintVector();
    solver_guess.createDecisionVector();
    solver_guess.solveOptimizationProblem(&cancel_flag);

    Eigen::Tensor<double, 3> cp_raw = solver_guess.getControlPoints();

    int dim1 = cp_raw.dimension(1); // should be 2
    int dim2 = cp_raw.dimension(2); // N control points


    all_control_points = Eigen::Tensor<double, 3>(p.n_vehicles, dim1, dim2);


    for (int d = 0; d < dim1; ++d) {
        for (int n = 0; n < dim2; ++n) {
            all_control_points(0, d, n) = cp_raw(0, d, n);
        }
    }

    Tf_guess = solver_guess.getTf();

    Eigen::Tensor<double, 3> control_points_guess = BezierUtils::elevateTensorDegree(all_control_points, p.bezier_degree- 5);

    MultipleVehiclePlanner solver_strategy(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    solver_strategy.setupSymbolic();
    solver_strategy.computeCostFunction();
    solver_strategy.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs, control_points_guess, Tf_guess, false);
    solver_strategy.createConstraintVector();
    solver_strategy.createDecisionVector();
    
    solver_strategy.solveOptimizationProblem(&cancel_flag);

    auto t1 = std::chrono::high_resolution_clock::now();


    result.solve_time_ms_strategy = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.status_strategy = solver_strategy.getOptimizationstatus();
    // --- Fallback success criterion (adapt once wasSuccessful() exists) ---
    // CasADi/IPOPT typically report something like "Solve_Succeeded".
    // If you switch solvers (qrqp/SQP), adjust the matched substrings.
    result.success_strategy =
        result.status_strategy.find("Succeeded") != std::string::npos ||
        result.status_strategy.find("success")   != std::string::npos ||
        result.status_strategy.find("SUCCESS")   != std::string::npos;

    

    t0 = std::chrono::high_resolution_clock::now();
    MultipleVehiclePlanner solver(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    solver.setupSymbolic();
    solver.computeCostFunction();
    solver.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs);
    solver.createConstraintVector();
    solver.createDecisionVector();

    

    
    solver.solveOptimizationProblem(&cancel_flag);
    t1 = std::chrono::high_resolution_clock::now();

    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.status = solver.getOptimizationstatus();

    // --- Fallback success criterion (adapt once wasSuccessful() exists) ---
    // CasADi/IPOPT typically report something like "Solve_Succeeded".
    // If you switch solvers (qrqp/SQP), adjust the matched substrings.
    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success")   != std::string::npos ||
        result.status.find("SUCCESS")   != std::string::npos;

    return result;
}

// ---------------------------------------------------------------------------
// Single trial: build + solve one random instance of a given scenario
// ---------------------------------------------------------------------------
TrialResult runSingleTrial_with_Collision(const ScenarioParams &p, RandomProblemGenerator &gen) {
    TrialResult result;

    auto current_states = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto goal_states     = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto circ_obs = gen.randomCircObs(p.n_circ_obs, p.area_size);
    auto elip_obs = gen.randomElipObs(p.n_elip_obs, p.area_size);
    auto line_obs = gen.randomLineObs(p.n_line_obs, p.area_size);

    MultipleVehiclePlanner solver_guess(p.bezier_degree, 1, p.nSplit, p.constr_flags);
    
    double Tf_guess = -std::numeric_limits<double>::infinity();
    Eigen::Tensor<double, 3> all_control_points;
    std::vector<double> Tf_values(p.n_vehicles, 0.0);
    std::vector<bool> success_first(p.n_vehicles, false);

    

    auto t0 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < p.n_vehicles; i++)
    {
        solver_guess.setupSymbolic();
        solver_guess.computeCostFunction();
        
        std::vector<vehicle_state> current_state_vec;
        std::vector<vehicle_state> goal_state_vec;

        current_state_vec.push_back(current_states[i]);
        goal_state_vec.push_back(goal_states[i]);
        solver_guess.setOptimizationProblem(current_state_vec, goal_state_vec, circ_obs, line_obs, elip_obs);
        
        solver_guess.createConstraintVector();
        solver_guess.createDecisionVector();

        std::atomic<bool> cancel_flag{false};

        solver_guess.solveOptimizationProblem(&cancel_flag);


        result.status_strategy = solver_guess.getOptimizationstatus();
        success_first[i] =
            result.status_strategy.find("Succeeded") != std::string::npos ||
            result.status_strategy.find("success")   != std::string::npos ||
            result.status_strategy.find("SUCCESS")   != std::string::npos;
        


        Eigen::Tensor<double, 3> cp_raw = solver_guess.getControlPoints();

        int dim1 = cp_raw.dimension(1); // should be 2
        int dim2 = cp_raw.dimension(2); // N control points

        if (i == 0) {
            all_control_points = Eigen::Tensor<double, 3>(p.n_vehicles, dim1, dim2);
        }

        for (int d = 0; d < dim1; ++d) {
            for (int n = 0; n < dim2; ++n) {
                all_control_points(i, d, n) = cp_raw(0, d, n);
            }
        }
        Tf_values[i] = solver_guess.getTf();
        Tf_guess = std::max(Tf_guess, Tf_values[i]);
        
        
    }

    std::vector<std::pair<int,int>> collisions;
    bool collision_detected = false;

    for (int i = 0; i < p.n_vehicles; ++i) {
        for (int j = i + 1; j < p.n_vehicles; ++j) {
            Eigen::MatrixXd P1 = BezierUtils::tensor_2_matrix(all_control_points, i);
            Eigen::MatrixXd P2 = BezierUtils::tensor_2_matrix(all_control_points, j);

            if (!BezierUtils::check_vehicle_distances(P1, P2, 1.5, 1)) {
                collisions.emplace_back(i, j);
                collision_detected = true;
            }
        }
    }

    double strategy_tf{0.0};
    if (collision_detected) {
        std::vector<std::pair<int,int>> sorted_collisions = collisions;

        std::sort(sorted_collisions.begin(), sorted_collisions.end(),
            [&](auto& a, auto& b) {
                double scoreA = Tf_values[a.first] + Tf_values[a.second];
                double scoreB = Tf_values[b.first] + Tf_values[b.second];
                return scoreA > scoreB; // prioritize high Tf conflicts
            });

        std::vector<int> to_remove = BezierUtils::selectTrajectoriesToRemove(p.n_vehicles, sorted_collisions, Tf_values);
        
        double Tf_min = 0.0;
        for (int i = 0; i < p.n_vehicles; ++i) {
            if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end()) {
                std::cout << "There are " << p.n_vehicles << ", and vehicle " << i << " must be removed" << std::endl;
                Tf_min = std::max(Tf_min, Tf_values[i]);
            } 
        }

        std::cout << "###############################################################" << std::endl;
        std::cout << "###############################################################" << std::endl;
        std::cout << "#                                                             #" << std::endl;
        std::cout << "# Now starting the new optimization without good trajectories #" << std::endl;
        std::cout << "#                                                             #" << std::endl;
        std::cout << "###############################################################" << std::endl;
        std::cout << "###############################################################" << std::endl;
        std::cout << "                                                           " << std::endl;
        
        MultipleVehiclePlanner solver_collision(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
        solver_collision.setupSymbolic();
        solver_collision.computeCostFunction();
        
        solver_collision.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs, all_control_points, Tf_guess, false);
        solver_collision.createConstraintVector(to_remove);
        std::cout << "##############                            ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############      PASSED CONSTR         ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############                            ###############" << std::endl;
        solver_collision.createDecisionVector(to_remove, Tf_min);
        std::cout << "##############                            ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############      PASSED DECIS         ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############                            ###############" << std::endl;
        
        std::atomic<bool> cancel_flag{false};
        solver_collision.solveOptimizationProblem(&cancel_flag);
        std::cout << "##############                            ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############      PASSED SOLVER         ###############" << std::endl;
        std::cout << "              ############################               " << std::endl;
        std::cout << "##############                            ###############" << std::endl;

        strategy_tf = solver_collision.getTf();
        result.status_strategy = solver_collision.getOptimizationstatus();
        result.success_strategy =
            result.status_strategy.find("Succeeded") != std::string::npos ||
            result.status_strategy.find("success")   != std::string::npos ||
            result.status_strategy.find("SUCCESS")   != std::string::npos;

    }
    else
    {
        result.success_strategy = true;
        for (int i = 0; i < p.n_vehicles; ++i) {
            strategy_tf = std::max(strategy_tf, Tf_values[i]);
            
            if(!success_first[i]){
                result.success_strategy = false;
            }
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    result.solve_time_ms_strategy = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.tf_strategy = strategy_tf;

    t0 = std::chrono::high_resolution_clock::now();

    MultipleVehiclePlanner solver(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);

    solver.setupSymbolic();
    solver.computeCostFunction();

    solver.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs);
    solver.createConstraintVector();
    solver.createDecisionVector();

    std::atomic<bool> cancel_flag{false};

    solver.solveOptimizationProblem(&cancel_flag);
    t1 = std::chrono::high_resolution_clock::now();

    result.tf = solver.getTf();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.status = solver.getOptimizationstatus();

    // --- Fallback success criterion (adapt once wasSuccessful() exists) ---
    // CasADi/IPOPT typically report something like "Solve_Succeeded".
    // If you switch solvers (qrqp/SQP), adjust the matched substrings.
    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success")   != std::string::npos ||
        result.status.find("SUCCESS")   != std::string::npos;

    return result;
}


// ---------------------------------------------------------------------------
// Run one scenario across n_trials random instances, aggregate stats
// ---------------------------------------------------------------------------
AggregateResult runScenario(const ScenarioParams &p, int n_trials, unsigned seed_base) {
    AggregateResult agg;
    agg.params = p;
    agg.n_trials = n_trials;

    std::vector<double> success_times;
    std::vector<double> tf_times;
    std::vector<Eigen::Tensor<double, 3>> ControlPts;
    std::vector<Eigen::Tensor<double, 3>> opt_traj;
    std::vector<int> ControlPts_trial;
    success_times.reserve(n_trials);
    int n_success = 0;

    std::vector<double> success_times_strategy;
    std::vector<double> tf_times_strategy;
    success_times_strategy.reserve(n_trials);
    int n_success_strategy = 0;

    std::vector<double> prc_maneuver_times;
    prc_maneuver_times.reserve(n_trials);

    for (int i = 0; i < n_trials; ++i) {
        RandomProblemGenerator gen(seed_base + static_cast<unsigned>(i));
        try {
            TrialResult r = runSingleTrial(p, gen);
            if (r.success) {
                ++n_success;
                success_times.push_back(r.solve_time_ms);
                tf_times.push_back(r.tf);
                ControlPts.push_back(r.ControlPts);
                opt_traj.push_back(r.opt_traj);
                ControlPts_trial.push_back(i);
            }
            if (r.success_strategy) {
                ++n_success_strategy;
                success_times_strategy.push_back(r.solve_time_ms_strategy);      
                tf_times_strategy.push_back(r.tf_strategy);      
            }
            if (r.success && r.success_strategy) {
                prc_maneuver_times.push_back((r.tf_strategy-r.tf)/r.tf);
            }

        } catch (const std::exception &e) {
            // Any thrown exception (e.g. infeasible NLP setup, solver crash,
            // cancellation) counts as a failed trial.
            std::cerr << "[" << p.label << "] trial " << i
                      << " threw: " << e.what() << "\n";
        }
    }

    agg.n_success = n_success;
    agg.failure_pct = 100.0 * (1.0 - static_cast<double>(n_success) / n_trials);

    agg.n_success_strategy = n_success_strategy;
    agg.failure_pct_strategy = 100.0 * (1.0 - static_cast<double>(n_success_strategy) / n_trials);
    agg.ControlPts = ControlPts;
    agg.opt_traj = opt_traj;
    agg.ControlPts_trial = ControlPts_trial;
    if (!success_times.empty()) {
        double sum = std::accumulate(success_times.begin(), success_times.end(), 0.0);
        double mean = sum / success_times.size();

        double sq_sum = 0.0;
        for (double t : success_times) sq_sum += (t - mean) * (t - mean);
        double stddev = success_times.size() > 1
            ? std::sqrt(sq_sum / (success_times.size() - 1))
            : 0.0;

        agg.avg_time_success_ms = mean;
        agg.worst_time_success_ms = *std::max_element(success_times.begin(), success_times.end());
        agg.std_time_success_ms = stddev;

        agg.tf_value = std::accumulate(tf_times.begin(), tf_times.end(), 0.0) / tf_times.size();
    }

    if (!success_times_strategy.empty()) {
        double sum = std::accumulate(success_times_strategy.begin(), success_times_strategy.end(), 0.0);
        double mean = sum / success_times_strategy.size();

        double sq_sum = 0.0;
        for (double t : success_times_strategy) sq_sum += (t - mean) * (t - mean);
        double stddev = success_times_strategy.size() > 1
            ? std::sqrt(sq_sum / (success_times_strategy.size() - 1))
            : 0.0;

        agg.avg_time_success_ms_strategy = mean;
        agg.worst_time_success_ms_strategy = *std::max_element(success_times_strategy.begin(), success_times_strategy.end());
        agg.std_time_success_ms_strategy = stddev;

        agg.tf_value_strategy = std::accumulate(tf_times_strategy.begin(), tf_times_strategy.end(), 0.0) / tf_times_strategy.size();

    }

    if (!prc_maneuver_times.empty()) {
        double sum = std::accumulate(prc_maneuver_times.begin(), prc_maneuver_times.end(), 0.0);
        double mean = sum / prc_maneuver_times.size();

        double sq_sum = 0.0;
        for (double t : prc_maneuver_times) sq_sum += (t - mean) * (t - mean);
        double stddev = prc_maneuver_times.size() > 1
            ? std::sqrt(sq_sum / (prc_maneuver_times.size() - 1))
            : 0.0;

        agg.avg_percentage_strategy = mean;
        agg.worst_percentage_strategy = *std::max_element(prc_maneuver_times.begin(), prc_maneuver_times.end());
        agg.std_percentage_strategy = stddev;
    }

    return agg;
}

// ---------------------------------------------------------------------------
// Scenario sweep builders — one function per axis you want to study.
// These vary ONE parameter at a time while holding a baseline fixed, which
// is what you want for clean per-axis plots in the chapter.
// ---------------------------------------------------------------------------
ScenarioParams baseline() {
    ScenarioParams p;
    p.label = "baseline";
    p.bezier_degree = 7;
    p.n_vehicles = 2;
    p.n_circ_obs = 2;
    p.n_elip_obs = 0;
    p.n_line_obs = 0;
    p.conservativeness = 0.3;
    return p;
}

std::vector<ScenarioParams> sweepVehicles(const std::vector<int> &values) {
    std::vector<ScenarioParams> out;
    for (int n : values) {
        ScenarioParams p = baseline();
        p.n_vehicles = n;
        p.label = "n_vehicles=" + std::to_string(n);
        out.push_back(p);
    }
    return out;
}

std::vector<ScenarioParams> sweepObstacles(const std::vector<int> &values) {
    std::vector<ScenarioParams> out;
    for (int n : values) {
        ScenarioParams p = baseline();
        p.n_circ_obs = n;
        p.label = "n_circ_obs=" + std::to_string(n);
        out.push_back(p);
    }
    return out;
}

std::vector<ScenarioParams> sweepDegree(const std::vector<int> &values) {
    std::vector<ScenarioParams> out;
    for (int d : values) {
        ScenarioParams p = baseline();
        p.bezier_degree = d;
        p.label = "bezier_degree=" + std::to_string(d);
        out.push_back(p);
    }
    return out;
}

std::vector<ScenarioParams> sweepConservativeness(const std::vector<double> &values) {
    std::vector<ScenarioParams> out;
    for (double c : values) {
        ScenarioParams p = baseline();
        p.conservativeness = c;
        std::ostringstream oss;
        oss << "conservativeness=" << c;
        p.label = oss.str();
        out.push_back(p);
    }
    return out;
}

// --- Plot 1: Bezier degree vs. time, 1 vehicle, 0 obstacles -----------------
std::vector<ScenarioParams> sweepDegreeSingleVehicleNoObstacles(const int start, const int end) {
    std::vector<ScenarioParams> out;
    for (int d = start; d <= end; d++) {
        ScenarioParams p;
        p.bezier_degree = d;
        p.n_vehicles = 1;
        p.n_circ_obs = 0;
        p.n_elip_obs = 0;
        p.n_line_obs = 0;
        p.conservativeness = 1.0;
        p.label = "bezier_degree=" + std::to_string(d);
        out.push_back(p);
    }
    return out;
}

// --- Plot 2: N vehicles vs. time, Bezier degree fixed at 9, 0 obstacles -----
std::vector<ScenarioParams> sweepVehiclesFixedDegree(const std::vector<int> &values, int fixed_degree) {
    std::vector<ScenarioParams> out;
    for (int n : values) {
        ScenarioParams p;
        p.bezier_degree = fixed_degree;
        p.n_vehicles = n;
        p.n_circ_obs = 0;
        p.n_elip_obs = 0;
        p.n_line_obs = 0;
        p.conservativeness = 0.3;
        p.label = "n_vehicles=" + std::to_string(n);
        out.push_back(p);
    }
    return out;
}

// ---------------------------------------------------------------------------
// CSV writer
// ---------------------------------------------------------------------------
void writeCsv(const std::vector<AggregateResult> &results, const std::string &path) {
    std::ofstream csv(path);
    csv << "label,n_vehicles,bezier_degree,n_circ_obs,n_elip_obs,n_line_obs,"
           "conservativeness,n_trials,"
           "n_success,failure_pct,avg_time_success_ms,worst_time_success_ms,std_time_success_ms,"
           "n_success_strategy,failure_pct_strategy,avg_time_success_ms_strategy,"
           "worst_time_success_ms_strategy,std_time_success_ms_strategy,"
           "avg_percentage_strategy,worst_percentage_strategy,std_percentage_strategy\n";

    for (const auto &r : results) {
        csv << r.params.label << ","
            << r.params.n_vehicles << ","
            << r.params.bezier_degree << ","
            << r.params.n_circ_obs << ","
            << r.params.n_elip_obs << ","
            << r.params.n_line_obs << ","
            << r.params.conservativeness << ","
            << r.n_trials << ","
            << r.n_success << ","
            << r.failure_pct << ","
            << r.avg_time_success_ms << ","
            << r.worst_time_success_ms << ","
            << r.std_time_success_ms << ","
            << r.n_success_strategy << ","
            << r.failure_pct_strategy << ","
            << r.avg_time_success_ms_strategy << ","
            << r.worst_time_success_ms_strategy << ","
            << r.std_time_success_ms_strategy << ","
            << r.avg_percentage_strategy << ","
            << r.worst_percentage_strategy << ","
            << r.std_percentage_strategy << "\n";
    }
    std::cout << "Wrote results to " << path << "\n";
}

// ---------------------------------------------------------------------------
// CSV writer
// ---------------------------------------------------------------------------
void writeCsv_simple(const std::vector<AggregateResult> &results, const std::string &path) {
    std::ofstream csv(path);
    csv << "label,bezier_degree,n_trials,n_success,failure_pct,tf_value,"
           "avg_time_success_ms,worst_time_success_ms,std_time_success_ms\n";

    for (const auto &r : results) {
        csv << r.params.label << ","
            << r.params.bezier_degree << ","
            << r.n_trials << ","
            << r.n_success << ","
            << r.failure_pct << ","
            << r.tf_value << ","
            << r.avg_time_success_ms << ","
            << r.worst_time_success_ms << ","
            << r.std_time_success_ms << "\n";
    }
    std::cout << "Wrote results to " << path << "\n";
}

void writeCsv_controlPts(const std::vector<AggregateResult>& results, const std::string& path)
{
    std::ofstream csv(path);

    if (!csv.is_open()) {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,control_point,x,y,tf\n";

    for (const auto& r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.ControlPts.size();
             ++trial_idx)
        {
            const auto& tensor = r.ControlPts[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_control_points =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2) {
                throw std::runtime_error(
                    "Expected control-point tensor dimension 1 to be 2.");
            }

            for (int k = 0; k < n_vehicles; ++k)
            {
                for (int i = 0; i < n_control_points; ++i)
                {
                    double x = tensor(k, 0, i);
                    double y = tensor(k, 1, i);

                    csv << r.params.label << ","
                        << r.params.bezier_degree << ","
                        << trial << ","
                        << k << ","
                        << i << ","
                        << x << ","
                        << y << ","
                        << r.tf_value << "\n";
                }
            }
        }
    }

    std::cout << "Wrote control points to " << path << "\n";
}

void writeCsv_trajectories(const std::vector<AggregateResult>& results, const std::string& path)
{
    std::ofstream csv(path);

    if (!csv.is_open()) {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,point_idx,x,y,tf\n";

    for (const auto& r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.opt_traj.size();
             ++trial_idx)
        {
            const auto& tensor = r.opt_traj[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_pts =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2) {
                throw std::runtime_error(
                    "Expected trajectory tensor dimension 1 to be 2.");
            }

            for (int k = 0; k < n_vehicles; ++k)
            {
                for (int i = 0; i < n_pts; ++i)
                {
                    double x = tensor(k, 0, i);
                    double y = tensor(k, 1, i);

                    csv << r.params.label << ","
                        << r.params.bezier_degree << ","
                        << trial << ","
                        << k << ","
                        << i << ","
                        << x << ","
                        << y << ","
                        << r.tf_value << "\n";
                }
            }
        }
    }

    std::cout << "Wrote trajectories points to " << path << "\n";
}

std::string todayDateString() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
    return std::string(buf);
}

// ---------------------------------------------------------------------------
// main: run each axis sweep as its own CSV, plus print a quick summary
// ---------------------------------------------------------------------------
int main() {
    const int N_TRIALS = 1;      // trials per scenario point, as requested
    const unsigned SEED_BASE = 10;
    const std::string date_tag = todayDateString();
    struct Sweep {
        std::string name;
        std::vector<ScenarioParams> scenarios;
    };
 
    std::vector<Sweep> sweeps = {
        // Plot 1: degree 5..15, 1 vehicle, 0 obstacles
        {"degree_1veh_0obs", sweepDegreeSingleVehicleNoObstacles(3,45)}
 
        // // Plot 2: n_vehicles 1..N, Bezier degree fixed at 9, 0 obstacles
        // {"vehicles_deg9_0obs", sweepVehiclesFixedDegree(
        //     {9}, /*fixed_degree=*/9)},

    };
 
    // Other sweeps (obstacles, conservativeness) are still available via
    // sweepObstacles()/sweepConservativeness() 

    for (const auto &sweep : sweeps) {
        std::vector<AggregateResult> results;
        results.reserve(sweep.scenarios.size());
 
        for (const auto &p : sweep.scenarios) {
            std::cout << "Running scenario: " << p.label
                      << " (" << N_TRIALS << " trials)...\n";
            AggregateResult agg = runScenario(p, N_TRIALS, SEED_BASE);
            std::cout << "  success=" << agg.n_success << "/" << agg.n_trials
                      << " (fail=" << agg.failure_pct << "%)"
                      << " avg=" << agg.avg_time_success_ms << " ms"
                      << " worst=" << agg.worst_time_success_ms << " ms\n";
            results.push_back(agg);
        }
 
        writeCsv(results, "src/farol2/farol2_motion_planning/test/results/benchmark_" + date_tag + "_original_plot_" + sweep.name + ".csv");
        writeCsv_simple(results, "src/farol2/farol2_motion_planning/test/results/benchmark_" + date_tag + "_simple.csv");
        writeCsv_controlPts(results, "src/farol2/farol2_motion_planning/test/results/benchmark_" +  date_tag + "_controlPts.csv");
        writeCsv_trajectories(results, "src/farol2/farol2_motion_planning/test/results/benchmark_" +  date_tag + "_trajectories.csv");
    }
 
    return 0;
}
