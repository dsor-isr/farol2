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
#include <filesystem>
#include <iomanip>
#include <memory>

#include "algorithms/multiple_vehicle_planner.hpp" // MultipleVehiclePlanner, vehicle_state

// Planner bounds and gains. Keep the generator's speed limits tied to these.
static constexpr double PLANNER_VEL_MIN = 0.001;
static constexpr double PLANNER_VEL_MAX = 1.0;
static constexpr double PLANNER_ACC_MIN = -0.2;
static constexpr double PLANNER_ACC_MAX = 0.2;
static constexpr double PLANNER_ANG_VEL_MIN = -0.4;
static constexpr double PLANNER_ANG_VEL_MAX = 0.4;
static constexpr double PLANNER_ANG_ACC_MIN = -0.1;
static constexpr double PLANNER_ANG_ACC_MAX = 0.1;
static constexpr double PLANNER_OBS_MIN = 1.0;
static constexpr double PLANNER_OBS_MAX = 50000.0;
static constexpr double PLANNER_RADIUS = 2.0;
static constexpr double PLANNER_ALPHA = 1.0;
static constexpr double PLANNER_BETA = 1.0;
static constexpr double PLANNER_GAMMA = 1.0;

// -----------------------------------------------------------------------------------
// Scenario definition: one point in the parameter sweep
// -----------------------------------------------------------------------------------
struct ScenarioParams
{
    std::string label; // human-readable tag, e.g. "N=4"
    int bezier_degree = 6;
    int n_vehicles = 2;
    std::vector<int> nSplit = {2, 1, 2};                           // as used by createConstraintVector etc.
    std::vector<uint8_t> constr_flags = {true, false, false, false}; // vel/acc/ang_vel/ang_acc on/off
    int n_circ_obs = 0;
    int n_elip_obs = 0;
    int n_line_obs = 0;
    double conservativeness = 0.5; // 0 = loose bounds, 1 = tight bounds
    double area_size = 150.0;      // spatial extent for random state/obstacle placement
};

struct TrialResult
{
    bool success = false;
    double solve_time_ms = 0.0;
    double tf = 0.0;
    Eigen::Tensor<double, 3> ControlPts;
    Eigen::Tensor<double, 3> opt_traj;
    std::string status;

    bool success_strategy = false;
    double solve_time_ms_strategy = 0.0;
    double tf_strategy = 0.0;
    Eigen::Tensor<double, 3> ControlPts_strategy;
    Eigen::Tensor<double, 3> opt_traj_strategy;
    std::string status_strategy;
};

struct BoxStats
{
    double q1 = std::nan("");
    double median = std::nan("");
    double q3 = std::nan("");
    double whisker_low = std::nan("");  // smallest value >= Q1 - 1.5*IQR
    double whisker_high = std::nan(""); // largest value  <= Q3 + 1.5*IQR
    std::vector<double> outliers;
};

struct AggregateResult
{
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

    BoxStats box_time_success;
    BoxStats box_time_success_strategy;
    BoxStats box_percentage_strategy;

    int n_success_strategy = 0;
    double failure_pct_strategy = 0.0;
    double avg_time_success_ms_strategy = std::nan("");
    double worst_time_success_ms_strategy = std::nan("");
    double std_time_success_ms_strategy = std::nan("");
    double tf_value_strategy = std::nan("");
    std::vector<Eigen::Tensor<double, 3>> ControlPts_strategy;
    std::vector<Eigen::Tensor<double, 3>> opt_traj_strategy;
    std::vector<int> ControlPts_trial_strategy;

    double avg_percentage_strategy = std::nan("");
    double worst_percentage_strategy = std::nan("");
    double std_percentage_strategy = std::nan("");
};

enum class TrialMode
{
    Standard,
    LowerOrderGuess,
    Collision
};

struct ScenarioPlanners
{
    std::unique_ptr<MultipleVehiclePlanner> single_vehicle;
    std::unique_ptr<MultipleVehiclePlanner> full_size;
    std::unique_ptr<MultipleVehiclePlanner> lower_order;

    explicit ScenarioPlanners(const ScenarioParams& p, TrialMode mode)
    {
        auto prepare = [](auto& planner)
        {
            planner->setBoundsAndGains(PLANNER_VEL_MIN, PLANNER_VEL_MAX,
                                       PLANNER_ACC_MIN, PLANNER_ACC_MAX,
                                       PLANNER_ANG_VEL_MIN, PLANNER_ANG_VEL_MAX,
                                       PLANNER_ANG_ACC_MIN, PLANNER_ANG_ACC_MAX,
                                       PLANNER_OBS_MIN, PLANNER_OBS_MAX,
                                       PLANNER_RADIUS, PLANNER_ALPHA, PLANNER_BETA, PLANNER_GAMMA);
            planner->setupSymbolic();
            planner->computeCostFunction();
        };

        switch (mode)
        {
        case TrialMode::Standard:
            full_size = std::make_unique<MultipleVehiclePlanner>(
                p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
            prepare(full_size);
            break;

        case TrialMode::LowerOrderGuess:
            lower_order = std::make_unique<MultipleVehiclePlanner>(
                5, p.n_vehicles, p.nSplit, p.constr_flags);
            full_size = std::make_unique<MultipleVehiclePlanner>(
                p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
            prepare(lower_order);
            prepare(full_size);
            break;

        case TrialMode::Collision:
            single_vehicle = std::make_unique<MultipleVehiclePlanner>(
                p.bezier_degree, 1, p.nSplit, p.constr_flags);
            full_size = std::make_unique<MultipleVehiclePlanner>(
                p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
            prepare(single_vehicle);
            prepare(full_size);
            break;
        }
    }
};

struct RunOptions
{
    TrialMode mode = TrialMode::Collision;
    int trials = 30;
    unsigned seed = 1099;
    std::filesystem::path output_dir =
        "src/farol2/farol2_motion_planning/test/results";
};

static TrialMode parseTrialMode(const std::string &value)
{
    if (value == "standard")
        return TrialMode::Standard;
    if (value == "lower-order")
        return TrialMode::LowerOrderGuess;
    if (value == "collision")
        return TrialMode::Collision;

    throw std::invalid_argument(
        "Unknown mode '" + value +
        "'. Use: standard, lower-order, collision.");
}

static std::string trialModeName(TrialMode mode)
{
    switch (mode)
    {
    case TrialMode::Standard:
        return "standard";
    case TrialMode::LowerOrderGuess:
        return "lower-order";
    case TrialMode::Collision:
        return "collision";
    }
    throw std::invalid_argument("Invalid trial mode.");
}

static RunOptions parseRunOptions(int argc, char **argv)
{
    RunOptions options;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];

        auto nextValue = [&](const std::string &option) -> std::string
        {
            if (i + 1 >= argc)
            {
                throw std::invalid_argument("Missing value for " + option);
            }
            return argv[++i];
        };

        if (arg == "--mode")
        {
            options.mode = parseTrialMode(nextValue(arg));
        }
        else if (arg == "--trials")
        {
            options.trials = std::stoi(nextValue(arg));
            if (options.trials <= 0)
            {
                throw std::invalid_argument("--trials must be greater than zero.");
            }
        }
        else if (arg == "--seed")
        {
            options.seed = static_cast<unsigned>(
                std::stoul(nextValue(arg)));
        }
        else if (arg == "--output-dir")
        {
            options.output_dir = nextValue(arg);
        }
        else if (arg == "--help")
        {
            std::cout
                << "Options:\n"
                << "  --mode standard|lower-order|collision\n"
                << "  --trials N\n"
                << "  --seed N\n"
                << "  --output-dir PATH\n";
            std::exit(0);
        }
        else
        {
            throw std::invalid_argument("Unknown option: " + arg);
        }
    }

    return options;
}

struct SpreadStats
{
    double mean{0.0}, median{0.0};
    double p16{0.0}, p84{0.0};         // empirical band
    double gmean{0.0};                 // geometric mean
    double g_lower{0.0}, g_upper{0.0}; // exp(mu -/+ s_log)
    double worst{0.0};
};

// -----------------------------------------------------------------------------------
// Scenario sweep builders — one function per axis you want to study.
// These vary ONE parameter at a time while holding a baseline fixed, which
// is what you want for clean per-axis plots in the chapter.
// -----------------------------------------------------------------------------------
ScenarioParams baseline()
{
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

std::vector<ScenarioParams> sweepDegreeSingleVehicleNoObstacles(const int start, const int end)
{
    std::vector<ScenarioParams> out;
    for (int d = start; d <= end; d++)
    {
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

std::vector<ScenarioParams> sweepNsplit(const int start, const int end)
{
    std::vector<ScenarioParams> out;
    for (int d = start; d <= end; d++)
    {
        ScenarioParams p;
        p.bezier_degree = 6;
        p.n_vehicles = 5;
        p.n_circ_obs = 0;
        p.n_elip_obs = 0;
        p.n_line_obs = 0;
        p.conservativeness = d;
        p.label = "nsplit=" + std::to_string(d);
        out.push_back(p);
    }
    return out;
}

std::vector<ScenarioParams> sweepVehiclesFixedDegree(const std::vector<int> &values, int fixed_degree)
{
    std::vector<ScenarioParams> out;
    for (int n : values)
    {
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

// -----------------------------------------------------------------------------------
// Other stuff for statistics
// -----------------------------------------------------------------------------------
static double percentileSorted(const std::vector<double> &v, double q)
{
    if (v.size() == 1)
        return v.front();
    const double pos = q * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    return v[lo] + (pos - lo) * (v[hi] - v[lo]);
}

static BoxStats computeBox(std::vector<double> v)
{
    BoxStats b;
    if (v.empty())
        return b;
    std::sort(v.begin(), v.end());

    b.q1 = percentileSorted(v, 0.25);
    b.median = percentileSorted(v, 0.50);
    b.q3 = percentileSorted(v, 0.75);

    const double iqr = b.q3 - b.q1;
    const double lo_fence = b.q1 - 1.5 * iqr;
    const double hi_fence = b.q3 + 1.5 * iqr;

    b.whisker_low = v.back();
    b.whisker_high = v.front();
    for (double t : v)
    {
        if (t < lo_fence || t > hi_fence)
        {
            b.outliers.push_back(t);
            continue;
        }
        b.whisker_low = std::min(b.whisker_low, t);
        b.whisker_high = std::max(b.whisker_high, t);
    }
    return b;
}

static SpreadStats computeSpread(std::vector<double> v, bool positive_only = true)
{
    SpreadStats s;
    if (v.empty())
        return s;

    std::sort(v.begin(), v.end());
    s.mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    s.median = percentileSorted(v, 0.5);
    s.p16 = percentileSorted(v, 0.16);
    s.p84 = percentileSorted(v, 0.84);
    s.worst = v.back();

    if (positive_only && v.front() > 0.0)
    {
        double mu = 0.0;
        for (double t : v)
            mu += std::log(t);
        mu /= v.size();

        double sq = 0.0;
        for (double t : v)
        {
            const double d = std::log(t) - mu;
            sq += d * d;
        }
        const double s_log = v.size() > 1 ? std::sqrt(sq / (v.size() - 1)) : 0.0;

        s.gmean = std::exp(mu);
        s.g_lower = std::exp(mu - s_log);
        s.g_upper = std::exp(mu + s_log);
    }
    return s;
}

void printBanner(const std::string &msg)
{
    const std::size_t W = 63;
    const std::size_t inner = W - 2;
    std::string text = msg.substr(0, inner);
    std::size_t left = (inner - text.size()) / 2;
    std::size_t right = inner - text.size() - left;
    std::string border(W, '#');
    std::string blank = "#" + std::string(inner, ' ') + "#";

    std::cout << border << "\n"
              << border << "\n"
              << blank << "\n"
              << "#" << std::string(left, ' ') << text << std::string(right, ' ') << "#\n"
              << blank << "\n"
              << border << "\n"
              << border << "\n"
              << std::endl;
}

static void printStates(const std::string& label, const std::vector<vehicle_state>& states)
{
    std::cout << label << ":\n" << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < states.size(); ++i)
        std::cout << "  vehicle " << i << ": x=" << states[i].x << ", y=" << states[i].y
                  << ", theta=" << states[i].theta << ", v=" << states[i].v << '\n';
}
// -----------------------------------------------------------------------------------
// CSV writers
// -----------------------------------------------------------------------------------
void writeCsv(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);

    auto boxHeader = [](const std::string &p)
    {
        return "," + p + "_q1," + p + "_median," + p + "_q3," +
               p + "_whisker_low," + p + "_whisker_high," + p + "_outliers";
    };

    auto writeBox = [&csv](const BoxStats &b)
    {
        std::ostringstream out;
        for (size_t i = 0; i < b.outliers.size(); ++i)
            out << (i ? ";" : "") << b.outliers[i];
        csv << "," << b.q1 << "," << b.median << "," << b.q3
            << "," << b.whisker_low << "," << b.whisker_high
            << "," << out.str();
    };

    csv << "label,n_vehicles,bezier_degree,n_circ_obs,n_elip_obs,n_line_obs,"
           "conservativeness,n_trials,"
           "n_success,failure_pct,avg_time_success_ms,worst_time_success_ms,std_time_success_ms,"
           "n_success_strategy,failure_pct_strategy,avg_time_success_ms_strategy,"
           "worst_time_success_ms_strategy,std_time_success_ms_strategy,"
           "avg_percentage_strategy,worst_percentage_strategy,std_percentage_strategy"
        << boxHeader("box_time")
        << boxHeader("box_time_strategy")
        << boxHeader("box_percentage")
        << "\n";

    for (const auto &r : results)
    {
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
            << r.std_percentage_strategy;

        writeBox(r.box_time_success);
        writeBox(r.box_time_success_strategy);
        writeBox(r.box_percentage_strategy);
        csv << "\n";
    }
    std::cout << "Wrote results to " << path << "\n";
}

void writeCsv_simple(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);
    csv << "label,bezier_degree,n_trials,n_success,failure_pct,tf_value,"
           "avg_time_success_ms,worst_time_success_ms,std_time_success_ms\n";

    for (const auto &r : results)
    {
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

void writeCsv_controlPts(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);

    if (!csv.is_open())
    {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,control_point,x,y,tf\n";

    for (const auto &r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.ControlPts.size();
             ++trial_idx)
        {
            const auto &tensor = r.ControlPts[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_control_points =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2)
            {
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

void writeCsv_trajectories(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);

    if (!csv.is_open())
    {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,point_idx,x,y,tf\n";

    for (const auto &r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.opt_traj.size();
             ++trial_idx)
        {
            const auto &tensor = r.opt_traj[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_pts =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2)
            {
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

void writeCsv_controlPts_strategy(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);

    if (!csv.is_open())
    {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,control_point,x,y,tf\n";

    for (const auto &r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.ControlPts_strategy.size();
             ++trial_idx)
        {
            const auto &tensor = r.ControlPts_strategy[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_control_points =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2)
            {
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

void writeCsv_trajectories_strategy(const std::vector<AggregateResult> &results, const std::string &path)
{
    std::ofstream csv(path);

    if (!csv.is_open())
    {
        throw std::runtime_error(
            "Could not open control point CSV: " + path);
    }

    csv << "label,bezier_degree,trial,vehicle,point_idx,x,y,tf\n";

    for (const auto &r : results)
    {
        for (size_t trial_idx = 0;
             trial_idx < r.opt_traj_strategy.size();
             ++trial_idx)
        {
            const auto &tensor = r.opt_traj_strategy[trial_idx];

            int trial = r.ControlPts_trial[trial_idx];

            const int n_vehicles =
                static_cast<int>(tensor.dimension(0));

            const int n_coords =
                static_cast<int>(tensor.dimension(1));

            const int n_pts =
                static_cast<int>(tensor.dimension(2));

            if (n_coords != 2)
            {
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

std::string todayDateString()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
    return std::string(buf);
}

// -----------------------------------------------------------------------------------
// Problem Generators
// -----------------------------------------------------------------------------------
struct RandomProblemGenerator
{
    std::mt19937 rng;

    explicit RandomProblemGenerator(unsigned seed) : rng(seed) {}

    std::vector<vehicle_state> randomStates(
        int n,
        double area,
        double radius)
    {
        std::uniform_real_distribution<double> pos(
            -area / 2.0,
            area / 2.0);

        std::uniform_real_distribution<double> theta(-M_PI, M_PI);

        const double min_dist = 2.0 * radius;
        const double min_dist_sq = min_dist * min_dist;

        std::vector<vehicle_state> states;
        states.reserve(n);

        for (int i = 0; i < n; ++i)
        {

            bool valid = false;
            vehicle_state candidate;

            while (!valid)
            {

                candidate.x = pos(rng);
                candidate.y = pos(rng);
                candidate.theta = theta(rng);
                candidate.v = std::clamp(0.02, PLANNER_VEL_MIN, PLANNER_VEL_MAX);
                valid = true;

                // Check candidate against all previously generated states
                for (const auto &s : states)
                {
                    const double dx = candidate.x - s.x;
                    const double dy = candidate.y - s.y;

                    const double dist_sq = dx * dx + dy * dy;

                    if (dist_sq < min_dist_sq)
                    {
                        valid = false;
                        break;
                    }
                }
            }

            states.push_back(candidate);
        }

        return states;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> randomCircObs(int n, double area)
    {
        Eigen::Matrix<double, 3, Eigen::Dynamic> obs(3, n);
        std::uniform_real_distribution<double> pos(-area / 2.0, area / 2.0);
        std::uniform_real_distribution<double> rad(1.0, 3.0);
        for (int i = 0; i < n; ++i)
        {
            obs(0, i) = pos(rng);
            obs(1, i) = pos(rng);
            obs(2, i) = rad(rng);
        }
        return obs;
    }

    Eigen::Matrix<double, 5, Eigen::Dynamic> randomElipObs(int n, double area)
    {
        Eigen::Matrix<double, 5, Eigen::Dynamic> obs(5, n);
        std::uniform_real_distribution<double> pos(-area / 2.0, area / 2.0);
        std::uniform_real_distribution<double> axis(1.0, 3.0);
        std::uniform_real_distribution<double> orient(-M_PI, M_PI);
        for (int i = 0; i < n; ++i)
        {
            obs(0, i) = pos(rng);
            obs(1, i) = pos(rng);
            obs(2, i) = axis(rng);       // semi-major
            obs(3, i) = axis(rng) * 0.6; // semi-minor
            obs(4, i) = orient(rng);
        }
        return obs;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> randomLineObs(int n, double area)
    {
        // Placeholder: same layout as circ obs for now; replace with actual
        // line-obstacle parametrization once defined.
        return randomCircObs(n, area);
    }
};

struct FormationProblemGenerator
{
    std::mt19937 rng;

    // Standard deviations of the perturbation on the initial states
    double sigma_pos;   // [m]
    double sigma_theta; // [rad]
    double sigma_v;     // [m/s]

    // Nominal initial states (mean) and fixed goals
    std::vector<vehicle_state> nominal_initial{
        {-5.0, 28.0, 0.8, 0.7},  // blue
        {1.0, 22.0, -0.4, 0.5},  // orange
        {-5.0, 18.0, -0.8, 0.7}, // yellow
        {-10.0, 12.0, 0.0, 0.5}, // purple
        {-8.0, 5.0, -0.4, 0.5}   // green
    };

    std::vector<vehicle_state> goal_states{
        {60.0, 12.0, 0.0, 0.7}, // blue
        {80.0, 12.0, 0.0, 0.7}, // orange
        {70.0, 17.0, 0.0, 0.7}, // yellow
        {80.0, 22.0, 0.0, 0.7}, // purple
        {60.0, 22.0, 0.0, 0.7}  // green
    };

    FormationProblemGenerator(unsigned seed,
                              double sigma_pos,
                              double sigma_theta,
                              double sigma_v)
        : rng(seed),
          sigma_pos(sigma_pos),
          sigma_theta(sigma_theta),
          sigma_v(sigma_v) {}

    // New perturbed set of initial states; call once per iteration
    std::vector<vehicle_state> sampleInitialStates()
    {
        std::normal_distribution<double> n_pos(0.0, sigma_pos);
        std::normal_distribution<double> n_theta(0.0, sigma_theta);
        std::normal_distribution<double> n_v(0.0, sigma_v);

        std::vector<vehicle_state> states;
        states.reserve(nominal_initial.size());

        for (const auto &nom : nominal_initial)
        {
            vehicle_state s = nom;
            s.x += n_pos(rng);
            s.y += n_pos(rng);
            s.theta += n_theta(rng);
            s.v = std::clamp(nom.v + n_v(rng), PLANNER_VEL_MIN, PLANNER_VEL_MAX);
            states.push_back(s);
        }
        return states;
    }

    const std::vector<vehicle_state> &goals() const { return goal_states; }
};

struct SquareProblemGenerator
{
    std::mt19937 rng;

    int n;              // number of vehicles = number of divisions
    double side;        // square side [m], centred at the origin
    double sigma_pos;   // std of position inside the cell [m]
    double sigma_theta; // std of heading [rad]
    double sigma_v;     // std of speed [m/s]

    static constexpr double UP_HEADING = M_PI / 2.0;

    SquareProblemGenerator(unsigned seed, int n, double side,
                           double sigma_pos, double sigma_theta, double sigma_v)
        : rng(seed), n(n), side(side),
          sigma_pos(sigma_pos), sigma_theta(sigma_theta), sigma_v(sigma_v) {}

    // bottom = true: lower half, bottom = false: upper half
    std::vector<vehicle_state> sampleStates(bool bottom, double v_mean)
    {
        const double cell_w = side / n;
        const double cell_h = side / 2.0;
        const double y_center = bottom ? -side / 4.0 : side / 4.0;

        // Each vehicle gets a distinct division
        std::vector<int> cells(n);
        std::iota(cells.begin(), cells.end(), 0);
        std::shuffle(cells.begin(), cells.end(), rng);

        std::normal_distribution<double> n_pos(0.0, 1.0);
        std::normal_distribution<double> n_theta(0.0, 1.0);
        std::normal_distribution<double> n_v(0.0, 1.0);

        std::vector<vehicle_state> states;
        states.reserve(n);

        for (int i = 0; i < n; ++i)
        {
            const double cx = -side / 2.0 + (cells[i] + 0.5) * cell_w;

            // Normal around the cell centre, clamped to stay inside the cell
            double dx = sigma_pos * n_pos(rng);
            double dy = sigma_pos * n_pos(rng);
            dx = std::clamp(dx, -cell_w / 2.0, cell_w / 2.0);
            dy = std::clamp(dy, -cell_h / 2.0, cell_h / 2.0);

            vehicle_state s;
            s.x = cx + dx;
            s.y = y_center + dy;
            s.theta = UP_HEADING + sigma_theta * n_theta(rng);
            s.v = std::clamp(0.15 + sigma_v * n_v(rng), PLANNER_VEL_MIN, PLANNER_VEL_MAX);
            states.push_back(s);
        }
        return states;
    }

    std::vector<vehicle_state> sampleInitialStates() { return sampleStates(true, 0.05); }
    std::vector<vehicle_state> sampleGoalStates() { return sampleStates(false, 0.05); }
};

// -----------------------------------------------------------------------------------
// Single trial: Simply run the optimization once without guess
// -----------------------------------------------------------------------------------
TrialResult runSingleTrial(const ScenarioParams &p, SquareProblemGenerator &gen, MultipleVehiclePlanner& full_size)
{
    TrialResult result;

    auto current_states = gen.sampleInitialStates();
    auto goal_states = gen.sampleGoalStates();

    // MultipleVehiclePlanner full_size(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);

    // full_size.setupSymbolic();
    // full_size.computeCostFunction();
    full_size.setOptimizationProblem(current_states, goal_states);
    full_size.createConstraintVector();
    full_size.createDecisionVector();

    std::atomic<bool> cancel_flag{false};

    full_size.solveOptimizationProblem(&cancel_flag);

    result.solve_time_ms = full_size.getTcomput();
    result.status = full_size.getOptimizationstatus();
    result.tf = full_size.getTf();
    result.ControlPts = full_size.getControlPoints();
    result.opt_traj = full_size.getOptimalTrajectories(200);

    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success") != std::string::npos ||
        result.status.find("SUCCESS") != std::string::npos;

    return result;
}

// -----------------------------------------------------------------------------------
// Single trial: Run the optimization with lower order guess and use it to initialize
// harder problem
// -----------------------------------------------------------------------------------
TrialResult runSingleTrial_with_lower_order_guess(const ScenarioParams &p, RandomProblemGenerator &gen, MultipleVehiclePlanner& lower_order, MultipleVehiclePlanner& full_size)
{
    TrialResult result;

    auto current_states = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto goal_states = gen.randomStates(p.n_vehicles, p.area_size, 1.7);
    auto circ_obs = gen.randomCircObs(p.n_circ_obs, p.area_size);
    auto elip_obs = gen.randomElipObs(p.n_elip_obs, p.area_size);
    auto line_obs = gen.randomLineObs(p.n_line_obs, p.area_size);

    std::atomic<bool> cancel_flag{false};
    double Tf_guess = -std::numeric_limits<double>::infinity();
    Eigen::Tensor<double, 3> all_control_points;

    auto t0 = std::chrono::high_resolution_clock::now();
    // MultipleVehiclePlanner lower_order(5, p.n_vehicles, p.nSplit, p.constr_flags);
    // lower_order.setupSymbolic();
    // lower_order.computeCostFunction();
    lower_order.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs);
    lower_order.createConstraintVector();
    lower_order.createDecisionVector();
    lower_order.solveOptimizationProblem(&cancel_flag);

    Eigen::Tensor<double, 3> cp_raw = lower_order.getControlPoints();

    int dim1 = cp_raw.dimension(1); // should be 2
    int dim2 = cp_raw.dimension(2); // N control points

    all_control_points = Eigen::Tensor<double, 3>(p.n_vehicles, dim1, dim2);

    for (int d = 0; d < dim1; ++d)
    {
        for (int n = 0; n < dim2; ++n)
        {
            all_control_points(0, d, n) = cp_raw(0, d, n);
        }
    }

    Tf_guess = lower_order.getTf();

    Eigen::Tensor<double, 3> control_points_guess = BezierUtils::elevateTensorDegree(all_control_points, p.bezier_degree - 5);

    // MultipleVehiclePlanner full_size(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    // full_size.setupSymbolic();
    // full_size.computeCostFunction();
    full_size.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs, control_points_guess, Tf_guess, false);
    full_size.createConstraintVector();
    full_size.createDecisionVector();

    full_size.solveOptimizationProblem(&cancel_flag);


    result.solve_time_ms_strategy = full_size.getTcomput() + lower_order.getTcomput();
    result.status_strategy = full_size.getOptimizationstatus();

    result.success_strategy =
        result.status_strategy.find("Succeeded") != std::string::npos ||
        result.status_strategy.find("success") != std::string::npos ||
        result.status_strategy.find("SUCCESS") != std::string::npos;

    // t0 = std::chrono::high_resolution_clock::now();
    // MultipleVehiclePlanner full_size(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    // full_size.setupSymbolic();
    // full_size.computeCostFunction();
    full_size.setOptimizationProblem(current_states, goal_states, circ_obs, line_obs, elip_obs);
    full_size.createConstraintVector();
    full_size.createDecisionVector();

    full_size.solveOptimizationProblem(&cancel_flag);

    result.solve_time_ms = full_size.getTcomput();
    result.status = full_size.getOptimizationstatus();

    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success") != std::string::npos ||
        result.status.find("SUCCESS") != std::string::npos;

    return result;
}

// -----------------------------------------------------------------------------------
// Single trial: Run the optimization for each vehicle individually check for collision
// and update for the ones that collide. Also run simply with guesses given by individual
// trajectories
// -----------------------------------------------------------------------------------
TrialResult runSingleTrial_with_Collision(const ScenarioParams &p, SquareProblemGenerator &gen, MultipleVehiclePlanner& single_vehicle_solver, MultipleVehiclePlanner& full_size)
{
    TrialResult result;

    auto current_states = gen.sampleInitialStates();
    auto goal_states = gen.sampleGoalStates();
    printStates("Initial states", current_states);
    printStates("Goal states", goal_states);

    // MultipleVehiclePlanner single_vehicle_solver(p.bezier_degree, 1, p.nSplit, p.constr_flags);

    double Tf_guess = -std::numeric_limits<double>::infinity();
    Eigen::Tensor<double, 3> all_control_points;
    std::vector<double> Tf_values(p.n_vehicles, 0.0);
    std::vector<bool> success_first(p.n_vehicles, false);

    result.solve_time_ms_strategy = 0.0;

    for (int i = 0; i < p.n_vehicles; i++)
    {
        // single_vehicle_solver.setupSymbolic();
        // single_vehicle_solver.computeCostFunction();

        std::vector<vehicle_state> current_state_vec;
        std::vector<vehicle_state> goal_state_vec;

        current_state_vec.push_back(current_states[i]);
        goal_state_vec.push_back(goal_states[i]);
        single_vehicle_solver.setOptimizationProblem(current_state_vec, goal_state_vec);

        single_vehicle_solver.createConstraintVector();
        single_vehicle_solver.createDecisionVector();

        std::atomic<bool> cancel_flag{false};

        single_vehicle_solver.solveOptimizationProblem(&cancel_flag);

        result.solve_time_ms_strategy = result.solve_time_ms_strategy + single_vehicle_solver.getTcomput();

        result.status_strategy = single_vehicle_solver.getOptimizationstatus();
        success_first[i] =
            result.status_strategy.find("Succeeded") != std::string::npos ||
            result.status_strategy.find("success") != std::string::npos ||
            result.status_strategy.find("SUCCESS") != std::string::npos;

        Eigen::Tensor<double, 3> cp_raw = single_vehicle_solver.getControlPoints();

        int dim1 = cp_raw.dimension(1); // should be 2
        int dim2 = cp_raw.dimension(2); // N control points

        if (i == 0)
        {
            all_control_points = Eigen::Tensor<double, 3>(p.n_vehicles, dim1, dim2);
        }

        for (int d = 0; d < dim1; ++d)
        {
            for (int n = 0; n < dim2; ++n)
            {
                all_control_points(i, d, n) = cp_raw(0, d, n);
            }
        }
        Tf_values[i] = single_vehicle_solver.getTf();
        if (success_first[i])
        {
            Tf_guess = std::max(Tf_guess, Tf_values[i]);
        }
    }

    std::vector<std::pair<int, int>> collisions;
    bool collision_detected = false;

    for (int i = 0; i < p.n_vehicles; ++i)
    {
        if (!success_first[i])
            continue;
        for (int j = i + 1; j < p.n_vehicles; ++j)
        {
            if (!success_first[j])
                continue;

            Eigen::MatrixXd P1 = BezierUtils::tensor_2_matrix(all_control_points, i);
            Eigen::MatrixXd P2 = BezierUtils::tensor_2_matrix(all_control_points, j);

            if (!BezierUtils::check_vehicle_distances(P1, P2, 1.5, 1))
            {
                collisions.emplace_back(i, j);
                collision_detected = true;
            }
        }
    }

    std::vector<int> failed_vehicles;
    for (int i = 0; i < p.n_vehicles; ++i)
    {
        if (!success_first[i])
            failed_vehicles.push_back(i);
    }

    double strategy_tf{0.0};
    if (collision_detected || !failed_vehicles.empty())
    {
        std::vector<int> to_remove;

        if (collision_detected)
        {
            std::vector<std::pair<int, int>> sorted_collisions = collisions;

            std::sort(sorted_collisions.begin(), sorted_collisions.end(),
                      [&](auto &a, auto &b)
                      {
                          double scoreA = Tf_values[a.first] + Tf_values[a.second];
                          double scoreB = Tf_values[b.first] + Tf_values[b.second];
                          return scoreA > scoreB; // prioritize high Tf conflicts
                      });

            to_remove = BezierUtils::selectTrajectoriesToRemove(p.n_vehicles, sorted_collisions, Tf_values);
        }

        // Failed vehicles are always re-computed in the joint solve
        for (int i : failed_vehicles)
        {
            if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end())
            {
                to_remove.push_back(i);
            }
        }
        std::sort(to_remove.begin(), to_remove.end());

        // Tf_min comes only from the vehicles that are kept
        double Tf_min = 0.0;
        for (int i = 0; i < p.n_vehicles; ++i)
        {
            if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end())
            {
                Tf_min = std::max(Tf_min, Tf_values[i]);
            }
        }

        // MultipleVehiclePlanner full_size(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
        // full_size.setupSymbolic();
        // full_size.computeCostFunction();

        full_size.setOptimizationProblem(current_states, goal_states, Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 0), Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 0), Eigen::Matrix<double, 5, Eigen::Dynamic>(5, 0), all_control_points, Tf_guess, false);
        full_size.createConstraintVector(to_remove);
        full_size.createDecisionVector(to_remove, Tf_min);
        std::atomic<bool> cancel_flag{false};
        full_size.solveOptimizationProblem(&cancel_flag);

        strategy_tf = full_size.getTf();
        result.solve_time_ms_strategy = result.solve_time_ms_strategy + full_size.getTcomput();
        result.status_strategy = full_size.getOptimizationstatus();
        result.ControlPts_strategy = full_size.getControlPoints();
        result.opt_traj_strategy = full_size.getOptimalTrajectories(2000);
        result.success_strategy =
            result.status_strategy.find("Succeeded") != std::string::npos ||
            result.status_strategy.find("success") != std::string::npos ||
            result.status_strategy.find("SUCCESS") != std::string::npos;
    }
    else
    {
        result.success_strategy = true;
        for (int i = 0; i < p.n_vehicles; ++i)
        {
            strategy_tf = std::max(strategy_tf, Tf_values[i]);

            if (!success_first[i])
            {
                result.success_strategy = false;
            }
        }
    }

    result.tf_strategy = strategy_tf;

    // MultipleVehiclePlanner full_size(p.bezier_degree, p.n_vehicles, p.nSplit, p.constr_flags);
    // // full_size.setFinalVelHeadMode(false,false);
    // full_size.setupSymbolic();
    // full_size.computeCostFunction();

    full_size.setOptimizationProblem(current_states, goal_states);
    full_size.createConstraintVector();
    full_size.createDecisionVector();

    std::atomic<bool> cancel_flag{false};

    full_size.solveOptimizationProblem(&cancel_flag);

    result.tf = full_size.getTf();
    result.solve_time_ms = full_size.getTcomput();
    result.status = full_size.getOptimizationstatus();
    result.ControlPts = full_size.getControlPoints();
    result.opt_traj = full_size.getOptimalTrajectories(2000);

    // --- Fallback success criterion (adapt once wasSuccessful() exists) ---
    // CasADi/IPOPT typically report something like "Solve_Succeeded".
    // If you switch full_sizes (qrqp/SQP), adjust the matched substrings.
    result.success =
        result.status.find("Succeeded") != std::string::npos ||
        result.status.find("success") != std::string::npos ||
        result.status.find("SUCCESS") != std::string::npos;

    return result;
}

AggregateResult runScenario(
    const ScenarioParams &p,
    int n_trials,
    unsigned seed_base,
    TrialMode mode = TrialMode::Collision)
{
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
    std::vector<Eigen::Tensor<double, 3>> ControlPts_strategy;
    std::vector<Eigen::Tensor<double, 3>> opt_traj_strategy;
    std::vector<int> ControlPts_trial_strategy;
    int n_success_strategy = 0;

    std::vector<double> prc_maneuver_times;
    prc_maneuver_times.reserve(n_trials);

    ScenarioPlanners planners(p, mode);

    for (int i = 0; i < n_trials; ++i)
    {
        const unsigned trial_seed =
            seed_base + static_cast<unsigned>(i) +
            n_trials * static_cast<unsigned>(p.bezier_degree);

        try
        {
            printBanner(
                "Starting trial " + std::to_string(i + 1) + "/" +
                std::to_string(n_trials) + " | N Vehicles = " +
                std::to_string(p.n_vehicles) + " | Degree = " +
                std::to_string(p.bezier_degree) + " | Mode = " +
                trialModeName(mode));

            TrialResult r;

            switch (mode)
            {
            case TrialMode::Standard:
            {
                SquareProblemGenerator gen(
                    trial_seed, p.n_vehicles, 1000.0, 0.0, 0.0, 0.0);
                r = runSingleTrial(p, gen, *planners.full_size);
                break;
            }

            case TrialMode::LowerOrderGuess:
            {
                RandomProblemGenerator gen(trial_seed);
                r = runSingleTrial_with_lower_order_guess(p, gen, *planners.lower_order, *planners.full_size);
                break;
            }

            case TrialMode::Collision:
            {
                SquareProblemGenerator gen(
                    trial_seed, p.n_vehicles, 800.0, 5.0, 0.0, 0.1);
                r = runSingleTrial_with_Collision(p, gen, *planners.single_vehicle, *planners.full_size);
                break;
            }
            }

            if (r.success)
            {
                std::cout << "[Trial " << i << "] ControlPts dimensions: "
                          << r.ControlPts.dimension(0) << " x "
                          << r.ControlPts.dimension(1) << " x "
                          << r.ControlPts.dimension(2) << std::endl;

                ++n_success;
                success_times.push_back(r.solve_time_ms);
                tf_times.push_back(r.tf);
                ControlPts.push_back(r.ControlPts);
                opt_traj.push_back(r.opt_traj);
                ControlPts_trial.push_back(i);
            }
            if (r.success_strategy)
            {
                std::cout << "[Trial " << i << "] ControlPts_strategy dimensions: "
                          << r.ControlPts_strategy.dimension(0) << " x "
                          << r.ControlPts_strategy.dimension(1) << " x "
                          << r.ControlPts_strategy.dimension(2) << std::endl;

                ++n_success_strategy;
                success_times_strategy.push_back(r.solve_time_ms_strategy);
                tf_times_strategy.push_back(r.tf_strategy);
                ControlPts_strategy.push_back(r.ControlPts_strategy);
                opt_traj_strategy.push_back(r.opt_traj_strategy);
                ControlPts_trial_strategy.push_back(i);
            }
            if (r.success && r.success_strategy)
            {
                prc_maneuver_times.push_back((r.tf_strategy - r.tf) / r.tf);
            }
        }
        catch (const std::exception &e)
        {
            // Any thrown exception (e.g. infeasible NLP setup, full_size crash,
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
    agg.ControlPts_strategy = ControlPts_strategy;
    agg.opt_traj_strategy = opt_traj_strategy;
    agg.ControlPts_trial_strategy = ControlPts_trial_strategy;
    if (!success_times.empty())
    {
        double sum = std::accumulate(success_times.begin(), success_times.end(), 0.0);
        double mean = sum / success_times.size();

        double sq_sum = 0.0;
        for (double t : success_times)
            sq_sum += (t - mean) * (t - mean);
        double stddev = success_times.size() > 1
                            ? std::sqrt(sq_sum / (success_times.size() - 1))
                            : 0.0;

        agg.avg_time_success_ms = mean;
        agg.worst_time_success_ms = *std::max_element(success_times.begin(), success_times.end());
        agg.std_time_success_ms = stddev;

        agg.tf_value = std::accumulate(tf_times.begin(), tf_times.end(), 0.0) / tf_times.size();

        agg.box_time_success = computeBox(success_times);
    }

    if (!success_times_strategy.empty())
    {
        double sum = std::accumulate(success_times_strategy.begin(), success_times_strategy.end(), 0.0);
        double mean = sum / success_times_strategy.size();

        double sq_sum = 0.0;
        for (double t : success_times_strategy)
            sq_sum += (t - mean) * (t - mean);
        double stddev = success_times_strategy.size() > 1
                            ? std::sqrt(sq_sum / (success_times_strategy.size() - 1))
                            : 0.0;

        agg.avg_time_success_ms_strategy = mean;
        agg.worst_time_success_ms_strategy = *std::max_element(success_times_strategy.begin(), success_times_strategy.end());
        agg.std_time_success_ms_strategy = stddev;

        agg.tf_value_strategy = std::accumulate(tf_times_strategy.begin(), tf_times_strategy.end(), 0.0) / tf_times_strategy.size();

        agg.box_time_success_strategy = computeBox(success_times_strategy);
    }

    if (!prc_maneuver_times.empty())
    {
        double sum = std::accumulate(prc_maneuver_times.begin(), prc_maneuver_times.end(), 0.0);
        double mean = sum / prc_maneuver_times.size();

        double sq_sum = 0.0;
        for (double t : prc_maneuver_times)
            sq_sum += (t - mean) * (t - mean);
        double stddev = prc_maneuver_times.size() > 1
                            ? std::sqrt(sq_sum / (prc_maneuver_times.size() - 1))
                            : 0.0;

        agg.avg_percentage_strategy = mean;
        agg.worst_percentage_strategy = *std::max_element(prc_maneuver_times.begin(), prc_maneuver_times.end());
        agg.std_percentage_strategy = stddev;

        agg.box_percentage_strategy = computeBox(prc_maneuver_times);
    }

    return agg;
}

int main(int argc, char **argv)
{
    const RunOptions options = parseRunOptions(argc, argv);

    const std::string date_tag = todayDateString();

    struct Sweep
    {
        std::string name;
        std::vector<ScenarioParams> scenarios;
    };

    std::vector<Sweep> sweeps = {
        {"vehicles_deg7", sweepVehiclesFixedDegree({10}, /*fixed_degree=*/6)},
    };

    namespace fs = std::filesystem;

    const fs::path results_dir =
        options.output_dir / date_tag / trialModeName(options.mode);
    fs::create_directories(results_dir);

    for (const auto &sweep : sweeps)
    {
        std::vector<AggregateResult> results;
        results.reserve(sweep.scenarios.size());

        for (const auto &p : sweep.scenarios)
        {
            std::cout << "Running scenario: " << p.label
                      << " (" << options.trials << " trials)...\n";

            AggregateResult agg = runScenario(
                p, options.trials, options.seed, options.mode);

            std::cout << "  success=" << agg.n_success << "/" << agg.n_trials
                      << " (fail=" << agg.failure_pct << "%)"
                      << " avg=" << agg.avg_time_success_ms << " ms"
                      << " worst=" << agg.worst_time_success_ms << " ms\n";
            results.push_back(agg);
        }

        writeCsv(
            results,
            (results_dir / ("benchmark_" + date_tag + "_original_plot_" + sweep.name + ".csv")).string());

        writeCsv_simple(
            results,
            (results_dir / ("benchmark_" + date_tag + "_simple_" + sweep.name + ".csv")).string());

        writeCsv_controlPts(
            results,
            (results_dir / ("benchmark_" + date_tag + "_controlPts_" + sweep.name + ".csv")).string());

        writeCsv_trajectories(
            results,
            (results_dir / ("benchmark_" + date_tag + "_trajectories_" + sweep.name + ".csv")).string());

        writeCsv_controlPts_strategy(
            results,
            (results_dir / ("benchmark_" + date_tag + "_controlPts_strategy_" + sweep.name + ".csv")).string());

        writeCsv_trajectories_strategy(
            results,
            (results_dir / ("benchmark_" + date_tag + "_trajectories_strategy_" + sweep.name + ".csv")).string());
    }

    return 0;
}


