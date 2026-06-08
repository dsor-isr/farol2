#include "MultipleVehicleMotionPlan.h"
#include "H5Cpp.h"
#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>   // C++17

// Generate timestamp like 142530
std::string makeTimestamp()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%H%M%S");  // hour-minute-second
    return oss.str();
}
std::string makeDateFolder()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d");
    return oss.str();
}


void saveTrajectory(const Eigen::Tensor<double,3>& CP, double Tf)
{
    // Base folder
    std::string baseFolder = std::string(getenv("HOME")) + "/dsor/trajectories";
    std::filesystem::create_directories(baseFolder);

    // Subfolder for current day
    std::string dateFolder = baseFolder + "/" + makeDateFolder();
    std::filesystem::create_directories(dateFolder);

    // File name
    std::string filename = dateFolder + "/data_" + makeTimestamp() + ".csv";

    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    // CSV header
    file << "Vehicle,PointIndex,X,Y\n";

    int NVehicles = CP.dimension(0);
    int num_points = CP.dimension(2);

    for(int k=0; k<NVehicles; ++k)
    {
        for(int c=0; c<num_points; ++c)
        {
            double x = CP(k,0,c);
            double y = CP(k,1,c);
            file << k << "," << c << "," << x << "," << y << "\n";
        }
    }

    // Save Tf at the end
    file << "Tf," << Tf << "\n";

    file.close();
    std::cout << "Saved trajectory CSV to " << filename << std::endl;
}



int main() {
    // Example inputs
    int bezier_degree = 6;
    std::vector<int> nSplit = {1, 1, 1};
    std::vector<bool> constr_flag = {true, true, true, true};

    int NVehicles = 3;
    MultipleVehicleMotionPlan solver(bezier_degree, NVehicles, nSplit, constr_flag);
/*
    solver.setBoundsAndGains(
        0.001, 0.3,     // vel min, max
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),     // acc min, max
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),     // ang vel min, max
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),     // ang acc min, max
        -0.1, 100, // obs min, max
        0.5,       // RADIUS
        1.0, 0.0, 0.0 // ALPHA, BETA, GAMMA
    );*/
    solver.setupSymbolic();
    solver.computeCostFunction();

    std::vector<State> current = {
        {0.0, 0.0, 0.0, 0.01},
        {0.0, 5.0, 0.5, 0.01},
        {0.0, 10.0, -0.5, 0.01}
    };
     std::vector<State> goal = {
        {10.0, 0.0, 0.0, 0.01},
        {10.0, 5.0, 0.0, 0.01},
        {15.0, 3.0, -0.0, 0.01}
    };

    // Empty matrices for now
    Eigen::Matrix<double,3,Eigen::Dynamic> circ_obs(3,0);
    Eigen::Matrix<double,3,Eigen::Dynamic> line_obs(3,0);
    Eigen::Matrix<double,3,Eigen::Dynamic> neighbor(3,0);
    int num_points = bezier_degree + 1;
    Eigen::Tensor<double, 3> ContP_guess(NVehicles, 2, num_points);

    // Define start and end points
    Eigen::Vector2d start(0.0, 0.0);
    Eigen::Vector2d end(0.0, 5.0);

    // Fill matrix with equally spaced points
    //for (int i = 0; i < num_points; ++i) {
    //    double t = static_cast<double>(i) / (num_points - 1); // t in [0,1]
    //    ContP_guess.col(i) = (1 - t) * start + t * end;
    //}

    double Tf_guess = 60.0;
    bool distributed = false;
    bool first_iter  = true;


    solver.setOptimizationProblem(
        current,
        goal,
        circ_obs,
        line_obs,
        ContP_guess,
        Tf_guess,
        first_iter
    );
    
    solver.createConstraintVector();
    solver.createDecisionVector();
    solver.solveOptimizationProblem();

    Eigen::Tensor<double, 3>  CP = solver.getControlPoints();
    auto Tf = solver.getTf();
    std::string save_dir =
    std::string(std::getenv("HOME")) + "/dsor/mat_results/trajectories";

    saveTrajectory(CP, Tf);


    std::cout << "Tf value: " << Tf << "\n";

    for(int k = 0; k < NVehicles; k++)
    {
        std::cout << "Vehicle " << k << ":\n   ";
        for (int r = 0; r < 2; r++) {
            for (int c = 0; c < num_points; c++) {
                std::cout << CP(k, r, c) << " ";
            }
            std::cout << "\n"; 
        }
    }
    

    return 0;
}