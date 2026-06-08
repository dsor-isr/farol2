// BezierUtils.h
#ifndef BEZIER_UTILS_H
#define BEZIER_UTILS_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <utility>
#include <unsupported/Eigen/CXX11/Tensor>

/**
 * @brief Here all utilities related to Bézier curves are implemented.
 *
 * This includes functions for computing derivatives, degree elevation, 
 * multiplication of Bézier curves, division of Bézier curves, and the 
 * de Casteljau algorithm. 
 * Additionally, there are functions for computing constraints for the 
 * optimization problem, related to dynamics and obstacles, as well as 
 * inter-vehicle constraints.
 */
namespace BezierUtils
{
    casadi::SX bezier_derivative_Casadi(const casadi::SX &P);
    casadi::SX elevate_degree_Casadi(const casadi::SX &P, int degree);
    casadi::SX multiply_bezier_Casadi(const casadi::SX &P1, const casadi::SX &P2);
    std::pair<casadi::SX, casadi::SX> divide_bezier_Casadi(const casadi::SX &P, double u);
    casadi::SX deCasteljau_Casadi(const casadi::SX &u, const casadi::SX &pts);

    Eigen::MatrixXd bezier_derivative(const Eigen::MatrixXd &P);
    Eigen::MatrixXd elevate_degree(const Eigen::MatrixXd &P, int degree);
    Eigen::MatrixXd multiply_bezier(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> divide_bezier(const Eigen::MatrixXd &P, double u);
    Eigen::VectorXd deCasteljau(double u, const Eigen::MatrixXd &pts);
    casadi::SX adaptive_dynamic_constraints(
        const casadi::SX &P,
        const casadi::SX &Tf,
        int nsplit,
        const std::vector<bool> &compute_constraints);

    casadi::SX adaptive_all_obstacle_constraints(
        const casadi::SX &P,
        const casadi::SX &circ_obs,
        const casadi::SX &line_obs,
        int nsplit);

    casadi::SX adaptive_inter_vehicle_constraints(
        const casadi::SX &P1,
        const casadi::SX &P2,
        double radius,
        int nsplit);
    
    Eigen::Tensor<double, 3> elevateTensorDegree(const Eigen::Tensor<double, 3>& tensor, int degree);

    bool check_vehicle_distances(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2, double radius, int nsplit);
    
    Eigen::MatrixXd tensor_2_matrix(const Eigen::Tensor<double, 3> &tensor, int vehicleIndex);
}

#endif // BEZIER_UTILS_H
