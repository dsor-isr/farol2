#include "BezierUtils.h"

// Bezier Operations using casadi symbolic expressions
casadi::SX BezierUtils::bezier_derivative_Casadi(const casadi::SX &P)
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

casadi::SX BezierUtils::elevate_degree_Casadi(const casadi::SX &P, int degree)
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

casadi::SX BezierUtils::multiply_bezier_Casadi(const casadi::SX &P1, const casadi::SX &P2)
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

std::pair<casadi::SX, casadi::SX> BezierUtils::divide_bezier_Casadi(const casadi::SX &P, double u)
{
    int n = P.size2(); // number of control points
    int dim = P.size1();

    casadi::SX P1 = casadi::SX::zeros(dim, n);
    casadi::SX P2 = casadi::SX::zeros(dim, n);

    casadi::SX temp = P;

    for (int r = 0; r < n; ++r)
    {
        P1(casadi::Slice(), r) = temp(casadi::Slice(), 0);
        P2(casadi::Slice(), n - r - 1) = temp(casadi::Slice(), temp.size2() - 1);

        casadi::SX next = casadi::SX::zeros(dim, temp.size2() - 1);
        for (int k = 0; k < temp.size2() - 1; ++k)
        {
            next(casadi::Slice(), k) = (1.0 - u) * temp(casadi::Slice(), k) + u * temp(casadi::Slice(), k + 1);
        }

        temp = next;
    }

    return {P1, P2};
}

casadi::SX BezierUtils::deCasteljau_Casadi(const casadi::SX &u, const casadi::SX &pts)
{
    casadi::SX temp = pts;
    int n = pts.size2();

    for (int r = 1; r < n; ++r)
    {
        for (int k = 0; k < n - r; ++k)
        {
            temp(casadi::Slice(), k) = (1.0 - u) * temp(casadi::Slice(), k) + u * temp(casadi::Slice(), k + 1);
        }
    }

    return temp(casadi::Slice(), 0);
}

// Bezier Operations used to obtain real values
Eigen::MatrixXd BezierUtils::bezier_derivative(const Eigen::MatrixXd &P)
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

Eigen::MatrixXd BezierUtils::elevate_degree(const Eigen::MatrixXd &P, int degree)
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

Eigen::MatrixXd BezierUtils::multiply_bezier(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2)
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

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> BezierUtils::divide_bezier(const Eigen::MatrixXd &P, double u)
{
    int n = P.cols();

    Eigen::MatrixXd P1(P.rows(), n);
    Eigen::MatrixXd P2(P.rows(), n);

    Eigen::MatrixXd temp = P;

    for (int r = 0; r < n; ++r)
    {
        P1.col(r) = temp.col(0);
        P2.col(n - r - 1) = temp.col(temp.cols() - 1);

        for (int k = 0; k < temp.cols() - 1; ++k)
        {
            temp.col(k) = (1.0 - u) * temp.col(k) + u * temp.col(k + 1);
        }

        temp.conservativeResize(temp.rows(), temp.cols() - 1);
    }

    return {P1, P2};
}

Eigen::VectorXd BezierUtils::deCasteljau(double u, const Eigen::MatrixXd &pts)
{
    Eigen::MatrixXd temp = pts;
    int n = pts.cols();

    for (int r = 1; r < n; ++r)
    {
        for (int k = 0; k < n - r; ++k)
        {
            temp.col(k) = (1.0 - u) * temp.col(k) + u * temp.col(k + 1);
        }
    }

    return temp.col(0);
}

casadi::SX BezierUtils::adaptive_dynamic_constraints(
    const casadi::SX &P,
    const casadi::SX &Tf,
    int nsplit,
    const std::vector<bool> &compute_constraints)
{
    using namespace casadi;
    
    struct Segment
    {
        SX curve;
        SX Tf_seg;
        int depth;
    };

    
    std::vector<Segment> queue;
    queue.push_back({P, Tf, nsplit});
    std::vector<SX> all_constraints;

    while (!queue.empty())
    {
        Segment seg = queue.back();
        queue.pop_back();

        if (seg.depth == 0)
        {
            SX segment_constraints;

            SX deriv_p, sec_deriv_p, third_deriv_p;

            // Compute derivatives as needed
            if (compute_constraints[0] || compute_constraints[1] || compute_constraints[2] || compute_constraints[3]) {
                deriv_p = BezierUtils::bezier_derivative_Casadi(seg.curve) / seg.Tf_seg;
            }
            if (compute_constraints[1] || compute_constraints[2] || compute_constraints[3]) {
                sec_deriv_p = BezierUtils::bezier_derivative_Casadi(deriv_p) / seg.Tf_seg;
            }
            if (compute_constraints[3]) {
                third_deriv_p = BezierUtils::bezier_derivative_Casadi(sec_deriv_p) / seg.Tf_seg;
            }

            // Velocity constraints
            if (compute_constraints[0])
            {
                SX vel_sq = BezierUtils::multiply_bezier_Casadi(deriv_p, deriv_p);
                vel_sq = vel_sq(0, Slice()) + vel_sq(1, Slice());
                segment_constraints = vel_sq.T();
            }

            // Angular velocity constraints
            if (compute_constraints[1])
            {
                SX num_ang = BezierUtils::multiply_bezier_Casadi(deriv_p(0, Slice()), sec_deriv_p(1, Slice()))
                            - BezierUtils::multiply_bezier_Casadi(deriv_p(1, Slice()), sec_deriv_p(0, Slice()));
                num_ang = BezierUtils::elevate_degree_Casadi(num_ang, 1);
                SX vel_sq = BezierUtils::multiply_bezier_Casadi(deriv_p, deriv_p);
                vel_sq = vel_sq(0, Slice()) + vel_sq(1, Slice());
                SX ang_vel_constr = num_ang / vel_sq;
                
                if (segment_constraints.size1() == 0) {
                    segment_constraints = ang_vel_constr.T();
                } else {
                    segment_constraints = vertcat(segment_constraints, ang_vel_constr.T());
                }
            }

            // Acceleration constraints
            if (compute_constraints[2])
            {
                SX num_acc_sqrt = BezierUtils::multiply_bezier_Casadi(deriv_p(0, Slice()), sec_deriv_p(0, Slice()))
                                + BezierUtils::multiply_bezier_Casadi(deriv_p(1, Slice()), sec_deriv_p(1, Slice()));
                SX num_acc = BezierUtils::multiply_bezier_Casadi(num_acc_sqrt, num_acc_sqrt);
                SX vel_sq = BezierUtils::multiply_bezier_Casadi(deriv_p, deriv_p);
                vel_sq = vel_sq(0, Slice()) + vel_sq(1, Slice());
                SX den = BezierUtils::elevate_degree_Casadi(vel_sq, num_acc.size2() - vel_sq.size2());
                SX acc_constr = num_acc / den;
                
                if (segment_constraints.size1() == 0) {
                    segment_constraints = acc_constr.T();
                } else {
                    segment_constraints = vertcat(segment_constraints, acc_constr.T());
                }
            }

            // Angular acceleration constraints
            if (compute_constraints[3])
            {
                SX vel_sq = BezierUtils::multiply_bezier_Casadi(deriv_p, deriv_p);
                vel_sq = vel_sq(0, Slice()) + vel_sq(1, Slice());

                SX num_ang = BezierUtils::multiply_bezier_Casadi(deriv_p(0, Slice()), sec_deriv_p(1, Slice()))
                            - BezierUtils::multiply_bezier_Casadi(deriv_p(1, Slice()), sec_deriv_p(0, Slice()));

                SX num_acc_sqrt = BezierUtils::multiply_bezier_Casadi(deriv_p(0, Slice()), sec_deriv_p(0, Slice()))
                                + BezierUtils::multiply_bezier_Casadi(deriv_p(1, Slice()), sec_deriv_p(1, Slice()));

                SX term_a = BezierUtils::multiply_bezier_Casadi(third_deriv_p(1, Slice()), deriv_p(0, Slice()))
                          - BezierUtils::multiply_bezier_Casadi(third_deriv_p(0, Slice()), deriv_p(1, Slice()));
                term_a = BezierUtils::multiply_bezier_Casadi(term_a, vel_sq);

                SX term_b = 3 * BezierUtils::multiply_bezier_Casadi(num_acc_sqrt, num_ang);
                SX num_ang_acc = term_a - term_b;
                SX den_ang_acc = BezierUtils::multiply_bezier_Casadi(vel_sq, vel_sq);
                num_ang_acc = BezierUtils::elevate_degree_Casadi(num_ang_acc, den_ang_acc.size2() - num_ang_acc.size2());
                SX ang_acc_constr = num_ang_acc / den_ang_acc;

                if (segment_constraints.size1() == 0) {
                    segment_constraints = ang_acc_constr.T();
                } else {
                    segment_constraints = vertcat(segment_constraints, ang_acc_constr.T());
                }
            }

            all_constraints.push_back(segment_constraints);
        }
        else
        {
            auto divided = BezierUtils::divide_bezier_Casadi(seg.curve, 0.5);
            queue.push_back({divided.first, seg.Tf_seg / 2, seg.depth - 1});
            queue.push_back({divided.second, seg.Tf_seg / 2, seg.depth - 1});
        }
    }

    return SX::vertcat(all_constraints);
}

casadi::SX BezierUtils::adaptive_all_obstacle_constraints(
    const casadi::SX &P,
    const casadi::SX &circ_obs,
    const casadi::SX &line_obs,
    int nsplit)
{
    using namespace casadi;
    
    struct Segment
    {
        SX curve;
        int depth;
    };

    std::vector<Segment> queue;
    queue.push_back({P, nsplit});
    std::vector<SX> all_constraints;

    while (!queue.empty())
    {
        Segment seg = queue.back();
        queue.pop_back();

        if (seg.depth == 0)
        {
            // Circular obstacles constraints
            int num_circ_obs = circ_obs.size2();  // Number of circular obstacles
            for (int i = 0; i < num_circ_obs; ++i)
            {
                // Extract obstacle parameters: [x, y, radius] for each column
                SX obs_x = circ_obs(0, i);
                SX obs_y = circ_obs(1, i);
                SX obs_r = circ_obs(2, i);
                
                SX obs_xy = SX::vertcat({obs_x, obs_y});
                SX P_new = seg.curve - obs_xy;
                SX P_obs = BezierUtils::multiply_bezier_Casadi(P_new, P_new);
                SX radii = obs_r * obs_r * SX::ones(1, P_obs.size2());
                SX constraint = (P_obs(0, Slice()) + P_obs(1, Slice()) - radii).T();
                all_constraints.push_back(constraint);
            }

            // Line obstacles constraints
            int num_line_obs = line_obs.size2();  // Number of line obstacles
            for (int i = 0; i < num_line_obs; ++i)
            {
                // Extract line parameters: [a, b, c] for line equation ax + by + c = 0
                SX a = line_obs(0, i);
                SX b = line_obs(1, i);
                SX c = line_obs(2, i);

                int N = seg.curve.size2();
                SX ones_row = SX::ones(1, N);
                SX result = a * seg.curve(0, Slice()) +
                           b * seg.curve(1, Slice()) +
                           c * ones_row;
                SX sign_ref = result(0);
                SX constraint = sign_ref * result(Slice(1, result.size2()));
                all_constraints.push_back(constraint.T());
            }
        }
        else
        {
            auto divided = BezierUtils::divide_bezier_Casadi(seg.curve, 0.5);
            queue.push_back({divided.first, seg.depth - 1});
            queue.push_back({divided.second, seg.depth - 1});
        }
    }

    return all_constraints.empty() ? SX(0) : SX::vertcat(all_constraints);
}

casadi::SX BezierUtils::adaptive_inter_vehicle_constraints(
    const casadi::SX &P1,
    const casadi::SX &P2,
    double radius,
    int nsplit)
{
    using namespace casadi;

    struct Segment
    {
        SX curve1;
        SX curve2;
        int depth;
    };

    std::vector<Segment> queue;
    queue.push_back({P1, P2, nsplit});

    std::vector<SX> all_constraints;

    while (!queue.empty())
    {
        Segment seg = queue.back();
        queue.pop_back();

        if (seg.depth == 0)
        {
            // Inter-vehicle distance constraint
            SX P_new = seg.curve1 - seg.curve2;
            SX P_obs = multiply_bezier_Casadi(P_new, P_new);

            SX radiuses = radius * radius * SX::ones(1, P_obs.size2());
            SX constraint =
                (P_obs(0, Slice()) + P_obs(1, Slice()) - radiuses).T();

            all_constraints.push_back(constraint);
        }
        else
        {
            // Subdivide both Bezier curves consistently
            auto divided1 = divide_bezier_Casadi(seg.curve1, 0.5);
            auto divided2 = divide_bezier_Casadi(seg.curve2, 0.5);

            queue.push_back({divided1.first,  divided2.first,  seg.depth - 1});
            queue.push_back({divided1.second, divided2.second, seg.depth - 1});
        }
    }

    return all_constraints.empty() ? SX(0) : SX::vertcat(all_constraints);
}

Eigen::MatrixXd BezierUtils::tensor_2_matrix(const Eigen::Tensor<double, 3> &tensor, int vehicleIndex)
{
    int numPoints = tensor.dimension(2);
    int dim = tensor.dimension(1);
    Eigen::MatrixXd mat(2, numPoints);

    for (int d = 0; d < dim; ++d) {
        for (int p = 0; p < numPoints; ++p) {
            mat(d, p) = tensor(vehicleIndex, d, p);
        }
    }

    return mat;
}

bool BezierUtils::check_vehicle_distances(const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2, double radius, int nsplit)
{
    struct Segment
    {
        Eigen::MatrixXd curve1;
        Eigen::MatrixXd curve2;
        int depth;
    };

    std::vector<Segment> queue;
    queue.push_back({P1, P2, nsplit});

    std::vector<Eigen::VectorXd> all_constraints;

    while (!queue.empty())
    {
        Segment seg = queue.back();
        queue.pop_back();

        if (seg.depth == 0)
        {
            // Inter-vehicle distance constraint
            Eigen::MatrixXd P_new = seg.curve1 - seg.curve2;
            Eigen::MatrixXd P_obs = multiply_bezier(P_new, P_new);

            Eigen::VectorXd radiuses = radius * radius * Eigen::VectorXd::Ones(P_obs.cols());
            Eigen::VectorXd constraint = (P_obs.row(0) + P_obs.row(1)).transpose() - radiuses;

            all_constraints.push_back(constraint);
        }
        else
        {
            // Subdivide both Bezier curves consistently
            auto divided1 = divide_bezier(seg.curve1, 0.5);
            auto divided2 = divide_bezier(seg.curve2, 0.5);

            queue.push_back({divided1.first,  divided2.first,  seg.depth - 1});
            queue.push_back({divided1.second, divided2.second, seg.depth - 1});
        }
    }
    for (const auto& constraint : all_constraints) {
        if ((constraint.array() < 0).any()) {
            return false; // At least one constraint is violated
        }
    }
    return true; // All constraints are satisfied
}

Eigen::Tensor<double, 3> BezierUtils::elevateTensorDegree(const Eigen::Tensor<double, 3>& tensor, int degree)
{
    const int dim0 = tensor.dimension(0);
    const int dim1 = tensor.dimension(1);
    const int dim2 = tensor.dimension(2);

    // First elevation to determine new matrix size
    Eigen::MatrixXd first_slice(dim1, dim2);
    for (int j = 0; j < dim1; ++j)
        for (int k = 0; k < dim2; ++k)
            first_slice(j, k) = tensor(0, j, k);

    Eigen::MatrixXd elevated = elevate_degree(first_slice, degree);

    const int new_dim1 = elevated.rows();
    const int new_dim2 = elevated.cols();

    Eigen::Tensor<double, 3> result(dim0, new_dim1, new_dim2);

    // Process all slices
    for (int i = 0; i < dim0; ++i)
    {
        Eigen::MatrixXd slice(dim1, dim2);

        for (int j = 0; j < dim1; ++j)
            for (int k = 0; k < dim2; ++k)
                slice(j, k) = tensor(i, j, k);

        Eigen::MatrixXd elevated_slice = elevate_degree(slice, degree);

        for (int j = 0; j < new_dim1; ++j)
            for (int k = 0; k < new_dim2; ++k)
                result(i, j, k) = elevated_slice(j, k);
    }

    return result;
}