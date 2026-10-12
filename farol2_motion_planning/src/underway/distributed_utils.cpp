#include "underway/distributed_utils.hpp"

bool DistributedUtils::areVoronoiNeighbors(const Eigen::Vector2d& p_i, int j, const std::vector<Eigen::Vector2d>& other_positions)
{
    const double eps = 1e-9;
    const Eigen::Vector2d p_j = other_positions[j];
    const Eigen::Vector2d diff = p_j - p_i;

    if (diff.squaredNorm() < eps * eps)
    {
        return false;
    }

    const Eigen::Vector2d mid = 0.5 * (p_i + p_j);
    const Eigen::Vector2d dir(-diff.y(), diff.x());

    double t_min = -std::numeric_limits<double>::infinity();
    double t_max = std::numeric_limits<double>::infinity();

    for (int k = 0; k < static_cast<int>(other_positions.size()); ++k)
    {
        if (k == j)
        {
            continue;
        }

        const Eigen::Vector2d p_k = other_positions[k];
        const Eigen::Vector2d d_ki = p_k - p_i;
        const double A = 2.0 * dir.dot(d_ki);
        const double B = p_k.squaredNorm() - p_i.squaredNorm() - 2.0 * mid.dot(d_ki);

        if (std::abs(A) < eps)
        {
            if (B < -eps)
            {
                return false;
            }
        }
        else if (A > 0.0)
        {
            t_max = std::min(t_max, B / A);
        }
        else
        {
            t_min = std::max(t_min, B / A);
        }

        if (t_min > t_max + eps)
        {
            return false;
        }
    }

    return (t_max - t_min) > eps;
}

VoronoiCell DistributedUtils::getVoronoiNeighbors(const Eigen::Vector2d& current_state, const std::vector<Eigen::Vector2d>& other_positions, double margin)
{
    VoronoiCell cell;

    for (int j = 0; j < static_cast<int>(other_positions.size()); ++j)
    {
        if (!areVoronoiNeighbors(current_state, j, other_positions))
        {
            continue;
        }

        const Eigen::Vector2d diff = other_positions[j] - current_state;
        const Eigen::Vector2d buffered_midpoint = 0.5 * (current_state + other_positions[j]) - margin * diff.normalized();
        const double a = diff.x();
        const double b = diff.y();
        const double c = -diff.dot(buffered_midpoint);

        cell.edges.emplace_back(a, b, c);
        cell.neighbor_indices.push_back(j);
    }

    return cell;
}

bool DistributedUtils::isInsideVoronoiCell(const Eigen::Vector2d& point, const Eigen::Vector2d& current_state, const VoronoiCell& cell)
{
    double eps = 1e-9;
    for (const Eigen::Vector3d& edge : cell.edges)
    {
        const double value = edge.x() * point.x() + edge.y() * point.y() + edge.z();
        const double current_value = edge.x() * current_state.x() + edge.y() * current_state.y() + edge.z();

        if (current_value >= 0.0)
        {
            if (value < -eps)
            {
                return false;
            }
        }
        else
        {
            if (value > eps)
            {
                return false;
            }
        }
    }

    return true;
}

Eigen::Vector2d DistributedUtils::getClosestPointInVoronoiCell(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, const VoronoiCell& cell)
{
    const double eps = 1e-9;

    if (isInsideVoronoiCell(goal_state, current_state, cell))
    {
        return goal_state;
    }

    std::vector<Eigen::Vector2d> candidates;

    for (const Eigen::Vector3d& edge : cell.edges)
    {
        const Eigen::Vector2d normal(edge.x(), edge.y());
        const double c = edge.z();
        const double normal_squared = normal.squaredNorm();

        if (normal_squared < eps)
        {
            continue;
        }

        const double distance = (normal.dot(goal_state) + c) / normal_squared;
        const Eigen::Vector2d projection = goal_state - distance * normal;

        if (isInsideVoronoiCell(projection, current_state, cell))
        {
            candidates.push_back(projection);
        }
    }

    for (int i = 0; i < static_cast<int>(cell.edges.size()); ++i)
    {
        for (int j = i + 1; j < static_cast<int>(cell.edges.size()); ++j)
        {
            const Eigen::Vector3d& e1 = cell.edges[i];
            const Eigen::Vector3d& e2 = cell.edges[j];

            const double det = e1.x() * e2.y() - e2.x() * e1.y();

            if (std::abs(det) < eps)
            {
                continue;
            }

            const double x = (e1.y() * e2.z() - e2.y() * e1.z()) / det;
            const double y = (e2.x() * e1.z() - e1.x() * e2.z()) / det;
            const Eigen::Vector2d intersection(x, y);

            if (isInsideVoronoiCell(intersection, current_state, cell))
            {
                candidates.push_back(intersection);
            }
        }
    }

    Eigen::Vector2d closest = candidates.front();
    double min_distance = (closest - goal_state).squaredNorm();

    for (const Eigen::Vector2d& candidate : candidates)
    {
        const double distance = (candidate - goal_state).squaredNorm();

        if (distance < min_distance)
        {
            min_distance = distance;
            closest = candidate;
        }
    }

    return closest;
}

Eigen::Vector2d DistributedUtils::getCurrentGoalVoronoiIntersection(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, const VoronoiCell& cell)
{
    const double eps = 1e-9;
    const Eigen::Vector2d direction = goal_state - current_state;

    double best_t = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_point;

    for (const Eigen::Vector3d& edge : cell.edges)
    {
        const Eigen::Vector2d normal(edge.x(), edge.y());
        const double c = edge.z();
        const double denominator = normal.dot(direction);

        if (std::abs(denominator) < eps)
        {
            continue;
        }

        const double t = -(normal.dot(current_state) + c) / denominator;

        if (t < -eps)
        {
            continue;
        }

        const Eigen::Vector2d intersection = current_state + t * direction;

        if (t < best_t)
        {
            best_t = t;
            best_point = intersection;
        }
    }

    return best_point;
}