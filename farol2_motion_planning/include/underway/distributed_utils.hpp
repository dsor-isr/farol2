#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>

struct VoronoiCell
{
    std::vector<Eigen::Vector3d> edges;
    std::vector<int> neighbor_indices;
};

namespace
DistributedUtils
{
    VoronoiCell getVoronoiNeighbors(const Eigen::Vector2d& current_state, const std::vector<Eigen::Vector2d>& other_positions, double margin);

    bool isInsideVoronoiCell(const Eigen::Vector2d& point, const Eigen::Vector2d& current_state, const VoronoiCell& cell);

    Eigen::Vector2d getClosestPointInVoronoiCell(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, const VoronoiCell& cell);

    Eigen::Vector2d getCurrentGoalVoronoiIntersection(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, const VoronoiCell& cell);


    bool areVoronoiNeighbors(const Eigen::Vector2d& p_i, int j, const std::vector<Eigen::Vector2d>& other_positions);
};