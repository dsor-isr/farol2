#pragma once
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

class HungarianAlgorithm {
public:
    double Solve(const Eigen::MatrixXd& costMatrix, std::vector<int>& assignment) {
        int nRows = costMatrix.rows();
        int nCols = costMatrix.cols();

        assignment.assign(nRows, -1);

        Eigen::MatrixXd cost = costMatrix;

        std::vector<double> u(nRows, 0), v(nCols, 0);
        std::vector<int> p(nCols, -1), way(nCols);

        for (int i = 0; i < nRows; ++i) {
            std::vector<double> minv(nCols, std::numeric_limits<double>::max());
            std::vector<char> used(nCols, false);
            int j0 = -1, j1 = 0;
            p.assign(nCols, -1);
            way.assign(nCols, -1);

            int i0 = i;
            do {
                used[j1] = true;
                double delta = std::numeric_limits<double>::max();
                int i1 = -1;
                for (int j = 0; j < nCols; ++j) {
                    if (!used[j]) {
                        double cur = cost(i0, j) - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j1;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for (int j = 0; j < nCols; ++j) {
                    if (used[j]) {
                        u[i0] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                i0 = p[j1];
            } while (p[j1] != -1);

            do {
                int j2 = way[j1];
                p[j1] = p[j2];
                j1 = j2;
            } while (j1 != -1);
        }

        for (int j = 0; j < nCols; ++j) {
            if (p[j] != -1)
                assignment[p[j]] = j;
        }

        double totalCost = 0.0;
        for (int i = 0; i < nRows; ++i) {
            if (assignment[i] >= 0)
                totalCost += costMatrix(i, assignment[i]);
        }

        return totalCost;
    }
};
