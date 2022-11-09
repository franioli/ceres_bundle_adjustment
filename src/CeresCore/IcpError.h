#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"

namespace CeresCore
{
    struct IcpError {
        IcpError(double* observed_x, double* observed_y, double* observed_z)
            : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z) {}

        template <typename T>
        bool operator()(const T* const point,
            T* residuals) const {

            if (std::isnan(*observed_x))
            {
                residuals[0] = T(dist_thresh);
                residuals[1] = T(dist_thresh);
                residuals[2] = T(dist_thresh);
                return true;
            }

            residuals[0] = (point[0] - T(*observed_x)) * 0.001;
            residuals[1] = (point[1] - T(*observed_y)) * 0.001;
            residuals[2] = point[2] - T(*observed_z);

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* CreateAutoDiff(double* observed_x,
            double* observed_y, double* observed_z) {
            return (new ceres::AutoDiffCostFunction<IcpError, 3, 3>(
                new IcpError(observed_x, observed_y, observed_z)));
        }

        double* observed_x;
        double* observed_y;
        double* observed_z;

    public:
        static double dist_thresh;
    };

}