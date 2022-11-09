#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace CeresCore
{
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).

    struct GroundPointError {
        GroundPointError(double observed_x, double observed_y, double observed_z)
            : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z) {}

        template <typename T>
        bool operator()(const T* const point,
            T* residuals) const {

            residuals[0] = point[0] - T(observed_x);
            residuals[1] = point[1] - T(observed_y);
            residuals[2] = point[2] - T(observed_z);

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* CreateAutoDiff(const double observed_x,
            const double observed_y, const double observed_z) {
            return (new ceres::AutoDiffCostFunction<GroundPointError, 3, 3>(
                new GroundPointError(observed_x, observed_y, observed_z)));
        }

        double observed_x;
        double observed_y;
        double observed_z;
    };
}