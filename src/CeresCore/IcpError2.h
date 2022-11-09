#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"

namespace CeresCore
{
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).

    struct IcpError2 {
        IcpError2(double* observed_x, double* observed_y, double* observed_z)
            : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z) {}

        template <typename T>
        bool operator()(const T* const point,
            T* residuals) const {

            if (std::isnan(*observed_x))
            {
                residuals[0] = T(4);
                /*residuals[1] = T(2);
                residuals[2] = T(2);*/
                return true;
            }

            T dx = point[0] - T(*observed_x);
            T dy = point[1] - T(*observed_y);
            T dz = point[2] - T(*observed_z);

            residuals[0] = dx * dx + dy * dy + dz * dz;

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* CreateAutoDiff(double* observed_x,
            double* observed_y, double* observed_z) {
            return (new ceres::AutoDiffCostFunction<IcpError2, 1, 3>(
                new IcpError2(observed_x, observed_y, observed_z)));
        }

        double* observed_x;
        double* observed_y;
        double* observed_z;
    };
}