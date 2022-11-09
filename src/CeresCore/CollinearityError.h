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

    struct CollinearityError {
        CollinearityError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

        template <typename T>
        bool operator()(const T* const sensor, 
            const T* const camera,            
            const T* const point,
            T* residuals) const  {
        // camera[0,1,2] are the angle-axis rotation (rodriguez vector).
        // camera[3,4,5] are the translation.
        T p[3];
        T pt[3];
        pt[0] = point[0] - camera[3];
        pt[1] = point[1] - camera[4];
        pt[2] = point[2] - camera[5];
        ceres::AngleAxisRotatePoint(camera, pt, p);

        // camera[0 = w,1 = h,2 = F,3 = Cx,4 = Cy,
        //        5 = K1,6 = K2,7 = K3,8 = K4,
        //        9 = P1, 10 = P2, 11 = B1, 12 = B2]

        T x = -p[0] / p[2];
        T y = p[1] / p[2];
        T r2 = x * x + y * y;
        T rad = (T(1.0) + sensor[5] * r2 + sensor[6] * r2 * r2 + sensor[7] * r2 * r2 * r2 + sensor[8] * r2 * r2 * r2 * r2);
        T xp = x * rad + sensor[9] * (r2 + T(2.0) * x * x) + T(2.0) * sensor[10] * x * y;
        T yp = y * rad + sensor[10] * (r2 + T(2.0) * y * y) + T(2.0) * sensor[9] * x * y;

        T u = sensor[0] * T(0.5) + sensor[3] + xp * sensor[2] + xp * sensor[11] + yp * sensor[12];
        T v = sensor[1] * T(0.5) + sensor[4] + yp * sensor[2];

        // The error is the difference between the predicted and observed position.
        residuals[0] = u - T(observed_x);
        residuals[1] = v - T(observed_y);

        return true;
    }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* CreateAutoDiff(const double observed_x,
            const double observed_y) {
            return (new ceres::AutoDiffCostFunction<CollinearityError, 2, 13, 6, 3>(
                new CollinearityError(observed_x, observed_y)));
        }

        double observed_x;
        double observed_y;
    };
}

