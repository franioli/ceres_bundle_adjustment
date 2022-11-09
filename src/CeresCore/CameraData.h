#pragma once
#include "ceres/ceres.h"

namespace CeresCore
{
    class CameraData
    {
    public:
        int id;
        std::vector<ceres::int32> fixed_parameters;
    };
}