#pragma once

#include "EnumToString.h"

namespace SLAM_Benchmark
{
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(DatasetName, (TUM)(KITTI)(EuRoC));
    
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(SystemName, (ORB_SLAM2)(ORB_SLAM3)(VINS_MONO)(ORB_SLAM2_CUBA));
}