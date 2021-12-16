#pragma once

#include "SystemRecorder.h"
#include "DatasetLoader.h"

#include "ORB_SLAM2_detailed_comments/include/System.h"

namespace SLAM_Benchmark
{
    class BenchmarkManager
    {
    public:

        static const std::string ORB_SLAM2_VOC_PATH;

        static const std::map<DatasetName, std::string> ORB_SLAM2_SETTING_MAP;

        static void benchmark(SystemName system_name, DatasetName dataset_name, const std::string dataset_path);

        static DatasetLoader *createDatasetLoader(DatasetName dataset_name, const std::string &dataset_path);

        static ORB_SLAM2::System *createSystem(SystemName system_name, DatasetName dataset_name, bool use_viewer);
    };
}