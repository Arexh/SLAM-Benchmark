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

        static const std::string ORB_SLAM3_VOC_PATH;

        static const std::string EuRoC_TimeStamps_PATH;

        static const std::string ORB_SLAM2_SETTING_PATH;

        static const std::string ORB_SLAM3_SETTING_PATH;

        static void benchmark_ORB_SLAM2(DatasetName dataset_name, const std::string dataset_path, const std::string setting_path, bool use_viewr, const std::string sequence = "");

        static void benchmark_ORB_SLAM3(DatasetName dataset_name, const std::string dataset_path, const std::string setting_path, bool use_viewer, const std::string sequence = "");

        static DatasetLoader *createDatasetLoader(DatasetName dataset_name, const std::string &dataset_path, const std::string &sequence = "");
    };
}