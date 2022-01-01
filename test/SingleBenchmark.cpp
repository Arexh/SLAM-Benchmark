#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "BenchmarkManager.h"

using namespace std;
using namespace SLAM_Benchmark;

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: system_name dataset_name path_to_sequence" << endl;
        return 1;
    }

    string system_name = argv[1];
    string dataset_name = argv[2];
    string dataset_path = argv[3];

    boost::algorithm::to_lower(system_name);
    boost::algorithm::to_lower(dataset_name);

    if (system_name == "orb_slam2")
    {
        if (dataset_name == "tum") {
            BenchmarkManager::benchmark_ORB_SLAM2(DatasetName::TUM, dataset_path, "TUM1.yaml", false);
        } else if (dataset_name == "euroc") {
            BenchmarkManager::benchmark_ORB_SLAM2(DatasetName::EuRoC, dataset_path, "EuRoC.yaml", false, "MH05");
        } else if (dataset_name == "kitti") {
            BenchmarkManager::benchmark_ORB_SLAM2(DatasetName::KITTI, dataset_path, "KITTI00-02.yaml", false);
        }
    }
    else if (system_name == "orb_slam3")
    {
        if (dataset_name == "tum") {
            BenchmarkManager::benchmark_ORB_SLAM3(DatasetName::TUM, dataset_path, "TUM1.yaml", false);
        } else if (dataset_name == "euroc") {
            BenchmarkManager::benchmark_ORB_SLAM3(DatasetName::EuRoC, dataset_path, "EuRoC.yaml", false, "MH05");
        } else if (dataset_name == "kitti") {
            BenchmarkManager::benchmark_ORB_SLAM3(DatasetName::KITTI, dataset_path, "KITTI00-02.yaml", false);
        }
    }
    return 0;
}
