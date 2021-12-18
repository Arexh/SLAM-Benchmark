#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <unistd.h>

#include <System.h>
#include "DatasetLoader.h"
#include "BenchmarkManager.h"

using namespace std;
using namespace SLAM_Benchmark;

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: path_to_sequence" << endl;
        return 1;
    }

    BenchmarkManager::benchmark_ORB_SLAM2(DatasetName::TUM, argv[1], "TUM1.yaml", true);

    return 0;
}
