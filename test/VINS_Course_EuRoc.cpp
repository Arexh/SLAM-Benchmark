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

    BenchmarkManager::benchmark_VINS_Course(DatasetName::EuRoC, argv[1], "EuRoC.yaml", false, "MH05");

    return 0;
}
