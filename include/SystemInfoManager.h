#include "SystemInfo.h"
#include <iostream>

namespace SLAM_Benchmark
{
    class SystemInfoManager
    {
    public:
        static void printAvailableInfoSummary();

        static void startMonitor();

        static void stopMonitor();

        static double simpleComputationTask(unsigned long long loops);
    };
}