#include "SystemInfoManager.h"
#include "Utility.h"
#include "stdio.h"
#include "matplotlibcpp.h"

using namespace SLAM_Benchmark;

int main()
{
    std::vector<std::thread> threads;
    Utility::thread_flag = true;
    threads.push_back(std::thread(Utility::simpleComputationTask, 1000000000));
    SystemInfo::init();
    threads.push_back(std::thread(Utility::timedTask, 100, SystemInfoManager::verboseCurrentSystemInfo));
    for (auto &th : threads) {
        th.join();
        Utility::thread_flag = false;
    }
    std::cout << SystemInfo::getCurrentProcessCPUPercent() << std::endl;
    // SystemInfoManager::printAvailableInfoSummary();
    return 0;
}