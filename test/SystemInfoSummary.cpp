#include "SystemInfoManager.h"
#include "stdio.h"
#include "matplotlibcpp.h"
#include "Utility.h"

using namespace SLAM_Benchmark;

int main()
{
    std::vector<std::thread> threads;
    bool thread_flag = true;
    threads.push_back(std::thread(Utility::simpleComputationTask, 1000000000));
    SystemInfo::init();
    threads.push_back(std::thread(Utility::timedTask, 100, thread_flag, SystemInfoManager::verboseCurrentSystemInfo));
    for (auto &th : threads) {
        th.join();
        thread_flag = false;
    }
    std::cout << SystemInfo::getCurrentProcessCPUPercent() << std::endl;
    return 0;
}