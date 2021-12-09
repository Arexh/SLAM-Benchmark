#include "SystemInfoManager.h"
#include "Utility.h"

#include <chrono>
#include <thread>
#include "math.h"

#include <vector>

using namespace std;

void printWhetherAvailable(const std::string &info, const double val);

double simpleBenchmark();

namespace SLAM_Benchmark
{

    void SystemInfoManager::printAvailableInfoSummary()
    {
        cout << "Summary of available system infomations" << endl;
        cout << "-----------------------------------------------------" << endl;

        printWhetherAvailable("CPU percent of current process: ", simpleBenchmark());
        printWhetherAvailable("Vritual memory used of current process: ", SystemInfo::getCurrentProcessVirtualMemoryUsed());
        printWhetherAvailable("Physical memory used of current process: ", SystemInfo::getCurrentProcessPhysicalMemoryUsed());

        printWhetherAvailable("CPU power (only Jetson Nano): ", SystemInfo::getCurrentCPUPower());
        printWhetherAvailable("GPU power (only Jetson Nano): ", SystemInfo::getCurrentCPUPower());
        printWhetherAvailable("SOC power (only Jetson Nano): ", SystemInfo::getCurrentCPUPower());
        printWhetherAvailable("Total power (only Jetson Nano): ", SystemInfo::getCurrentTotalPower());

        cout << "-----------------------------------------------------" << endl;
    }

    void SystemInfoManager::verboseCurrentSystemInfo()
    {
        std::cout << "CPU usage: " << SystemInfo::getCurrentProcessCPUPercent() << "%, "
            << "Physical Memory usage: " << SystemInfo::getCurrentProcessPhysicalMemoryUsed() << "KB, "
            << "Virtual Memory usage: " << SystemInfo::getCurrentProcessVirtualMemoryUsed()
            << "KB" << std::endl;
    }
}

double simpleBenchmark()
{
    // run a simple task (multi-threads), return the cpu percent used in this process
    SLAM_Benchmark::SystemInfo::init();
    std::vector<std::thread> threads;
    for (int i = 0; i < SLAM_Benchmark::SystemInfo::processor_num; i++)
        threads.push_back(std::thread(SLAM_Benchmark::Utility::simpleComputationTask, 100000000));
    for (auto &th : threads)
        th.join();
    return SLAM_Benchmark::SystemInfo::getCurrentProcessCPUPercent();
}

inline void printWhetherAvailable(const std::string &info, const double val)
{
    cout << info << (val <= 0 ? "Unavailable" : "Available") << " (current value: " << val << ")" << endl;
}