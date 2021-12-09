#include "SystemInfoManager.h"

#include <chrono>
#include <thread>
#include "math.h"

#include <vector>

using namespace std;

namespace SLAM_Benchmark
{

    void printWhetherAvailable(const std::string &info, const double val);
    
    double simpleBenchmark();

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

    // copy from: https://stackoverflow.com/questions/35525543/multithread-program-in-c-shows-the-same-performance-as-a-serial-one
    double SystemInfoManager::simpleComputationTask(unsigned long long loops)
    {
        volatile double x;
        for (unsigned long long i = 0; i < loops; i++)
        {
            x = sin(sqrt(i) / i * 3.14159);
        }
        return x;
    }

    double simpleBenchmark()
    {
        // run a simple task (multi-threads), return the cpu percent used in this process
        SystemInfo::init();
        std::vector<std::thread> threads;
        for (int i = 0; i < SystemInfo::processor_num; i++)
            threads.push_back(std::thread(SystemInfoManager::simpleComputationTask, 100000000));
        for (auto &th : threads)
            th.join();
        return SystemInfo::getCurrentProcessCPUPercent();
    }

    inline void printWhetherAvailable(const std::string &info, const double val)
    {
        cout << info << (val <= 0 ? "Unavailable" : "Available") << " (current value: " << val << ")" << endl;
    }

}