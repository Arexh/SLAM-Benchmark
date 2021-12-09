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
    struct SystemInfoRecord* SystemInfoManager::history_record;
    
    thread SystemInfoManager::record_thread;

    atomic_bool SystemInfoManager::thread_flag;
    
    bool SystemInfoManager::cpu_power_available = SystemInfo::getCurrentCPUPower() != -1;

    bool SystemInfoManager::gpu_power_available = SystemInfo::getCurrentGPUPower() != -1;

    bool SystemInfoManager::soc_power_available = SystemInfo::getCurrentSOCPower() != -1;

    void SystemInfoManager::printAvailableInfoSummary()
    {
        cout << "Summary of available system infomations" << endl;
        cout << "-----------------------------------------------------" << endl;

        cout << "Running a simple benchmark..." << endl;
        printWhetherAvailable("CPU percent of current process: ", simpleBenchmark());
        printWhetherAvailable("Vritual memory used of current process: ", SystemInfo::getCurrentProcessVirtualMemoryUsed());
        printWhetherAvailable("Physical memory used of current process: ", SystemInfo::getCurrentProcessPhysicalMemoryUsed());

        printWhetherAvailable("CPU power (only Jetson Nano): ", SystemInfo::getCurrentCPUPower());
        printWhetherAvailable("GPU power (only Jetson Nano): ", SystemInfo::getCurrentGPUPower());
        printWhetherAvailable("SOC power (only Jetson Nano): ", SystemInfo::getCurrentSOCPower());
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

    void SystemInfoManager::startMonitor(const uint32_t interval)
    {
        history_record = new SystemInfoRecord;
        Utility::thread_flag = true;
        SystemInfo::init();
        record_thread = thread(Utility::timedTask, 100, recordInfo);
    }

    struct SystemInfoRecord* SystemInfoManager::stopMonitor()
    {
        Utility::thread_flag = false;
        record_thread.join();
        return history_record;
    }

    void SystemInfoManager::recordInfo()
    {
        history_record->cpu_percent_meter.update(SystemInfo::getCurrentProcessCPUPercent());
        history_record->virtual_memory_meter.update(SystemInfo::getCurrentProcessVirtualMemoryUsed());
        history_record->physical_memory_meter.update(SystemInfo::getCurrentProcessPhysicalMemoryUsed());
        if (cpu_power_available)
            history_record->cpu_power_meter.update(SystemInfo::getCurrentCPUPower());
        if (gpu_power_available)
            history_record->gpu_power_meter.update(SystemInfo::getCurrentGPUPower());
        if (soc_power_available)
            history_record->soc_power_meter.update(SystemInfo::getCurrentSOCPower());
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
    cout << info << (val == -1 ? "Unavailable" : "Available") << " (current value: " << val << ")" << endl;
}