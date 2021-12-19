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
    struct SystemInfoRecord *SystemInfoManager::m_history_record;

    thread SystemInfoManager::m_record_thread;

    atomic_bool SystemInfoManager::m_thread_flag;

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
        m_history_record = new SystemInfoRecord;
        Utility::thread_flag = true;
        SystemInfo::init();
        m_record_thread = thread(Utility::timedTask, 100, recordInfo);
    }

    struct SystemInfoRecord *SystemInfoManager::stopMonitor()
    {
        Utility::thread_flag = false;
        m_record_thread.join();
        return m_history_record;
    }

    void SystemInfoManager::recordInfo()
    {
        m_history_record->cpu_percent_meter.update(SystemInfo::getCurrentProcessCPUPercent());
        m_history_record->virtual_memory_meter.update(SystemInfo::getCurrentProcessVirtualMemoryUsed());
        m_history_record->physical_memory_meter.update(SystemInfo::getCurrentProcessPhysicalMemoryUsed());
        if (isCPUPowerAvailable())
            m_history_record->cpu_power_meter.update(SystemInfo::getCurrentCPUPower());
        if (isGPUPowerAvailable())
            m_history_record->gpu_power_meter.update(SystemInfo::getCurrentGPUPower());
        if (isSOCPowerAvailable())
            m_history_record->soc_power_meter.update(SystemInfo::getCurrentSOCPower());
    }

    bool SystemInfoManager::isCPUPowerAvailable()
    {
        return SystemInfo::getCurrentCPUPower() != -1;
    }

    bool SystemInfoManager::isGPUPowerAvailable()
    {
        return SystemInfo::getCurrentGPUPower() != -1;
    }

    bool SystemInfoManager::isSOCPowerAvailable()
    {
        return SystemInfo::getCurrentSOCPower() != -1;
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