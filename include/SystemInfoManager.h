#pragma once

#include "SystemInfo.h"
#include "Meter.h"

#include <iostream>
#include <atomic>

namespace SLAM_Benchmark
{
    struct SystemInfoRecord
    {
        Meter<double> cpu_percent_meter;
        Meter<int> virtual_memory_meter;
        Meter<int> physical_memory_meter;
        Meter<int> cpu_power_meter;
        Meter<int> gpu_power_meter;
        Meter<int> soc_power_meter;
    };

    class SystemInfoManager
    {
    private:
        static SystemInfoRecord* m_history_record;
        static std::thread m_record_thread;
        static std::atomic_bool m_thread_flag;

        static void recordInfo();

    public:
        static void printAvailableInfoSummary();

        static void verboseCurrentSystemInfo();

        static void startMonitor(const uint32_t interval = 100);

        static struct SystemInfoRecord* stopMonitor();

        static bool isCPUPowerAvailable();

        static bool isGPUPowerAvailable();

        static bool isSOCPowerAvailable();
    };
}