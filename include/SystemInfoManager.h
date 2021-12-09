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
        static SystemInfoRecord* history_record;
        static std::thread record_thread;
        static std::atomic_bool thread_flag;
        static bool cpu_power_available;
        static bool gpu_power_available;
        static bool soc_power_available;

        static void recordInfo();

    public:
        static void printAvailableInfoSummary();

        static void verboseCurrentSystemInfo();

        static void startMonitor(const uint32_t interval = 100);

        static struct SystemInfoRecord* stopMonitor();

    };
}