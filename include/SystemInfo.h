#pragma once

#include "sys/times.h"
#include "sys/vtimes.h"
#include <string>
#include <thread>

namespace SLAM_Benchmark
{

    class SystemInfo
    {
    private:
        static const std::string JETSON_GPU_POWER_FILE_PATH;
        static const std::string JETSON_CPU_POWER_FILE_PATH;
        static const std::string JETSON_SOC_POWER_FILE_PATH;
        static clock_t last_cpu, last_sys_cpu, last_user_cpu;

    public:
        static const int processor_num;

        static void init();
        /* 
        CPU percent usage:
            SystemInfo::init();
            executa_heavy_computation();
            getCurrentProcessCPUPercent(); // get CPU percent
        */
        static double getCurrentProcessCPUPercent();
        // Memory usage in KB
        static int getCurrentProcessVirtualMemoryUsed();
        // Memory usage in KB
        static int getCurrentProcessPhysicalMemoryUsed();
        // Current GPU power (in mW, only available in Jetson board)
        static int getCurrentCPUPower();
        // Current SOC power (in mW, only available in Jetson board)
        static int getCurrentGPUPower();
        // Current CPU power (in mW, only available in Jetson board)
        static int getCurrentSOCPower();
        // Current total power (in mW, only available in Jetson board)
        static int getCurrentTotalPower();
    };

}