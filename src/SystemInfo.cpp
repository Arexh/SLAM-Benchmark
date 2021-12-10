#include "SystemInfo.h"
#include "Utility.h"

int parseLine(char *line);

int readPowerFromFile(const std::string &file_name);

namespace SLAM_Benchmark
{
    // copy from: https://inaj012.medium.com/nvidia-jetson-nano-xavier-power-monitoring-62d374e9dc81
    const std::string SystemInfo::JETSON_GPU_POWER_FILE_PATH = "/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_power1_input";

    const std::string SystemInfo::JETSON_CPU_POWER_FILE_PATH = "/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_power2_input";

    const std::string SystemInfo::JETSON_SOC_POWER_FILE_PATH = "/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_power0_input";

    const int SystemInfo::processor_num = std::thread::hardware_concurrency();

    clock_t SystemInfo::last_cpu = 0, SystemInfo::last_sys_cpu = 0, SystemInfo::last_user_cpu = 0;

    void SystemInfo::init()
    {
        struct tms time_sample;
        last_cpu = times(&time_sample);
        last_sys_cpu = time_sample.tms_stime;
        last_user_cpu = time_sample.tms_utime;
    }

    // copy from: https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
    double SystemInfo::getCurrentProcessCPUPercent()
    {
        struct tms time_sample;
        clock_t now;
        double percent;

        now = times(&time_sample);
        if (now <= last_cpu || time_sample.tms_stime < last_sys_cpu ||
            time_sample.tms_utime < last_user_cpu)
        {
            //Overflow detection. Just skip this value.
            percent = -1.0;
        }
        else
        {
            percent = (time_sample.tms_stime - last_sys_cpu) +
                      (time_sample.tms_utime - last_user_cpu);
            percent /= (now - last_cpu);
            percent *= 100;
        }
        last_cpu = now;
        last_sys_cpu = time_sample.tms_stime;
        last_user_cpu = time_sample.tms_utime;

        return Utility::roundDecimal(percent, 2);
    }

    int SystemInfo::getCurrentProcessVirtualMemoryUsed()
    {
        FILE *file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];

        while (fgets(line, 128, file) != NULL)
        {
            if (strncmp(line, "VmSize:", 7) == 0)
            {
                result = parseLine(line);
                break;
            }
        }
        fclose(file);
        //Note: this value is in KB!
        return result;
    }

    int SystemInfo::getCurrentProcessPhysicalMemoryUsed()
    {
        FILE *file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];

        while (fgets(line, 128, file) != NULL)
        {
            if (strncmp(line, "VmRSS:", 6) == 0)
            {
                result = parseLine(line);
                break;
            }
        }
        fclose(file);
        //Note: this value is in KB!~
        return result;
    }

    int SystemInfo::getCurrentCPUPower()
    {
        return readPowerFromFile(JETSON_CPU_POWER_FILE_PATH);
    }

    int SystemInfo::getCurrentGPUPower()
    {
        return readPowerFromFile(JETSON_GPU_POWER_FILE_PATH);
    }

    int SystemInfo::getCurrentSOCPower()
    {
        return readPowerFromFile(JETSON_SOC_POWER_FILE_PATH);
    }

    int SystemInfo::getCurrentTotalPower()
    {
        int total_power = 0;

        int cpu_power = getCurrentCPUPower();
        int gpu_power = getCurrentGPUPower();
        int soc_power = getCurrentSOCPower();

        total_power += cpu_power != -1 ? cpu_power : 0;
        total_power += gpu_power != -1 ? gpu_power : 0;
        total_power += soc_power != -1 ? soc_power : 0;

        return total_power;
    }
}

int readPowerFromFile(const std::string &file_name)
{
    if (SLAM_Benchmark::Utility::checkIfFileExists(file_name))
    {
        return SLAM_Benchmark::Utility::readFileContentInt(file_name);
    }
    else
    {
        return -1;
    }
}

int parseLine(char *line)
{
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char *p = line;
    while (*p < '0' || *p > '9')
        p++;
    line[i - 3] = '\0';
    i = atoi(p);
    return i;
}