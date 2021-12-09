#include "SystemInfoManager.h"
#include "Utility.h"
#include <iostream>

using namespace std;

int main()
{

    cout << "Running a simple benchmark..." << endl;
    SLAM_Benchmark::SystemInfoManager::startMonitor();
    SLAM_Benchmark::Utility::simpleComputationTask(100000000);
    struct SLAM_Benchmark::SystemInfoRecord* record = SLAM_Benchmark::SystemInfoManager::stopMonitor();

    cout << "CPU: " << record->cpu_percent_meter.getMean() << "%" << endl;
    cout << "Virtual Memory: " << record->virtual_memory_meter.getMean() << "KB" << endl;
    cout << "Physical Memory: " << record->physical_memory_meter.getMean() << "KB" << endl;
    cout << "CPU Power: " << record->cpu_power_meter.getMean() << "mW" << endl;
    cout << "GPU Power: " << record->gpu_power_meter.getMean() << "mW" << endl;
    cout << "SOC Power: " << record->soc_power_meter.getMean() << "mW" << endl;

    return 0;
}