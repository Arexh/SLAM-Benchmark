#include "SystemRecorder.h"
#include "Utility.h"

#include <thread>
#include <iostream>

int main()
{
    std::vector<std::thread> threads;
    SLAM_Benchmark::Utility::thread_flag = true;
    SLAM_Benchmark::SystemRecorder system_info_recorder(SLAM_Benchmark::SystemName::ORB_SLAM2);
    system_info_recorder.recordSystemStart();
    threads.push_back(std::thread(SLAM_Benchmark::Utility::simpleComputationTask, 1000000000));
    // threads.push_back(std::thread(SLAM_Benchmark::Utility::timedTask, 100, SLAM_Benchmark::SystemInfoManager::verboseCurrentSystemInfo));
    for (auto &th : threads) {
        th.join();
        SLAM_Benchmark::Utility::thread_flag = false;
    }
    system_info_recorder.recordSystemStop();
    std::cout << system_info_recorder.summary() << std::endl;
    return 0;
}